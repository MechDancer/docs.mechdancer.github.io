---
title: 状态机
category: 控制
order: 2
---
Gtihub 仓库：mechdancerlib [的一部分](https://github.com/MechDancer/mechdancerlib/blob/master/main/src/main/java/org/mechdancer/ftclib/algorithm/StateMachine.kt)

## Abstract

*state machine* is a light-weight tool facilitate autonomous programming. It provides capability of collaborative working in single thread driven through loop. Thread safety is not supported.

## Motivation

Typically, a op mode will be like this:

```kotlin
class DemoOpMode : BaseOpMode<UnicornRobot>() {
    override fun initTask() {

    }

    override fun loopTask() {

    }

    override fun stopTask() {
    }
}
```

As for `BaseOpMode`, it's a encryption of `OpMode`, which you can find in [mechdancerlib](https://www.mechdancer.org/Documentation/mechdancerlib/). Notice that the main logic we should write will be in `loopTask()`, which called circularly. For example, there is a simple requirement: robot moves forward for one second, then uses locator to move to a (5,5). Think about this process, it's kind of a action chain, including delay and close-loop control. Those two are time-consuming procedures, as they ought to work in a loop. However, there is a problem that how to ensure that two actions can be run precisely in loop. Obviously, blocking the thread is not allowed, it's the key that loop control structure should be converted to linear actions. Let's see the implementation of the requirement mentioned above through linear state machine:

```
demoLinearStateMachine
    .add {
        robot.chassis.descartes {
            x = 1.0
            y = .0
            w = .0
        }
        NEXT
    }
    .add(Delay(1000))
    .add(PIDMove(vector3DOf(0, 0, 0), .1, 0.2))
```

 What needs to be done next is calling the `invoke()` of the state machine in loop:

```kotlin
override fun loopTask() {
    demoLinearStateMachine()
}
```

Actions are described as transfer of states via state machine, so that they can be run correctly in loop.

## Overview

### Primitives

```kotlin
typealias StateMember<T> = (() -> T)

typealias StateMachine = StateMember<Boolean>
```

`StateMember<T>` is a alias of `(() -> T)`, which is lambda without parameter, return type `T`. In other words, arbitrary such lambda can be seen as a state. `StateMachine` is `StateMember<Boolean>`, a specific state, returning if this state machine is finished.

```kotlin
const val FINISH = true
const val CONTINUE = false

const val NEXT = true
const val REPEAT = false
```

`FINISH` and `CONTINUE` mark if state machine should switch state, `NEXT` and `REPEAT` mark if one state is finished. As you seen, all primitive definitions are `typealias` or `const val`, thus cost of running can be reduced.

### Operations

```kotlin
infix fun <T> StateMember<*>.join(other: StateMember<T>): StateMember<T> = { this(); other() }

operator fun StateMachine.plus(other: StateMachine): StateMachine = {
    val a = this()
    val b = other()
    a || b
}

operator fun StateMachine.times(other: StateMachine): StateMachine = {
    val a = this()
    val b = other()
    a && b
}

fun <T> StateMember<T>.run(): T? = invoke()

fun StateMachine.runToFinish() {
    while (run() == CONTINUE);
}
```

`join()` can combine two linear members into one linearly. Binary operators `+` and `*` are similar to operators between `Boolean`:`||` and `&&` correspondingly, which mean that when either `this` or `other` finish continue, when both `this` and `other` finish continue.  `runToFinish()` will block current thread until state machine's invoke returning `true`.

## State machines

For easy to use, some preset state machine are provided:

* `LinearStateMachine`

* `LinearStateMachineWithWatchDog`
* `RepeatingLinearStateMachine`
* `StepStateMachine`
* `StandardStateMachine`
* `Delay`

For more information please see below.

### Linear state machines

Linear state machine is a concept that running states sequentially. Even if there are few linear state machine definitions provided, they don't have any superclass or interface contracting. They are named in this way due to the fact that they have similar behaviors.

```kotlin
open class LinearStateMachine : StateMachine {
    private val states: Queue<StateMember<Boolean>> = LinkedList<StateMember<Boolean>>()

    fun add(state: StateMember<Boolean>): LinearStateMachine =
        apply { states.offer(state) }

    override fun invoke(): Boolean {
        val current = states.peek()
        if (current != null && run(current) == NEXT) states.poll()
        return if (states.peek() == null) NEXT else REPEAT
    }
```

Common linear state machine add state members into a internal queue simply, and poll state member from queue when state machine is running.

 ```kotlin
open class LinearStateMachineWithWatchDog : StateMachine {
    private val driver = LinearStateMachine()

    fun add(state: StateMember<Boolean>, timeout: Long): LinearStateMachineWithWatchDog =
        apply {
            driver.add(state + Delay(timeout))
        }

    override fun invoke() = driver()
}
 ```

The only difference from common linear state machine is that each state member has a time limit, which let state machine transfer when it's timeout.

```kotlin
open class RepeatingLinearStateMachine : StateMachine {
    private val states: MutableList<StateMember<Unit>> = mutableListOf()
    private var index = 0

    fun add(state: StateMember<Unit>): RepeatingLinearStateMachine =
        apply { states.add(state) }

    override fun invoke(): Boolean {
        if (states.isEmpty()) return FINISH
        states[index++]()
        index %= states.size
        return if (0 == index) FINISH else CONTINUE
    }
}
```

Repeating linear state machine is a special linear state machine, which do not discard states when that is finished, will run state members in loop.

### Step state machine

Step state machine doesn't save state members, because current running state member will return state member to run later.

```kotlin
class StepStateMachine(private var current: StateMember<StateMember<*>?>?) : StateMachine {
    override fun invoke(): Boolean {
        @Suppress("UNCHECKED_CAST")
        current = current?.invoke() as StateMember<StateMember<*>?>?
        return if (current == null) FINISH else CONTINUE
    }
}
```

If current state returns `null`, state machine will finish.

### Delay

Usually, delay plays an important role in control. Therefore, function delay are implemented through state machine as well.

```kotlin
class Delay(delay: Long) : StateMachine {
    private val timer = MyTimer(delay)
    private var waiting: StateMember<StateMember<*>?> = { null }

    init {
        waiting = { if (timer.isFinished) null else waiting }
    }

    private val driver = StepStateMachine { timer.start(); waiting }

    override fun invoke() = driver()
}
```

Notice `driver`, the internal implementation of delay is a step state machine. `waiting` is a state member, returns `null` when time is up, itself otherwise. Actually this is a recursive lambda, step state machine uses such state member as mentioned before.

### Stellate

```kotlin
class StellateStateMachine<T>(private val core: StateMember<T>) : StateMachine {
    private val states = HashMap<T, StateMember<Unit>>()

    fun add(key: T, state: StateMember<Unit>): StellateStateMachine<T> =
        apply { states[key] = state }

    override fun invoke(): Boolean {
        val currentKey = run(core)
        if (!states.containsKey(currentKey)) return FINISH
        states[currentKey]?.invoke()
        return CONTINUE
    }
}
```

Stellate state machine is unique because `core` will be run in every loop which returns the key to next common state. Common states can be add with a key using `add()`.