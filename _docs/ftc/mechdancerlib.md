---
title: 机械舞者 FTC 编程库
category: FTC
order: 1
---
Gtihub 仓库：[mechdancerlib](https://github.com/MechDancer/mechdancerlib)
## Abstract

*mechdancerlib* is a FTC robot library written by Kotlin. It provides structurize device tree of the robot, including the encapsulation and dispatching, which make program easier to use.

## Motivation

FTC Control System has kept out of the category of embedded development, thus it's very friendly to people who is new to programming, however, the usability of it is not good enough. For example: 

```kotlin
abstract class UnicornBaseOpMode : OpMode() {

    protected lateinit var dumperArm: DCMotor
    private lateinit var dumperPushRod: Servo

    private lateinit var dustpan: Servo

    protected lateinit var collectorArm: DCMotor
    protected lateinit var collectorIntake: CRServo

    protected lateinit var expanderMotor: Motor
    protected lateinit var expanderLock: Servo

    protected lateinit var lifterTouch: RevTouchSensor

    protected lateinit var lifter: DCMotor

    override fun init() {
        lifterTouch = hardwareMap.get(...)
        lifter = hardwareMap.get(...)
        collectorArm = hardwareMap.get(...)
        collectorIntake = hardwareMap.get(...)
        expanderMotor = hardwareMap.get(...)
        expanderLock = hardwareMap.get(...)
        dustpan = hardwareMap.get(...)
        dumperPushRod = hardwareMap.get(...)
        dumperArm = hardwareMap.get(...)
    }

    override fun loop() {
		...
    }
    
    override fun stop() {
		...
    }

}
```

Pay attention to `init()`, all devices attached to this robot which defined in one class (although it's abstract), should be bind through `hardwereMap`. Those devices are flattened in program, and it's not easy to use or manage. Therefore, we defined a thing called *Structure*, we can distinctly layered the program the robot  through combining many *structures*. This is a part of our teleop:

```kotlin
 with(robot) {
     //Chassis
     chassis.descartes {
         x = master.leftStick.y
         y = master.leftStick.x
         w = 0.7 * master.rightStick.x
     }
     //Helper
     expander.expandState = when {
         helper.up.bePressed()   -> Expanding
         helper.down.bePressed() -> Shrinking
         else                    -> Expander.ExpandState.Stop
     }
     expander.lockState = when {
         helper.left.isPressing()  -> Lock
         helper.right.isPressing() -> Unlock
         else                      -> expander.lockState
     }
     collector.armState = when {
         helper.leftStick.y > 0.5  -> Lifting
         helper.leftStick.y < -0.5 -> Dropping
         else                      -> Collector.ArmState.Stop
     }
 }
```

Control details of devices are hidden, outer user should only focus on the whole robot control logic.

## Overview

### Terminology

* A *structure* — is an specific definition of a robot part, like a rocker arm or a chassis. Once it is attached to robot as substructure, it will has a similar life-cycle.
* A *monomeric structure* — is a kind of *structure* which does not have substructure. All real devices are *monomeric structures* because they can't have childes.
* A *composite structure* — a *structure* that have substructures. For example, a mecanum chassis is a *composite structure* which has four motors as substructure.
* *Robot* — is a unique *composite structure*, all structures are attached to it.

### Structure interface and classes

```kotlin
interface Structure {
    
    val name: String

    fun run()

    override fun toString(): String
    
}
```

This is a simple *structure* definition. It has a name, can be to string. `run()` function defines what this  *structure* should do when robot is running.

```kotlin
abstract class MonomericStructure(override val name: String) : Structure

abstract class CompositeStructure(override val name: String) : Structure {
    abstract val subStructures: List<Structure>
}
```

The only difference between a *composite structure* and a *monomeric structure* is that a *composite structure* can have substructures attached to it.

### Device

Device is a core of control system. Encapsulations done by FIRST are almost perfect, however, the only drawback is that the output and input data range are not unified. On the basis of correcting this, we create wrappers to let devices adapt this library. Furthermore, we high abstract the behavior of devices to make control more pure. We split devices which only perform output into *effector*, others into *sensor*.

#### Effectors

##### Motor

```kotlin
interface Motor : Structure {

    var power: Double

    var direction: Direction

    override var lock: Boolean
    
    enum class Direction(internal val sign: Int) {
        FORWARD(+1),
        REVERSE(-1);
    }
}
```

An user who control this motor should only notice the `power` and `direction` (ignore `lock` for the moment), implementing details is not important. The range of  `power` is from -1.0 to 1.0.

##### Continuous Servo

```kotlin
interface ContinuousServo : Structure {
    
    var power: Double

    var pwmOutput: Boolean
    
}
```

The only difference from motor is that it's a pwm controlled device, whose output can be shutdown. The range of `power` is from -1.0 to 1.0.

##### Servo

```kotlin
interface Servo : Structure {
    
    var position: Double

    var pwmOutput: Boolean
    
}
```

It is a pwm device having a position output.  The range of `position` depends on device configuration. 

> The unit of `position` is $rad$, which you have to specified in configuration.

#### Sensors

##### Encoder

```kotlin
interface Encoder : Structure {

    val position: Double

    val speed: Double

    fun reset(off: Double)

}
```

We separate the encoder from the motor. We believe that motors and encoders should be independent. This is a fly in the ointment in FIRST design. Certainly, they bind to a same `DCMotor` in low-level code. 

> The unit of `position` is $rad$, `speed` is $rad/s$.

##### Touch Sensor

```kotlin
interface TouchSensor : Structure {

    val force: Double

    fun bePressed(): Boolean

    fun isPressing(): Boolean

    fun isReleasing(): Boolean
    
}
```

`bePressed()` returns the current state of button, `isPressing()` returns true when the first time the button be pressed, `isReleasing()` is similar to `isPressing()`. We process `Gamepad` in the same way, please see [Gamepad](#Button).

##### Color Sensor

```kotlin
interface ColorSensor : Structure {

    val colorData: ColorData

    var enableLed: Boolean

    data class ColorData(
        val red: Int, 
        val green: Int, 
        val blue: Int, 
        val alpha: Int, 
        val argb: Int) 

}
```

##### Voltage Sensor

```kotlin
interface VoltageSensor : Structure {

    val voltage: Double
    
}
```

### Pre-cast structures

We provides following common structures:

#### Motor with encoder

As mentioned before, we divide the `Motor` and the `Encoder` into two parts according to their behavior. Although this is reasonable, in some cases it brings a lot of inconvenience. Therefore, a combination of motor and encoder is established. It is a `CompositeStructure`, have two substructures: a motor and a encoder. Through the combination of two devices, some new  behaviors come out. At the beginning of the design, we thought for a long time whether this class should be polymorphic. In other word, can `MotorWithEncoder` be `cast` to `Motor` or `Encoder`? i.e. have the behaviors of both of the above. That sounds right, but it's a special case. Think about it: can a mechanical arm with only one motor be counted as a motor? Actually, it is the pure combination of the two devices, and it just has the behavior of the two that makes polymorphism reasonable. And actually that's wrong. However for the sake of constraint its functions, we let it inherit class `Motor` and `Encoder`, let its substructures  work as delegates. Let's see some details:

```kotlin
interface MotorWithEncoder : Motor, Encoder, Structure {

    var mode: Mode

    var targetSpeed: Double

    var targetPosition: Double

    override var lock: Boolean

    enum class Mode {
        SPEED_CLOSE_LOOP, OPEN_LOOP, POSITION_CLOSE_LOOP,STOP
    }


}
```

What behaviors mentioned above are that this combination has kinds of abilities to close-loop-control.

#### Chassis

Chassis plays an important part of robot, so we provide basic definition of this structure:

```kotlin
abstract class Chassis(motorsConfig: Array<Pair<String, Motor.Direction>>, enable: Boolean)
    : CompositeStructure("null_chassis"), AutoCallable, SmartLogger {

    override val subStructures: List<Motor> =
        motorsConfig.map { MotorImpl(it.first, enable, it.second) }

    open var powers = DoubleArray(motorsConfig.size) { .0 }
        get() = field.standardizeBy(maxPower)
        set(value) {
            if (value.size != field.size)
                warn("Illegal size powers: ${value.size} ≠ ${field.size}")
            else
                field = value
        }

    var maxPower = 1.0
        set(value) {
            if (value !in -1.0..1.0)
                warn("Illegal max power: $maxPower ∉ [-1,1]")
            else
                field = value
        }

    /**
     * Standardizes powers
     *
     * If the input power has a value greater than the input maximum constraint,
     * the maximum is adjusted to the constraint value,
     * and the other values are scaled down.
     *
     * @param maxPower  maximum power constraint ∈ [-1,1]
     */
    private fun DoubleArray.standardizeBy(maxPower: Double) =
        map(::abs).max()!!.let {
            if (it <= abs(maxPower))
                maxPower.sign
            else
                maxPower / it
        }.let {
            DoubleArray(size) { i ->
                this[i] * it
            }
        }

    override fun run() {
        subStructures.forEachIndexed { index: Int, motor: Motor ->
            motor.power = powers[index]
        }
    }
}

```

As you've seen, chassis is a structure managing some motors assembly, which provide ability to process relationship between motors, including **standardize**. Therefore, is provided farther:

```kotlin
abstract class Omnidirectinal
(motorsConfig: Array<Pair<String, Motor.Direction>>, enable: Boolean)
    : Chassis(motorsConfig, enable) {

    /**
     * Descartes control parameters
     */
    data class Descartes(var x: Double,
                         var y: Double,
                         var w: Double) {
        operator fun times(other: Descartes) =
            Descartes(x * other.x, y * other.y, w * other.w)
    }

    /**
     * Polar control parameters
     */
    data class Polar(var rho: Double = .0,
                     var theta: Double = .0,
                     var omega: Double = .0) {
        val block: Descartes.() -> Unit = {
            x = rho * cos(theta)
            y = rho * sin(theta)
            w = omega
        }
    }

    /**
     * Tank control parameters
     */
    data class TankMode(var left: Double = .0,
                        var right: Double = .0,
                        var horizon: Double = .0) {
        val block: Descartes.() -> Unit = {
            x = left + right
            y = horizon
            w = left - right
        }
    }
    /**
     * Transforms descartes parameters to powers of each wheel.
     */
    protected abstract fun Descartes.transform(): DoubleArray

    override fun run() {
        if (!advancedControlMode)
            powers = (weights * descartes).transform()
        super.run()
    }
}
```

Omnidirectinal is a kind of omni-directional mobile in *cartesian coordinate system*, there are three methods to control it, including `Descartes`, `TankMode`, and `Polar`. Notice that subclass need to declare motors and `transform` which describes how to drive this chassis. For convenience, drive way of mecanum chassis is built-in.

```kotlin
open class Mecanum(override val name: String = "chassis",
                   enable: Boolean,
                   lfMotorDirection: Motor.Direction = Motor.Direction.REVERSE,
                   lbMotorDirection: Motor.Direction = Motor.Direction.REVERSE,
                   rfMotorDirection: Motor.Direction = Motor.Direction.FORWARD,
                   rbMotorDirection: Motor.Direction = Motor.Direction.FORWARD,
                   lfMotorName: String = "LF",
                   lbMotorName: String = "LB",
                   rfMotorName: String = "RF",
                   rbMotorName: String = "RB"
) : Omnidirectinal(arrayOf(
    lfMotorName to lfMotorDirection, lbMotorName to lbMotorDirection,
    rfMotorName to rfMotorDirection, rbMotorName to rbMotorDirection), enable) {


    override fun Descartes.transform() =
        doubleArrayOf(
            +x + y - w,
            +x - y - w,
            +x - y + w,
            +x + y + w)
}
```

Consequently, there is no need for you to write duplicated code anymore.

### Robot

Structure tree is the core of this library, every node of which is a structure. The foregoing implies that all devices have become *monomeric structure* which don't have substructures. Robot is the root of this tree, middleware structures we defined are leaves  under the robot, devices are at the lowest nodes. Therefore, parent nodes just need to manage theirs sons, call `run()` of them. Because of the transitivity, robot is the parent of all structures.  Lets see an example:

```kotlin
class DummyRobot : Robot(
    "dummyRobot",
    Mecanum(enable = true),
    DummyArm(),
    DeviceFactory.colorSensor("colorSensor") {
        enable = true
    }
) {

    @Inject
    lateinit var colorSensor: ColorSensor

    @Inject
    lateinit var voltageSensor: VoltageSensor

    @Inject("mecanumChassis")
    lateinit var chassis: Mecanum

    @Inject
    private lateinit var dummyArm: DummyArm

    var armState: DummyArm.ArmState = DummyArm.ArmState.DOWN

    override fun run() {
        dummyArm.armState = armState
    }
}
```

Substructures' instantiation are provided via the constructor of  Robot, and are used through `Inject`. You may think that it's redundant that constructing a structure on your own but which can't be a member of parent structure. In this case, substructures are created by your self, however, you can't manage the life cycle of a structure frequently.

### Op Mode

Op mode is the entry point of whole robot control program. All thins we can do are based on op mode, and it has a `hardwareMap` providing us a easy why to control device interfaces.  Since we have our robot, we can write our op mode. Devices are bind to theirs corresponding structure wrappers in `init`, robot's `run()` is called in `loop`. We do something quite import, so we ban the overriding of life cycle call back functions and provide new methods to let user to override:

```kotlin
abstract class BaseOpMode<T : Robot> : OpModeWithRobot<T>() {

    protected val robot: T = createRobot()

    ...
    
    abstract fun initTask()

    open fun initLoopTask() {}

    open fun startTask() {}

    abstract fun loopTask()

    abstract fun stopTask()

}
```

Notice at `robot`, `BaseOpMode` have a generic parameter `T:Robot`, which means robot instance can be constructed through its type. However, we have to ensure that robot class can't be `abstract` and muse have a public constructor without parameters.

### Gamepad

Let's see the definition in robot core:

```java
public class Gamepad {

	public float left_stick_x = 0f;

	public float left_stick_y = 0f;

	public float right_stick_x = 0f;

	public float right_stick_y = 0f;

	public boolean dpad_up = false;

	public boolean dpad_down = false;

	public boolean dpad_left = false;

	public boolean dpad_right = false;

	public boolean a = false;

	public boolean b = false;

	public boolean x = false;

	public boolean y = false;

	public boolean start = false;

	public boolean back = false;

	public boolean left_bumper = false;

	public boolean right_bumper = false;

	public boolean left_stick_button = false;

	public boolean right_stick_button = false;

	public float left_trigger = 0f;

	public float right_trigger = 0f;
	
}
```

FIRST provides a convenient way to let us use gamepad directly, however, there are some little problems:

* Buttons and joysticks lie in one class flatly, `float` and `double` members mix together.

* Joysticks have reversed vertical axis.

* The way to realize pressing a button only one time (even keep pressing) to execute some actions  one time (e.g. when user press `x` button, a motor should start to run, etc.) really make people confused.

Thus, we create few classes to reach better encapsulation.

#### Button

```kotlin
class Button : IGamePadComponent<Boolean> {

    private var last = false

    fun bePressed() = raw

    fun isPressing() = !last && bePressed()

    fun isReleasing() = last && !bePressed()

    override var raw: Boolean = false
        set(value) {
            last = field
            field = value
        }
}
```

As you see, `bePressed()` returns the current state of the button, while you keep pressing, `isPressing()` returns true only one time, `isReleasing()` returns true when you stop pressing.

#### Stick

```kotlin
class Stick : IGamePadComponent<DoubleArray> {

    val x get() = raw[0]

    val y get() = raw[1]

    val theta get() = Math.atan2(y, x)

    val radius get() = sqrt(x * x + y * y)

    var feel = 1.0

    var diedArea = .0

    private fun mapExpression(x: Double, f: Double) =
        (f / ((1 - f) * (x + 1) + f) - f) / (f - 1)

    private fun nonlinearMap(x: Double, feel: Double, diedArea: Double) = when {
        Math.abs(x) < diedArea -> .0
        feel == 1.0            -> x
        x < 0                  -> mapExpression(x, feel)
        else                   -> -mapExpression(-x, feel)
    }

    override var raw: DoubleArray = doubleArrayOf(.0, .0)
        set(value) {
            field = doubleArrayOf(
                nonlinearMap(value[0], feel, diedArea),
                nonlinearMap(-value[1], feel, diedArea))
        }

}
```

We provides a *feel* represents tactile coefficient, hence, the stick data will be more smooth. Besides, we reversed back y axis.

#### Trigger

```kotlin
   var pressingThreshold = 0.7

    private var last = false

    fun bePressed() = raw > pressingThreshold

    fun isPressing() = !last && bePressed()

    fun isReleasing() = last && !bePressed()

    override var raw: Double = .0
        set(value) {
            last = field > pressingThreshold
            field = value
        }
```

Usually, triggers are used as button is very common, so we add a `pressingThreshold` to make it become to a button.

#### Overall

```kotlin
class Gamepad {

    val leftBumper = Button()
    val rightBumper = Button()

    val a = Button()
    val b = Button()
    val x = Button()
    val y = Button()

    val up = Button()
    val down = Button()
    val left = Button()
    val right = Button()
    
    val leftStick = Stick()
    val rightStick = Stick()

    val leftTrigger = Trigger()
    val rightTrigger = Trigger()
    
}
```

## Sample

Let's see a part of our robot:

```kotlin
class Dustpan : AbstractStructure(
    "dustpan", {
    servo("servo") {
        origin = .0
        ending = PI
        enable = true
    }
}), Resettable {

    @Inject
    private lateinit var servo: Servo

    @Volatile
    private var isBusy = false

    var state: State by Delegates.observable(State.Down) { _, _, value ->
        servo.position = when (value) {
            State.Up   -> DUSTPAN_UP_RAD
            State.Down -> DUSTPAN_DOWN_RAD
        }
    }


    fun dump(times: Int) {
        if (isBusy) return
        isBusy = true
        GlobalScope.launch {
            repeat(times) {
                state = State.Up
                delay(DUSTPAN_RUNNING_DELAY)
                state = State.Down
                delay(DUSTPAN_RUNNING_DELAY)
            }
            isBusy = false
        }
    }

    enum class State {
        Up, Down
    }

    override fun reset() {
        state = State.Down
    }

    override fun toString(): String = "${javaClass.simpleName} | State: $state"


}
```

It is called `Dustpan`, which is driven by a 180° servo. According the design of library, dustpan is a structure, having device servo as its substructure. Then we abstract two state of this part — `Up` and `Down`, which represent two position of servo. For easy to modify parameters and reduce coupling, `DUSTPAN_UP_RAD` and `DUSTPAN_DOWN_RAD` lie in other file. Outside can change the value of variable `state` to change the real state of this robot part.

```kotlin
class UnicornRobot : Robot(
    "unicorn_robot",
    ...
    Dustpan(),
    ...
) {
	...

    @Inject
    lateinit var dustpan: Dustpan
}
```

We have to add it as substructure of robot mentioned before. Since then, we can write a teleop to control it:

```kotlin
class UnicornRemote : RemoteControlOpMode<UnicornRobot>() {
	...
    override fun loop(master: Gamepad, helper: Gamepad){
    	//Master
    	 if (helper.y.isPressing()) 
         	robot.dustpan.dump(1)
         when {
         	master.y.bePressed() -> robot.dustpan.state=State.Up
         	master.b.bePressed() -> robot.dustpan.state=State.Down
         }
    }
    ...
}
```

