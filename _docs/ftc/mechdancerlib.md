---
title: 机械舞者 FTC 编程库
category: FTC
order: 1
---
Gtihub 仓库：[mechdancerlib](https://github.com/MechDancer/mechdancerlib)
## 摘要

*mechdancerlib* 是一个使用 Kotlin 语言编写的机器人编程库。本库提供了对于机器人设备树的结构化，包括各个具体设备的封装和调度，这将会简化编程中的一些重复操作，让程序更加易用。

## 动机

FTC 机器人控制系统已经脱出了传统嵌入式开发范畴，当然，这对新人入门是非常友好的（不需要踩很多坑）。即便如此，它的易用性还有待提升。一起看看下面这个例子：

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

所有机器人附属的硬件设备都需要在这里定义。通过面向接口编程，具体设备的功能实现被隐藏了起来，向用户开放的仅是抽象的接口。因此，有着这些定义好的接口的变量需要在 `init()` 拿到底层创建实例的引用，换句话说就是向 `hardwaremap` 绑定。而且这些设备是平铺定义在类中的，并不便于管理。所以一个叫做 *结构* 的东西被定义了，它可以将机器人程序结构转化为多个层次结构的组合。看看有了它之后的遥控程序：

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

每个设备的具体控制细节被隐藏了，取而代之的是机器人部件的行为，外部只需要关心遥控的具体逻辑。

## 概览

### 术语

* *结构* — 是一个特定的机器人部件，像是单摇臂、底盘。当它与机器人挂载后边与之享有共同的生命周期。
* *单体结构* — 是一种没有子结构的*结构*。所有电机、传感器这种现实中的设备都属于*单体结构*，因为它们不可再分，没有属于自己的子结构。
* *复合结构* — 是一种拥有子结构的*结构*。例如一个麦克纳姆底盘，它就是一个拥有四个电机作为子结构的*符合结构*。
* *机器人* — 是一个特殊的*复合结构*。正常情况下所有*结构* 都是它的子结构。

### 结构的接口与类

```kotlin
interface Structure {
    
    val name: String

    fun run()

    override fun toString(): String
    
}
```

这就是*结构* 的定义，很简单吧！它具有名字，在机器人调度时具有能力去运行。

```kotlin
abstract class MonomericStructure(override val name: String) : Structure

abstract class CompositeStructure(override val name: String) : Structure {
    abstract val subStructures: List<Structure>
}
```

*符合结构* 与*单体结构* 的唯一不同在这里体现出来——后者不能有子结构。

### 设备

设备是控制系统的核心，是我们控制的目标。FIRST（Qualcomm）提供的封装已经很完美了，如果只考虑单个设备，唯一的不足在于输入和输出的单位没有统一，这给控制上带来了一些不必要的麻烦。为了修正这一点，我们为每个设备都创建了一个包包装器，同时引入了结构来适配整个库。此外，通过高度抽象设备的具体行为，控制的纯净可以得到保证。由此，设备可以被分为两类：*效应器* 与*传感器*。

#### 效应器

##### 电机

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

对于控制者而言，他们只需要关心 `power` ——功率，以及 `direction` ——方向（暂时先忽略 `lock`）。方向一般用于修正电机安装在左侧或经过传动后方向需要修正。这些功能的具体实现并不重要，总之只有这些参数是可控的。

> `power` 的范围是 `[-1.0,1.0]`。

##### 连续舵机

```kotlin
interface ContinuousServo : Structure {
    
    var power: Double

    var pwmOutput: Boolean
    
}
```

连续舵机与电机的显著不同在于它是通过 pwm 控制的设备，而 pwm 输出在运行期具有被禁用的能力。后面还有这类设备。从用途来考虑从，连续舵机没有必要再单独抽象出方向。

> `power` 的范围是 `[-1.0,1.0]`。

##### Servo

```kotlin
interface Servo : Structure {
    
    var position: Double

    var pwmOutput: Boolean
    
}
```

舵机与连续舵机很相似，只不过可控的是位置而不是功率。`position` 的范围取决于设备结构配置中的定义。当然，作为 pwm 控制的设备是可以禁用输出的。

> `position` 的单位是 $rad$，例如常见的 180° 舵机中 `position` 的取值范围为 `[0,PI]`。

#### 传感器

##### 编码器

```kotlin
interface Encoder : Structure {

    val position: Double

    val speed: Double

    fun reset(off: Double)

}
```

编码器与电机分离成了不同的借口。讲道理，编码器和电机应是相互独立的，它们相互可以独自存在。当然，在底层中编码器和电机还是绑定到了一个 `DCMotor` 上。 

> `position` 的单位是 $rad$, `speed` 是 $rad/s$。

##### 触碰传感器

```kotlin
interface TouchSensor : Structure {

    val force: Double

    fun bePressed(): Boolean

    fun isPressing(): Boolean

    fun isReleasing(): Boolean
    
}
```

`bePressed()` 返回传感器按钮的当前状态， `isPressing()` 返回在循环周期中是否这是第一个按钮被按下的周期， `isReleasing()` 和 `isPressing()` 相似，不过返回的为是否第一个松开的周期。`Gamepad` 应用了相同处理，具体细节可见[手柄](#Button)。

##### 颜色传感器

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

`ColorData` 为封装好的传感器数据，可以直接从 `colorData` 变量获取。`enableLed` 负责开关传感器上的 LED 灯（如果支持的话）。

##### 电压传感器

```kotlin
interface VoltageSensor : Structure {

    val voltage: Double
    
}
```

电压传感器与其他设备不同，它**不能**自行定义或创建实例。若要使用，在后文提到的 `Robot` 结构中在 `enableVoltageSensor` 构造器参数传入 `true`，并使用 `@Inject` 注入即可。

### 预置结构

为了便于使用，库中定义了以下通用结构：

#### 电机 x 编码器

正如上文所述， `Motor` 和 `Encoder` 根据它们的行为被分离成了两部分。从意义上来说，这是合理的，但对于使用者而言却加大了复杂度。因此，电机与编码器的结合设备就这样诞生了。它是一个*复合结构* ，拥有两个子结构：一个电机和一个编码器。简单的两个设备结合到一起诞生了一些新的功能。在库设计之初，作者思考了很久`MotorWithEncoder` 这个类是否应该是多态的。换句话说就是这类的实例是否可以被强制转换为 `Motor` 或者 `Encoder`，并让使用者能够调用它们的函数（尽管在结合之后某些函数可能是无意义的）。这听起来是正确的，其实这种结合是一种特殊情况。考虑以下情况：一个单电机机械臂可以被看作是一个电机吗？显然不行。电机 x 编码器仅仅是两个设备放到一起，而并不是一个新的事物既是编码器又是电机，只不过恰好组合后提供的新行为看起来让多态是合理的，其实不然。但是为了约束结合物的行为，它还是继承了 `Motor` 和 `Encoder`，并将实现委托给两个成员，~~真香~~。来看看具体定义：

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

可以看到，以上的结合提供了闭环的能力。

#### 底盘

底盘是机器人重要且常用的一部分，所以库中给了一个抽象：

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

如你所见，底盘聚合管理了一些电机，并处理电机之间的关系，例如**功率标准化**。有了底盘显然不够，还得来个全向底盘：

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

全向底盘是一种全自由度移动平台。许多方式来控制它，包括：笛卡尔、坦克、极坐标。最终输出都会转成笛卡尔坐标系中的速度向量。该模型的子类需要声明底盘中的电机，并且将笛卡尔速度向量对应到物理模型的输出。为了方便使用，麦克纳姆底盘内置进了库中。

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

终于不用重复写这些代码了～

### 机器人

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

