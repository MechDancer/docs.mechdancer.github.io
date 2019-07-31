---
title: TC 标准化计划（去世）
category: FTC
order: 3
---
共有三个项目，Github 组织：[StandaloneTC](https://github.com/StandaloneTC)

## Abstract

*Standalone tc* is a simple implementation of the concept of responsive robot control system development. In robot programming, in most cases, the sensor is read in the loop and the current is output to the device. Such a program structure is not friendly to development because there must be coupling between instructions and controls. Reactive  system can avoid all problems, although the bottom layer is still looping through the driver. *Standalone tc* is divided into three parts: *host* - upper-level responsive dependencies, *slave* - lower-level controller communication, *protocol* - upper-lower and lower-level communication protocols. Among them [*dataflow*](https://www.mechdancer.org/Documentation/dataflow/) is responsible for the establishment of the upper layer responsive network. In addition, *host* provides tools for managing relationships between components from [*dependency*](https://www.mechdancer.org/Documentation/dependency/).

Like its name **Standalone**, its purpose is to build a reactive robot control system that is not specific to a development platform. Therefore, when you change the platform, you only need to modify the implementation to the lower layer. We implemented this architecture first on FTC, so *slave* is the FTCRobotController App.

## Overview

### Terminology

* *Effector* — is a device that is only responsible for output, and its data node is *target*.
* *Sensor* — is a device that is only responsible for input, and its data node is *source*.
* *Robot* — is a *dynamic scope*, can load components.
* *Robot component* — is a special component with `init` and `stop` lifecycles.

### Effector

The data node of all Effector types is `OutputDriver<T>`, which can be passed to the next node periodically when receiving data in the dataflow. Effector data nodes in the link should be the last end, and it will communicate to the network even when the robot starts. Let's see most common effectors.

#### Motor

```kotlin
class Motor(name: String, private val direction: Direction = Direction.FORWARD) :
    NamedComponent<Motor>(name), Device, PowerOutput {

    override val power = OutputDriver<Double> { raw ->
        raw.checkedValue(-1.0..1.0)?.let {
            it * direction.sign
        } ?: logger.warning("Invalid motor power value: $raw").run { null }
    }

    override fun toString(): String = "${javaClass.simpleName}[$name]"

    enum class Direction(val sign: Double) {
        FORWARD(1.0), REVERSED(-1.0)
    }

    override fun stop() {
        power.close()
    }

}
```

Motor has a data node `power`.

#### Servo

```kotlin
class Servo(name: String, range: ClosedFloatingPointRange<Double>) : NamedComponent<Servo>(name),
    Device, PositionOutput, PwmOutput {

    private val mapper = Lens(-1.0, 1.0, range.start, range.endInclusive)

    override val position: OutputDriver<Double> = OutputDriver { raw ->
        raw.checkedValue(range)?.let {
            mapper(it)
        } ?: logger.warning("Invalid servo position: $raw").run { null }
    }

    override val pwmEnable: OutputDriver<Boolean> = OutputDriver()

    override fun toString(): String = "${javaClass.simpleName}[$name]"

}
```

Servo has two data nodes: `position` and `pwmEnable`.

### Sensor

All Sensors are *source* nodes. When a sensor value is updated in network communication, data is placed in its data node. It should be the starting point in the link. Let's look at the most common sensors.

#### Encoder

```kotlin
class Encoder(name: String, cpr: Double = 360.0) : NamedComponent<Encoder>(name), Sensor<EncoderData> {
    private val value = AtomicReference(EncoderData(.0, .0))

    private val ratio = 2 * PI / cpr

    /** Current position */
    val position get() = value.get().position

    /** Current speed */
    val speed get() = value.get().speed

    override val updated: ISource<EncoderData> = broadcast()

    override fun update(new: EncoderData) {
        val transformed = EncoderData(position * ratio, speed * ratio)
        if (value.getAndSet(transformed) != transformed)
            updated tryPost transformed
    }

    override fun toString(): String = "${javaClass.simpleName}[$name]"

}
```

It can be seen that in addition to the reactive flow of data, it also stores the last data for combined time operations.

### Robot

All devices are *component*, the robot is a *dynamic scope*, so the device can be **setup** to the bot. As mentioned in the [*dependency*](https://www.mechdancer.org/Documentation/dependency/) article, there may be some composite components in the robot that can rely on instances of the device. The robot initializes the `RobotComponent` in it when it initializes and configures the connection between the network communication and the dataflow.

### Use case

Let's see teleop program:

```kotlin
object RemoteControl : RobotProgram<UnicornRobot>() {

    private val master = robot.master
    private val helper = robot.helper

    override fun init() {
        //Chassis
        master.updated - {
            MecanumChassis.Descartes(
                it.leftStickY,
                it.leftStickX,
                -it.rightStickX
            )
        } - robot.chassis.descartesControl


        //Expander

        //Helper up: Expanding/Stop
        helper.up.pressing - { Expander.State.Expanding } - Expander.state
        helper.up.releasing - { Expander.State.Stop } - Expander.state
        //Helper down: Shirking/Stop
        helper.down.pressing - { Expander.State.Shrinking } - Expander.state
        helper.down.releasing - { Expander.State.Stop } - Expander.state
        //Helper left and right: Lock/Unlock
        helper.left.pressing - { true } - Expander.lock
        helper.right.pressing - { false } - Expander.lock

        //Collector

        //Helper left stick: Lifting/Dropping/Stop
        helper.leftStick.valueChanged - {
            when {
                it.y > 0.5  -> Collector.ArmState.Lifting
                it.y < -0.5 -> Collector.ArmState.Dropping
                else        -> Collector.ArmState.Stop
            }
        } - Collector.arm
        //Helper right trigger: Collecting/Stop
        helper.rightTrigger.pressing - { Collector.CollectorState.Collecting } - Collector.core
        helper.rightTrigger.releasing - { Collector.CollectorState.Stop } - Collector.core
        //Helper left bumper: Spiting/Stop
        helper.leftBumper.pressing - { Collector.CollectorState.Spiting } - Collector.core
        helper.leftBumper.releasing - { Collector.CollectorState.Stop } - Collector.core

        //Dumper TODO: Arguments

        //Master right bumper: Pushes rod 2 times
        master.rightBumper.pressing - { 2 } - Dumper.pushRod
        master.y.pressing - { 0.35 + if (Dumper.inSadArea) 0.2 else .0 } - Dumper.power
        master.y.releasing - { .0 } - Dumper.power
        master.a.pressing - { 0.5 + if (Dumper.inSadArea) 0.2 else .0 } - Dumper.power
        master.b.releasing - { .0 } - Dumper.power
        //Helper y: Enable dumper lock
        helper.y.pressing linkTo { Dumper.lockEnable = !Dumper.lockEnable }

        //Lifter

        //When master left trigger is pressed, robot lands to ground
        //Presses right trigger to interrupt
        master.leftTrigger.pressing - { Lifter.State.Landing } - Lifter.state
        master.rightTrigger.pressing - {
            if (Lifter.lastState == Lifter.State.Landing)
                Lifter.State.Stop
            else
                Lifter.State.Lifting
        } - Lifter.state
        master.rightTrigger.releasing - { Lifter.State.Stop } - Lifter.state

        //Dustpan
        //Master left bumper: dumps 1 time
        master.leftBumper.pressing - { 1 } - Dustpan.dump
    }
}
```

We should configure the link relationship when `init`. As shown in the snippet, when the gamepad1 value is updated, the remote sensing data is wrapped into a *chassis speed vector* and then flows into the *descartes controller* node of the chassis. When the button `up` of gampad2 is pressed, the state `Expanding` will flow into `state` of `Expander`. Because it is a remote control program, most of the links *source* are handles.