---
title: FTC RobotLog 对于 Slf4j 接口的实现
category: FTC
order: 2
---

Gtihub 仓库：[slf4j-robotlog](https://github.com/MechDancer/slf4j-robotlog/)



## 摘要

*Slf4j-robotlog* 是 FTC 机器人编程中对 1.7 版本 *slf4j* 日志接口的绑定。它可以将一些使用 *slf4j* 日志接口依赖库的日志通过 `RobotLog` 输出。



## 动机

在 JVM 生态系统中，许多库使用了 *slf4j* 接口规范日志输出。在 FTC 机器人编程中，日志的输出方式是通过由 `RobotCore` 中提供的工具类 `RobotLog` 实现的。由于整个机器人控制框架中所有日志都使用了该工具类，再添加 Android Binder 未免有些不妥。因此，本库旨在讲 *slf4j* 接口实现绑定至 `RobotLog`。



## 实现

*slf4j* 定义了许多日志输出方法接口，例如 `logger.info(...)`、`logger.debug(...)` 等。但这些接口缺乏着具体实现，提供这部分的结构叫做 *binder*。在运行期，*slf4j* 初始化时会寻找 Classpath 中 *binder* 的实现，因此只需新建一个实现了 *slf4j* 规范的日志器，在具体实现中调用 `RobotLog` 工具类即可。



## 使用方法

只需要将本库加入到依赖中，不需任何额外配置，所有依赖 *slf4j* 的日志便可正常输出。
