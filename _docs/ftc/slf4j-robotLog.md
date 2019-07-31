---
title: FTC RobotLog 对于 Slf4j 接口的实现
category: FTC
order: 2
---

Gtihub 仓库：[slf4j-robotlog](https://github.com/MechDancer/slf4j-robotlog/)

## Abstract

*Slf4j-robotlog* is a binder for the *slf4j* logger interface version 1.7 for FTC robot programming. It can convert the output of some libraries through the *slf4j* log interface to `RobotLog` output.

## Motivation

In the Java ecosystem, many library logs are used *slf4j* interface. But the logger provided in FTC programming is `RobotLog`, which also provides the ability to save the dump log to a file when the Op mode ends. However *slf4j* is just a specification of a logger without implementation. Common implementation in web programming is *log4j*, used by *slf4j* through a *binder*. *slf4j* will look for binders'implementation in the classpath at runtime. The binder uses *log4j* as the wrapper to implement the slf4j convention interface. Similarly, we use `RobotLog` as the wrapper to implement *slf4j* specification.

## Usage

Just add this dependency to your project, all *slf4j* logs will perfectly output.