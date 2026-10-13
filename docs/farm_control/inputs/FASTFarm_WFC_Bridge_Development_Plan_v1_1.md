# FAST.Farm 场控接口与 ROSCO Bridge 开发方案 v1.1

## 1. 项目目标

建立一套同时支持离线仿真和实时仿真机的 FAST.Farm 场控接口。

目标：

-   RCP 场控程序通过 Modbus TCP 与实时仿真机交互；
-   离线模式通过 ZeroMQ 复用相同场控接口；
-   场控优化输出目标诱导因子 a_ref 和绝对偏航目标；
-   ROSCO Bridge 将场控指令转换为 ROSCO 可执行输入；
-   AWAE 提供多点空间流场数据；
-   为未来基于 Ct 的诱导因子闭环控制保留接口。

## 2. 总体系统架构

数据流：

RCP \| \| Modbus TCP / ZeroMQ \| 调度核 \| +----------------+ \| \| Farm
State AWAE Observation Manager Manager \| \| \| ROSCO Bridge Wind Mapper
\| ROSCO Controller \| OpenFAST

设计原则：

1.  调度核负责数据同步，不负责控制律。
2.  ROSCO Bridge 是唯一外部控制入口。
3.  10 ms 控制周期禁止执行通信和阻塞操作。
4.  AWAE 多点数据不能直接进入 ROSCO，必须经过空间映射。

# 3. 软件模块修改计划

## P0 软件基线确认

### ROSCO

建议迁移目标：

-   ROSCO v2.10.x

必须完成：

1.  当前 ROSCO fork 与官方版本 diff。
2.  检查：
    -   ROSCO_Types.f90
    -   ControllerBlocks.f90
    -   Controllers.f90
    -   DISCON.F90
3.  保留 FAST.Farm 集成修改。
4.  完成单机 OpenFAST+ROSCO 回归。

禁止：

-   直接覆盖 Fortran 文件。
-   混用不同版本 DISCON 参数。

# 4. 新增 ROSCO Bridge

目录：

bridge/

-   induction_bridge.c
-   ct_estimator.c
-   awae_adapter.c
-   observation_manager.c
-   wind_mapper.c
-   yaw_adapter.c
-   safety_manager.c

## 4.1 输入

来自调度核：

-   turbine_id
-   induction_ref
-   yaw_target_heading
-   rotor_speed
-   generator_speed
-   rotor_thrust
-   blade_pitch
-   disk_wind_speed
-   disk_turbulence

## 4.2 输出

发送 ROSCO：

-   r_pitch
-   r_speed
-   r_torque
-   min_pitch_ref
-   pitch_offset
-   yaw_target

# 5. 诱导因子控制策略

## Mode 0

原 ROSCO 控制。

所有外部 ratio=1。

## Mode 1

功率比例开环：

R_power =

a(1-a)\^2 / \[a0(1-a0)\^2\]

输出：

r_pitch = R_power

## Mode 2

Ct 前馈：

Ct_ref = 4a(1-a)

利用：

Ct(lambda,beta)

求：

beta_target

输出：

min_pitch_ref

## Mode 3

未来闭环：

Ct_error = Ct_ref - Ct_est

输出：

pitch_offset

# 6. AWAE 数据管理

AWAE 不再输出单一风机风速。

输出：

Observation Point Frame

包含：

-   point_id
-   x,y,z
-   wind_x
-   wind_y
-   wind_z
-   wind_speed
-   turbulence

新增模块：

Observation Manager

负责：

-   点位管理
-   时间同步
-   数据缓存

Wind Mapper

负责：

观察点 -\> 风机轮盘等效风速

# 7. 调度核修改

新增任务：

## Fast Task

周期：

10 ms

负责：

-   OpenFAST状态
-   ROSCO状态
-   Bridge执行

禁止：

-   Modbus
-   ZeroMQ
-   AWAE解析

## Slow Task

周期：

1 s

负责：

-   Modbus
-   AWAE
-   WD
-   RCP同步

## Mapping Task

周期：

1 s

负责：

AWAE空间数据映射。

# 8. Modbus设计要求

RCP:

Server

RT:

Client

数据区域：

1.  Command Block

RCP -\> RT

包括：

-   induction_ref
-   yaw_target
-   timestamp
-   sequence

2.  Turbine State Block

RT -\> RCP

包括：

-   power
-   speed
-   pitch
-   Ct

3.  AWAE Frame Buffer

RT -\> RCP

包括：

-   observation point
-   position
-   wind vector
-   TI

支持：

-   初期30台
-   扩展100台以上

# 9. 实时性要求

10 ms路径：

允许：

-   读取缓存
-   控制计算

禁止：

-   socket
-   malloc
-   mutex等待

# 10. 验证计划

阶段1：

单机ROSCO Bridge测试。

阶段2：

FAST.Farm离线ZeroMQ测试。

阶段3：

实时机Modbus测试。

阶段4：

30台风机测试。

阶段5：

100台扩展测试。

验收指标：

1.  指令链路正确。
2.  多点AWAE数据完整。
3.  诱导因子开环响应正确。
4.  为Ct闭环控制提供完整数据链路。
