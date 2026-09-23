# joint_sdk 测试设计文档

## 1. 目标

本文档定义 `joint_sdk` 当前公共接口的测试范围、测试分层、用例设计和验收标准，覆盖单元测试与集成测试。

测试目标：

- 验证 `JointManager` 对 Adapter、关节生命周期、JSON 创建、批量控制和状态读取的管理行为。
- 验证 `BaseJoint` 派生类接口在关节坐标系、反向、限位、状态回调和异常语义上的一致性。
- 验证 `RJoint` 直连电机控制、限位保护、状态转换和校准流程。
- 验证 `DoubleCoupleJoint`、`MasterJoint`、`SlaveJoint` 闭链关节配对、求解器控制映射、状态缓存和错误处理。
- 验证 JSON Schema 和 JSON 配置反序列化接口。
- 验证 CMake 测试开关、依赖链接和安装导出相关行为。

## 2. 测试范围

### 2.1 纳入范围

- `include/BaseJoint.h`
- `include/RJoint/RJoint.h`
- `include/DoubleCoupleJoint/DoubleCoupleJoint.h`
- `include/DoubleCoupleJoint/MasterJoint.h`
- `include/DoubleCoupleJoint/SlaveJoint.h`
- `include/JointManager.h`
- `include/JsonSchemaGenerator.h`
- `tests/test_adapter.*`
- `external/couple_joint_solver` 与 `joint_sdk` 的接口集成
- CMake 构建选项：`JOINT_SDK_BUILD_TESTS`、`COUPLE_JOINT_BUILD_TESTS`

### 2.2 排除范围

- `external/boost_preprocessor` 第三方源码内部逻辑。
- `EncosMotorDriver` 底层驱动自身实现，只验证 `joint_sdk` 对其接口的调用效果。
- Web 前端和 WASM 绑定。
- 真实硬件稳定性、总线时序、电气安全测试。硬件测试仅作为集成测试扩展项。

## 3. 测试环境

基础环境：

- C++17
- CMake 3.20+
- GTest
- nlohmann_json
- EncosMotorDriver
- Eigen3
- Boost.Preprocessor 子模块
- couple_joint_solver 子模块

推荐命令：

```bash
cmake -S . -B build -DJOINT_SDK_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

测试关闭构建：

```bash
cmake -S . -B build-no-tests -DJOINT_SDK_BUILD_TESTS=OFF
cmake --build build-no-tests
```

## 4. 测试夹具与通用数据

### 4.1 TestAdapter

用途：

- 拦截 `MotorMessage` 下发内容。
- 模拟电机状态上报。
- 模拟零点复位 ACK。
- 避免单元测试依赖真实硬件。

必备能力：

- `Send()` 记录下发消息。
- `SimulateOnMessage()` 注入反馈消息。
- `ReadMessages()` 验证消息消费。
- `SetAutoAckZeroReset()` 支持 `SetZero()` 流程。
- `GetBuses()` 明确返回空集合，不伪造真实总线枚举。

### 4.2 标定数据

闭链关节测试应使用一组可通过 `couple_joint::MechanismSolver` 标定的固定数据，包含：

- `left_joint_center`
- `right_joint_center`
- `left_planar_endpoint`
- `right_planar_endpoint`
- `roll_center`
- `pitch_center`
- `left_spatial_endpoint`
- `right_spatial_endpoint`
- `rotation_order`
- 可选 `rotate_direction`

可复用的 couple 库示例标定参数：

- `external/couple_joint_solver/examples/ankle.json`：踝关节机构标定样例。
- `external/couple_joint_solver/examples/waist.json`：腰部机构标定样例。

这两个文件可作为 `couple_joint::CalibrationInput` 的测试数据来源。用于 `DoubleCoupleJoint::Config` 测试时，只取文件内的 `calibration` 字段映射到 `Config::calibration`，再由测试夹具额外补齐 `motorParams1`、`motorParams2`、`axis`、`limit`、`maxSpd`、`maxTor`、`reverse`。文件中的 `rollLimit` / `pitchLimit` 属于 couple 示例配置，若需要纳入 `joint_sdk` 用例，应显式转换为对应 axis 的 `limit`。

### 4.3 电机 PVT 范围

应提供可复用的 `MotorPVTRanges`：

- `kp`
- `kd`
- `position`
- `speed`
- `torque`
- `current`
- `kt`

重点边界：

- 正常 `kt > 0`
- 非法 `kt == 0`
- 非法 `kt < 0`
- 非法 `Range.min > Range.max`
- 很小的 `position` / `speed` / `current` 范围，用于验证裁剪和失败路径

## 5. 单元测试设计

### 5.1 TestAdapter

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-TA-001 | 构造与 `Ok()` | 默认构造 `TestAdapter` | `Ok()` 返回 true |
| UT-TA-002 | `Send()` 记录消息 | 发送单条/多条 `MotorMessage` | `GetSentMessages()` 保持原顺序和原内容 |
| UT-TA-003 | `ClearSentMessages()` | 先发送消息再清空 | 发送缓冲区为空 |
| UT-TA-004 | `SimulateOnMessage()` 注入反馈 | 注入多条不同 bus/motor 的反馈 | `ReadMessages()` 可读出对应消息 |
| UT-TA-005 | `ReadMessages()` 过滤与消费 | 指定 `bus_idx` / `motor_idx` 读取两次 | 首次返回过滤结果，第二次为空 |
| UT-TA-006 | `SetAutoAckZeroReset(true)` | 发送零点复位广播命令 | 自动回注 ACK，`ReadMessages()` 可读取 |
| UT-TA-007 | `SetAutoAckZeroReset(false)` | 发送相同命令 | 不自动回注 ACK |
| UT-TA-008 | `GetBuses()` | 默认 `TestAdapter` | 返回空 map，测试夹具不声明真实总线 |
| UT-TA-009 | 自动零点 ACK 负例 | 自动 ACK 开启后发送 ID 不匹配、长度不足或命令字不为 zero reset 的消息 | 不回注 ACK，`ReadMessages()` 为空 |

### 5.2 BaseJoint、通用类型与电机创建

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-BJ-001 | `JointError` 枚举稳定性 | 枚举值转换为 `int` | 已公开错误码顺序符合预期 |
| UT-BJ-002 | `JointException::what()` | 构造异常消息 | `what()` 返回原始消息 |
| UT-BJ-003 | `JointException::Error()` | 构造指定 `JointError` | 返回相同错误码 |
| UT-BJ-004 | `Range<T>` JSON 解析 | `{"min": -1, "max": 1}` | `min/max` 正确 |
| UT-BJ-005 | `MotorInit` JSON 自动模式 | 空对象 | 得到 `MotorInitAuto` |
| UT-BJ-006 | `MotorInit` JSON 型号模式 | `{"model": "..."}` | 得到 `MotorInitModel` |
| UT-BJ-007 | `MotorInit` JSON 自定义范围 | `{"ranges": ...}` | 得到 `MotorInitParams` |
| UT-BJ-008 | `MotorParams` JSON 解析 | adapter/bus/motor/initParam | 字段完整赋值 |
| UT-BJ-009 | `MotorInit` 优先级 | 同时包含 `ranges` 和 `model` | 优先解析为 `MotorInitParams` |
| UT-BJ-010 | 非法电机型号 | 未知 `model` 字符串 | 抛 `std::invalid_argument` |
| UT-BJ-011 | `JointManager<>::CreateMotor()` 缺失 adapter | `adapterId` 不存在 | 抛 `std::invalid_argument` |
| UT-BJ-012 | `JointException` 继承语义 | 以 `JointException` 抛出并用 `std::runtime_error` / `std::exception` 捕获 | 可被标准异常基类捕获，`what()` 和 `Error()` 信息保持不丢失 |
| UT-BJ-013 | `JointManager<>::CreateMotor()` 初始化分支 | `MotorInitAuto`、`MotorInitModel`、`MotorInitParams` 三种 `initParam` | 分别调用对应的 `Bus::GetMotor()` 重载，motor id 与范围/型号参数不丢失 |
| UT-BJ-014 | `JointManager<>::CreateMotor()` bus 获取失败 | adapter 存在但 `busId` 无效，`GetBus(busId)` 抛异常或返回空 | 异常/失败语义明确向上传播，不解引用空 bus，不生成无效 `Motor` |
| UT-BJ-015 | 保留错误码运行时契约 | 触发 pending 创建、不支持校准接口等当前公开路径 | 不抛 `kDoubleCoupleJointPending` / `kDoubleCoupleJointUnsupportedControl`；若未来使用必须新增明确场景测试 |
| UT-BJ-016 | `JointManager<>::CreateMotor()` motor 获取失败 | adapter/bus 存在但 `motorId` 无效，`Bus::GetMotor()` 抛异常或返回空 | 异常/失败语义明确向上传播，不解引用空 `Motor`，不生成无效关节 |
| UT-BJ-017 | `JointManager<>::CreateMotor()` 空 adapter 指针 | `AdapterMap` 中存在 `adapterId`，但对应 `BaseAdapterPtr == nullptr` | 抛明确异常，不解引用空 adapter，不生成无效 `Motor` |
| UT-BJ-018 | `JointManager<>::CreateMotor()` public static 接口 | 从测试或外部 helper 直接以 `MotorParams + AdapterMap` 调用 | 可独立创建 `Motor`，`BaseJoint` 不再暴露或承担电机创建职责 |
| UT-BJ-019 | `kLimitSlack` 常量契约 | 直接读取 `kLimitSlack` | 值为 2 度对应弧度且为正有限值；RJoint 限位保护用例使用同一常量预期 |
| UT-BJ-020 | `JointStatus` 字段语义 | 构造含位置/速度/扭矩/温度/错误码的 `JointStatus`，包含 NaN 字段 | 字段可逐项保存，不丢失 NaN；派生关节状态转换用例均按该结构输出 |
| UT-BJ-021 | `MotorInit` 非对象 JSON | `initParam` 为 string/number/bool/null/array | `from_json(MotorInit)` 抛 JSON 类型异常，不把非对象误解析为 `MotorInitAuto` |
| UT-BJ-022 | `MotorParams` 字段类型错误 | `adapterId` 为 bool/float/null/object/array，`busId/motorId` 非 integer，或 `initParam` 类型非法 | `from_json(MotorParams)` 抛 JSON 类型异常；不得生成带默认 id 或默认 adapter 的电机配置 |
| UT-BJ-025 | `MotorParams` 适配器引用类型分派 | `adapterId` 为字符串、非负整数、负数或超出 uint32 的整数 | 字符串得到名称引用（`AdapterIdIsIndex()` 为 false），非负整数得到索引引用且 `AdapterIdString()` 为十进制形式；越界/负数抛 `std::invalid_argument` |
| UT-BJ-023 | `MotorParams` 可缺省字段 | 缺少 `slaveId`/`canfd`，或 `canfd` 为 number/string | 缺失时取 `slaveId = 0`、`canfd = false`；`canfd` 非 bool 时抛 JSON 类型异常；`BusIndex()` 在 `slaveId = 0` 时等于 `busId` |
| UT-BJ-024 | `MotorParams` 合成总线索引与物理身份 | 显式 `slaveId = 3, busId = 4`，与手写 `busId = (3 << 16) \| 4` 配置并列 | `BusIndex()` 均为 `(3 << 16) \| 4`；`MakePairKey` 判为同一物理电机，`slaveId` 不同则判为不同电机 |

### 5.3 RJoint

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-RJ-001 | 构造成功 | 合法 `Config` 和预创建 `Motor` | 绑定底层 `Motor` 成功，读取 PVT range 和 `kt` |
| UT-RJ-002 | 拒绝非法 `kt` | `kt <= 0` 或非有限值 | 抛 `std::invalid_argument` |
| UT-RJ-003 | `PVTControl()` PVT 裁剪 | `pos` 超过 `limit`，`spd/torque` 超过最大值 | 下发位置、速度、扭矩被裁剪到安全范围 |
| UT-RJ-004 | `PosControl()` 参数处理 | 目标超过 `limit`，`speed/current <= 0` 或超过最大值 | 位置被裁剪，速度/电流使用 `maxSpd_` 和 `maxCur_` |
| UT-RJ-005 | `SpdControl()` 速度裁剪 | 超过 `maxSpd` | 速度裁剪到范围内 |
| UT-RJ-006 | `CurControl()` 电流裁剪 | 超过最大电流 | 电流裁剪到范围内 |
| UT-RJ-007 | `TorControl()` 扭矩裁剪 | 超过 `maxTor` | 扭矩裁剪到范围内 |
| UT-RJ-008 | 反向控制 | `reverse=true` | 位置/速度/扭矩符号翻转 |
| UT-RJ-009 | 限位保护 | 当前位置靠近上下限且继续越界运动 | 越界方向速度/电流/扭矩置零 |
| UT-RJ-010 | `SetZero()` | mock ACK 成功 | 返回 true |
| UT-RJ-011 | `GotoLimit()` | 正/负方向 | 返回关节坐标系 offset |
| UT-RJ-012 | `GotoZero()` | offset | 传给电机时考虑 `reverse` |
| UT-RJ-013 | `PosAsOffset()` | 底层 `GetParameter<Position>()` 有效/无效，覆盖 `dir` 与 `reverse` 组合 | 基于底层参数和对应限位计算关节坐标系 offset；底层位置无效返回 NaN |
| UT-RJ-014 | `Calibrate()` 成功路径 | `GotoLimit` 有效且 `GotoZero` 成功 | 返回 true |
| UT-RJ-015 | `Calibrate()` 失败路径 | `GotoLimit` 返回 NaN | 返回 false |
| UT-RJ-016 | `GetStatus()` 有缓存 | 注入电机状态 | 返回关节坐标系 `JointStatus` |
| UT-RJ-017 | `GetStatus(autoSendCommand=true)` 无缓存 | 无状态 | 返回空并下发反馈命令 |
| UT-RJ-018 | `SetOnStatus()` | 注入状态 | 回调收到关节坐标系状态 |
| UT-RJ-019 | 取消回调 | `SetOnStatus(nullptr)` | 后续状态不再触发回调 |
| UT-RJ-020 | `Stop()` | 指定 mode/current/feedback | 参数原样传递到底层电机 |
| UT-RJ-021 | `SetLimit()` | `reverse=false/true` | 内部限位按反向规则存储 |
| UT-RJ-022 | `GetMotorPosition()` | `read_cache=true/false` | 有缓存时优先读缓存，无缓存时回退 `GetParameter` |
| UT-RJ-023 | `GetReversedPosition()` | `reverse=false/true` | 返回关节坐标系位置 |
| UT-RJ-024 | `SetZero()` 失败路径 | mock `ResetZeroPos()` 返回 false | 返回 false，不产生额外控制命令 |
| UT-RJ-025 | `GotoZero()` 失败路径 | mock `GotoZero()` 返回 false | 返回 false，`reverse` 仍按关节坐标系转换 |
| UT-RJ-026 | `Calibrate()` 第二阶段失败 | `GotoLimit()` 返回有效 offset，但 `GotoZero()` 返回 false | `Calibrate()` 返回 false |
| UT-RJ-027 | `GetStatus()` 速度字段缺失 | 位置有效、`speed=NaN` | `JointStatus.pos` 正常，`JointStatus.spd` 保持 NaN |
| UT-RJ-028 | `GetStatus()` 电流字段缺失 | 位置有效、`current=NaN` | `JointStatus.pos` 正常，`JointStatus.tor` 保持 NaN |
| UT-RJ-029 | `SetOnStatus()` 部分反馈字段透传 | 回调状态中 `speed` 或 `current` 为 NaN | 回调输出逐字段保持 NaN，不伪造有效速度/扭矩 |
| UT-RJ-030 | `GetMotorPosition()` 缓存 `NaN` 回退 | `GetStatus()` 有缓存对象，但 `position=NaN` | 回退调用 `GetParameter<Position>()`，不返回 NaN 缓存值 |
| UT-RJ-031 | `GetStatus()` 生命周期扣减参数 | 调用 `GetStatus(false, lifeCycleDeduction=N)` | `N` 原样传到底层 `Motor::GetStatus()`，返回语义不受默认值掩盖 |
| UT-RJ-032 | `GetMotorPosition()` 生命周期扣减参数 | `read_cache=true` 且传入非默认 `life_cycle_deduction` | 底层缓存读取使用该扣减值；缓存缺失或 NaN 时仍回退 `GetParameter<Position>()` |
| UT-RJ-033 | `SpdControl()` 电流参数处理 | `current <= 0`、`current > maxCur_` | 速度裁剪后下发；电流使用或裁剪为 `maxCur_` |
| UT-RJ-034 | 控制接口 `feedback` 透传 | `PosControl/SpdControl/CurControl/TorControl` 使用非默认 `feedback` | 下发到底层电机的反馈参数与输入一致 |
| UT-RJ-035 | 校准运动参数透传 | `GotoLimit/GotoZero/Calibrate` 使用非默认 `spd/cur/timeout`，覆盖 `reverse=false/true` | `spd/cur/timeout` 原样传到底层电机；`reverse` 只影响方向或 offset，`Calibrate()` 两阶段使用同一组参数 |
| UT-RJ-036 | `PVTControl()` `kp/kd` 透传 | 使用非默认 `kp/kd`，同时触发位置/速度/扭矩裁剪 | `kp/kd` 原样传到底层电机；只有位置、速度、扭矩按关节规则转换和裁剪 |
| UT-RJ-037 | `GetStatus(autoSendCommand=false)` 无缓存 | 无缓存状态且关闭自动请求 | 返回 `std::nullopt`，不下发反馈请求命令 |
| UT-RJ-038 | `Config` 数值语义拒绝 | `limit.min > limit.max`、`maxSpd <= 0`、`maxTor <= 0` | 创建 `RJoint` 失败并抛明确异常，不允许进入可控状态 |
| UT-RJ-039 | `PVTControl()` 非有限输入 | `kp/kd/pos/spd/torque` 任一为 NaN/Inf | 底层电机先 `Stop(FullBrake)`，再抛 `std::invalid_argument`；不得下发包含 NaN/Inf 的 PVT 命令 |
| UT-RJ-040 | `PosControl()` 非有限输入 | `position/speed/current` 任一为 NaN/Inf | 底层电机先 `Stop(FullBrake)`，再抛 `std::invalid_argument`；不得下发包含 NaN/Inf 的位置命令 |
| UT-RJ-041 | 速度/电流/扭矩控制非有限输入 | `SpdControl(speed/current)`、`CurControl(current)`、`TorControl(torque)` 任一数值为 NaN/Inf | 底层电机先 `Stop(FullBrake)`，再抛 `std::invalid_argument`；不得下发包含 NaN/Inf 的控制命令 |
| UT-RJ-042 | `Stop()` 非有限电流 | `Stop(mode, current=NaN/Inf, feedback)` | 仍下发停止命令，但停止电流固定为 0，不向底层传递 NaN/Inf |
| UT-RJ-043 | 构造拒绝空 `Motor` | `RJoint(config, nullptr)` | 抛 `std::invalid_argument`，不解引用空指针 |

### 5.4 DoubleCoupleJoint 配置与配对

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-DC-001 | `MakePairKey()` 同电机 | 两个 `MotorParams` 指向同一物理电机 | 返回空字符串 |
| UT-DC-002 | `MakePairKey()` 顺序敏感 | `(a,b)` 与 `(b,a)` | key 不同 |
| UT-DC-003 | 首次 AddJoint | 某电机对第一次创建 | 创建 `SlaveJoint`，进入 pending |
| UT-DC-004 | 第二次 AddJoint | 同电机对、不同 axis、同标定 | 创建 `MasterJoint`，完成绑定 |
| UT-DC-005 | 第三次 AddJoint | 同电机对再次创建 | 抛 `kDoubleCoupleJointDuplicatePair` |
| UT-DC-006 | 重复 axis | 两份 config 均为 Roll 或 Pitch | 抛 `kDoubleCoupleJointDuplicateAxis` |
| UT-DC-007 | 标定不一致 | 两份 calibration 不一致 | 抛 `kDoubleCoupleJointCalibrationMismatch` |
| UT-DC-008 | 标定失败 | 无效几何输入 | 抛 `kDoubleCoupleJointCalibrationFailed` |
| UT-DC-009 | `CalibrationEqual()` 容差 | 点误差在/超出容差 | 在容差内 true，否则 false |
| UT-DC-010 | 可选 `rotateDirection` | 缺失/存在字段 | `use_rotate_direction` 正确 |
| UT-DC-011 | `rotationOrder` 字符串解析 | 合法字符串 | 转为对应枚举 |
| UT-DC-012 | `rotationOrder` 数字解析 | 0 / 1 | 转为对应枚举 |
| UT-DC-013 | 非法 axis | 非 roll/pitch 字符串 | 抛 `std::invalid_argument` |
| UT-DC-014 | couple 踝关节示例标定 | `external/couple_joint_solver/examples/ankle.json` 的 `calibration` | 可转换为 `CalibrationInput`，roll/pitch 两轴配对成功 |
| UT-DC-015 | couple 腰部示例标定 | `external/couple_joint_solver/examples/waist.json` 的 `calibration` | 可转换为 `CalibrationInput`，roll/pitch 两轴配对成功 |
| UT-DC-016 | 同电机配置拒绝配对 | `motorParams1` 和 `motorParams2` 指向同一物理电机 | 抛 `kDoubleCoupleJointSameMotor` |
| UT-DC-017 | 缺失 pending slave 的 master 创建 | `motor1/motor2` 均非空但 `slave=nullptr` | 抛 `kDoubleCoupleJointNotPaired` |
| UT-DC-018 | 部分电机注入拒绝 | `motor1/motor2` 只有一个非空 | 抛 `std::invalid_argument`，不创建 `MasterJoint`，不绑定 slave |
| UT-DC-019 | 第二次配对失败后可恢复 | 先创建 pending slave，再以非法 config 配对，最后用合法 config 重试 | 第一次失败不破坏 pending 状态，最终仍可生成 `MasterJoint` |
| UT-DC-020 | `CalibrationEqual()` 比较 `rotateDirection` | 两侧都启用 `rotateDirection`，但方向向量不同 | 返回 false |
| UT-DC-021 | `CalibrationEqual()` 忽略未启用的 `rotateDirection` | 两侧都未启用 `rotateDirection`，结构体残留值不同 | 返回 true |
| UT-DC-022 | `MakePairKey()` 唯一性 | 仅 adapter、bus 或 motor id 不同的多组电机参数 | 生成不同且非空的 key，不误判为同一物理电机或同一 pair |
| UT-DC-023 | `CalibrationEqual()` 单侧启用 `rotateDirection` | 一侧 `use_rotate_direction=true`，另一侧为 false | 返回 false，不把单侧方向约束误判为同一标定 |
| UT-DC-024 | pending 首轴延迟校验 | 首次 `AddJoint<DoubleCoupleJoint>()` 使用会导致 solver 失败的 calibration | 只创建 pending `SlaveJoint`，不创建电机、不运行 solver；第二轴配对时才抛 `kDoubleCoupleJointCalibrationFailed`，原 pending 可恢复 |
| UT-DC-025 | 闭链 `Config` 数值语义拒绝 | `limit.min > limit.max`、`maxSpd <= 0`、`maxTor <= 0`、PVT range `min > max` | 配对完成前或创建 `MasterJoint` 阶段失败并抛明确异常，不允许生成可控 master |
| UT-DC-026 | 配对预校验优先级 | pending slave 已存在，第二轴重复 axis 或 calibration 不一致，且 manager 无 adapter | 先抛 `kDoubleCoupleJointDuplicateAxis` / `kDoubleCoupleJointCalibrationMismatch`，不得先进入 `CreateMotor()` 失败 |
| UT-DC-027 | `CreateJoint()` master 路径参数契约 | `motor1/motor2` 均非空且 slave 合法，或任一参数不满足契约 | 合法时创建 master 并绑定 slave；非法时异常类型明确，pending slave 不被破坏 |
| UT-DC-028 | `CreateJoint()` slave 路径参数契约 | `motor1 == nullptr && motor2 == nullptr`，覆盖 `slave=nullptr` 与误传非空 slave | 始终创建新的未绑定 `SlaveJoint`；不创建电机、不绑定传入 slave、不运行 solver |
| UT-DC-029 | `MakePairKey()` 物理身份定义 | `adapterId/busId/motorId` 相同但 `initParam` 不同，或 `initParam` 相同但物理 id 不同 | 只用 adapter/bus/motor id 判定物理身份；同物理电机返回空，不同物理电机生成不同 key |
| UT-DC-030 | `CalibrationEqual()` 比较 `rotation_order` | 几何点完全一致但 `rotation_order` 不同 | 返回 false，不把不同求解顺序误判为同一标定 |

### 5.5 MasterJoint

`MasterJoint` 既要覆盖主轴对外接口，也要覆盖 `SlaveJoint` 转发后的从轴执行接口。`*ForSlave()` 用例验证从轴命令进入 `MasterJoint` 后的运动学解算、双电机下发、缓存提交和保护逻辑，不只验证函数被调用。

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-MJ-001 | `SetZero()` 成功 | 两电机 reset ACK 成功，零位可解 | 返回 true，内部目标位姿归零 |
| UT-MJ-002 | 构造期当前姿态初始化 | 创建 `MasterJoint` 时两电机当前位置可正解 | 构造函数读取两电机位置并初始化目标位姿缓存，后续控制无需先调用 `SetZero()` |
| UT-MJ-003 | `Stop()` | 指定 mode/current/feedback | 两个电机均收到停止命令，参数原样透传 |
| UT-MJ-004 | `SetLimit()` | `reverse=false/true` | 内部限位按反向规则存储 |
| UT-MJ-005 | 主轴 `PVTControl()` | 主轴 PVT 目标 | 缓存主轴位置、速度、扭矩和增益，等待同批次第二条轴命令 |
| UT-MJ-006 | 从轴 `PVTControlForSlave()` | 已有主轴缓存，再控制从轴 PVT | 保留主轴数据并更新从轴数据；在实际反馈位姿一次性映射两轴速度、扭矩和增益，任一电机超范围时整体缩放 |
| UT-MJ-007 | 主轴 `PosControl()` | 主轴位置目标可达 | 两电机收到位置命令，提交主轴目标缓存 |
| UT-MJ-008 | 从轴 `PosControlForSlave()` | 已有主轴缓存，再控制从轴位置 | 保留主轴目标，更新从轴目标，双电机命令正确 |
| UT-MJ-009 | 主轴 `SpdControl()` | 主轴速度命令（按合帧下发，需同批次再调用从轴一次） | 第一条调用只缓存轴目标且不下发电机命令；第二条调用用两轴槽位执行联合逆速度映射到双电机速度；任一电机超 speed range 时整体缩放；有有效双电机位置反馈时按反馈刷新目标缓存，无反馈时不改缓存 |
| UT-MJ-010 | 从轴 `SpdControlForSlave()` | 从轴速度命令（按合帧下发，需同批次再调用主轴一次） | 与 UT-MJ-009 相同，仅轴身份来自从轴 |
| UT-MJ-011 | 主轴 `TorControl()` | 主轴扭矩命令 | 逆扭矩映射到双电机扭矩；任一电机超 torque range 时整体缩放；有有效双电机位置反馈时按反馈刷新目标缓存，无反馈时不改缓存 |
| UT-MJ-012 | 从轴 `TorControlForSlave()` | 从轴扭矩命令 | 逆扭矩映射到双电机扭矩；任一电机超 torque range 时整体缩放；有有效双电机位置反馈时按反馈刷新目标缓存，无反馈时不改缓存 |
| UT-MJ-013 | 主轴 `CurControl()` | 主轴电流命令 | 按主轴扭矩比拆分到双电机电流；任一电机超 current range 时整体缩放；有有效双电机位置反馈时按反馈刷新目标缓存，无反馈时不改缓存 |
| UT-MJ-014 | 从轴 `CurControlForSlave()` | 从轴电流命令 | 按从轴扭矩比拆分到双电机电流；任一电机超 current range 时整体缩放；有有效双电机位置反馈时按反馈刷新目标缓存，无反馈时不改缓存 |
| UT-MJ-015 | 目标超电机范围 | 极小电机 position 范围 | 有有效双电机位置反馈时下发当前位置保持；无有效位置反馈时退回 `FullBrake`；不提交不可执行目标 |
| UT-MJ-016 | 电流范围裁剪 | 很小 current 范围 | 双电机电流等比例缩放且均不超范围 |
| UT-MJ-017 | 近奇异方向保护 | near singular 且命令进入奇异区 | 有有效双电机位置反馈时下发当前位置保持；无有效位置反馈时退回 `FullBrake`；不提交危险目标 |
| UT-MJ-018 | `GetStatus()` Pull 主轴 | 注入双电机状态后拉取主轴 | 返回主轴合成状态并提交反馈缓存 |
| UT-MJ-019 | `GetStatusForSlave()` Pull 从轴 | 注入双电机状态后拉取从轴 | 返回从轴合成状态并提交反馈缓存 |
| UT-MJ-020 | `GetStatus(autoSendCommand=true)` 无缓存 | 缺任一电机状态 | 返回空并向两电机发反馈命令 |
| UT-MJ-021 | `SetOnStatus()` Push 主轴 | 两电机状态均到达 | 主轴回调收到合成状态，缓存更新 |
| UT-MJ-022 | `SetOnStatusForSlave()` Push 从轴 | 两电机状态均到达 | 从轴回调收到合成状态，缓存更新 |
| UT-MJ-023 | 单边反馈 | 只收到一个电机状态 | 不触发关节回调，不更新缓存 |
| UT-MJ-024 | 状态诊断聚合 | 两电机温度/错误码不同 | 温度取较大值，错误码取更严重值 |
| UT-MJ-025 | `GetMotor1Position()` / `GetMotor2Position()` | 缓存与直接读参数 | 返回原始电机坐标系位置 |
| UT-MJ-026 | 不支持的校准接口 | `GotoLimit/GotoZero/PosAsOffset/Calibrate` | 返回 NaN 或 false |
| UT-MJ-027 | `SetZero()` ACK 失败 | 任一电机 reset ACK 失败 | 返回 false，不把目标位姿强制归零 |
| UT-MJ-028 | 构造期当前姿态不可解 | 创建 `MasterJoint` 时两电机当前位置无法正解或存在硬奇异 | 构造失败并抛 `kDoubleCoupleJointSolveFailed`，不进入可控状态 |
| UT-MJ-029 | 主轴 reverse 控制与状态 | 主轴 `reverse=true` | 控制输入、状态输出和限位均符合关节坐标系约定 |
| UT-MJ-030 | 从轴 reverse 控制与状态 | 从轴 `reverse=true` | 从轴 `*ForSlave()` 路径仍保持关节坐标系一致性 |
| UT-MJ-031 | 反馈缺失速度字段 | 双电机位置有效，速度缺失或 NaN | 位置更新成功，`JointStatus.spd` 为 NaN |
| UT-MJ-032 | 反馈缺失电流字段 | 双电机位置有效，电流缺失或 NaN | 位置更新成功，`JointStatus.tor` 为 NaN |
| UT-MJ-033 | 反馈位置缺失时回退缓存位姿 | 双电机位置缺失，但速度或电流存在 | 位姿沿用缓存，速度/力矩基于缓存位姿求解 |
| UT-MJ-034 | 正向位置失败时跳过速度/力矩求解 | 双电机位置存在但 `SolveForward` 不可执行 | 位置回退缓存，速度/力矩不基于错误位姿更新 |
| UT-MJ-035 | 构造后未 `SetZero()` 读主轴状态 | 已完成配对且构造期姿态已验证，未调用 `SetZero()` | 按普通状态读取路径返回合成状态或空状态，不抛零位未设置异常 |
| UT-MJ-036 | 构造后未 `SetZero()` 读从轴状态 | 已完成配对且构造期姿态已验证，未调用 `SetZero()` | 按普通状态读取路径返回合成状态或空状态，不抛零位未设置异常 |
| UT-MJ-037 | 主轴回调解绑不影响从轴 | 先注册主轴/从轴回调，再对主轴 `SetOnStatus(nullptr)` | 主轴不再回调，从轴仍持续收到状态 |
| UT-MJ-038 | 从轴回调解绑不影响主轴 | 先注册主轴/从轴回调，再对从轴 `SetOnStatusForSlave(nullptr)` | 从轴不再回调，主轴仍持续收到状态 |
| UT-MJ-039 | 最后一个回调解绑 | 清空最后一个已注册回调 | 底层两个 motor 的状态回调被卸载，后续反馈不再触发 joint 回调 |
| UT-MJ-040 | `GetMotor1Position()` / `GetMotor2Position()` 缓存 `NaN` 回退 | `GetStatus()` 有缓存对象，但 `position=NaN` | 回退调用 `GetParameter<Position>()`，不返回 NaN 缓存值 |
| UT-MJ-041 | 主轴回调无需 `SetZero()` | 构造后直接 `SetOnStatus()` 并注入双电机状态 | 双电机状态齐全后正常触发回调并更新缓存 |
| UT-MJ-042 | 从轴回调无需 `SetZero()` | 构造后直接 `SetOnStatusForSlave()` 并注入双电机状态 | 双电机状态齐全后正常触发回调并更新缓存 |
| UT-MJ-043 | 主轴 `PosControl()` 速度/电流参数处理 | `speed <= 0`、`speed > maxSpd`、`current <= 0`、`current > maxTor/kt` | 速度使用或裁剪到主轴 `maxSpd`；电流 `<=0` 下发 0，超限时按每个电机 `kt` 裁剪 |
| UT-MJ-044 | 从轴 `PosControlForSlave()` 速度/电流参数处理 | `speed <= 0`、`speed > maxSpd`、`current <= 0`、`current > maxTor/kt` | 速度使用或裁剪到从轴 `maxSpd`；电流 `<=0` 下发 0，超限时按每个电机 `kt` 裁剪 |
| UT-MJ-045 | 主轴 `CurControl()` 非有限输入 | `current=NaN/Inf` | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存 |
| UT-MJ-046 | 从轴 `CurControlForSlave()` 非有限输入 | `current=NaN/Inf` | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存 |
| UT-MJ-047 | 主轴零输入不提交预估缓存 | `SpdControl(0)`、`TorControl(0)`、`CurControl(0)` 后再控制另一轴 | 两轴调用成帧后下发零命令；无反馈时主轴目标缓存保持原值，有反馈时按反馈正解刷新目标缓存 |
| UT-MJ-048 | 从轴零输入不提交预估缓存 | `SpdControlForSlave(0)`、`TorControlForSlave(0)`、`CurControlForSlave(0)` 后再控制主轴 | 两轴调用成帧后下发零命令；无反馈时从轴目标缓存保持原值，有反馈时按反馈正解刷新目标缓存 |
| UT-MJ-049 | 回调析构清理 | 注册主轴/从轴回调后销毁 `MasterJoint`，再注入双电机状态 | 析构时卸载底层 motor 回调，后续状态不会访问已销毁对象、不触发 joint 回调 |
| UT-MJ-050 | 状态读取生命周期扣减参数 | 主轴 `GetStatus(false, N)` 与从轴 `GetStatusForSlave(false, N)` | 两个底层电机 `GetStatus()` 均收到相同 `N` |
| UT-MJ-051 | 电机位置 helper 生命周期扣减参数 | `GetMotor1Position(true, N)` / `GetMotor2Position(true, N)` | 对应底层电机缓存读取使用 `N`；缓存缺失或 NaN 时仍回退参数读取 |
| UT-MJ-052 | 控制接口 `feedback` 透传 | 主轴/从轴 `PosControl/SpdControl/CurControl/TorControl` 使用非默认 `feedback` | 两台电机收到的反馈参数与输入一致 |
| UT-MJ-053 | `SpdControl()` 电流参数处理 | 主轴/从轴 `SpdControl()` 传入 `current <= 0` 或超出 `maxTor/kt`，两轴传入不同 `current` | `current <= 0` 下发 0；超限时按各电机 `kt` 裁剪，速度映射仍按统一比例缩放；两台电机各取两条调用中较小的电流上限 |
| UT-MJ-081 | 动态类两轴合帧 | `SpdControl`/`TorControl`/`CurControl` 两轴同批次都提供非零目标 | 第一条调用后不产生电机命令；第二条调用在最后有效位姿一次性完成双轴联合映射，两个轴的目标均保留，结果不依赖第二次调用属于哪个轴 |
| UT-MJ-082 | 动态类半帧与模式切换 | 挂起速度半帧后改调 `TorControl`，或改调 `PosControl` | 挂起的半帧被丢弃且不下发混合模式命令；本次调用作为新批次第一条，同模式第二条才成帧 |
| UT-MJ-083 | 动态类未命令轴沿用上次值 | 两轴成帧后，下一批次只调用其中一轴 | 另一轴沿用该轴上次命令值参与联合映射；`Stop` 与 `SetZero` 后同样场景按 0 参与映射 |
| UT-MJ-084 | `MapJointCurrent` 单轴等价性 | `CurControl` 单轴非零、另一轴 0 | 双电机电流分配与两轴联合映射前的 `MapAxisCurrent` 结果逐位一致 |
| UT-MJ-054 | `SetZero()` 成功后再次失败 | 先成功 `SetZero()` 并控制到非零缓存，再模拟第二次 reset ACK 失败或零位不可解 | 第二次返回 false，不把目标位姿强制归零；控制路径不依赖零位门禁 |
| UT-MJ-055 | 闭链拒绝非法 `kt` | 任一电机 `kt <= 0` 或非有限值 | `MasterJoint` 创建失败并抛明确异常，不进入可控状态，不允许后续电流/位置控制绕过电流上限 |
| UT-MJ-056 | 闭链 `PVTControl()` 关节增益转换 | 主轴/从轴 PVT 使用不同 `kp/kd`，且实际反馈位姿与目标位姿不同 | 在实际反馈位姿按 `diag(J^T K J)` / `diag(J^T D J)` 融合两轴增益，两台电机收到各自转换后的 `kp/kd` |
| UT-MJ-057 | 构造后未 `SetZero()` 的 `Stop()` | master 已完成配对且未调用 `SetZero()`，直接 `Stop(mode,current,feedback)` | 不检查零位，两台电机立即收到停止命令，参数原样透传 |
| UT-MJ-058 | `GetStatus(autoSendCommand=false)` 无缓存 | 已 `SetZero()`，缺任一电机缓存状态且关闭自动请求 | 返回 `std::nullopt`，不向任一电机下发反馈请求命令 |
| UT-MJ-059 | 近奇异安全方向 | 当前位姿 near singular，但速度/扭矩/电流命令方向远离奇异区 | 不触发保护，控制命令正常下发；有反馈时按反馈刷新目标缓存，无反馈时缓存保持原值 |
| UT-MJ-060 | 闭链 `PVTControl()` 非有限输入 | 主轴/从轴 PVT 的 `pos/spd/torque` 任一为 NaN/Inf | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存 |
| UT-MJ-061 | 闭链 `SpdControl()` 非有限输入 | 主轴/从轴 `speed=NaN/Inf` | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存 |
| UT-MJ-062 | 闭链 `TorControl()` 非有限输入 | 主轴/从轴 `torque=NaN/Inf` | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存 |
| UT-MJ-063 | `ControlPVT()` 目标位置越限钳制 | `pos` 超出 `params.limit_.max` 或低于 `params.limit_.min`，覆盖 `reverse=false/true` | `JointToSolver` 后在 solver 空间与 `limit_`（已由 `BaseJoint` 按 reverse 翻转）比较并钳制到边界，解算后电机位置反映钳制结果，正常提交缓存 |
| UT-MJ-064 | `ControlPosition()` 目标位置越限钳制 | `position` 超出限位边界，覆盖主轴/从轴两条路径 | 同 UT-MJ-063，位置钳制到限位边界后下发双电机位置命令，正常提交缓存 |
| UT-MJ-065 | 闭链 `PosControl()` 非有限输入 | 主轴/从轴 `position/speed/current` 任一为 NaN/Inf | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存，不下发包含 NaN/Inf 的目标位置命令 |
| UT-MJ-066 | 闭链 `PVTControl()` 增益非有限输入 | 主轴/从轴 `kp/kd` 任一为 NaN/Inf | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存，不下发包含 NaN/Inf 的 PVT 命令 |
| UT-MJ-070 | 闭链 PVT 混合闭环 | 两台电机均有有限位置缓存，主从轴调用 `PVTControl(..., PVTControlMode::HostKpMotorKd)` | SDK 计算 `Kp*位置误差 + 前馈扭矩` 并按当前反馈 `J^T` 映射；双电机 PVT 的 Kp=0，Kd=diag(J^T Kd J)，目标速度逆映射，SDK 不重复计算速度误差 |
| UT-MJ-080 | 闭链 PVT 完整上位机闭环 | 主从轴选择 `HostTorque`，覆盖非零反馈速度、反向关节和大 Kd | SDK 计算完整 PD 加前馈，按关节总限扭和电机限扭映射，发送两条 TorControl；不再下发本地 Kd |
| UT-MJ-077 | 本地阻尼下限 | 两种 PVT 模式分别输入 Kd=0 和小的正 Kd | 正的映射 Kd 至少为 0.1（受协议上限约束），显式零保持为零 |
| UT-MJ-078 | 保护保持位置内缩 | 电机反馈已超过软件限位，触发保持 | 复用位控下发逻辑，保持目标限制到两侧各内缩 2°；区间为空则 FullBrake |
| UT-MJ-079 | 左右踝逆解跨圈 | 使用 EC-R130-V2 左右镜像标定，反馈跨过求解器 theta 的 π 主值边界 | theta 减 initial_theta 后再次归一化，输出在 [-π,π] 且保护目标不跳 ±2π |
| UT-JM-LOG | 所有关节保护告警 | Manager 创建普通、连续和耦合主从关节，各控制模式触发非法输入或幅值限幅 | Manager 指定日志器收到 warning，包含关节名称、控制模式和原因；抛异常的一次性故障逐次告警，持续成立的限幅只在进入保护时输出一条 `state=enter`、退出时输出一条 `state=exit` |
| UT-MJ-071 | 上位机力矩模式无缓存回退 | 任一电机无有效状态缓存时分别选择 `HostTorque`、`HostKpMotorKd` | 自动回退普通电机 PVT，仍使用转换后的双电机 `kp/kd`，不得下发基于无效反馈计算的扭矩 |
| UT-MJ-072 | PVT 重载兼容性 | 分别通过 `BaseJoint`、master 和 slave 调用五参数及六参数重载 | 五参数版本等价转发六参数且模式为 `MotorPVT`；slave 完整转发枚举参数 |
| UT-MJ-073 | PVT 混合模式合帧 | 同一两指令批次覆盖模式 0/1/2 的全部九种组合 | 只有两次调用选择相同上位机模式才启用对应闭环；模式不一致按普通 PVT 下发，结果不受调用顺序影响 |
| UT-MJ-074 | PVT 两轴动态量合帧 | roll/pitch 同批次均提供非零 `spd/torque` | 在最后有效实际位姿一次性执行双轴逆速度和逆力矩映射，两个轴的动态量均保留，结果不依赖第二次调用属于哪个轴 |
| UT-MJ-075 | PVT 半帧反馈隔离 | 第一轴目标已缓存、第二轴尚未提交时到达双电机位置反馈 | 反馈仅刷新实际位姿 Jacobian 缓存，不覆盖 pending 第一轴目标；第二轴提交后下发完整两轴目标 |
| UT-MJ-076 | 清零后的 PVT 实际位姿缓存 | 在非零反馈位姿调用 `SetZero()`，覆盖旧缓存、单电机先到及 reset 边界排队回调，随后执行普通 PVT | 按原子坐标系代次丢弃 reset 前/期间的 status；两台电机均收到 reset 后新帧前按零位 Jacobian 映射，之后恢复实际位姿刷新 |
| UT-MJ-067 | 闭链 `Stop()` 非有限电流 | master/slave `Stop(mode, current=NaN/Inf, feedback)`，覆盖构造后和 `SetZero()` 后 | 不检查零位；两电机均收到停止命令，停止电流固定为 0，不向底层传递 NaN/Inf |
| UT-MJ-068 | Pull 状态合成异常隔离 | 双电机状态均存在，但状态合成过程中因 slave 弱引用失效或轴参数不可用抛 `JointException` | `GetStatus()` / `GetStatusForSlave()` 返回 `std::nullopt`，不向调用方传播该异常，不提交错误反馈缓存 |
| UT-MJ-069 | Push 状态合成异常隔离 | 已注册主轴或从轴回调，双电机状态触发合成时抛 `JointException` | 忽略本次反馈，不调用 joint 回调，不提交错误反馈缓存，后续合法反馈仍可继续处理 |

### 5.6 SlaveJoint

`SlaveJoint` 是代理层测试。这里验证未绑定保护、绑定后是否转发到正确的 `MasterJoint::*ForSlave()` 或主关节接口；运动学解算和双电机命令正确性由 `MasterJoint` 的从轴用例覆盖。

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-SJ-001 | 未配对控制 | 未绑定 master 的 slave 调控制接口 | 抛 `kDoubleCoupleJointNotPaired` |
| UT-SJ-002 | 已配对 `SetZero()` | 调用 slave `SetZero()` | 委托到 master |
| UT-SJ-003 | `PVTControl()` 转发 | 调用 slave PVT | 调用 `MasterJoint::PVTControlForSlave()` |
| UT-SJ-004 | `PosControl()` 转发 | 调用 slave 位置控制 | 调用 `MasterJoint::PosControlForSlave()` |
| UT-SJ-005 | `SpdControl()` 转发 | 调用 slave 速度控制（按合帧下发，需同批次再调用另一轴一次） | 调用 `MasterJoint::SpdControlForSlave()`，并作为合帧第一条缓存从轴目标 |
| UT-SJ-006 | `CurControl()` 转发 | 调用 slave 电流控制 | 调用 `MasterJoint::CurControlForSlave()` |
| UT-SJ-007 | `TorControl()` 转发 | 调用 slave 扭矩控制 | 调用 `MasterJoint::TorControlForSlave()` |
| UT-SJ-008 | 已配对 Stop | 调用 `Stop()` | 委托 `MasterJoint::Stop()`，两个电机同时停止 |
| UT-SJ-009 | `GetStatus()` 转发 | 调 slave 状态 | 调用 `MasterJoint::GetStatusForSlave()`，返回从轴合成状态 |
| UT-SJ-010 | `SetOnStatus()` 转发 | 注册 slave 回调 | 调用 `MasterJoint::SetOnStatusForSlave()`，收到从轴合成状态 |
| UT-SJ-011 | `SetLimit()` | `reverse=false/true` | slave 运行时参数正确影响 master 从轴映射 |
| UT-SJ-012 | 不支持的校准接口 | `GotoLimit/GotoZero/PosAsOffset/Calibrate` | 返回 NaN 或 false |
| UT-SJ-013 | 未配对转发接口 | 未绑定 master 时调用 `SetZero/Stop/GetStatus/SetOnStatus` | 统一抛 `kDoubleCoupleJointNotPaired` |
| UT-SJ-014 | `SetOnStatus(nullptr)` 转发 | 已绑定 slave 后清空回调 | 委托 `MasterJoint::SetOnStatusForSlave(nullptr)`，停止从轴侧回调 |
| UT-SJ-015 | 已配对未 `SetZero()` 读状态 | slave 已绑定 master 且构造期姿态已验证，但未 `SetZero()` | `GetStatus()` 委托 master 正常读取，不抛零位未设置异常 |
| UT-SJ-016 | pending 阶段 `SetLimit()` | slave 尚未绑定 master 时调用 `SetLimit(reverse=false/true)` | 不抛 `kDoubleCoupleJointNotPaired`，本地限位按 reverse 规则保存，后续配对后影响从轴运行时参数 |
| UT-SJ-017 | `GetStatus()` 生命周期扣减转发 | 已绑定 slave 调用 `GetStatus(false, N)` | 委托 `MasterJoint::GetStatusForSlave(false, N)`，扣减值不丢失 |
| UT-SJ-018 | 控制接口完整参数转发 | 已绑定 slave 调 `PosControl/SpdControl/CurControl/TorControl/Stop`，使用非默认 `current/feedback` | 委托到 master 的参数与输入一致，不被代理层改写；动态类控制按合帧在第二条轴调用时以完成调用的 `feedback` 下发 |
| UT-SJ-019 | `PVTControl()` 完整参数转发 | 已绑定 slave 调 `PVTControl(kp, kd, pos, spd, torque)` | `kp/kd/pos/spd/torque` 全部原样委托到 `MasterJoint::PVTControlForSlave()`，代理层不改写参数 |
| UT-SJ-020 | 已绑定未 `SetZero()` 的 `Stop()` | slave 已绑定 master，但 master 尚未 `SetZero()` | `Stop()` 仍委托到 master 并停止两台电机，不抛零位未设置异常 |
| UT-SJ-021 | pending 阶段不支持校准类接口 | 未绑定 master 的 slave 调 `GotoLimit/GotoZero/PosAsOffset/Calibrate` | 直接返回 NaN 或 false，不抛 `kDoubleCoupleJointNotPaired`，不创建电机、不完成配对 |

### 5.7 JointManager

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-JM-001 | 默认类型 | `DefaultJointManager` | 等价于 `JointManager<RJoint, DoubleCoupleJoint>` |
| UT-JM-002 | `CreateAdapter()` 幂等 | 同名 adapter 创建两次 | 只保留一个实例，不抛错 |
| UT-JM-003 | `AddJoint<RJoint>()` | 合法 config | 返回 `RJoint` 实例，可 `GetJoint()` |
| UT-JM-004 | 重名 AddJoint | 同名关节重复添加 | 抛 `std::invalid_argument` |
| UT-JM-005 | `AddJoint<DoubleCoupleJoint>()` pending | 首个闭链关节 | 返回 `SlaveJoint` |
| UT-JM-006 | `AddJoint<DoubleCoupleJoint>()` active | 第二个闭链关节 | 返回 `MasterJoint` |
| UT-JM-007 | `AddJoint(json)` RJoint | type/name/config JSON | 创建 RJoint |
| UT-JM-008 | `AddJoint(json)` DoubleCoupleJoint | type/name/config JSON | 创建闭链关节 |
| UT-JM-009 | `AddJoint(json)` 未知 type | 未注册 type | 抛 `std::invalid_argument` |
| UT-JM-010 | `GetJoint()` 缺失 | 不存在名称 | 返回 nullptr |
| UT-JM-011 | `GetStatus()` | 多关节状态可用 | 返回 name -> status map |
| UT-JM-012 | `GetStatus()` 异常传播 | 某关节抛 `JointException` | 异常透传 |
| UT-JM-013 | `PosControl()` 批量控制 | name -> pos map | 对存在关节下发控制，缺失关节跳过 |
| UT-JM-014 | `PosControl()` 异常传播 | 某关节抛 `JointException` | 异常透传 |
| UT-JM-015 | `Calibrate()` 成功 | 所有关节校准成功 | 正常返回 |
| UT-JM-016 | `Calibrate()` 失败 | 某关节返回 false | Stop 后抛 `std::runtime_error` |
| UT-JM-017 | `SetOnStatus()` | 注册指定关节回调 | 状态到达触发回调 |
| UT-JM-018 | `SetAdaptersStatusMaxLifeCycle()` | 设置生命周期 | 传递到底层 adapter |
| UT-JM-019 | `GetAvailableJointTypes()` | 默认 manager | 包含 `RJoint`、`DoubleCoupleJoint` |
| UT-JM-020 | `GetJointSchema()` | 默认 manager | 返回两个非空 schema |
| UT-JM-021 | `AddJoint(json)` 缺字段 | 缺 `type/name/config` 或 config 缺必填字段 | 抛 JSON 解析异常 |
| UT-JM-022 | `AddJoint(json)` 重名 | 通过 JSON 重复添加同名关节 | 抛 `std::invalid_argument` |
| UT-JM-023 | pending 配对失败后恢复 | 添加第一个闭链关节，第二次配对失败，再次合法配对 | 失败不污染 manager 内部 pending 状态 |
| UT-JM-024 | `GetStatus()` 忽略空状态关节 | 某关节返回 `std::nullopt`，其他关节正常 | 结果仅包含有状态关节 |
| UT-JM-025 | `SetOnStatus()` 目标缺失 | 注册不存在名称 | 无异常、无副作用 |
| UT-JM-026 | `Calibrate()` 缺失名称跳过 | `dirs` 中包含不存在关节 | 现有目标照常执行，不存在名称被忽略 |
| UT-JM-027 | `GetStatus(autoSendCommand=true)` 透传 | 某 RJoint 或闭链关节无缓存状态 | manager 返回结果中忽略该关节，同时对应关节下发反馈请求命令 |
| UT-JM-028 | pending 闭链异常传播 | manager 内包含未配对的闭链 slave，调用需要绑定 master 的接口 | 抛 `kDoubleCoupleJointNotPaired`，不吞异常 |
| UT-JM-029 | `SetOnStatus(nullptr)` 注销透传 | 已注册回调后传入 `nullptr` | 正确转发到目标关节并停止后续状态通知 |
| UT-JM-030 | pending slave `Calibrate()` 路径 | manager 内只有未配对的闭链 slave，调用 `Calibrate({name, dir})` | slave `Calibrate()` 返回 false 后 manager 尝试 `Stop()`，最终抛 `kDoubleCoupleJointNotPaired` |
| UT-JM-031 | 闭链第二轴电机顺序反转 | 第一次添加 `(a,b)`，第二次添加同一对电机但配置为 `(b,a)` | 不完成配对，第二次也形成新的 pending slave；原 pending 仍可继续用合法同顺序配置配对 |
| UT-JM-032 | pending slave `SetOnStatus()` 路径 | manager 内只有未配对的闭链 slave，调用 `SetOnStatus(name, cb)` | 异常透传为 `kDoubleCoupleJointNotPaired`，不注册回调、不触发后续状态通知 |
| UT-JM-033 | `GetStatus()` 生命周期扣减透传 | manager 管理 RJoint、闭链 master/slave，调用 `GetStatus(autoSendCommand, N)` | 所有被读取关节都收到相同 `lifeCycleDeduction=N`，异常路径仍按原接口透传 |
| UT-JM-034 | `GetStatus(autoSendCommand=false)` 无缓存链路 | manager 管理无缓存 RJoint 与已设零位闭链关节，关闭自动请求 | 返回 map 忽略无状态关节，不下发任何反馈请求命令 |
| UT-JM-035 | `CreateAdapter()` 同名不同类型 | 先用名称 N 创建类型 A，再用同名 N 创建类型 B | manager 保留第一次创建的 adapter，不替换、不再次调用 `MakeAdapter()` |
| UT-JM-036 | `CreateAdapter()` 非法类型失败 | 使用未知或底层拒绝的 `adapterType` | `MakeAdapter()` 的异常/失败向上传播，manager 不登记无效 adapter，后续可用合法类型重试同名创建 |
| UT-JM-037 | `PosControl()` 批量非事务语义 | 多个关节按名称排序执行，中间某关节抛异常 | 异常向上传播；已执行关节不回滚，异常后的关节不继续下发 |
| UT-JM-038 | `Calibrate()` 批量非事务语义 | 多个关节按输入 map 顺序校准，中间某关节失败 | 失败关节先执行 Stop 再抛异常；已校准关节不回滚，异常后的关节不继续校准 |
| UT-JM-039 | `AddJoint(json)` envelope 类型错误 | `type/name` 不是 string，或 `config` 为 null/非 object | 抛 nlohmann JSON 类型异常或明确 `std::invalid_argument`；manager 不登记 joint，不污染 pending/active pair |
| UT-JM-040 | `AddJoint(json)` config 解析失败后状态保持 | JSON envelope 合法，但 `Config::FromJson()` 因字段类型、缺字段、非法 axis/model 失败 | 异常向上传播；同名 joint 不被登记，闭链 pending/active pair 不变化，修正 JSON 后可重试 |
| UT-JM-041 | `CreateAdapter()` 返回空 adapter | `MakeAdapter()` 未抛异常但返回 `nullptr` | manager 视为创建失败并抛明确异常，不登记空 adapter；后续同名合法 adapter 可重试 |
| UT-JM-042 | `SetAdaptersStatusMaxLifeCycle()` 多 adapter 与边界值 | manager 管理多个 adapter，分别设置 `0`、负数、`INT_MAX` 和普通正数 | 每个 adapter 均收到同一输入值，不遗漏、不改写边界值、不因空 manager 崩溃 |
| UT-JM-043 | `DefaultJointManager` 暴露模板 `AddJoint<T>()` | `DefaultJointManager manager; manager.AddJoint<RJoint>(name, config)` | 模板接口与 `AddJoint(json)` 同时可见；缺 adapter 时运行期抛 `std::invalid_argument` 而不是编译期不可调用 |
| UT-JM-044 | CAN FD 默认状态与空 manager 切换 | 新建 manager，不添加 joint，重复调用 enable/disable/is | 默认 false；enable 后 true，disable 后 false；空 manager 调用无异常、幂等 |
| UT-JM-045 | CAN FD 切换已存在 RJoint | manager 创建 RJoint 后发送控制，再 enable/disable 后重复发送 | 已存在 RJoint 的后续报文 `frame_flags` 随 manager 状态切换，`IsJointMotorCanfd()` 与实际报文一致 |
| UT-JM-046 | CAN FD 状态应用到未来 RJoint | enable 后创建 RJoint，disable 后再创建另一个 RJoint | 新创建电机继承创建时的 manager CAN FD 状态 |
| UT-JM-047 | CAN FD 对 pending slave 无副作用 | 只有未配对 `SlaveJoint` 时 enable/disable | 不抛异常、不创建电机、不下发消息，pending 仍可继续配对 |
| UT-JM-048 | CAN FD 切换已存在 MasterJoint 双电机 | 闭链配对完成后 enable/disable，并调用 `Stop()` 或控制接口 | 两台电机报文同时切换 CAN FD flag，不遗漏任一电机 |
| UT-JM-049 | CAN FD 状态应用到未来闭链 master | pending slave 已存在，enable 后添加第二轴完成配对 | master 创建阶段两台预创建电机均继承 CAN FD 状态；disable 后后续报文恢复 classic CAN |
| UT-JM-050 | `AddJoint()` 创建电机后再注入关节 | RJoint 与闭链第二轴分别通过 manager 添加，构造阶段不接收 `AdapterMap` | manager 统一调用 `CreateMotor()`，构造函数只接收非空 `Motor`；构造失败不登记无效 joint、不破坏 pending 状态 |
| UT-JM-051 | 重名优先级 | 已存在同名 joint，再通过模板或 JSON 路径用同名传入非法 RJoint config、非法 JSON config 或闭链重复 pair config | 优先抛重名 `std::invalid_argument`；JSON 路径不得读取或解析 `config`，不得继续进入其他配对或电机创建错误 |
| UT-JM-052 | 闭链第二轴电机创建失败后状态保持 | pending slave 已存在且配对约束合法，但 `CreateMotor()` 因 adapter/bus/motor 失败 | 异常向上传播；原 pending 不被删除，active pair 不新增，修正电机配置后仍可完成配对 |
| UT-JM-053 | `AddJoint(json)` type 精确匹配 | `type` 使用错误大小写、前后空白或同名非注册类型 | 不做模糊匹配，抛 `std::invalid_argument`；manager 不登记 joint/pending |
| UT-JM-054 | `AddJoint(json)` 未知 type 错误优先级 | `type` 未注册且 `config` 同时缺字段或字段类型非法 | 优先抛未知类型 `std::invalid_argument`；不得解析 `config`，manager 不登记 joint/pending |
| UT-JM-055 | 闭链 master 构造失败后状态保持 | pending slave 已存在且电机创建成功，但 `MasterJoint` 构造因非法 `kt` 或 solver 标定失败抛异常 | 原 pending 不被删除，active pair 不新增，master 名称不登记；修正配置后仍可用原 pending 完成配对 |
| UT-JM-056 | 单 Adapter Motor 文件日志命名 | 编码 bus id 表示 slave 3、bus 4，motor id 为 5 | `EnableLog(path)` 创建 `s3b4m5_status.csv.zstd` 和 `s3b4m5_command.csv.zstd` |
| UT-JM-057 | 多 Adapter Motor 文件日志命名 | 两个 Adapter 下具有相同 slave/bus/motor id | 文件分别使用 `<adapterName>_sXbXmX` 基础名，不发生跨 Adapter 名称冲突 |
| UT-JM-058 | Motor 文件日志生命周期 | `EnableLog()`、`IsLogged()`、`DisableLog()` | Manager 状态与全部已跟踪 Motor 一致，关闭时刷新全部日志 |
| UT-JM-059 | 日志状态应用到未来 Motor | 空 Manager 启用日志后添加关节 | 新创建 Motor 自动按当前路径和 Adapter 数量启用日志 |
| UT-JM-060 | 重复物理 Motor 日志去重 | 多个关节引用相同 adapter/slave/bus/motor | 同一个 Motor 只跟踪和切换一次，不重复创建日志会话 |
| UT-JM-061 | 日志路径稳定性与文件名安全 | 使用相对输出路径后切换工作目录；多 Adapter 名包含路径分隔符 | 后续 Motor 仍写入最初解析的绝对目录；Adapter 名被转义且不能逃逸目标目录 |
| UT-JM-062 | 日志启用期间关节构造失败 | `EnableLog()` 后添加配置非法的 RJoint 或闭链 master | 失败 Motor 和 Bus 不进入跟踪集合、不残留活动日志，Manager 可继续使用 |
| UT-JM-063 | 日志启用期间增加第二个 Adapter | 单 Adapter 已记录 Motor 时创建第二个 Adapter | 已有 Motor 保持原日志文件名和记录状态；此后新 Adapter 的 Motor 使用带 Adapter 前缀的名称 |
| UT-JM-064 | Adapter 建立顺序 id 索引 | 依次创建多个 Adapter，对其中一个同名重复创建 | id 按创建顺序从 `"0"` 递增；同名重复创建不新增 id，两个索引都指向最新实例 |
| UT-JM-065 | `adapterId` 按取值类型选索引 | 同一个 `"0"` 同时存在于建立顺序 id 索引与某个 Adapter 名称，且两者指向不同 Adapter | 整数引用只查 id 索引，字符串引用只查名称索引，都不做跨索引回退；命中与未命中分别返回对应 Adapter 与抛 `std::invalid_argument` |
| UT-JM-066 | `JointManager(json)` 配置构造 | 传入含 `adapters` 与 `joints` 数组的 JSON 对象 | 先按 `adapters` 顺序创建 Adapter，再按 `joints` 顺序创建关节；关节既可用整数索引也可用 Adapter 名称引用适配器 |
| UT-JM-067 | `JointManager(json)` 部分与空配置 | 传入 `{}`、只有 `adapters`、只有空 `joints` 三种对象 | 均构造成功且不含任何关节；缺省键不报错 |
| UT-JM-068 | `JointManager(json)` 非法配置 | 顶层非对象；`adapters`/`joints` 非数组；适配器缺 `type`/`name`；关节类型未知或名称重复；关节引用不存在的适配器 | 分别抛 `std::invalid_argument` 或 JSON 类型异常；不留下半构造状态 |

### 5.8 JSON Schema Generator

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-JS-001 | RJoint schema | `RJoint::Config::ToJsonSchema()` | 包含 motorParams/limit/maxSpd/maxTor/reverse |
| UT-JS-002 | DoubleCouple schema | `DoubleCoupleJoint::Config::ToJsonSchema()` | 包含 motorParams1/2、axis、calibration |
| UT-JS-003 | schema definitions | 包含 Range/MotorParams/CalibrationInput | `$ref` 正确 |
| UT-JS-004 | required 字段 | 所有必填字段 | `required` 列表完整 |
| UT-JS-005 | MotorModel enum | `GetAllMotorModelStrings()` | schema 中型号枚举同步 |
| UT-JS-006 | `FromJson()` 缺字段 | 缺必填字段 | 抛 nlohmann json 异常 |
| UT-JS-007 | schema 可解析 | `ToJsonSchema()` 返回字符串 | 可被 JSON parser 成功解析 |
| UT-JS-008 | axis 大小写兼容 | `roll/Roll/pitch/Pitch` | 均可解析到正确 `DoubleCoupleJointAxis` |
| UT-JS-009 | 非法 `rotationOrder` 字符串 | 未知字符串 | 抛 `std::invalid_argument` |
| UT-JS-010 | 非法 `rotationOrder` 整数 | 非 0/1 数值 | 抛 `std::invalid_argument` |
| UT-JS-011 | 非法 motor model | schema 合法但 `model` 字符串未知 | 抛 `std::invalid_argument` |
| UT-JS-012 | 额外字段兼容 | JSON 中带未使用字段 | `FromJson()` 忽略额外字段，不影响已知字段解析 |
| UT-JS-013 | `MotorInit` 解析优先级 | `initParam` 同时包含 `ranges` 和 `model` | `FromJson()` 优先按 `ranges` 解析为固定范围初始化 |
| UT-JS-014 | schema 对 auto `initParam` 额外字段严格校验 | auto init 分支带未知字段 | JSON Schema 校验失败；该用例与 `FromJson()` 宽松兼容用例分开验证 |
| UT-JS-015 | schema 对 `initParam` 分支互斥 | 同一个 `initParam` 同时包含 `ranges` 和 `model` | JSON Schema `oneOf` 校验失败；该用例与 `FromJson()` 的 `ranges` 优先级宽松解析分开验证 |
| UT-JS-016 | `Range` schema 必填与类型 | `Range` 缺少 `min/max` 或 `min/max` 非 number | JSON Schema 校验失败，防止 limit 与 PVT range 缺字段进入配置 |
| UT-JS-017 | `Point3` schema 必填与类型 | calibration 中任一 `Point3` 缺少 `x/y/z` 或字段非 number | JSON Schema 校验失败，防止标定点结构不完整 |
| UT-JS-018 | nested object 额外字段契约 | `Range`、`Point3`、model init、ranges init 分支带未知字段 | 当前 schema 对这些对象保持宽松兼容；该预期与 auto `initParam` 严格拒绝额外字段分开验证 |
| UT-JS-019 | JSON 与业务数值校验分层 | JSON 字段类型合法但 `limit.min > limit.max`、`maxSpd <= 0`、`maxTor <= 0`、PVT range `min > max` | schema/`FromJson()` 与构造阶段的职责清晰；最终不得创建可控关节，失败类型和阶段在测试中固定 |
| UT-JS-020 | 顶层 config 额外字段契约 | `RJoint::Config` 或 `DoubleCoupleJoint::Config` 顶层对象带未知字段 | 当前 schema 与 `FromJson()` 均保持宽松兼容；未知字段被忽略，不影响已知字段解析 |
| UT-JS-021 | `MotorParams` / `CalibrationInput` 额外字段契约 | `MotorParams`、`CalibrationInput` 对象带未知字段 | 当前 schema 与 `FromJson()` 均保持宽松兼容；未知字段被忽略，不影响电机参数或标定参数解析 |
| UT-JS-022 | `AddJoint(json)` envelope 额外字段契约 | JSON envelope 带除 `type/name/config` 外的额外字段 | 额外字段被忽略；合法 `type/name/config` 仍决定创建行为 |
| UT-JS-023 | `rotationOrder` 类型错误 | `rotationOrder` 为非整数 number、bool、object 或 array | `FromJson()` 抛 nlohmann JSON 类型异常或明确 `std::invalid_argument`；不得生成默认 rotation order |
| UT-JS-024 | `axis` 类型错误 | `axis` 为 number、bool、object、array 或 null | `FromJson()` 抛 nlohmann JSON 类型异常；不得默认成 roll 或 pitch |
| UT-JS-025 | `MotorParams` 字段类型 schema 契约 | `adapterId` 非 string，`busId/motorId/slaveId` 非 integer，`canfd` 非 boolean，`initParam` 非 object | JSON Schema 校验失败；该预期与 `FromJson()` 类型异常保持一致 |
| UT-JS-026 | `MotorParams` schema 与解析键一致 | 比较 `GetMotorParamsSchema()` 的 `properties`/`required` 与 `from_json(MotorParams)` 实际读取的键 | `properties` 覆盖 `adapterId/slaveId/busId/motorId/canfd/initParam`；`required` 仅含 `adapterId/busId/motorId/initParam` |

### 5.9 CMake 构建开关

| ID | 测试项 | 输入 | 期望 |
|----|--------|------|------|
| UT-CM-001 | 默认构建 | `JOINT_SDK_BUILD_TESTS=OFF` | 只构建库和必要依赖，不生成测试目标 |
| UT-CM-002 | 开启测试 | `JOINT_SDK_BUILD_TESTS=ON` | 生成 `joint_sdk_tests` 和 `mechanism_solver_test` |
| UT-CM-003 | 子模块测试联动 | 顶层测试关闭 | `mechanism_solver_test` 目标不存在 |
| UT-CM-004 | 子模块测试开启 | 顶层测试开启 | CTest 注册 `mechanism_solver_test` |
| UT-CM-005 | 安装导出 | `cmake --install` | 导出 `joint_sdkTargets.cmake` 和 package config |
| UT-CM-006 | 缺失 Boost.Preprocessor 子模块 | `external/boost_preprocessor/include/boost/preprocessor.hpp` 不存在 | CMake 配置阶段 `FATAL_ERROR` |
| UT-CM-007 | 缺失 `couple_joint_solver` 子模块 | `external/couple_joint_solver/CMakeLists.txt` 不存在 | CMake 配置阶段 `FATAL_ERROR` |
| UT-CM-008 | 安装头文件完整性 | 执行 `cmake --install` | 安装目录包含 `joint_sdk` 公开头文件及 `boost/` 头 |
| UT-CM-009 | 安装后最小消费工程 | 独立 consumer `find_package(joint_sdk CONFIG REQUIRED)` 并链接 | 配置、编译、链接成功，依赖 `EncosMotorDriver/nlohmann_json/couple_joint_solver` 自动解析 |
| UT-CM-010 | 安装版本文件 | 执行 `cmake --install` | 安装目录包含 `joint_sdkConfigVersion.cmake`，consumer 指定兼容版本时可通过配置 |
| UT-CM-011 | 安装导入目标 | 独立 consumer 使用 `target_link_libraries(app PRIVATE joint_sdk::joint_sdk)` | 导入目标存在，include 路径、Boost.Preprocessor 头和传递依赖均可用 |
| UT-CM-012 | 关闭测试时不依赖 GTest | 未安装或隐藏 GTest，配置 `JOINT_SDK_BUILD_TESTS=OFF` | CMake 配置和普通构建仍通过，不执行 `find_package(GTest)` |
| UT-CM-013 | 安装包依赖版本传播 | 执行 `cmake --install` 后检查生成的 `joint_sdkConfig.cmake` | `find_dependency(encos_driver 3.3.2 REQUIRED CONFIG)` 与顶层 `find_package(encos_driver 3.3.2 REQUIRED CONFIG)` 保持一致 |

## 6. 集成测试设计

### 6.1 软件集成测试

软件集成测试使用 `TestAdapter`、真实 `Bus/Motor` 接口和真实 `couple_joint_solver`，不连接真实硬件。

| ID | 场景 | 步骤 | 验收 |
|----|------|------|------|
| IT-SW-001 | RJoint 全流程 | 创建 adapter -> 添加 RJoint -> 位置控制 -> 注入状态 -> 读取状态 | 下发消息正确，状态为关节坐标系 |
| IT-SW-002 | RJoint reverse 全流程 | `reverse=true` 重复 IT-SW-001 | 控制输入和状态输出符号一致 |
| IT-SW-003 | RJoint 校准流程 | 模拟限位/零点 ACK -> `Calibrate()` | 成功返回并下发正确命令序列 |
| IT-SW-004 | 双闭链配对流程 | 添加 pitch slave -> 添加 roll master，构造期读取当前双电机位置 | slave/master 绑定成功，当前姿态可正解并初始化缓存，不下发 zero reset |
| IT-SW-005 | 双闭链 PVT 控制 | 配对完成后不调用 `SetZero()`，直接 roll PVT，再 slave pitch PVT | 两次控制均解算双电机目标；速度/扭矩任一电机超范围时整体缩放，缓存位姿正确 |
| IT-SW-006 | 双闭链位置控制 | 控制 roll/pitch 到指定值 | 电机位置接近期望求解结果 |
| IT-SW-007 | 双闭链速度控制 | 发送 roll 或 pitch 速度（每周期两轴各一次调用） | 每周期只下发一帧（两台电机各一条记录）；电机速度符合两轴联合逆速度映射，任一电机超范围时整体缩放且均不超范围 |
| IT-SW-008 | 双闭链扭矩控制 | 发送 roll 或 pitch 扭矩 | 电机扭矩符合逆扭矩映射；任一电机超范围时整体缩放且均不超范围 |
| IT-SW-009 | 双闭链电流控制 | 发送 roll 或 pitch 电流 | 电流按扭矩比拆分；任一电机超范围时整体缩放且均不超范围 |
| IT-SW-010 | Pull 状态缓存 | 控制到 A -> 注入实际零位 -> `GetStatus()` -> 控制另一轴 | 后续目标基于实际零位而非旧缓存 |
| IT-SW-011 | Push 状态缓存 | 注册回调 -> 注入双电机状态 -> 控制另一轴 | 后续目标基于回调更新后的位姿 |
| IT-SW-012 | 状态缺失 | 只注入一个电机状态 | 不触发合成状态，不更新闭链缓存 |
| IT-SW-013 | 批量状态读取 | manager 管理多个 RJoint 和闭链关节 | 返回所有可用状态，异常按接口透传 |
| IT-SW-014 | 批量位置控制 | manager 批量控制多个关节 | 存在关节均收到命令，缺失名称跳过 |
| IT-SW-015 | JSON 创建混合关节 | 从 JSON 创建 RJoint 和闭链关节 | 类型分发正确，schema 与 FromJson 一致 |
| IT-SW-016 | 异常路径集成 | 重复配对、重复轴、标定失败、构造期姿态不可解 | 抛出预期 `JointError` |
| IT-SW-017 | CTest 集成 | `JOINT_SDK_BUILD_TESTS=ON` 后运行 CTest | 所有注册测试通过 |
| IT-SW-018 | no-tests 构建集成 | `JOINT_SDK_BUILD_TESTS=OFF` 构建 | 普通构建通过，测试目标不存在 |
| IT-SW-019 | 踝关节示例闭链集成 | 使用 `ankle.json` 的 `calibration` 补齐 roll/pitch 两份 `DoubleCoupleJoint::Config` | 配对、构造期当前姿态验证、小角度位置控制和状态合成通过 |
| IT-SW-020 | 腰部示例闭链集成 | 使用 `waist.json` 的 `calibration` 补齐 roll/pitch 两份 `DoubleCoupleJoint::Config` | 配对、构造期当前姿态验证、小角度位置控制和状态合成通过 |
| IT-SW-021 | RJoint PVT 控制集成 | 创建 RJoint -> `PVTControl()` -> 注入状态 -> `GetStatus()` | PVT 下发参数正确，状态读取仍为关节坐标系 |
| IT-SW-022 | 闭链配对失败后恢复 | 添加 pending slave -> 非法 master 配对失败 -> 合法 master 重试 | manager 内部 pending 状态可恢复，最终配对成功 |
| IT-SW-023 | 闭链部分反馈字段集成 | 注入仅有位置、缺速度或缺电流的双电机反馈 | 位姿正常更新，缺失通道保持 NaN 或不更新 |
| IT-SW-024 | RJoint 辅助接口集成 | 创建 RJoint -> 注入状态 -> 调 `GetMotorPosition()` / `GetReversedPosition()` | 返回值与缓存状态及 reverse 配置一致 |
| IT-SW-025 | JSON 错误路径集成 | 传入缺字段、非法 axis、非法 `rotationOrder`、未知 joint type 的 JSON | 抛出预期 JSON 或 `std::invalid_argument` 异常 |
| IT-SW-026 | 安装包消费集成 | `cmake --install` 到 staging 目录后，独立 consumer `find_package(joint_sdk)` 并链接 `joint_sdk::joint_sdk` | 最小工程能配置、编译、链接通过，版本文件和导入目标可用 |
| IT-SW-027 | 闭链未 `SetZero()` 状态读取 | 配对完成且构造期姿态已验证，未 `SetZero()` 时分别调用 master、slave、manager `GetStatus()` | 三条路径按普通状态读取行为返回状态或空结果，不抛零位未设置异常 |
| IT-SW-028 | manager `autoSendCommand` 透传 | manager 下某些关节无缓存状态，调用 `GetStatus(true)` | 无缓存关节会下发反馈请求命令，有缓存关节正常返回 |
| IT-SW-029 | 回调注销链路 | 注册 master/slave/manager 回调后依次传入 `nullptr` | 对应路径停止回调，未注销的其他路径继续工作 |
| IT-SW-030 | RJoint 部分反馈字段集成 | 注入位置有效、速度或电流缺失的电机状态 | `GetStatus()` 与回调输出保持逐字段 NaN 传播 |
| IT-SW-031 | 位置缓存 `NaN` 回退读参数 | 对 RJoint 和闭链 helper 注入带 `position=NaN` 的缓存状态 | `GetMotorPosition()` / `GetMotor1Position()` / `GetMotor2Position()` 回退到底层参数读取 |
| IT-SW-032 | 闭链回调无需 `SetZero()` | 分别通过 master、slave、manager 注册回调，直接注入双电机状态 | 双电机状态齐全后三条路径都开始正常收到状态 |
| IT-SW-033 | 闭链位置控制参数处理 | master/slave 分别调用 `PosControl()`，覆盖默认速度、超限速度、零电流、超限电流 | 双电机位置命令正确；速度和电流符合各轴 `maxSpd/maxTor/kt` 规则 |
| IT-SW-034 | 闭链非有限电流输入 | master/slave 分别调用 `CurControl(NaN/Inf)` | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，后续合法控制仍基于旧缓存 |
| IT-SW-035 | pending slave 限位延迟生效 | 创建 pending slave 后先 `SetLimit()`，再添加 master 完成配对并控制从轴 | 从轴控制使用 pending 阶段设置的限位与 reverse 规则 |
| IT-SW-036 | 闭链反转电机顺序不配对 | 添加 `(a,b)` pending slave，再添加 `(b,a)` 另一轴配置 | 不生成 master，第二个关节也是 pending slave；随后用 `(a,b)` 合法第二轴配置可完成原 pending 配对 |
| IT-SW-037 | 闭链零输入缓存保持 | 控制到非零位姿后，对 master/slave 分别发送零速度、零扭矩、零电流，再控制另一轴 | 零输入只下发零命令；无反馈时后续控制仍基于零输入前缓存，有反馈时按反馈正解刷新缓存 |
| IT-SW-038 | manager pending slave 校准异常 | manager 只管理未配对闭链 slave 时调用 `Calibrate()` | 抛 `kDoubleCoupleJointNotPaired`，测试记录这是 pending 状态的异常路径 |
| IT-SW-039 | manager pending slave 回调注册异常 | manager 只管理未配对闭链 slave 时调用 `SetOnStatus(name, cb)` 并注入相关电机状态 | 抛 `kDoubleCoupleJointNotPaired`，不完成回调注册，后续状态注入不触发该回调 |
| IT-SW-040 | JSON schema 与 `FromJson()` 契约分离 | 同一配置分别走 schema 校验和 `FromJson()` 解析，覆盖顶层额外字段与 auto `initParam` 额外字段 | 顶层额外字段可被 `FromJson()` 忽略；auto `initParam` 未知字段按 schema 严格拒绝，二者预期不混淆 |
| IT-SW-041 | 状态生命周期扣减链路 | RJoint、闭链 master/slave、manager 分别使用非默认 `lifeCycleDeduction` 读取状态 | 扣减值完整传到底层电机缓存读取；`autoSendCommand` 行为保持不变 |
| IT-SW-042 | 控制参数透传链路 | RJoint 与闭链 master/slave 使用非默认 `feedback`，速度控制覆盖 `current <= 0` 与超限 current | 下发命令中的 feedback/current 与各接口规则一致，双电机路径两侧一致；合帧速度控制取两轴较小电流上限与完成调用的 feedback |
| IT-SW-043 | 闭链回调生命周期 | 注册闭链主/从轴回调后销毁配对对象，再向相关电机注入状态 | 底层回调已卸载，不触发 joint 回调，不访问已销毁对象 |
| IT-SW-044 | 闭链二次 `SetZero()` 失败 | `SetZero()` 成功并控制后，再模拟 reset 失败或零位不可解 | 第二次返回 false，不把目标位姿强制归零；后续控制/读状态不依赖零位门禁 |
| IT-SW-045 | PVT 增益坐标转换链路 | RJoint、闭链 master/slave 分别调用 `PVTControl()`，使用非默认 `kp/kd`，并使实际反馈位姿与目标位姿不同 | RJoint 保持直接电机增益；闭链按实际反馈位姿雅可比融合两轴关节增益后下发各电机增益；位置、速度、扭矩仍按各接口规则保护 |
| IT-SW-046 | RJoint 校准参数链路 | RJoint 使用非默认 `spd/cur/timeout` 执行 `GotoLimit/GotoZero/Calibrate` | 限位搜索和回零两阶段均收到同一组运动参数，reverse 只影响方向或 offset |
| IT-SW-047 | 闭链非法 `kt` 防护 | 闭链两电机中任一电机 PVT range 使用 `kt <= 0` 或 NaN/Inf | 创建或初始化阶段失败并给出明确异常，不允许进入控制流程或下发未受保护电流 |
| IT-SW-048 | 电机初始化分支链路 | RJoint 与闭链分别使用 auto、model、自定义 ranges 创建电机 | 三种初始化路径都能创建目标电机；自定义 ranges 在后续裁剪和 `kt` 校验中生效 |
| IT-SW-049 | `autoSendCommand=false` 状态读取链路 | RJoint、闭链 master/slave、manager 在无缓存时读取状态且关闭自动请求 | 返回空状态或忽略该关节，不下发反馈请求命令；切换为 true 时才下发请求 |
| IT-SW-050 | 闭链未 `SetZero()` Stop 链路 | 闭链完成配对但未 `SetZero()`，分别从 master 和 slave 调用 `Stop()` | 两台电机均收到停止命令，且不抛零位未设置异常 |
| IT-SW-051 | JSON Schema `initParam` 互斥链路 | 同一配置分别走 schema 校验和 `FromJson()`，`initParam` 同时包含 `ranges` 和 `model` | schema 校验失败；`FromJson()` 按 `ranges` 优先级解析，二者契约保持清晰分离 |
| IT-SW-052 | 近奇异安全方向链路 | 将闭链状态推到 near singular，分别发送远离奇异区的速度/扭矩/电流命令 | 控制命令正常下发并保持范围保护；只有进入奇异区方向才触发位置保持或 `FullBrake` 兜底 |
| IT-SW-053 | no-tests 无 GTest 构建链路 | 在不提供 GTest 的环境配置 `JOINT_SDK_BUILD_TESTS=OFF` | 配置和构建通过；开启测试时才要求 GTest 并注册测试目标 |
| IT-SW-054 | JSON Schema nested object 契约链路 | RJoint 与闭链 JSON 分别覆盖 `Range`、PVT `ranges`、calibration `Point3` 的缺字段、类型错误和额外字段 | 缺字段/类型错误按 schema 拒绝；额外字段按 UT-JS-018 的当前宽松契约处理，`FromJson()` 宽松解析预期不被 schema 用例混淆 |
| IT-SW-055 | adapter/bus 创建失败链路 | manager 创建非法 adapter 类型，或用存在的 adapter 添加无效 `busId` 关节 | 失败语义向上传播，manager 不留下无效 adapter/joint 状态，修正配置后可重试 |
| IT-SW-056 | 保留错误码运行时链路 | 覆盖首次闭链 AddJoint pending、pending slave 公开接口异常、MasterJoint 不支持校准接口 | 当前运行时不抛 `kDoubleCoupleJointPending` / `kDoubleCoupleJointUnsupportedControl`，只抛已有明确场景的 `JointError` 或返回 false/NaN |
| IT-SW-057 | 安装包依赖版本链路 | install 后独立 consumer 通过 `find_package(joint_sdk CONFIG REQUIRED)` 间接解析依赖 | consumer 必须解析到满足 `encos_driver 3.3.2` 的包；低版本依赖环境配置失败并给出 CMake 诊断 |
| IT-SW-058 | 配置数值语义链路 | 通过 JSON 创建 RJoint 与闭链，分别传入非法 limit、非正 maxSpd/maxTor、非法 PVT ranges | schema/FromJson/构造链路的失败阶段符合 UT-JS-019；manager 不留下可控 joint 或污染 pending/active pair |
| IT-SW-059 | pending 闭链延迟校验链路 | 添加首个 invalid-calibration 闭链关节，再添加第二轴触发配对 | 首次只形成 pending，不创建电机/solver；第二次抛标定失败，原 pending 仍可用合法第二轴恢复配对 |
| IT-SW-060 | 闭链非有限控制输入链路 | master/slave 分别调用 PVT、速度、力矩、电流，输入 NaN/Inf | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，目标缓存不提交，后续合法控制基于旧缓存 |
| IT-SW-061 | manager 批量接口非事务链路 | manager 管理多个关节，批量 `PosControl()` 或 `Calibrate()` 中间目标抛异常 | 已执行目标保持其下发/校准副作用，失败目标异常透传，后续目标不继续执行 |
| IT-SW-062 | motor 获取失败链路 | adapter 和 bus 存在但 `motorId` 无效，分别创建 RJoint 与闭链 master | `GetMotor()` 失败向上传播，manager 不登记无效 joint，不污染 pending/active pair，修正 motorId 后可重试 |
| IT-SW-063 | 闭链限位钳制链路 | master/slave 分别发送超出 `limit.max`/`limit.min` 的 PVT/PosControl 命令，覆盖 `reverse=false/true` | 位置被钳制到限位边界，电机命令正常下发，不触发 Stop 或异常，后续控制基于钳制后的缓存位姿 |
| IT-SW-064 | RJoint 非有限控制输入链路 | RJoint 分别调用 PVT、位置、速度、电流、扭矩、Stop，输入 NaN/Inf | 控制接口先 Stop 并抛 `std::invalid_argument`，Stop 接口电流归零；底层消息不包含 NaN/Inf |
| IT-SW-065 | 闭链位置与 PVT 增益非有限输入链路 | master/slave 分别调用 `PosControl(position/speed/current=NaN/Inf)` 和 `PVTControl(kp/kd=NaN/Inf)` | 有反馈时两电机位置保持，无反馈时 `FullBrake`；抛 `std::invalid_argument`，不提交目标缓存，底层消息不包含 NaN/Inf |
| IT-SW-066 | JSON envelope 与额外字段契约链路 | manager `AddJoint(json)` 覆盖 `type/name/config` 类型错误、config null/非 object、顶层/MotorParams/CalibrationInput 额外字段 | 类型错误不登记 joint/pending；额外字段按 UT-JS-020/021 宽松忽略，合法字段仍创建成功 |
| IT-SW-067 | 空 adapter 防护链路 | `MakeAdapter()` 返回空 adapter，或 `AdapterMap` 中存在空 adapter 指针后创建 RJoint/闭链 | 失败语义向上传播，manager 不登记空 adapter 或无效 joint，不污染 pending/active pair，修正后可重试 |
| IT-SW-068 | adapter 生命周期配置链路 | manager 管理多个 adapter，调用 `SetAdaptersStatusMaxLifeCycle()` 设置 `0`、负数、`INT_MAX` 和普通正数 | 所有 adapter 均收到原始输入值；空 manager 调用无异常、无副作用 |
| IT-SW-069 | CAN FD manager 状态链路 | manager 创建 RJoint、pending slave、闭链 master，穿插 enable/disable 并发送控制/Stop | `IsJointMotorCanfd()`、已有电机、未来电机和闭链双电机报文 flag 一致；pending slave 不创建电机、不下发消息 |
| IT-SW-070 | 模板 AddJoint 与 JSON AddJoint 共存链路 | `DefaultJointManager` 先用 `AddJoint<RJoint>(name, config)`，再用 `AddJoint(json)` 添加其他关节 | 两种 public API 均可调用，重名和未知类型仍按原异常契约处理 |
| IT-SW-071 | 配对校验优先级链路 | 无 adapter 或无效 adapter 环境下，第二轴重复 axis / calibration mismatch；另测部分 motor 注入 | 重复 axis / calibration mismatch 先抛对应 `JointError`，不被 CreateMotor 失败覆盖；部分 motor 注入抛 `std::invalid_argument` |
| IT-SW-072 | manager 电机创建所有权链路 | RJoint 和闭链都通过 manager 创建，构造函数仅接收预创建 `Motor` | manager 是唯一电机创建入口；构造期空 motor 被拒绝，失败时不登记无效 joint、不污染 pending/active pair |
| IT-SW-073 | 闭链 master 创建失败恢复链路 | pending slave 已存在，先用合法配对但无效 motor 配置触发 `CreateMotor()` 失败，再修正 motor 配置重试 | 失败后 pending 仍保留且 active pair 未形成；重试后可创建 master 并绑定原 slave |
| IT-SW-074 | JSON type 精确匹配链路 | `AddJoint(json)` 使用错误大小写/空白 type 后，再使用合法 type 重试 | 非精确 type 不登记任何状态；合法 JSON 后可正常创建 |
| IT-SW-075 | JSON 未知 type 优先级链路 | 未注册 `type` 搭配非法 `config`，随后用合法 type/config 重试 | 第一次只报告未知 type 且不污染状态；第二次可正常创建关节 |
| IT-SW-076 | 闭链 master 构造失败恢复链路 | pending slave 已存在，第二轴使用会导致非法 `kt` 或 solver 标定失败的配置，再修正配置重试 | 构造失败后 pending 仍保留且 active pair 未形成；重试后可创建 master 并绑定原 slave |
| IT-SW-077 | JSON 重名优先级链路 | manager 已有 joint，再用同名 JSON 搭配缺失、非 object 或字段类型非法的 `config` 添加 | 只抛重名 `std::invalid_argument`，不解析 `config`，不污染 joint/pending/active pair |
| IT-SW-078 | pending slave 不支持校准接口链路 | manager 或直接工厂创建 pending slave 后调用 `GotoLimit/GotoZero/PosAsOffset/Calibrate` | 这些 helper 返回 NaN 或 false，不抛未配对异常；与 manager `Calibrate()` 后续 `Stop()` 触发未配对异常的路径分开验证 |
| IT-SW-079 | 闭链状态合成异常隔离链路 | 闭链完成配对并注册回调后制造状态合成 `JointException`，再恢复合法反馈 | pull 路径返回空，push 路径忽略本次回调；缓存不被错误反馈污染，恢复后仍可正常合成状态 |

### 6.2 硬件在环集成测试

硬件测试需在独立环境执行，必须有急停、限位和电流保护。

| ID | 场景 | 前置条件 | 验收 |
|----|------|----------|------|
| IT-HW-001 | Adapter 创建与总线连接 | 真实通信接口可用 | `CreateAdapter()` 成功，adapter `Ok()` |
| IT-HW-002 | RJoint 位置小幅运动 | 电机空载或安全夹具 | 位置到达误差在阈值内，无错误码 |
| IT-HW-003 | RJoint 速度小幅运动 | 限速限流已配置 | 速度方向与命令一致，可 Stop |
| IT-HW-004 | RJoint Stop | 任意运动中 | 指定停止模式生效 |
| IT-HW-005 | RJoint SetZero | 电机安全位置 | 零点重置成功，状态位置接近 0 |
| IT-HW-006 | 闭链 SetZero | 两电机在标定零位附近 | reset 成功，零位可解 |
| IT-HW-007 | 闭链 roll 小角度位置控制 | 机构无干涉 | roll 方向正确，pitch 漂移在阈值内 |
| IT-HW-008 | 闭链 pitch 小角度位置控制 | 机构无干涉 | pitch 方向正确，roll 漂移在阈值内 |
| IT-HW-009 | 闭链 Stop | 控制过程中 | 两电机同时停止 |
| IT-HW-010 | 状态回调 | 持续上报状态 | Push 状态频率和数值稳定 |
| IT-HW-011 | 通信断开 | 断开 adapter 或 bus | SDK 返回空状态或明确异常，不崩溃 |
| IT-HW-012 | 错误码传播 | 注入/触发底层错误 | `JointStatus.error` 能向上暴露 |
| IT-HW-013 | RJoint 电流小幅输出 | 限流已配置，电机处于安全姿态 | 电流方向与命令一致，可及时 Stop |
| IT-HW-014 | RJoint 扭矩小幅输出 | 限扭与夹具已配置 | 扭矩方向与命令一致，可及时 Stop |
| IT-HW-015 | 闭链 PVT 小幅控制 | 机构已 SetZero 且无干涉 | 双电机下发平稳，目标轴方向正确，无异常停机 |
| IT-HW-016 | 闭链速度小幅控制 | 限速限流已配置 | 双电机速度方向正确，近奇异区不会继续危险运动 |
| IT-HW-017 | 闭链电流/扭矩小幅控制 | 限流限扭与夹具已配置 | 双电机输出稳定，可 Stop，状态无新增错误码 |

硬件测试限制：

- 每个位置控制测试先使用小角度，例如 `0.02 rad`。
- 所有测试必须设置合理 `maxSpd`、`maxTor` 和电流限制。
- 闭链测试必须确认 calibration 与实际机构一致。
- 闭链控制前必须确认构造期当前双电机姿态已通过正解验证；正常上电流程不得依赖重新 `SetZero()` 建立可控状态。

## 7. 回归测试要求

以下行为属于必须长期保留的回归覆盖：

- `RJoint` 拒绝非正 `kt`。
- 闭链关节也必须拒绝任一电机非法 `kt`，不得在电流、位置或速度控制中绕过最大值保护。
- `TestAdapter` 只允许合法 zero reset 广播命令触发自动 ACK，ID、长度或命令字不匹配时不得回注 ACK。
- `JointException` 必须保持 `std::runtime_error` / `std::exception` 可捕获语义，不能只保留自定义错误码接口。
- `JointManager::GetStatus()` 不吞掉 `JointException`。
- `JointManager::PosControl()` 不吞掉 `JointException`。
- `MasterJoint::GetStatus()` Pull 模式会提交闭链反馈缓存。
- `MasterJoint::SetOnStatus()` Push 模式会提交闭链反馈缓存。
- `MasterJoint` 构造时必须读取两台电机当前位置并执行正解验证，失败时抛 `kDoubleCoupleJointSolveFailed`，不得进入可控状态。
- `MasterJoint` 构造后无需调用 `SetZero()` 即可控制；`SetZero()` 仅表示真实重置电机零点，失败时不得把目标位姿强制归零。
- 闭链 `GetStatus()` 在未 `SetZero()` 时必须按普通状态读取路径运行，不得抛零位未设置异常。
- `MasterJoint` 正向反馈在位置缺失、速度缺失、电流缺失时必须按分通道规则处理，不得伪造有效值。
- `MasterJoint` 状态合成过程中若出现 `JointException`，pull 路径必须返回空，push 路径必须忽略本次反馈，且不得提交错误反馈缓存。
- `MasterJoint` 主轴/从轴回调解绑必须彼此隔离，只有最后一个回调被清空时才卸载底层 motor 回调。
- `MasterJoint` 析构时必须卸载底层 motor 回调，防止状态上报访问已销毁对象。
- 闭链回调注册后只等待双电机状态齐全；不得再依赖 `SetZero()` 门禁来决定是否分发合成状态。
- 顶层 `JOINT_SDK_BUILD_TESTS=OFF` 时不生成 `mechanism_solver_test`。
- `DoubleCoupleJoint` 必须拒绝同电机配置，并且第二次配对失败不得破坏 pending slave 的恢复能力。
- `DoubleCoupleJoint` 首轴 pending 创建阶段不得创建电机或运行 solver；标定失败等需要 master 的校验必须延迟到第二轴配对阶段，失败后 pending 仍可恢复。
- 闭链配对预校验必须先于 manager 创建 master 电机执行；重复 axis 和 calibration mismatch 不得被 adapter/motor 创建失败覆盖。
- `DoubleCoupleJoint::CreateJoint()` 使用 motor 指针区分 slave/master 路径时，只有两个 motor 均为空才创建 slave，只有两个 motor 均非空才允许进入 master，部分 motor 注入必须失败。
- `DoubleCoupleJoint::MakePairKey()` 必须对 adapter、bus、motor id 都保持区分，顺序敏感且不同物理电机组合不得碰撞。
- 闭链配对要求两次 `AddJoint` 的电机顺序一致；第二次使用反转电机顺序时不得完成配对，必须形成独立 pending。
- `DoubleCoupleJoint::CalibrationEqual()` 在启用 `rotateDirection` 时必须比较方向向量，未启用时不得受残留值干扰。
- `DoubleCoupleJoint::CalibrationEqual()` 必须把单侧启用 `rotateDirection` 判定为不一致。
- `MotorInit` JSON 在同时存在 `ranges` 和 `model` 时必须优先解析 `ranges`。
- `MotorInit`、`MotorParams`、`axis`、`rotationOrder` 的 JSON 类型错误必须明确失败，不得静默生成默认 adapter、默认 id、默认 roll/pitch 或默认 rotation order。
- `JointManager::AddJoint(json)` 遇到未注册 `type` 时不得解析 `config`，未知 type 错误优先级必须稳定，且不得污染 joint/pending 状态。
- `JointManager::AddJoint(json)` 遇到重名时必须在读取或解析 `config` 前失败，重名错误优先级必须稳定。
- 闭链 master 构造阶段若因非法 `kt` 或 solver 标定失败抛异常，manager 必须保留原 pending slave，不得登记 active pair 或 master 名称，修正配置后仍可恢复配对。
- `JointManager<>::CreateMotor()` 的 auto、model、自定义 ranges 三个初始化分支必须都有覆盖，防止后续改动只验证 JSON variant 而漏掉实际 `Bus::GetMotor()` 调用。
- `JointManager<>::CreateMotor()` 遇到无效 bus 时必须明确失败，不得解引用空 bus 或返回无效 motor。
- `JointManager<>::CreateMotor()` 遇到无效 motor id 或 `GetMotor()` 返回空时必须明确失败，不得解引用空 motor 或登记无效关节。
- `JointManager<>::CreateMotor()` 遇到空 adapter 指针时必须明确失败，不得解引用空 adapter 或登记无效关节。
- `RJoint` 和 `MasterJoint` 构造函数只接受预创建电机，空 `Motor` 必须明确拒绝，不得通过空指针崩溃。
- `DefaultJointManager` 必须同时暴露模板 `AddJoint<T>(name, config)` 和 `AddJoint(json)`，新增 JSON overload 不得隐藏模板接口。
- JSON Schema 的 `initParam.oneOf` 必须保持互斥：同时包含 `ranges` 与 `model` 时 schema 校验失败，但 `FromJson()` 仍按 `ranges` 优先级宽松解析。
- JSON Schema 对 `Range` 和 `Point3` 的必填字段与 number 类型必须严格校验；对 `Range`、`Point3`、model init、ranges init 的额外字段当前保持宽松兼容。
- JSON Schema 与 `FromJson()` 对顶层 config、`MotorParams`、`CalibrationInput` 的额外字段当前保持宽松兼容；若改为严格拒绝，必须同步调整 schema、解析和测试预期。
- JSON Schema、`FromJson()` 和构造阶段对业务数值约束的职责必须固定；非法 limit、非正 `maxSpd/maxTor`、PVT range `min > max` 不得最终生成可控关节。
- 删除废弃错误码后，测试不得再引用 `kDoubleCoupleJointKinematicsNotImplemented`。
- 当前 public API 不得把 `kDoubleCoupleJointPending` 或 `kDoubleCoupleJointUnsupportedControl` 作为运行时异常抛出；如果未来启用这些错误码，必须先补明确语义和对应测试。
- `ankle.json` 和 `waist.json` 的 couple 示例标定参数必须长期保持可用于闭链关节配对与解算测试。
- 闭链 PVT、速度、力矩、电流映射到双电机后，任一电机超出对应范围时必须按统一比例缩放，保持双电机比例且均不超范围。
- RJoint PVT 保持 `kp/kd` 直接传递；闭链 PVT 的速度、扭矩和关节增益转换必须使用实际反馈位姿 Jacobian，并长期覆盖实际位置缓存回退、两轴动态量合帧及上位机力矩模式行为。
- 闭链位置控制必须长期覆盖 `speed/current` 参数处理：`speed <= 0` 使用轴最大速度，超速裁剪；`current <= 0` 下发 0，超限电流按 `maxTor/kt` 裁剪。
- `RJoint` 与闭链控制接口必须长期覆盖非默认 `feedback` 透传；速度控制还必须覆盖 `current <= 0` 与超限 current 的接口语义。
- `RJoint` PVT、位置、速度、电流、扭矩控制遇到 `NaN/Inf` 输入时必须 Stop 并抛 `std::invalid_argument`，不得向底层电机下发包含非有限值的控制命令；`Stop()` 的非有限电流必须归零。
- 闭链内部保护路径遇到非有限输入、不可执行目标或进入奇异区方向时，若已有双电机有效位置反馈，必须用 `PosControl` 维持两台电机当前位置；若缺少任一有效位置反馈，才退回 `FullBrake`。
- 闭链非有限控制输入必须抛 `std::invalid_argument`，不得提交目标缓存或向底层电机下发包含 NaN/Inf 的目标控制命令；公开 `Stop()` 的非有限电流仍必须归零。
- 闭链速度、扭矩、电流控制的零输入必须只下发零命令；无反馈时不改目标缓存，有有效双电机位置反馈时按反馈正解刷新缓存。
- 闭链五类控制（PVT/Pos/Spd/Tor/Cur）必须长期保持合帧下发：半帧不下发电机命令，第二条轴调用对两轴做联合映射后一次下发两台电机；两轴动态量必须同时保留，不得出现后一次调用覆盖前一次轴分量的行为。
- 持续成立的保护工况（限位保持、命令限幅、反馈与求解回退）必须长期按「进入/退出」边沿输出日志：每个关节的每个错误点各持有一个标志位，耦合关节的两条轴、两台电机分别持有独立标志位，条件持续期间不得逐周期重复告警；会抛异常的一次性故障点仍逐次输出。
- 闭链 near singular 状态下，远离奇异区方向的速度、扭矩、电流命令不得误触发内部保护；只有进入奇异区方向才触发位置保持或 `FullBrake` 兜底。
- 闭链完成配对但未 `SetZero()` 时，`Stop()` 仍必须可用并下发到两台电机，不得被零位状态检查阻断。
- pending `SlaveJoint::SetLimit()` 必须允许在未绑定 master 时调用，配对后从轴控制仍应使用该运行时限位。
- pending `SlaveJoint` 的 `GotoLimit/GotoZero/PosAsOffset/Calibrate` 必须直接返回 NaN 或 false，不得抛 `kDoubleCoupleJointNotPaired` 或触发配对/电机创建。
- manager 对 pending `SlaveJoint` 执行 `Calibrate()` 时，`Stop()` 转发会触发 `kDoubleCoupleJointNotPaired`，不得误写成普通 `std::runtime_error` 路径。
- manager 对 pending `SlaveJoint` 执行 `SetOnStatus()` 时必须透传 `kDoubleCoupleJointNotPaired`，不得完成回调注册。
- `JointManager::CreateAdapter()` 以 adapter name 作为幂等键，同名不同类型不得替换已有 adapter；非法类型失败后不得登记无效 adapter，合法配置仍可重试。
- `JointManager::CreateAdapter()` 遇到 `MakeAdapter()` 返回空指针时必须视为失败，不得登记空 adapter；`SetAdaptersStatusMaxLifeCycle()` 必须把边界值原样传给所有 adapter。
- `JointManager::AddJoint(json)` 遇到 envelope 字段类型错误、`config` 为 null/非 object 或 config 解析失败时必须抛异常，且不得登记 joint、pending pair 或 active pair。
- `JointManager::EnableJointMotorCanfd()` / `DisableJointMotorCanfd()` 必须同时作用于已创建 RJoint、已配对 MasterJoint 的两台电机和后续新创建电机；pending slave 不持有电机，切换时不得创建电机或抛异常。
- `JointManager::PosControl()` 和 `JointManager::Calibrate()` 是非事务批量接口：异常前已执行的关节不回滚，异常后的关节不继续执行。
- `FromJson()` 对额外字段保持宽松兼容，但 JSON Schema 对 auto `initParam` 的未知字段保持严格拒绝；两类测试不得混写成同一个预期。
- 安装后的 `joint_sdkConfig.cmake` 必须能在最小 consumer 工程中被 `find_package(joint_sdk CONFIG REQUIRED)` 正常消费。
- 安装后的 `joint_sdkConfig.cmake` 必须传播 `encos_driver 3.3.2` 依赖版本要求，不能退化为无版本 `find_dependency`。
- 安装后的 package 必须提供 `joint_sdkConfigVersion.cmake` 和可链接的 `joint_sdk::joint_sdk` 导入目标。
- `RJoint` 的 `SetZero()` / `Calibrate()` 失败路径、校准 `spd/cur/timeout` 参数透传以及部分反馈字段 NaN 传播必须长期保留覆盖。
- `RJoint` 与 `MasterJoint` 的位置辅助读取接口在缓存缺失或缓存位置为 NaN 时都必须回退到底层 `GetParameter<Position>()`。
- `JointManager::GetStatus(autoSendCommand=true)`、自定义 `lifeCycleDeduction` 和 `SetOnStatus(nullptr)` 必须把请求/生命周期扣减/注销行为准确透传到目标关节。
- `GetStatus(autoSendCommand=false)` 必须在 RJoint、闭链和 manager 链路上保持无副作用：无缓存时只返回空或忽略该关节，不下发反馈请求命令。
- `JOINT_SDK_BUILD_TESTS=OFF` 必须允许没有 GTest 的普通构建环境通过配置和构建。

## 8. 通过标准

### 8.1 单元测试通过标准

- 所有 GTest 用例通过。
- 新增接口或行为变更必须包含对应回归测试。
- 异常路径必须验证具体异常类型或 `JointError`。
- 对浮点解算结果使用合理误差，例如 `1e-3` 到 `2e-3`，不得使用精确相等。
- 状态回调测试必须覆盖完整双电机反馈和不完整反馈。

### 8.2 集成测试通过标准

- `cmake --build build` 成功。
- `ctest --test-dir build --output-on-failure` 全部通过。
- `JOINT_SDK_BUILD_TESTS=OFF` 普通构建成功且不生成测试目标。
- 软件集成测试不依赖真实硬件。
- 硬件集成测试在安全前置条件满足时通过，失败必须记录硬件状态、adapter、bus、motor id、命令参数和错误码。

## 9. 推荐 CI 流程

```bash
cmake -S . -B build -DJOINT_SDK_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure

cmake -S . -B build-no-tests -DJOINT_SDK_BUILD_TESTS=OFF
cmake --build build-no-tests
```

可选静态检查：

```bash
clang-format --dry-run --Werror include/*.h tests/*.cc tests/*.h
```

## 10. 测试报告模板

```text
测试版本:
Git commit:
测试环境:
构建命令:
测试命令:

单元测试:
- 总数:
- 通过:
- 失败:

软件集成测试:
- 总数:
- 通过:
- 失败:

硬件集成测试:
- 硬件型号:
- Adapter:
- Bus:
- Motor ID:
- 通过:
- 失败:

失败记录:
- 用例 ID:
- 现象:
- 日志:
- 下发命令:
- 电机反馈:
- 初步原因:
- 处理结论:
```
