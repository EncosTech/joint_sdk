# joint_sdk

`joint_sdk` 是基于 EncosMotorDriver 的关节控制 SDK，header-only C++17 模板库。
提供 **RJoint**（带位置限位的直连旋转关节）、**ContinuousJoint**（可无限旋转的
直连关节）和 **DoubleCoupleJoint**（双电机关节）三种抽象，统一封装关节控制与状态反馈。

## 目录

- [安装依赖](#安装依赖)
- [构建](#构建)
- [安装](#安装)
- [测试](#测试)
- [Motor 文件日志](#motor-文件日志)
- [关节保护与告警](#关节保护与告警)
- [ContinuousJoint 连续关节](#continuousjoint-连续关节)
- [RJoint 零阻尼控制](#rjoint-零阻尼控制)
- [JSON 配置创建](#json-配置创建)
- [WASM 绑定](#wasm-绑定)
  - [构建 WASM](#构建-wasm)
  - [运行 WASM 测试](#运行-wasm-测试)
  - [JavaScript API 示例](#javascript-api-示例)
- [依赖](#依赖)
- [目录结构](#目录结构)

## 安装依赖

以下命令以 Ubuntu 24.04 为例。基础构建依赖由 apt 安装；Boost.Preprocessor 和
`couple_joint_solver` 随 Git 子模块获取：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git ninja-build pkg-config \
  libeigen3-dev nlohmann-json3-dev \
  libcap-dev libcap2-bin libzstd-dev libspdlog-dev libfmt-dev

git clone --recurse-submodules \
  https://git.in.encos.cn/EncosRobotics-Stack/joint_sdk.git
cd joint_sdk
git submodule update --init --recursive
```

原生 joint_sdk 通过 CMake 查找已安装的 `encos_driver >= 3.3.2`。如果机器上尚未安装，
先从源码安装到与 joint_sdk 相同的前缀：

```bash
git clone --recurse-submodules \
  https://git.in.encos.cn/bismarckkk/motor_driver.git "$HOME/src/motor_driver"
cmake -S "$HOME/src/motor_driver" -B "$HOME/src/motor_driver/build-install" \
  -DENCOS_ENABLE_INSTALL=ON \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local"
cmake --build "$HOME/src/motor_driver/build-install" -j
cmake --install "$HOME/src/motor_driver/build-install"
```

如需构建 C++ 测试和执行代码检查，再安装：

```bash
sudo apt install -y libgtest-dev clang-format clang-tidy
```

## 构建

```bash
git submodule update --init --recursive
cmake -B build \
  -DJOINT_SDK_BUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$HOME/.local"
cmake --build build
```

## 安装

基础关节接口和零阻尼包装层均作为公共头文件安装：

```bash
cmake -B build-install \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local" \
  -DCMAKE_PREFIX_PATH="$HOME/.local"
cmake --build build-install -j
cmake --install build-install
```

安装后的 CMake 包提供基础关节接口：

```cmake
find_package(joint_sdk CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE joint_sdk::joint_sdk)
```

`robot_identity` 独立拥有 Pinocchio-based `Robot` runtime、硬件采集和参数辨识流程；本 SDK
只提供基础关节控制接口及零阻尼补偿包装层。

## 测试

```bash
./build/joint_sdk_tests
# 或
cd build && ctest
```

## Motor 文件日志

原生 C++ 的 `JointManager` 可以统一开启底层 Motor 的状态和控制命令日志：

```cpp
encos::DefaultJointManager manager;
// 创建 Adapter 和关节后：
manager.EnableLog("/tmp/joint-logs");
// ...执行控制与状态读取...
manager.DisableLog();  // 刷新并关闭全部日志
```

单 Adapter Manager 使用 `s<slaveId>b<busId>m<motorId>` 作为文件基础名；多 Adapter
Manager 增加 `<adapterName>_` 前缀。driver 会为每台 Motor 生成对应的
`_status.csv.zstd` 与 `_command.csv.zstd` 文件。日志启用期间新增的关节电机也会自动记录，
可通过 `manager.IsLogged()` 查询整体状态。Adapter 名中的路径分隔符等非文件名安全字符会
使用 `%HH` 转义。该文件日志能力不适用于 WASM。

## 关节保护与告警

控制期间必须连续调用控制命令，建议控制频率 **≥500 Hz**（周期 ≤2 ms）。
耦合关节的**所有控制模式**（`PVTControl`、`PosControl`、`SpdControl`、`TorControl`、
`CurControl`）都按合帧下发：每个周期应完成主、从两轴各一次调用，第一条调用只缓存该轴
目标，第二条调用把两轴目标一起做联合映射后一次下发两台电机。半帧期间不产生任何电机
命令；反馈刷新与限位、奇异方向保护在连续调用中持续生效，单次调用不能持续执行这些判断。
结束控制时显式调用 `Stop`；停止会取消尚未发出的半帧并清空两轴目标缓存，后续控制从新批次开始。

`JointManager` 创建的所有关节（`RJoint`、`ContinuousJoint`、耦合主从轴）共用
`manager.GetLogger()` 记录保护日志，包含关节名称、控制模式和原因；数值限幅还记录
请求值和实际值。可用 `manager.SetLogger(logger)` 替换已有及后续关节的保护日志器，
用 `manager.SetLogLevel(encos::LogLevel::Warn)` 设置输出级别。默认日志器在 SDK 内共享。
这类日志与 `EnableLog()` 的电机 CSV 文件日志独立。

保护日志按**进入/退出边沿**输出：限位保持、命令限幅、反馈与求解回退这类条件会在控制
循环中持续成立，因此每个关节的每个错误点各持有一个标志位——条件成立时输出一条
`state=enter` 的 warning，条件解除时输出一条 `state=exit` 的 info，持续期间不再复述，
高频控制循环下也不会淹没其它日志。耦合关节的两条轴、两台电机各自持有独立的标志位，
日志中用 `scope` 字段区分。会抛出异常的一次性故障（非法输入、不支持的操作、逆解失败等）
仍逐次输出 warning，因为异常已经终止了该条控制流。

保护位控与普通位控复用位置下发逻辑：`RJoint` 调用自身 `PosControl`；耦合关节复用
双电机位置下发函数，避免重新进入双轴合帧与死点检测。电机保持目标限制在软件限位
两侧各内缩 2° 的区间；若区间为空或无法保持，则回退 `FullBrake` 并告警。
耦合电机保持目标限幅后会重新验证闭链位姿及两个虚拟关节限位，验证失败时也回退
`FullBrake`。旋转关节在控制前读取位置反馈抛出异常时，先尝试 `FullBrake`，再传播原始异常。
耦合解算的相对零位电机角度在减去初始角后归一化到 `[-π, π]`，防止跨圈目标。
原始电机反馈用于保持时只限幅、不取模，避免超出 π 后反向跳圈。
连续旋转关节的累计反馈位置不做此归一化。

耦合 `PVTControl` 的第六个参数为枚举类 `encos::PVTControlMode`；省略时使用模式 0。

| 值 | 枚举成员 | 控制方式 |
| --- | --- | --- |
| 0 | `MotorPVT` | 将关节目标及增益映射到双电机，直接下发电机 PVT |
| 1 | `HostTorque` | 上位机计算 `Kp * 位置误差 + Kd * 速度误差 + 前馈扭矩`，按关节 `maxTor` 限幅后映射并下发电机 `TorControl` |
| 2 | `HostKpMotorKd` | 上位机计算 `Kp * 位置误差 + 前馈扭矩`；映射后的 Kd 与速度目标下发电机 PVT，本地执行速度闭环，电机 `Kp=0` |

模式 1、2 通过当前反馈位姿的 `Jᵀ` 映射力矩，并按电机扭矩范围统一缩放。
模式 2 的 `Kd` 使用 `diag(Jᵀ Kd J)` 对角近似，SDK 不重复计算速度误差；
这里 `J` 满足 `关节速度 = J * 电机速度`。
模式 0、2 将正的映射 `Kd` 下限设为 0.1，再受电机协议范围限制；显式零阻尼仍为零。
同一合帧批次的两轴必须选择相同的上位机模式才生效；模式不一致时回退普通电机 PVT。
`SpdControl`/`TorControl`/`CurControl` 三种动态模式也按批次下发，同一批次的两条调用必须
使用同一种模式；模式不一致或中途切换到 `PVTControl`/`PosControl` 时，挂起的半帧被丢弃，
本次调用作为新批次的第一条处理。
模式 1 需要有效位置和速度反馈，模式 2 需要有效位置反馈；反馈不可用时沿用普通 PVT 回退。

```cpp
roll->PVTControl(kp_roll, kd_roll, pos_roll, spd_roll, tor_roll,
                 encos::PVTControlMode::HostTorque);
pitch->PVTControl(kp_pitch, kd_pitch, pos_pitch, spd_pitch, tor_pitch,
                  encos::PVTControlMode::HostTorque);
```

TypeScript 从包入口导入 `PVTControlMode`，通过 `pvtControl` 第六个参数传入；
导出的 `PVTControlCommand.torqueMode` 类型也同步为该枚举。
这是接口变更：旧 `false` 迁移为 `MotorPVT`（0），旧 `true` 迁移为 `HostKpMotorKd`（2），
不能把旧 `true` 转成数值 1；WASM C ABI 的整型参数也按新的 0/1/2 语义解释。
非法枚举值会被拒绝。普通 `RJoint` 的三种合法模式均沿用自身 PVT；`ContinuousJoint` 仍不支持 PVT。

耦合 `PosControl` 合帧保存两轴的位置、速度及电流约束：按整批位移和两轴速度上限计算共同
运动时间，再映射电机速度；两个电机分别采用两次调用中较小的电流上限。
由于电机接收的是独立速度上限，SDK 还按目标位姿的速度映射限制两轴最坏组合速度，
实际下发速度可能比共同运动时间计算值更低。

耦合 `SpdControl`/`TorControl` 合帧后在最后有效位姿一次性执行双轴逆速度和逆力矩映射，
两轴动态量同时保留（不会出现后一次调用覆盖前一次轴分量的情况）；`CurControl` 按当前传动
关系把两轴电流换算为被控轴扭矩，按各轴 `maxTor` 限幅后再按力矩比对双电机做电流分配，
并按电机电流范围统一缩放。两种调用的电机电流上限取较小值，`feedback` 取第二条调用。
未在本批次命令过的轴沿用该轴上次命令值（从未命令过则为 0），`Stop`、`SetZero` 以及保护拦截
（位姿不可执行或朝奇异区运动）会清空这些缓存。两轴都朝越限方向或朝奇异区运动时，拦截轴自身的命令置零，另一轴仍按本批次目标参与映射；
电机级越限则两台电机一起置零。

## ContinuousJoint 连续关节

`ContinuousJoint` 直接继承 `BaseJoint`，配置中不包含 `limit`，不会执行任何位置限位保护。
它支持速度、电流、扭矩和停止控制，并保留最大速度、最大电流与最大扭矩的幅值限制；
`PosControl`、`PVTControl`、`SetLimit` 以及限位/校准相关接口会抛出
`kContinuousJointUnsupportedOperation`。

连续关节会使用 `Motor::GetPVTRanges().position` 的跨度展开反馈位置。当编码器反馈从上限
跳到下限或反向跳变时，`GetStatus()` 与状态回调返回的 `pos` 会保持连续；成功执行
`SetZero()` 后重新开始累计。

```cpp
encos::ContinuousJoint::Config config{};
config.motorParams = motor_params;
config.maxSpd = 2.0f;
config.maxTor = 3.0f;
config.reverse = false;
auto joint = manager.AddJoint<encos::ContinuousJoint>("wheel", config);
```

也可以通过默认 Manager 的 JSON `AddJoint` 创建，`type` 使用 `"ContinuousJoint"`，
`config` 字段与上例一致且无需 `limit`。

> `RJoint`、`ContinuousJoint`、双电机关节以及 Manager 的 `GetStatus(autoSend=true)`
> 都可能在缓存缺失时发送零目标 PVT 反馈命令。关节运动过程中不得启用 `autoSend`，
> 否则可能改变当前运动状态。

## RJoint 零阻尼控制

`ZeroDampingRJoint` 是与 `BaseJoint` 接口一致的 RJoint 包装层，内部以 1 kHz
重发最近一条控制指令，并使用带因果速度低通和速度滞回的对称 Stribeck 模型。
当前版本只支持 `RJoint`，实时补偿包含摩擦和关节侧等效惯量；重力参数不参与实时控制。惯量
辨识值保持为物理参数，实时控制只使用标定文件中的保守比例并对惯量项单独限幅，避免完整
`J * acceleration` 在滤波延迟下造成过强加速度反馈。

- PVT、扭矩和电流模式叠加摩擦与惯量前馈；扭矩模式在底层按扭矩系数换算为电流控制，位置和
  速度模式只按原指令重发。
- 500 ms 没有新指令后切换到电流模式，只发送摩擦与惯量补偿。
- 超时后不再使用历史指令判断静摩擦方向，但滤波后的实测速度超过启动阈值时会重新启动补偿。
- 速度补偿尚未启动时，累计位置变化超过标定的位置意图死区会按移动方向输出 80% 的辨识静摩擦；
  单个编码器计数不触发，连续 50 ms 没有新位移或速度达到启动阈值时会关闭该低速支路。
- `Stop` 会锁存，直到收到下一条控制指令。
- `GetStatus()` 和状态回调中的 `tor` 已扣除最近实际保留的摩擦与惯量补偿；接口不增加电流字段。

```cpp
#include "RJoint/ZeroDampingRJoint.h"

auto control = nlohmann::json::parse(friction_json).at("joints").at(joint_name).at("control");
auto zero_damping = std::make_shared<encos::ZeroDampingRJoint>(
    raw_rjoint, encos::StribeckConfig::FromJson(control));

zero_damping->TorControl(0.0f);
```

包装层是软实时控制，不提供操作系统级硬实时保证。销毁包装层时会停止工作线程并尝试对底层
RJoint 执行 `FullBrake`。

该包装层会真实启动电机并持续发送摩擦与惯量补偿力矩。使用前应确保急停可用、机械环境允许
关节自由运动，并确认标定文件中的最大补偿力矩适合对应关节。补偿统一通过电流模式下发。

## JSON 配置创建

`JointManager<JointTypes...>` 提供一个接收 JSON 的构造函数：先按 `adapters` 数组创建
适配器，再按 `joints` 数组创建关节，两个数组都按给定顺序处理。默认构造保持不变。

```cpp
const nlohmann::json config = {
    {"adapters", {{{"type", "EthercatIGH"}, {"name", "0"}}}},
    {"joints",
     {{{"name", "left_shoulder_pitch_joint"},
       {"type", "RJoint"},
       {"config",
        {{"reverse", true},
         {"motorParams",
          // 整数按建立顺序索引引用适配器；字符串则按适配器名称引用
          {{"adapterId", 0},
           {"busId", 65536},
           {"motorId", 1},
           {"initParam", {{"model", "EC_A4310_P2"}}}}},
         {"limit", {{"min", -1.57}, {"max", 2.57}}},
         {"maxSpd", 9.33},
         {"maxTor", 36}}}}}},
};

encos::DefaultJointManager manager(config);  // 适配器与关节一次性建立
```

- `adapters` 与 `joints` 均可缺省；出现但类型不是数组时抛 `std::invalid_argument`。
- `motorParams.adapterId` 的**取值类型决定引用方式**：整数按适配器建立顺序索引解析，
  字符串按适配器 `name` 解析，不做跨索引回退。索引从 `0` 开始，只统计实际创建的适配器
  （同名重复创建不新增索引）。所以上例的 `0` 与 `"0"`（假如同名适配器存在）会指向不同
  的适配器；同一台物理电机的多处配置应使用同一种引用形式。
- 单个适配器/关节的失败语义与 `CreateAdapter()`、`AddJoint(json)` 一致，异常直接向上传播。

## WASM 绑定

`joint_sdk` 提供 Emscripten WASM 构建，并附带 TypeScript 包 `@encos/joint-sdk`，
可在 Node.js 和浏览器中使用。

### 构建 WASM

脚本会自动 `source "$HOME/emsdk/emsdk_env.sh"`，因此只需要确保 Emscripten SDK 安装在该路径：

```bash
./scripts/build-wasm.sh
```

产物位于 `build-wasm/`：

- `JointSDKWasm.js`
- `JointSDKWasm.wasm`

### 运行 WASM 测试

```bash
./scripts/test-wasm.sh
```

该脚本会依次完成 WASM 构建、TypeScript 包构建、Node（Vitest）测试和浏览器（Playwright）测试。

也可以手动运行：

```bash
cd npm
pnpm install
pnpm run build
pnpm run test:unit
pnpm run test:browser   # 需要先启动 dev server
```

### JavaScript API 示例

```typescript
import { createJointRuntime, MotorStopMode } from '@encos/joint-sdk'

const runtime = await createJointRuntime()
console.log(runtime.version())

const manager = runtime.createJointManager({
  adapterType: 'Fake',
  adapterName: 'fake_adapter',
})

// 也可以先建空 manager，再按 JSON 配置创建适配器
// const manager = runtime.createJointManager({})
// await manager.createAdapter({ type: 'Fake', name: 'fake_adapter' })

const joint = manager.addJoint({
  type: 'RJoint',
  name: 'joint_0',
  config: {
    motorParams: {
      adapterId: 'fake_adapter',
      // slaveId 与 busId 合成底层总线索引 slaveId << 16 | busId，slaveId 缺省为 0
      slaveId: 0,
      busId: 0,
      motorId: 1,
      // 逐电机启用 CAN FD；缺省 false，全局开关被显式调用后以全局开关为准
      canfd: false,
      initParam: { model: 'EC_A4310_P2' },
    },
    limit: { min: -1, max: 1 },
    maxSpd: 2,
    maxTor: 3,
    reverse: false,
  },
})

await joint.posControl(0.25, 1, 2, 0)
const status = joint.getStatus()
console.log(status)

const command = manager.fake?.getLastCommand()
console.log(command)

await joint.stop(MotorStopMode.FullBrake)
manager.dispose()
runtime.dispose()
```

### Fake 调试工具

在 `adapterType: 'Fake'` 下，可通过 `manager.fake` 注入反馈、读取电机快照与解码命令记录：

```typescript
import { createJointRuntime, FakeCommandKind } from '@encos/joint-sdk'

const runtime = await createJointRuntime()
const manager = runtime.createJointManager({ adapterType: 'Fake', adapterName: 'fake' })

// 注入电机反馈
manager.fake!.injectFeedback(1, 1, {
  position: 0.5,
  speed: 0.1,
  current: 0.2,
  motorTemperature: 30,
  mosTemperature: 40,
})

// 读取最近的命令
const command = manager.fake!.getLastCommand()
console.log(command?.kind === FakeCommandKind.PosControl)

manager.dispose()
runtime.dispose()
```

## 依赖

- C++17
- encos_driver >= 3.3.2（原生构建必须预先安装；使用 main/v3 的 `motor/` 头文件）
- nlohmann/json（Ubuntu 包：`nlohmann-json3-dev`）
- Boost.Preprocessor（`external/boost_preprocessor/` 子模块）
- couple_joint_solver（`external/couple_joint_solver/` 子模块）
- GoogleTest、clang-format、clang-tidy（仅测试和代码检查需要）

WASM 构建额外依赖：

- Emscripten SDK
- Node.js 20+
- pnpm

## 目录结构

```
include/          # C++ 头文件
src/wasm/         # Emscripten WASM 运行时与 C API
npm/              # TypeScript 包 @encos/joint-sdk
scripts/          # 构建/测试脚本
tests/            # 测试代码
  wasm-node/      # Node.js WASM 测试
  wasm-browser/   # 浏览器 WASM 测试
.gitea/workflows/ # Gitea CI 工作流
```

## 许可证与发布

项目自有源码采用 [MIT License](LICENSE)，第三方子模块和依赖保留原许可证。
DEB 包包含完整 LICENSE 和 Debian copyright；npm 包声明 `license: MIT` 并包含完整 LICENSE。
构建产物下载与 GitHub 发布配置见 [发布说明](doc/github-release.md)。
