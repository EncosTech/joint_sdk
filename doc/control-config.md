# control.json 配置说明

本文按当前 joint_sdk 的 `JointManager`、各关节 `Config::FromJson()` 和电机参数解析实现编写。
`control.json` 是调用方约定的文件名，SDK 接收的是 JSON 对象，不会自动寻找该文件。
配置描述硬件连接与关节参数；零阻尼标定数据另存于 `friction.json`。

## 1. 完整示例

以下是结构示例，型号、连接、限位和增益相关数值需按实际设备填写，不能直接作为实机标定值。

```json
{
  "adapters": [
    {
      "type": "EthercatIGH",
      "name": "0",
      "extend": {}
    }
  ],
  "joints": [
    {
      "name": "left_hip_pitch_joint",
      "type": "RJoint",
      "config": {
        "motorParams": {
          "adapterId": "0",
          "slaveId": 1,
          "busId": 0,
          "motorId": 1,
          "canfd": false,
          "initParam": {
            "model": "EC_A4310_P2"
          }
        },
        "limit": {"min": -1.0, "max": 1.0},
        "maxSpd": 2.0,
        "maxTor": 3.0,
        "reverse": false
      }
    }
  ]
}
```

读取方式：

```cpp
#include <fstream>
#include "JointManager.h"

std::ifstream input("control.json");
nlohmann::json config;
input >> config;
encos::DefaultJointManager manager(config);
```

构造管理器会创建适配器及关节，不是离线校验操作。离线检查使用本文最后的 Schema 方法。

## 2. 顶层结构和创建顺序

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `adapters` | 对象数组 | 否 | 按数组顺序创建适配器 |
| `joints` | 对象数组 | 否 | 全部适配器处理完后，按数组顺序创建关节 |

顶层必须为对象。缺省数组表示不创建对应对象；出现时必须为数组，不能为 `null`。
SDK 当前允许额外字段，但不代表这些字段会生效。

`DefaultJointManager` 支持 `RJoint`、`ContinuousJoint`、`DoubleCoupleJoint`；
自定义 `JointManager<...>` 只支持模板参数中列出的关节类型。

## 3. adapters：适配器

| 字段 | 类型 | 必填 / 默认 | 说明 |
|---|---|---|---|
| `type` | 字符串 | 必填 | 驱动插件类型，例如 `EthercatIGH`、`Fake`；取决于已安装插件 |
| `name` | 字符串 | 必填 | 接口名称，同时是适配器身份标识 |
| `extend` | 对象或 null | 缺省或 null 视为 `{}` | 传给驱动插件的扩展配置 |

`extend` 内部字段由插件定义，SDK 不统一规定其结构。不要将某个插件的配置直接套给其他插件。
只有支持相应扩展构造接口的插件会收到非空扩展配置。

同名适配器不新增建立顺序索引。已存在且扩展为空时保持幂等；
非空扩展交给驱动增量处理，其中 `buses` 可更新总线扩展，顶层公共参数不能重新应用。

## 4. joints：关节条目

每个条目都需要：

| 字段 | 类型 | 说明 |
|---|---|---|
| `name` | 字符串 | 关节唯一名称；同一个管理器内不能重复 |
| `type` | 字符串 | 三种关节类型之一，区分大小写 |
| `config` | 对象 | 与 type 对应的配置，不能为 null |

### RJoint

普通旋转关节，所有下列字段均必填：

| 字段 | 类型 | 单位 / 说明 |
|---|---|---|
| `motorParams` | 对象 | 单个电机配置，见下一节 |
| `limit` | Range | 关节侧位置范围，rad |
| `maxSpd` | 数值 | 最大关节速度，rad/s |
| `maxTor` | 数值 | 最大关节力矩，N·m |
| `reverse` | 布尔值 | 是否翻转关节与底层电机之间的方向 |

### ContinuousJoint

连续旋转关节，必填 `motorParams`、`maxSpd`、`maxTor`、`reverse`，含义同上。
不配置 `limit`：额外写入该字段不会为连续关节启用位置限位。

### DoubleCoupleJoint

闭链双电机机构的一个输出自由度，每对电机需要两个条目，分别表示 roll 和 pitch。

| 字段 | 必填 | 说明 |
|---|---|---|
| `motorParams1`、`motorParams2` | 是 | 两个电机配置，两份轴配置中的顺序必须一致 |
| `axis` | 是 | `roll` / `Roll` 或 `pitch` / `Pitch` |
| `calibration` | 是 | 机构几何标定参数，见第 6 节 |
| `limit` | 是 | 当前输出轴的关节位置范围，rad |
| `maxSpd`、`maxTor`、`reverse` | 是 | 当前输出轴的速度、力矩上限及方向 |
| `motorLimit1`、`motorLimit2` | 否 | 各电机相对位置限位，Range，rad；省略时使用电机协议位置范围 |

首个轴暂存为 Slave，第二个轴建立 Master 并完成配对。因此两个条目都创建完成后才能使用。
两轴名称必须不同，axis 必须不同，几何标定必须一致；不能让电机与自身配对。
同一电机在两份配置中应使用一致的适配器引用形式，不混用整数索引和字符串名称。
两份配置均提供同一电机限位时也需要保持一致；限位还要通过运行时协议范围检查。
可选限位应省略或提供完整 Range，不写 `null`。

不要把 `MasterJoint`、`SlaveJoint` 或旧名 `DoubleMotorJoint` 作为 JSON 的 type。
PVT 计算模式由运行时 API 设置，不是上述 Config 中的字段。

## 5. MotorParams：电机配置

| 字段 | 类型 | 必填 / 默认 | 说明 |
|---|---|---|---|
| `adapterId` | 字符串或整数 | 必填 | 字符串按适配器 name 引用；整数按建立顺序从 0 索引 |
| `busId` | 整数 | 必填 | 总线编号，按驱动插件解释 |
| `slaveId` | 整数 | 默认 0 | 从站编号，存储为 uint16；正常配置范围 0–65535 |
| `motorId` | 整数 | 必填 | 总线上的电机 ID，支持范围由协议/驱动决定 |
| `canfd` | 布尔值 | 默认 false | 是否对该电机启用 CAN FD；管理器全局设置优先 |
| `initParam` | 对象 | 必填 | 电机初始化方式，见下面三种形式 |

`adapterId: 0` 和 `adapterId: "0"` 不等价：前者查索引，后者查名称，不跨索引回退。
整数引用必须在 uint32 范围内，即 0–4294967295，还必须对应已经创建的适配器。

总线索引计算为 `(slaveId << 16) | busId`。推荐分别填写从站和总线号，
例如 slaveId=1、busId=0 对应 65536；旧配置将 busId 直接写为 65536 并省略 slaveId
也保持这个结果。不要将已合成的 busId 再与不匹配的 slaveId 混合。

### 自动初始化

```json
{}
```

通过驱动自动获取/识别电机，不表示所有插件或设备都支持自动识别。

### 按型号初始化

```json
{"model": "EC_A4310_P2"}
```

型号枚举来自当前安装驱动的 `GetAllMotorModelStrings()`，已同步到 Schema 的
`definitions.MotorParams.properties.initParam`。升级驱动后应重新导出枚举。

### 自定义协议范围

```json
{
  "ranges": {
    "kp": {"min": 0.0, "max": 500.0},
    "kd": {"min": 0.0, "max": 5.0},
    "position": {"min": -12.5, "max": 12.5},
    "speed": {"min": -18.0, "max": 18.0},
    "torque": {"min": -30.0, "max": 30.0},
    "current": {"min": -30.0, "max": 30.0},
    "kt": 0.5
  }
}
```

全部七项必填。位置、速度、力矩、电流单位分别为 rad、rad/s、N·m、A；
`kt` 为力矩系数 N·m/A，关节构造要求其有限且大于 0。
这些是底层电机协议范围，不是关节软限位。示例数值仅用于说明结构。
Range 均为含 `min`、`max` 两个数值的对象，不能用数组代替。

三种初始化方式应择一填写。解析器实际优先取 `ranges`，其次取 `model`，否则自动初始化；
Schema 使用 oneOf 规范输入，可能拒绝解析器容忍的混合或多余字段。

## 6. calibration：闭链几何标定

| 字段 | 必填 | 说明 |
|---|---|---|
| `leftJointCenter`、`rightJointCenter` | 是 | 左右电机关节中心 |
| `leftPlanarEndpoint`、`rightPlanarEndpoint` | 是 | 左右平面侧端点 |
| `rollCenter`、`pitchCenter` | 是 | roll、pitch 旋转中心 |
| `leftSpatialEndpoint`、`rightSpatialEndpoint` | 是 | 左右空间侧端点 |
| `rotationOrder` | 是 | 旋转顺序 |
| `rotateDirection` | 否 | 可选方向向量；存在时启用方向约束 |

所有点和方向向量均使用 `{"x": 数值, "y": 数值, "z": 数值}`。
几何点使用同一坐标系和一致的长度单位，遵守实际机构及求解器约定；不能随意混合 CAD 单位。

`rotationOrder` 支持：

- `PitchThenRoll`、`pitch_then_roll` 或整数 `0`。
- `RollThenPitch`、`roll_then_pitch` 或整数 `1`。

几何样例可参考 [踝关节](../external/couple_joint_solver/examples/ankle.json)、
[腰部](../external/couple_joint_solver/examples/waist.json)。只将其中的 calibration
部分映射到本 SDK 配置，并按真实机构设置两轴限位；不要将示例几何直接用于其他机构。
结构校验通过不保证闭链标定可解，几何合法性和两轴配对还需运行时校验。

## 7. 与临时标定工具的区别

`tools/joint_sine_collector` 和 `tools/joint_zero_damping_test` 使用自己的旧配置读取层，
目前仅提取适配器 `type`、`name`，**不会传递 `extend`**。依赖扩展配置的硬件不能仅凭
`DefaultJointManager` 支持 extend 就认为这两个工具同样支持。

这两个工具只支持 RJoint；`--all` 选择其中的 RJoint，显式选择其他类型会报错。
工具读取时要求 `adapters`、`joints` 都存在，区别于 SDK 构造函数允许缺省。
`friction.json` 的 `joints.<名称>.control` 是补偿标定结果，不属于 control.json。

## 8. JSON Schema 使用与边界

[joint_schema.json](joint_schema.json) 保留按名称索引的 Schema 集合：

| 键 | 校验对象 |
|---|---|
| `RJoint` | 单个 RJoint 的 config 对象 |
| `ContinuousJoint` | 单个 ContinuousJoint 的 config 对象 |
| `DoubleCoupleJoint` | 单个 DoubleCoupleJoint 的 config 对象 |
| `ControlConfig` | 完整 control.json 对象（新增） |

必须提取对应键的值作为独立 Schema 文档，不能直接把集合本身传给校验器。
各入口使用 Draft 7；关节入口由当前代码的 `GetJointSchema()` 导出，
ControlConfig 合并共享 definitions 并补充适配器、关节条目和顶层结构。

在已安装 Python `jsonschema` 的环境中，从仓库根目录执行：

```python
import json
from pathlib import Path
from jsonschema import Draft7Validator

schemas = json.loads(Path("doc/joint_schema.json").read_text())
schema = schemas["ControlConfig"]
Draft7Validator.check_schema(schema)
config = json.loads(Path("control.json").read_text())
Draft7Validator(schema).validate(config)
```

Schema 主要检查类型、必填字段及枚举，保留 SDK 的额外字段宽松策略。
它不检查关节名称唯一、适配器引用是否存在、插件可用性、电机反馈、min/max 关系、
物理参数的全部范围约束、闭链配对及标定是否可解。通过 Schema 不等于可以安全启动硬件。
