# C++ 求解器使用指南

本文面向直接集成 `MechanismSolver` 的 C++ 调用者，重点说明：

- 如何准备标定输入；
- 如何按顺序调用位置、速度和力矩相关接口；
- 如何判断返回状态是否可用；
- 如何读取当前状态、几何和传动信息。

机构理论、符号定义和推导过程请参考 [mechanism.md](mechanism.md)。

## 1. 适用范围

本文只讨论 C++ 层的求解器接口：

- 头文件：`src/mechanism_solver.h`
- 主要类：`couple_joint::MechanismSolver`

本文不展开以下内容：

- 闭链约束和雅可比的理论推导；
- WASM 绑定和前端调用方式；
- Three.js 可视化层的使用方法。

## 2. 核心类型概览

### 2.1 `RotationOrder`

用于指定中间连杆的旋转顺序：

- `couple_joint::RotationOrder::PitchThenRoll`
- `couple_joint::RotationOrder::RollThenPitch`

这个字段必须在标定输入中显式设置。

### 2.2 `Point3`

用于表达三维点或三维向量：

```cpp
couple_joint::Point3 p{1.0, 2.0, 3.0};
```

### 2.3 `CalibrationInput`

用于构造求解器的标定输入。它对应 `docs/mechanism.md` 中的几何符号：

- `left_joint_center` / `right_joint_center` 对应 `O1 / O2`
- `left_planar_endpoint` / `right_planar_endpoint` 对应零位 `P1 / P2`
- `left_spatial_endpoint` / `right_spatial_endpoint` 对应零位 `S1 / S2`
- `roll_center` / `pitch_center` 对应 `O_roll / O_pitch`
- `rotate_direction` 可选，用于直接指定输入平面法向 / 转轴方向

### 2.4 `MechanismSolver`

这是整个 C++ 求解器的入口。它内部维护当前机构状态，并提供：

- 位置逆解：`SolveInverse()`
- 位置正解：`SolveForward()`
- 速度映射：`SolveForwardVelocity()` / `SolveInverseVelocity()`
- 力矩映射：`SolveForwardTorque()` / `SolveInverseTorque()`
- 状态与几何读取：`GetState()` / `GetGeometry()` / `GetTransmission()`

### 2.5 `SolveResult` 与 `VelocityTorqueResult`

这两个结果类型分别用于：

- 位置求解结果；
- 速度或力矩映射结果。

调用方通常首先检查：

- `status`
- `transmission`

对于位置正解，还常常需要查看：

- `residual`
- `iterations`

### 2.6 `SolveStatus`

最常见的状态处理方式是：

- `Success`：正常使用
- `NearSingular`：结果通常仍可用，但应提示风险
- 其他状态：按失败处理

### 2.7 `CalibrationStatus`

用于表达构造阶段的标定结果。典型使用方式是：

- 构造 `MechanismSolver`
- 读取 `GetCalibrationStatus()`
- 使用 `MechanismSolver::StatusMessage(...)` 获取可读消息

## 3. 输入约定与标定数据准备

### 3.1 基本约定

构造 `MechanismSolver` 之前，必须先准备一组完整的 `CalibrationInput`。其中：

- `O1 / O2` 是两个输入转动副中心；
- `P1 / P2` 是零位时两个平面输入连杆末端；
- `S1 / S2` 是零位时中间连杆与空间连接杆的连接点；
- `O_roll` 和 `O_pitch` 可以相同，也可以不同；
- `rotation_order` 必须设置为支持的枚举值。

### 3.2 `rotate_direction` 的使用建议

有两种标定输入方式：

1. 不设置 `rotate_direction`

这时求解器会根据 `O1`、`O2`、`P1`、`P2` 自动推断输入平面法向，默认方向定义为：

```text
(O1 - P1) x (O1 - O2)
```

也就是以左侧输入连杆 `O1 -> P1` 和基轴 `O1 -> O2` 按右手定则确定默认旋转轴方向。此时要求这四个点能够确定平面，且满足共面约束。

2. 设置 `rotate_direction`，并将 `use_rotate_direction` 置为 `true`

这时求解器直接使用你提供的方向作为输入平面的法向 / 转轴方向。适合外部已经知道法向方向，或者标定点来自带姿态信息的系统。

无论采用哪种方式，roll 的正方向都与电机旋转轴正方向一致；pitch 的正方向定义为 `O1 -> O2` 轴线方向。

### 3.3 构造后必须先检查标定状态

`MechanismSolver` 的构造函数会立即执行标定和零位校验，因此构造后第一件事应当是：

```cpp
couple_joint::MechanismSolver solver(input);
if (solver.GetCalibrationStatus() != couple_joint::CalibrationStatus::Success) {
  std::cerr << couple_joint::MechanismSolver::StatusMessage(
      solver.GetCalibrationStatus()) << "\n";
  return;
}
```

如果标定状态不是 `CalibrationStatus::Success`，不要继续调用位置、速度或力矩相关接口。

## 4. 标准调用流程

推荐按下面顺序使用求解器：

1. 填充 `CalibrationInput`
2. 构造 `MechanismSolver solver(input)`
3. 调用 `GetCalibrationStatus()` 并结合 `MechanismSolver::StatusMessage(...)` 检查标定结果
4. 根据业务需求进行一次有效的位置求解
5. 读取 `GetState()`、`GetGeometry()` 或 `GetTransmission()`
6. 在当前有效状态上执行速度映射或力矩映射
7. 根据 `SolveStatus` 做失败处理或近奇异预警

这里有两个很重要的使用约束：

- 速度和力矩接口依赖“当前内部状态处”的传动信息，因此通常应先进行一次成功的位置求解；
- 不可达、退化、未收敛等失败结果不应继续作为有效姿态使用。

内部状态更新语义建议一并记住：

- `SolveInverse()` 和 `SolveForward()` 在返回 `Success` 或 `NearSingular` 时，会把内部状态更新到新姿态；
- 位置求解失败时，不应假设内部状态已经切换到本次目标姿态；
- 如果调用方需要记录“求解前状态”，应在调用位置接口前先取一次 `GetState()`。

## 5. 读取求解器信息

### 5.1 `GetParameters()`

用于读取由标定推导出的规范几何参数，例如：

- `base_distance`
- `left_planar_length`
- `right_planar_length`
- `left_spatial_length`
- `right_spatial_length`

适用场景：

- 调试标定结果；
- 记录机构规范参数；
- 与外部配置进行比对。

### 5.2 `GetState()`

返回当前内部状态：

```cpp
const couple_joint::MechanismState state = solver.GetState();
```

包含：

- `theta1`
- `theta2`
- `roll`
- `pitch`

适用场景：

- 读取最近一次有效位置求解后的状态；
- 在日志或 UI 中显示当前机构姿态。

### 5.3 `GetGeometry()`

返回当前几何信息，包括：

- 当前 `O1` / `O2`
- 当前 `P1(theta1)` / `P2(theta2)`
- 当前 `S1(q)` / `S2(q)`
- 当前 `roll_axis` / `pitch_axis`

适用场景：

- 可视化显示；
- 调试姿态变换；
- 检查旋转中心和旋转轴是否符合预期。

### 5.4 `GetTransmission()`

返回当前状态处的微分传动信息：

- `fu`，对应 `F_theta`
- `fq`，对应 `F_q`
- `velocity_jacobian`，对应 `J_{q theta}`
- `reduction`，对应 `J_{theta q}`

以及：

- 奇异值；
- 条件数；
- 奇异与近奇异标志。

适用场景：

- 分析当前姿态下的传动特性；
- 判断是否接近奇异；
- 在调试时查看速度 / 力矩映射依据。

## 6. 位置求解

### 6.1 位置逆解：`SolveInverse(roll, pitch)`

当你已知目标输出姿态 `q = [roll, pitch]^T`，希望得到驱动角 `theta = [theta1, theta2]^T` 时，调用：

```cpp
const couple_joint::SolveResult result = solver.SolveInverse(roll, pitch);
```

典型处理方式：

```cpp
if (result.status != couple_joint::SolveStatus::Success &&
    result.status != couple_joint::SolveStatus::NearSingular) {
  std::cerr << "inverse solve failed\n";
  return;
}
```

说明：

- `Success` 表示正常求解成功；
- `NearSingular` 表示结果可用，但当前位置接近奇异；
- `UnreachablePose` 表示目标姿态不可达；
- `DegenerateSideEquation` 表示几何退化。

在 `Success` 或 `NearSingular` 时，求解器内部状态会更新，此时可以继续调用：

- `GetState()`
- `GetGeometry()`
- `GetTransmission()`

### 6.2 位置正解：`SolveForward(theta1, theta2)`

当你已知驱动角，希望求解输出姿态时，调用：

```cpp
const couple_joint::SolveResult result = solver.SolveForward(theta1, theta2);
```

它内部使用阻尼牛顿迭代。

建议同时检查：

- `status`
- `residual`
- `iterations`

例如：

```cpp
const auto result = solver.SolveForward(theta1, theta2);
if (result.status == couple_joint::SolveStatus::ForwardNotConverged) {
  std::cerr << "forward solve did not converge\n";
  return;
}
if (result.status == couple_joint::SolveStatus::SingularJacobian) {
  std::cerr << "forward solve hit singular Jacobian\n";
  return;
}
```

使用建议：

- 如果需要以某个驱动角组合为基准做速度 / 力矩计算，通常应先做一次有效的 `SolveForward()`；
- 若返回 `ForwardNotConverged`，不应假设内部状态已经变为目标姿态；
- 若返回 `Success` 或 `NearSingular`，可继续读取当前状态和传动信息。

## 7. 速度映射

速度接口都是基于“当前内部状态”处的局部线性映射，因此推荐先进行一次有效的位置求解，再调用这些接口。

### 7.1 速度正解：`SolveForwardVelocity(theta1_dot, theta2_dot)`

输入驱动角速度，输出 `roll_dot` 和 `pitch_dot`：

```cpp
const auto result = solver.SolveForwardVelocity(1.2, -0.4);
```

适用场景：

- 已知当前姿态和电机速度，求中间连杆的瞬时广义速度；
- 做局部速度传播。

### 7.2 速度逆解：`SolveInverseVelocity(roll_dot, pitch_dot)`

输入目标输出广义速度，输出所需驱动角速度：

```cpp
const auto result = solver.SolveInverseVelocity(roll_dot, pitch_dot);
```

适用场景：

- 已知当前姿态和目标输出速度，求驱动侧速度指令；
- 做局部逆运动学速度映射。

### 7.3 使用注意事项

- 速度接口不负责更新位置状态；
- 它们只在当前状态附近有效；
- 如果当前位置奇异或接近奇异，应重点检查 `result.transmission`。

## 8. 力矩映射

力矩接口同样依赖当前内部状态处的局部传动关系。

### 8.1 力矩正解：`SolveForwardTorque(theta1_torque, theta2_torque)`

输入驱动力矩，输出输出侧广义力矩：

```cpp
const auto result = solver.SolveForwardTorque(theta1_torque, theta2_torque);
```

适用场景：

- 分析当前姿态下电机力矩在输出侧的映射；
- 估计当前姿态下的输出能力。

### 8.2 力矩逆解：`SolveInverseTorque(roll_torque, pitch_torque)`

输入输出广义力矩，输出所需驱动力矩：

```cpp
const auto result = solver.SolveInverseTorque(roll_torque, pitch_torque);
```

适用场景：

- 已知输出需求，反推当前姿态下的驱动力矩；
- 进行准静态力矩分配分析。

### 8.3 使用注意事项

- 力矩接口是低速准静态意义下的映射；
- 它们依赖当前姿态处的 `J_{q theta}`；
- 在奇异或近奇异区域，力矩映射会变得敏感，应重点检查 `sigma_min_j` 和 `condition_j`。

## 9. `SolveStatus` 处理建议

### 9.1 `Success`

正常使用。

### 9.2 `NearSingular`

一般仍可继续使用，但建议：

- 记录日志；
- 给上层做预警；
- 在控制侧增加限幅或保护。

### 9.3 `InvalidCalibration`

这表示在“标定失败的求解器实例”上继续调用了位置、速度或力矩接口。应先查看：

- `GetCalibrationStatus()`
- `MechanismSolver::StatusMessage(solver.GetCalibrationStatus())`

然后再检查：

- 输入点是否正确；
- 旋转顺序是否设置；
- 平面法向或共面关系是否成立。

### 9.4 `DegenerateSideEquation`

逆解过程中的单侧几何方程退化。通常表示：

- 当前几何配置不适合解算；
- 输入数据接近退化布局。

### 9.5 `UnreachablePose`

目标姿态不可达。此时不要继续使用本次位置结果。

### 9.6 `SingularJacobian`

当前局部雅可比奇异。常见影响：

- 位置正解不稳定；
- 速度 / 力矩映射不可用；
- 逆向速度或力矩求解失败。

### 9.7 `ForwardNotConverged`

位置正解迭代未收敛。建议：

- 检查输入驱动角是否合理；
- 检查当前位置是否接近奇异；
- 必要时从更接近的初值状态开始解算。

## 10. 常见使用模式

### 10.1 用目标姿态驱动机构

适合“给定 roll / pitch，求电机角”的场景：

1. 构造求解器
2. 调用 `SolveInverse(roll, pitch)`
3. 检查 `status`
4. 读取 `GetState()` 里的 `theta1` / `theta2`

### 10.2 用当前驱动角恢复姿态

适合“已知编码器角度，求机构姿态”的场景：

1. 构造求解器
2. 调用 `SolveForward(theta1, theta2)`
3. 检查 `status`、`residual`、`iterations`
4. 读取 `GetState()`、`GetGeometry()`

### 10.3 在已知姿态上做速度分析

1. 先做一次成功的位置逆解或正解
2. 调用 `SolveForwardVelocity()` 或 `SolveInverseVelocity()`
3. 检查 `status`
4. 如有需要，读取 `transmission.velocity_jacobian`

### 10.4 在已知姿态上做力矩分析

1. 先做一次成功的位置逆解或正解
2. 调用 `SolveForwardTorque()` 或 `SolveInverseTorque()`
3. 检查 `status`
4. 如有需要，读取 `transmission.reduction` 或奇异值指标

## 11. 完整 C++ 示例

下面的示例直接复用了仓库测试中已经验证过的一组标定参数。

```cpp
#include "mechanism_solver.h"

#include <iostream>

int main() {
  couple_joint::CalibrationInput input;
  input.left_joint_center = {-35.462080740, 29.491263548, 58.546474855};
  input.right_joint_center = {-35.462080740, -29.491263548, 58.546474855};
  input.left_planar_endpoint = {-35.5, 11.61629736, 56.42889465};
  input.right_planar_endpoint = {-35.5, -11.61629736, 56.42889465};
  input.roll_center = {0.0, 0.0, 0.0};
  input.pitch_center = {0.0, 0.0, 0.0};
  input.left_spatial_endpoint = {-36.6, 26.0, -5.0};
  input.right_spatial_endpoint = {-36.6, -26.0, -5.0};
  input.rotation_order = couple_joint::RotationOrder::PitchThenRoll;

  couple_joint::MechanismSolver solver(input);
  if (solver.GetCalibrationStatus() != couple_joint::CalibrationStatus::Success) {
    std::cerr << "solver init failed: "
              << couple_joint::MechanismSolver::StatusMessage(
                     solver.GetCalibrationStatus())
              << "\n";
    return 1;
  }

  const couple_joint::SolveResult inverse_result = solver.SolveInverse(0.03, -0.02);
  if (inverse_result.status != couple_joint::SolveStatus::Success &&
      inverse_result.status != couple_joint::SolveStatus::NearSingular) {
    std::cerr << "inverse solve failed, status=" << static_cast<int>(inverse_result.status) << "\n";
    return 1;
  }

  const couple_joint::MechanismState state = solver.GetState();
  const couple_joint::MechanismGeometry geometry = solver.GetGeometry();
  const couple_joint::TransmissionInfo transmission = solver.GetTransmission();

  std::cout << "theta1 = " << state.theta1 << "\n";
  std::cout << "theta2 = " << state.theta2 << "\n";
  std::cout << "roll   = " << state.roll << "\n";
  std::cout << "pitch  = " << state.pitch << "\n";
  std::cout << "left spatial point = ("
            << geometry.left_spatial_endpoint.x << ", "
            << geometry.left_spatial_endpoint.y << ", "
            << geometry.left_spatial_endpoint.z << ")\n";
  std::cout << "sigma_min(J) = " << transmission.sigma_min_j << "\n";

  const couple_joint::VelocityTorqueResult forward_velocity =
      solver.SolveForwardVelocity(1.2, -0.4);
  if (forward_velocity.status == couple_joint::SolveStatus::Success ||
      forward_velocity.status == couple_joint::SolveStatus::NearSingular) {
    std::cout << "roll_dot  = " << forward_velocity.roll << "\n";
    std::cout << "pitch_dot = " << forward_velocity.pitch << "\n";
  }

  const couple_joint::VelocityTorqueResult inverse_torque =
      solver.SolveInverseTorque(2.0, -0.75);
  if (inverse_torque.status == couple_joint::SolveStatus::Success ||
      inverse_torque.status == couple_joint::SolveStatus::NearSingular) {
    std::cout << "theta1_torque = " << inverse_torque.theta1 << "\n";
    std::cout << "theta2_torque = " << inverse_torque.theta2 << "\n";
  }

  return 0;
}
```

## 12. 接口速查表

| 接口 | 输入 | 输出 | 是否依赖当前状态 | 典型用途 |
| --- | --- | --- | --- | --- |
| `GetCalibrationStatus()` | 无 | `CalibrationStatus` | 否 | 检查标定是否成功 |
| `StatusMessage(CalibrationStatus)` | 标定状态 | C 字符串 | 否 | 获取标定状态说明 |
| `StatusMessage(SolveStatus)` | 求解状态 | C 字符串 | 否 | 获取求解状态说明 |
| `GetParameters()` | 无 | `MechanismParameters` | 否 | 查看规范几何参数 |
| `GetState()` | 无 | `MechanismState` | 是 | 查看当前 `theta / q` |
| `GetGeometry()` | 无 | `MechanismGeometry` | 是 | 查看当前几何点和旋转轴 |
| `GetTransmission()` | 无 | `TransmissionInfo` | 是 | 查看当前传动与奇异性 |
| `SolveInverse(roll, pitch)` | 输出姿态 | `SolveResult` | 是，使用当前分支作为参考 | 由 `q` 求 `theta` |
| `SolveForward(theta1, theta2)` | 驱动角 | `SolveResult` | 是，使用当前 `q` 作为初值 | 由 `theta` 求 `q` |
| `SolveForwardVelocity(theta1_dot, theta2_dot)` | 驱动速度 | `VelocityTorqueResult` | 是 | 由 `dot(theta)` 求 `dot(q)` |
| `SolveInverseVelocity(roll_dot, pitch_dot)` | 输出速度 | `VelocityTorqueResult` | 是 | 由 `dot(q)` 求 `dot(theta)` |
| `SolveForwardTorque(theta1_torque, theta2_torque)` | 驱动力矩 | `VelocityTorqueResult` | 是 | 由驱动力矩求输出广义力矩 |
| `SolveInverseTorque(roll_torque, pitch_torque)` | 输出广义力矩 | `VelocityTorqueResult` | 是 | 由输出广义力矩求驱动力矩 |

## 13. 使用建议汇总

- 构造后总是先检查 `GetCalibrationStatus()`；
- 做速度 / 力矩映射前，通常先做一次成功的位置求解；
- 上层若允许继续使用近奇异结果，建议只接受 `Success` 和 `NearSingular`；
- 如果你需要可视化或调试姿态，优先使用 `GetGeometry()`；
- 如果你需要分析局部传动特性，优先使用 `GetTransmission()`。
