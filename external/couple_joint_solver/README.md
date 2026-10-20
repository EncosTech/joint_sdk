# couple_joint

`couple_joint` 是一个六连杆闭链机构求解器仓库，包含两部分：

- C++ 核心求解器：负责机构的正逆位置求解、速度映射和力矩映射
- Web 可视化前端：将求解器编译为 WebAssembly，并通过 React + Three.js 进行三维交互预览

这个仓库适合两类使用场景：

- 直接在 C++ 项目中集成 `MechanismSolver`
- 用浏览器加载标定参数，进入三维预览界面调试机构姿态与传动特性

## 功能概览

当前仓库主要提供以下能力：

- 位置逆解：给定 `roll / pitch`，求解两个输入驱动角 `theta1 / theta2`
- 位置正解：给定 `theta1 / theta2`，迭代求解 `roll / pitch`
- 速度映射：支持 `theta` 空间与 `q = [roll, pitch]` 空间之间的速度映射
- 力矩映射：支持驱动力矩与输出广义力矩之间的映射
- 奇异性分析：返回 Jacobian、最小奇异值、条件数、近奇异标志
- WASM 导出：将 C++ 求解器编译为浏览器可调用模块
- 三维可视化：在前端界面中查看机构几何、姿态和传动信息

## 文档入口

建议按下面顺序阅读：

- [docs/mechanism.md](docs/mechanism.md)
  机构定义、坐标系、闭链约束、正逆解与速度/力矩推导的理论说明
- [docs/usage.md](docs/usage.md)
  C++ 求解器的接口使用指南，说明如何构造 `MechanismSolver`、如何处理状态码、如何读取状态与传动信息

如果你想直接看源码入口，可以从下面几个文件开始：

- [src/mechanism_solver.h](src/mechanism_solver.h)
  求解器对外 API 定义
- [src/mechanism_solver.cc](src/mechanism_solver.cc)
  位置求解主流程与状态处理
- [src/wasm_bindings.cc](src/wasm_bindings.cc)
  Emscripten 绑定，定义 C++ 到前端的导出接口
- [web/src/main.tsx](web/src/main.tsx)
  前端应用入口
- [web/src/pages/ConfigPage.tsx](web/src/pages/ConfigPage.tsx)
  参数配置页
- [web/src/pages/ViewerPage.tsx](web/src/pages/ViewerPage.tsx)
  三维预览页

## 仓库结构

```text
.
├── src/                    # C++ 求解器与 WASM 绑定
├── tests/                  # C++ 原生测试
├── benchmarks/             # 原生 benchmark
├── web/                    # React + Three.js 前端
├── scripts/                # 构建脚本
├── docs/                   # 理论文档与使用说明
└── third_party/            # 第三方依赖（如 nanobench 子模块）
```

## 环境要求

### C++ / 原生构建

- `cmake >= 3.20`
- 支持 C++17 的编译器
- `Eigen3`

### WASM / 前端

- `pnpm`
- `Node.js`
- Emscripten SDK，当前构建脚本默认从 `/home/bismarck/emsdk` 加载环境

## C++ 构建与测试

### 构建原生测试程序

```bash
cmake -B build -DCOUPLE_JOINT_BUILD_TESTS=ON
cmake --build build
```

### 运行测试

```bash
./build/mechanism_solver_test
```

或者通过 `ctest`：

```bash
ctest --test-dir build
```

## 原生 Benchmark

如果需要构建 benchmark，请先拉取子模块：

```bash
git submodule update --init --recursive
```

然后执行：

```bash
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DCOUPLE_JOINT_BUILD_BENCHMARKS=ON
cmake --build build-bench --target mechanism_solver_bench
./build-bench/mechanism_solver_bench
```

说明：

- benchmark 不走 `ctest`
- 建议使用 `Release` 模式，`Debug` 下性能数据没有参考意义

## 构建 WASM

前端三维预览依赖 WASM 产物。构建命令如下：

```bash
pnpm run build:wasm
```

该命令会调用 `scripts/build_wasm.sh`，输出：

- `web/public/wasm/couple_joint_solver.js`
- `web/public/wasm/couple_joint_solver.wasm`

如果这一步没有成功，前端无法正常加载求解器。

## 启动前端可视化

### 安装依赖

```bash
pnpm install
```

### 启动开发服务器

```bash
pnpm run dev
```

Vite 会启动本地开发服务，默认可通过 `http://localhost:5173` 访问。

### 进入三维预览界面

前端启动后，使用流程如下：

1. 打开配置页
2. 填写或导入机构标定参数
3. 点击“保存并进入”
4. 进入三维预览页，拖动滑块查看 `theta / roll / pitch` 变化
5. 在右侧面板查看当前位置的传动矩阵、奇异值和近奇异提示

如果页面停留在配置页，通常优先检查：

- `pnpm run build:wasm` 是否已经执行成功
- 标定参数是否满足机构几何约束
- 浏览器控制台是否有 WASM 加载错误

### 构建前端发布版本

```bash
pnpm run build
```

产物输出到：

```text
dist/
```

### 本地预览发布版本

```bash
pnpm run preview
```

## 前端相关说明

前端当前基于以下技术栈：

- React 19
- TypeScript
- Vite
- Three.js

整体交互分为两个页面：

- 配置页：输入、导入、导出标定参数
- 预览页：显示机构三维场景、姿态滑块和传动信息

## 常用开发路径

如果你是第一次接手这个仓库，推荐顺序如下：

1. 阅读 [docs/mechanism.md](docs/mechanism.md) 了解机构与推导
2. 阅读 [docs/usage.md](docs/usage.md) 了解 C++ 求解器 API
3. 运行 `cmake -B build && cmake --build build`
4. 运行 `ctest --test-dir build`
5. 运行 `pnpm install`
6. 运行 `pnpm run build:wasm`
7. 运行 `pnpm run dev`，进入前端三维预览界面
