# fdtd2d 使用手册

## 简介

fdtd2d 是用 C11 写的二维声学 FDTD 仿真器。它计算二维声压场，v1 只支持均匀介质。声源可以是点源或线源，信号为简谐波。物体可以放置刚性障碍物，外边界可选 rigid、pressure_release 或 PML。结果以离线 PNG 热力图输出。所有数值使用 SI 单位。

## 构建

前提：C11 编译器、CMake 3.20 或更高版本、libpng 开发包（同时需要 libm）。tomlc17 已随源码提供，无需安装。当前只支持 POSIX 系统。默认构建类型是 Release，可执行文件为 `../build/fdtd2d`。

```sh
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

| CMake 选项 | 默认 | 作用 |
|---|---|---|
| `FDTD2D_BUILD_TESTS` | ON | 构建单元测试、物理验证测试和示例的 `--check` 冒烟测试 |
| `FDTD2D_WERROR` | OFF | 把编译警告当作错误 |
| `FDTD2D_SANITIZE` | OFF | 启用 AddressSanitizer 和 UBSan |

## 快速开始

先用 `--check` 检查模型，它只解析配置并打印摘要，不仿真，也不创建输出目录：

```sh
./build/fdtd2d --check examples/point_source_free_field.toml
```

摘要如下（`api/model.md` 中的实际输出，有删节）：

```
grid:  400 x 300 cells physical, 440 x 340 cells total (incl. PML)
time:  dt = 7.21538e-06 s, n_steps = 555, requested duration = 0.004 s, simulated 0.00400453 s
       points per wavelength = 34.30 at 2000 Hz
boundary: x_min pml, x_max pml, y_min pml, y_max pml
sources: 1
```

确认无误后运行：

```sh
./build/fdtd2d examples/point_source_free_field.toml
```

输出写入示例的 `[output].directory`，即 `out/point_source_free_field`。另一个示例 `../examples/line_source_obstacles.toml` 含线源和障碍物，输出到 `out/line_source_obstacles`。

## 命令行

用法：`fdtd2d [options] CONFIG.toml`，必须且只能给出一个配置文件。

| 选项 | 说明 |
|---|---|
| `--check` | 解析配置、构建模型、打印摘要后退出，不仿真 |
| `-o DIR`, `--output DIR` | 输出目录，覆盖 `[output].directory`；不存在时自动创建 |
| `-q`, `--quiet` | 不打印进度信息（警告仍会打印） |
| `-h`, `--help`；`--version` | 显示帮助；显示版本 |

| 退出码 | 含义 |
|---|---|
| 0 | 成功，或只显示了帮助、版本 |
| 1 | 运行错误（配置、模型、写文件、解变为非有限值），信息以 `error: ` 开头写到标准错误 |
| 2 | 命令行用法错误（未知选项、缺少配置文件、给了多个配置文件、`-o` 缺少参数） |

摘要写到标准输出。进度（每 10% 一行，结束时一行 `done: ...`）和警告写到标准错误。

## 配置文件

TOML 格式，所有表都按白名单检查键名，拼错的键会报错。要求浮点数的键可以写整数。最小示例：

```toml
[simulation]
duration = 0.004            # [s]
[grid]
x_range = [0.0, 2.0]        # [m]，PML 加在外侧
y_range = [0.0, 1.5]
dx      = 0.005             # [m]，x_range 长度必须是 dx 的整数倍
[medium]
density = 1.21              # [kg/m^3]
c       = 343.0             # [m/s]
[[source]]
shape  = { type = "point", position = [0.6, 0.75] }
signal = { type = "harmonic", frequency = 2000.0, amplitude = 2e-3 }
```

没有写的 `[boundary]` 四边默认都是 PML，`[output]` 使用默认值。完整的字段定义、约束和错误信息见 [config.md](api/config.md)。

### `[simulation]`、`[grid]`、`[medium]`（必需）

| 键 | 类型 | 默认 | 含义 |
|---|---|---|---|
| `simulation.duration` | 数值 | 必需 | 仿真时长 [s]，> 0 |
| `simulation.cfl` | 数值 | 0.7 | Courant 数，0 < cfl ≤ 1，由它推导 dt |
| `simulation.dt` | 数值 | 无 | 直接指定时间步长 [s]；不能与 `cfl` 同时出现 |
| `grid.x_range`, `grid.y_range` | `[min, max]` | 必需 | 物理域范围 [m] |
| `grid.dx`, `grid.dy` | 数值 | `dx` 必需，`dy` 等于 `dx` | 网格步长 [m] |
| `medium.density` | 数值 | 必需 | 密度 [kg/m³]，> 0 |
| `medium.c` | 数值 | 必需 | 声速 [m/s]，> 0 |

### `[boundary]`、`[boundary.pml]`（可选）

| 键 | 类型 | 默认 | 含义 |
|---|---|---|---|
| `x_min` `x_max` `y_min` `y_max` | 字符串 | `"pml"` | `"rigid"`、`"pressure_release"` 或 `"pml"` |
| `pml.thickness` | 整数 | 20 | PML 层数（单元），≥ 1，所有 PML 边共用 |
| `pml.reflection` | 数值 | 1e-6 | 理论反射系数 R，0 < R < 1 |
| `pml.order` | 数值 | 2 | 衰减剖面的多项式阶数，≥ 0 |

### `[[obstacle]]` 和 `[[source]]`（可选，可重复）

`[[obstacle]]` 只有键 `shape`（刚性障碍物）。`[[source]]` 有 `shape` 和 `signal`，没有声源时会有警告。`shape` 写成内联表，`type` 决定其余的键：

| type | 用于 | 键 |
|---|---|---|
| `rect` | 障碍物 | `min = [x, y]`, `max = [x, y]` |
| `circle` | 障碍物 | `center = [x, y]`, `radius` |
| `polygon` | 障碍物 | `vertices = [[x, y], ...]`，至少 3 个顶点 |
| `point` | 声源 | `position = [x, y]` |
| `segment` | 声源 | `start = [x, y]`, `end = [x, y]` |

声源的 `signal` 目前只有 `harmonic`：

| 键 | 默认 | 含义 |
|---|---|---|
| `frequency` | 必需 | 频率 [Hz]，> 0 |
| `amplitude` | 必需 | 点源是体积速度 Q [m²/s]，线源是单位长度体积速度 Q′ [m/s] |
| `phase` | 0 | 初相 [rad] |
| `ramp_cycles` | 3 | 起振渐变周期数，≥ 0 |

### `[output]`（可选）

| 键 | 类型 | 默认 | 含义 |
|---|---|---|---|
| `directory` | 字符串 | `"out"` | 输出目录，非空；`-o` 可覆盖 |
| `every` | 整数 | 10 | 每 N 步输出一帧，≥ 1；最后一步总会输出 |
| `color_limit` | 数值或 `"auto"` | `"auto"` | 色标范围 [Pa]，数值须 > 0 |
| `show_pml` | 布尔 | false | 是否把 PML 区域也画出来 |
| `pixel_scale` | 整数 | 1 | 每个单元的像素边长，1 到 16 |

## 输出

| 文件 | 内容 |
|---|---|
| `summary.txt` | 版本、配置文件路径和模型摘要，仿真开始前写出 |
| `p_XXXXXX.png` | 声压热力图。`XXXXXX` 是步号，补零到至少 6 位，从第 1 步起，不含初始时刻 |
| `frames.csv` | 每帧一行的索引 |

`frames.csv` 的列：

| 列 | 含义 |
|---|---|
| `step`, `time_s`, `file` | 步号、时间 [s]、PNG 文件名（不含目录） |
| `color_limit_pa` | 该帧实际使用的色标上限 [Pa] |
| `p_min_pa`, `p_max_pa` | 该帧范围内非固体单元的最小、最大声压 [Pa]，不受色标截断影响 |

图像没有色标和文字，需要数值时查 `color_limit_pa`。颜色含义：

| 颜色 | 含义 |
|---|---|
| 蓝色、近白、红色 | 负声压、零、正声压；深蓝对应 −limit，深红对应 +limit |
| 灰色 | 固体（障碍物）单元 |
| 黑色细线 | 物理域边界，仅在 `show_pml = true` 且网格带有 PML 时绘出 |

- `show_pml = false` 只画物理域。
- `color_limit` 为数值时所有帧共用同一个范围，便于比较。绝对值超过它的声压按端点颜色显示，因此设得太小会使强的区域饱和成一片深色。
- `color_limit = "auto"` 时每帧取 ±max\|p\|，不同帧的颜色不可直接比较。
- 单元总是画成正方形像素，`dx ≠ dy` 时图像在一个方向上被拉伸。

## 常见问题

| 信息 | 处理 |
|---|---|
| `grid: x_range length 2.003 is not a multiple of dx = 0.005 (nearest valid dx: 0.00499501)` | 改用提示的 dx，或调整 range，使长度是步长的整数倍 |
| `simulation: dt = ... s is above the CFL limit ... s (cfl = ... > 1)` | 减小 `dt`，或删掉 `dt` 改用 `cfl` |
| `source[0]: point (5, 1) is outside the physical domain [0, 1] x [0, 1]` | 把声源位置移到物理域内 |
| `source[0]: point (...) lies inside a rigid obstacle` | 把声源移出障碍物 |
| `output: frames would be 80000 x 60000 pixels, above the PNG limit of 65535; reduce pixel_scale` | 减小 `pixel_scale`，或增大 `dx`、`dy`。`--check` 也会报这个错 |
| `solution became non-finite at step ... ; check dt/cfl and source amplitudes` | 检查 `dt`、`cfl` 和声源振幅 |
| `warning: only ... points per wavelength at ... Hz (recommended >= 10); ...` | 减小 `dx`、`dy` 或降低声源频率，否则数值色散明显 |
| `warning: obstacle[0] (circle) extends into the PML (... cells); ...` | 把障碍物移出 PML，否则它会反射声波、削弱吸收 |

配置层的错误格式为 `<文件>:<行号>: <键路径>: <问题>`，只报告第一处，例如 `grid.dxx: unknown key` 表示键名拼错。

## 链接

- [theory.md](theory.md)：理论说明
- [api/README.md](api/DEV.md)：模块文档总览
