# model：解析器第二层（SimConfig → 网格模型）

头文件：`src/model.h`　实现：`src/model.c`　依赖：config（只读 `SimConfig` 结构体，不调用其函数）、grid、medium、boundary、geometry、waveform、source、common

`model_build()` 把与网格无关的 `SimConfig` 转换成求解器可以直接使用的 `Model`。它负责所有与网格相关的检查和预计算：网格尺寸、时间步长、介质编号图、刚性障碍物掩码、PML 衰减剖面以及声源离散。solver 只读取 `Model`，不做任何几何或参数推导。

**校验分工**：用户输入只在 config 层校验一次（`config_load_*`：取值范围、有限性、枚举、形状、PML 参数、介质、信号）。`model_build()` 不重复这些检查，只检查依赖网格的内容：域长度能否被 dx、dy 整除，单元数和步数是否溢出，`dt` 是否超过 CFL 上限，声源是否落在域外或障碍物内。

## 类型

### `Model`

```c
typedef struct Model {
    Grid grid;                  /* 见 grid.md */

    double dt;                  /* 时间步长 [s] */
    long n_steps;               /* 总步数 */
    double duration;            /* 用户请求的仿真时长 [s] */
    double cfl;                 /* 实际 Courant 数 */

    MediumTable media;          /* owned；第 0 项为背景介质 */
    MediumId *medium_id;        /* owned [nx*ny]，单元介质编号 */

    uint8_t *solid;             /* owned [nx*ny]，1 = 刚性障碍物单元 */
    size_t n_solid;             /* 固体单元个数 */

    BoundaryType boundary[SIDE_COUNT];
    PmlParams pml;
    int pml_cells[SIDE_COUNT];  /* 各侧 PML 层数，非 PML 侧为 0 */

    double *sigma_x_cell;       /* owned [nx]   单元中心处的 x 向衰减 [1/s] */
    double *sigma_x_face;       /* owned [nx+1] x 面处 */
    double *sigma_y_cell;       /* owned [ny] */
    double *sigma_y_face;       /* owned [ny+1] */

    ModelSource *sources;       /* owned [n_sources]，见 source.md */
    int n_sources;
} Model;
```

`Model` 构建完成后不再修改，求解期间只读。v1 中 `medium_id` 全为 0；预留这个字段是为了以后支持多介质区域。

## 函数

### `model_build`

```c
int model_build(const SimConfig *cfg, Model *m, FdtdError *err);
```

| 参数 | 说明 |
|---|---|
| `cfg` | 非 `NULL`，由 `config_load_*` 生成（或满足同样的约束）的配置，只读，调用后仍归调用方所有 |
| `m` | 非 `NULL`，输出，会被先清零 |
| `err` | 错误信息，可为 `NULL` |

前提：`cfg` 满足 config 校验后的全部约束，包括 x_range、y_range 有限且上界大于下界，dx、dy、duration、dt 为有限正数，`0 < cfl ≤ 1`，边界类型是合法枚举，PML 参数合法，介质、形状和信号合法，列表与个数一致。`model_build()` 不检查这些，传入不满足约束的配置属于未定义行为。

成功返回 0，`m` 拥有全部内存，用 `model_free` 释放。失败返回 -1，此时 `*m` 为全零，不需要释放。`cfg` 中的数据会被复制或转换，`Model` 不引用 `cfg` 的内存，因此可以在 `model_build` 之后立即 `config_free(cfg)`。

处理步骤如下，任一步失败即返回。

**1. 网格**

- 物理域单元数取 `nx_phys = round((x1 − x0)/dx)`，y 方向同理。
- 若 `|(x1−x0)/dx − nx_phys| > 1e-6·max(1, (x1−x0)/dx)`，报错并给出最接近的可整除步长。错误信息示例：
  ```
  grid: x_range length 2.003 is not a multiple of dx = 0.005 (nearest valid dx: 0.00499501)
  ```
- 若 `(x1−x0)/dx` 超过 `INT_MAX`，或不足一个单元，同样报错。
- PML 侧各增加 `pml.thickness` 层，非 PML 侧为 0。PML 参数（厚度、R、m）已由 config 校验，这里直接使用。
- 总单元数 `nx = off_x + nx_phys + pml_cells[x_max]`，y 方向同理；`nx+1` 和 `ny+1` 必须在 `int` 范围内，`nx·ny` 必须在 `size_t` 范围内。

**2. 介质**

- 用背景介质初始化介质表，`medium_id` 全部置 0。
- c_max 和 c_min 分别取介质表中的最大和最小声速。

**3. 时间步长**

记 `S = c_max·√(1/dx² + 1/dy²)`。

| 配置 | 结果 |
|---|---|
| 未给 `dt` | `dt = cfl / S`，cfl 默认 0.7。极慢介质配合极大单元时 S 很小但有限，`dt` 会上溢为无穷，此时报错 `simulation: the derived time step ... overflows` |
| 给了 `dt` | 实际 `cfl = dt·S`；若大于 1 则报错并给出 dt 的上限 |

步数 `n_steps = max(1, ceil(duration/dt − 1e-9))`，实际仿真时长为 `n_steps·dt`，不小于请求的 `duration`。步数超过 `long` 范围时报错。

二维交错网格 leapfrog 格式的稳定条件是 `cfl ≤ 1`。dx = dy 时等价于 `c·dt/dx ≤ 1/√2`。

**4. 刚性障碍物**

单元中心落在任一障碍物形状内（`shape_contains`）即标记为固体，包括 PML 内的单元。只遍历形状包围盒外扩一格的范围。以下两种情况会发出警告：
- 障碍物不包含任何单元中心：它太小或位于网格之外，不会起作用。
- 障碍物伸进了 PML：PML 中的刚性单元会反射声波，削弱吸收效果。

曲线边界按单元呈阶梯状近似，精度取决于 dx。

**5. PML 衰减剖面**

```
σ(d) = σ_max · (d / L)^m,       0 ≤ d ≤ L
σ_max = −(m + 1) · c_ref · ln(R) / (2L)
```

各符号含义：
- d：从物理域边缘向 PML 内部的距离 [m]。
- L：PML 厚度，x 侧为 `thickness·dx`，y 侧为 `thickness·dy`。
- m：多项式阶数，即 `pml.order`。
- R：理论反射系数，即 `pml.reflection`。
- c_ref：参考声速，取 c_max。

σ 分别在单元中心（供 p 使用）和面（供 v 使用）处取值。物理域内、物理域与 PML 的交界面上以及非 PML 侧，σ 都为 0。以 x_min 侧为例，交界面位于 x 面 `off_x`：
- 面 i 的深度为 `d = (off_x − i)·dx`。
- 单元 i 的深度为 `d = (off_x − i − 0.5)·dx`。

**6. 网格分辨率检查**

f_max 取所有声源的 `signal_max_frequency` 最大值。若 `c_min / (f_max·max(dx, dy)) < 10`，发出警告。常用经验是每波长至少 10 个网格点，否则数值色散明显。

**7. 声源**

逐个调用 `source_discretize`，规则见 source.md。错误信息前加 `source[k]`。没有声源时发出警告。

### `model_free`

```c
void model_free(Model *m);
```

释放全部内存并清零。可作用于全零结构体，可重复调用，`m == NULL` 时什么都不做。

### `model_print_summary`

```c
void model_print_summary(FILE *out, const Model *m);
```

输出多行可读的模型摘要，`out` 和 `m` 必须非 `NULL`。`fdtd2d --check` 和输出目录下的 `summary.txt` 都使用它。下面是 `fdtd2d --check examples/point_source_free_field.toml` 的实际输出：

```
grid:  400 x 300 cells physical, 440 x 340 cells total (incl. PML)
       dx = 0.005 m, dy = 0.005 m
       domain x = [0, 2] m, y = [0, 1.5] m
       pml cells: x_min 20, x_max 20, y_min 20, y_max 20
time:  dt = 7.21538e-06 s, n_steps = 555, requested duration = 0.004 s, simulated 0.00400453 s
       cfl = 0.7
       points per wavelength = 34.30 at 2000 Hz
medium: [0] rho = 1.21 kg/m^3, c = 343 m/s, K = 142355 Pa
boundary: x_min pml, x_max pml, y_min pml, y_max pml
obstacles: 0 solid cells of 149600
sources: 1
  [0] point source, 4 cells, harmonic signal, f = 2000 Hz, amplitude = 0.002
```

## 常量

| 宏 | 值 | 说明 |
|---|---|---|
| `MODEL_MIN_POINTS_PER_WAVELENGTH` | 10.0 | 每波长网格点数的警告阈值 |

## 示例

```c
SimConfig cfg;
Model model;
FdtdError err = {{0}};
if (config_load_file("case.toml", &cfg, &err) != 0 ||
    model_build(&cfg, &model, &err) != 0) {
    fprintf(stderr, "error: %s\n", err.msg);
    config_free(&cfg);              /* 对全零结构体安全 */
    return 1;
}
config_free(&cfg);                  /* model 不引用 cfg 的内存 */
model_print_summary(stdout, &model);
/* ... solver_init(&solver, &model, &err) ... */
model_free(&model);
```

## 扩展方式

**多介质区域**
1. 在 config 中解析 `[[medium_region]]`，每个区域包含 shape、density 和 c。
2. 在步骤 2 中用 `medium_table_add` 追加介质，并把区域内单元的 `medium_id` 写成新编号。
3. 时间步长和 PML 已经使用 c_max，分辨率检查已经使用 c_min，solver 已经按单元读取介质，这三处都不用改。

**辐射面声源**：在 source 中实现面状形状的离散，model 不用改。
