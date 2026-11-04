# solver：二维线性声学 FDTD 时间推进

头文件：`src/solver.h`　实现：`src/solver.c`　依赖：common、model、medium、waveform、libm（`Grid`、`BoundaryType`、`ModelSource` 的定义经 `model.h` 引入）

本模块在交错网格上用 leapfrog 格式推进二维线性声学方程（无黏性理想流体，介质静止），外边界采用刚性、压力释放或分裂场 PML。solver 只读取 `Model`，所有几何、介质和声源数据都已由 model 预先算好。solver 自己预计算更新系数，分配并推进场变量，提供压力和能量的读取函数。

## 在流程中的位置

```
config ──► model ──► solver ──► render
(TOML)    (网格化)   (时间推进)  (PNG)
                         ▲
                     main.c 驱动
```

`main.c` 的调用顺序如下：

1. `config_load_file` → `model_build`，然后 `model_print_summary`。
2. `solver_init(&solver, &model, &err)`。
3. 循环 `n = 1 .. model.n_steps`：调用 `solver_step`；每隔 `output.every` 步以及最后一步，调用 `solver_get_pressure` 把全网格压力（含 PML 单元）复制到调用方的缓冲区，检查是否有限，再交给 render。
4. `solver_free`，然后 `model_free`。

`solver_step` 不检查数值是否发散。非有限值由 `main.c` 在取出压力后检查。

## 控制方程

求解器使用分裂场形式的方程（与 `solver.h` 一致）：

```
∂px/∂t + σx·px = −K ∂vx/∂x + K·q/2
∂py/∂t + σy·py = −K ∂vy/∂y + K·q/2          p = px + py
∂vx/∂t + σx·vx = −(1/ρ) ∂p/∂x
∂vy/∂t + σy·vy = −(1/ρ) ∂p/∂y
```

- K = ρc² 是体积模量，ρ 是密度，q 是声源项（见“声源”一节）。
- σx、σy 是 PML 阻尼 [1/s]，由 model 给出。物理域内和非 PML 侧 σ = 0。
- σ = 0 时两个压力方程相加得到 `∂p/∂t = −K ∇·v + K·q`，速度方程化为 `∂v/∂t = −(1/ρ) ∇p`，与 source.md 的源项形式一致。此时 px、py 各自没有独立的物理意义，只有 p = px + py 有意义。
- vx 只受 σx 阻尼，vy 只受 σy 阻尼。px 只受 σx 阻尼，py 只受 σy 阻尼。PML 角区两个方向的阻尼同时存在。

## 未知量与位置

p 存放在单元中心，vx 存放在 x 面，vy 存放在 y 面。布局和下标见 grid.md：

| 量 | 位置 | 数组长度 | 下标 |
|---|---|---|---|
| px、py | 单元中心 | `nx*ny` | `j*nx + i` |
| vx | x 面 | `(nx+1)*ny` | `j*(nx+1) + i` |
| vy | y 面 | `nx*(ny+1)` | `j*nx + i` |

这里的 `nx`、`ny` 含 PML 单元，是整个计算网格的单元数。

下面是一行（固定 j）中 `nx = 4` 个单元和 5 个 x 面：

```
face i:   0         1         2         3         4
          |----p0---|----p1---|----p2---|----p3---|
          vx0       vx1       vx2       vx3       vx4
cell i:        0         1         2         3
```

p 在单元中心，vx 在竖线表示的面上。n 个单元有 n+1 个面，面 i 位于单元 i−1 与单元 i 之间。最左和最右的竖线是最外侧面 `i = 0` 和 `i = nx`。

- 内部面 `i = 1 .. nx−1` 两侧各有一个单元，速度由两侧压力之差更新。
- 最外侧的面 `i = 0` 和 `i = nx` 只有一侧有单元，压力差不完整，必须由边界条件决定速度。这就是每行有 `nx+1` 个 x 面的原因。
- y 方向相同：每列有 `ny+1` 个 y 面，`j = 0` 和 `j = ny` 是外侧面。

## 时间推进

### 时间层

p 在整数时刻 `t_n = n·dt`，v 在半整数时刻 `t_{n+½}`。完成 `step` 步后，p 在 `step·dt`，v 在 `(step − ½)·dt`。初始状态全为 0。

### `solver_step` 的顺序

1. **速度** `v^{n+½}`：由 `v^{n−½}` 和 `p^n` 计算。先更新内部面（`vx` 的 `i = 1..nx−1`，`vy` 的 `j = 1..ny−1`），再更新压力释放侧的最外侧面。
2. **压力** `p^{n+1}`：由 `p^n` 和 `v^{n+½}` 计算 px、py。
3. **声源**：在 `t_{n+½} = (step + ½)·dt` 处求时间函数值，把增量加到声源单元的 px、py 上。
4. **时钟**：`step++`，`time = step·dt`。`time` 由乘法得到，不累加，所以没有累积误差。

`solver.h` 把第 2、3 项合并为一步描述；代码里声源在压力更新之后单独加入，结果等价。

### 半隐式阻尼系数

对每个量，阻尼项取新旧时刻的平均值。设 `h = σ·dt/2`：

```
a = (1 − h) / (1 + h)
c = 1 / (1 + h)
f_new = a·f_old − c·dt·rhs
```

σ = 0 时 a = c = 1，退化为无阻尼的 leapfrog。系数按轴向存放成一维表，按所在位置的 σ 计算：

| 表 | 长度 | 用于 |
|---|---|---|
| `ax_c`、`cx_c` | `nx` | px（单元 i 处的 σx） |
| `ay_c`、`cy_c` | `ny` | py（单元 j 处的 σy） |
| `ax_f`、`cx_f` | `nx+1` | vx（x 面 i 处的 σx） |
| `ay_f`、`cy_f` | `ny+1` | vy（y 面 j 处的 σy） |

### 离散更新公式

记 `p = px + py`，`bx = dt/ρ_face`（x 面），`by = dt/ρ_face`（y 面），`kdt = K·dt`（单元）。以下是代码实际执行的更新。

速度（`update_vx_row`、`update_vy_row`），其中 p 取 `p^n`：

```
vx(i,j) ← ax_f[i]·vx(i,j) − cx_f[i]·bx(i,j)/dx · ( p(i,j) − p(i−1,j) )
vy(i,j) ← ay_f[j]·vy(i,j) − cy_f[j]·by(i,j)/dy · ( p(i,j) − p(i,j−1) )
```

压力（`update_p_row`），其中 v 取 `v^{n+½}`：

```
px(i,j) ← ax_c[i]·px(i,j) − cx_c[i]·kdt(i,j)/dx · ( vx(i+1,j) − vx(i,j) )
py(i,j) ← ay_c[j]·py(i,j) − cy_c[j]·kdt(i,j)/dy · ( vy(i,j+1) − vy(i,j) )
```

热循环按 j 为外层、i 为内层逐行遍历，行指针带 `restrict`。更新不含针对固体或面类型的分支，因为这些情况已经编码在系数里（见下一节），所以含固体单元的算例每步成本与不含固体时相同。

### 稳定条件

二维交错网格 leapfrog 的稳定条件是

```
cfl = c·dt·√(1/dx² + 1/dy²) ≤ 1
```

c 取介质表的最大声速 c_max。该条件由 model 检查：未给 `dt` 时按 `cfl`（默认 0.7）反算 `dt`；给了 `dt` 且 `cfl > 1` 时 `model_build` 报错。**`solver_init` 不读取 `m->cfl`，也不重新检查**，所以手工构造或改动过 `dt` 的 `Model` 若超出稳定条件，solver 会照常运行并发散。

## 边界与固体

边界规则全部预先编码在系数数组里：`kdt`（单元）、`bx`、`by`（面）和阻尼表。`solver_step` 对所有单元和内部面使用同一套公式。

### 系数的构造

- `kdt[c] = K·dt`，固体单元为 0。固体单元由 `Model.solid` 标记。
- 内部 x 面（`i = 1 .. nx−1`）：两侧单元都是流体时，`bx = dt/ρ_face`，`ρ_face` 是两侧单元密度的算术平均；否则 `bx = 0`。内部 y 面同理。
- 最外侧面默认 `bx = by = 0`，只有压力释放侧例外（见下）。
- `bx = 0` 或 `by = 0` 的面，更新公式变成 `v ← a·v`，v 初值为 0，因此恒为 0。

### 刚性与 PML 的最外侧面

`rigid` 侧和 `pml` 侧的最外侧面上 v 恒为 0。热循环从不更新这些面，系数 `bx`、`by` 也是 0。PML 的最外缘因此是刚性壁，波穿过 PML、被最外缘反射、再穿回物理域，往返衰减的理论值由 R 决定（见 PML 一节）。

### 压力释放

`pressure_release` 侧要求边界面上 p = 0。实现方式是在面外侧放一个镜像虚单元，取反对称：

```
p_ghost = −p_inside
```

虚单元中心与内部单元中心分处面的两侧，距面各 dx/2，所以 p = 0 的平面正好在面上。最外侧面两侧的压力差为 `p_inside − p_ghost = 2·p_inside`（x_min、y_min 侧），或 `p_ghost − p_inside = −2·p_inside`（x_max、y_max 侧）。最外侧面速度的更新为：

```
x_min:  vx(0,j)  ← ax_f[0]·vx(0,j)  − cx_f[0]·bx(0,j)/dx   ·  2·( px + py )(0,j)
x_max:  vx(nx,j) ← ax_f[nx]·vx(nx,j) − cx_f[nx]·bx(nx,j)/dx · (−2)·( px + py )(nx−1,j)
y_min:  vy(i,0)  ← ay_f[0]·vy(i,0)  − cy_f[0]·by(i,0)/dy   ·  2·( px + py )(i,0)
y_max:  vy(i,ny) ← ay_f[ny]·vy(i,ny) − cy_f[ny]·by(i,ny)/dy · (−2)·( px + py )(i,ny−1)
```

- 这四个更新在 `update_release_faces` 中，位于内部面速度更新之后、压力更新之前，因此使用的是 `p^n`。
- 最外侧面的 `bx`、`by` 取内侧单元的密度：`dt/ρ_inside`。内侧单元是固体时为 0，速度保持 0。
- 压力释放侧没有 PML，model 给这些面的 σ 是 0，所以这些最外侧面的 `a = c = 1`。solver 不单独设置这两个值，直接由 σ = 0 算出；测试 `test_coefficients` 检查四个压力释放侧的最外侧面都满足 `a = c = 1`。

### 固体单元

固体单元由 model 标记（障碍物形状覆盖单元中心的单元，PML 内的单元也算）。

- `kdt = 0`：px、py 的更新项为 0，声源增量也为 0，所以 p 恒为 0。
- 与固体单元相邻的面，`bx = 0` 或 `by = 0`，所以 v 恒为 0。这就是刚性：固体表面上法向速度为 0。
- 曲线边界按单元呈阶梯状近似。

## PML 衰减剖面

σ 由 model 计算（`build_pml`、`fill_sigma_axis`），solver 只把它转成上面的 a、c。每个 PML 侧用相同的公式：

```
σ(d) = σ_max · (d / L)^m,         0 ≤ d ≤ L
σ_max = −(m + 1) · c_ref · ln(R) / (2L)
```

- d：从物理域边缘向 PML 内部的距离 [m]。
- L：PML 厚度，x 侧为 `thickness·dx`，y 侧为 `thickness·dy`。
- m：`pml.order`。R：`pml.reflection`，即理论反射系数。c_ref：c_max。

σ 在单元中心取一次（用于 p），在面上取一次（用于 v）。以 x_min 侧为例，物理域与 PML 的交界面是 x 面 `off_x`：

| 位置 | 深度 d |
|---|---|
| x 面 i（`i ≤ off_x`） | `(off_x − i)·dx` |
| 单元 i（`i < off_x`） | `(off_x − i − 0.5)·dx` |

交界面上 d = 0，物理域内 σ = 0。PML 最外缘的 x 面（`i = 0`）上 σ 达到 σ_max，但那个面的 v 恒为 0。x_max、y_min、y_max 侧同理，y 侧用 dy。`rigid` 和 `pressure_release` 侧的 σ 为 0。

PML 里的 p 是 px + py。垂直入射的波穿过 PML、被最外缘的刚性壁反射、再穿回物理域，沿途两次衰减，往返衰减因子的理论值就是 R。这是连续介质中的值，离散网格上的实际反射更大（boundary.md 有同样的说明）。本文档没有单独测量某个入射角的反射系数，验证一节给出的是封闭盒内总能量的衰减。

## 声源

声源是软源（单极子体积注入），加到压力方程的源项里，而不是直接设定压力。因此波经过声源位置时不会被反射。

在 `solver_step` 的第 3 步，对每个声源、每个声源单元 c（权重 `w`）：

```
inc  = ½ · kdt[c] · w · s(t_{n+½})
px[c] += inc
py[c] += inc
```

- 总增量 `K·dt·w·s` 平均分给 px 和 py，每份一半。
- `s(t)` 由 `signal_eval(&src->signal, t_{n+½})` 给出，`t_{n+½} = (step + ½)·dt`。声源在物理域内，那里 σ = 0，所以增量不需要乘阻尼系数 c。
- 权重来自 source.md：

| 类型 | 归一化 | 幅值 A 的含义 | 单位 |
|---|---|---|---|
| 点源 | Σ w·dx·dy = 1 | 单位深度的体积速度 Q | m²/s |
| 线源 | Σ w·dx·dy = 线段长度 [m] | 单位长度、单位深度的体积速度 Q′ | m/s |

权重的单位因此不同：点源的 w 是 1/m²，线源的 w 是 1/m（w 乘以面积等于长度）。两种情况下 `w·s` 的单位都是 1/s，即单位体积的体积注入率。

- 总注入量与网格尺寸无关，网格加密时源强度不变。
- 第一步的检验：从静止出发走一步后，`Σ p·dx·dy = K·dt·s(dt/2)`，而且 px 与 py 相等（见验证一节）。
- `m->sources` 和 `m->n_sources` 在每次 `solver_step` 中都会重新读取。因此调用方可以在两步之间把 `n_sources` 改成 0 来关闭声源，测试里就这么做。恢复 `n_sources` 之后才能 `model_free`，否则 `model_free` 不会释放全部 `ModelSource`。

## 多介质预留

solver 已经按单元读取介质，v1 只有一种介质，`medium_id` 全为 0：

- K 按单元取值：`kdt[c] = K(medium_id[c])·dt`。
- ρ_face 在面上取两侧单元密度的算术平均，因此介质界面落在两个单元之间时，界面上的面用平均密度。
- 时间步长和 PML 参考声速使用 c_max，由 model 处理。

测试 `test_two_media` 和 `test_two_media_horizontal` 在 `Model` 构建之后向介质表追加一种介质（ρ = 1000 kg/m³，c = 300 m/s），把一半单元改成新编号，重新 `solver_init` 后检查：`bx`、`by` 与上述平均规则一致（相对误差 1e-14 内），且封闭盒的守恒量仍然成立（偏差 < 1e-11）。没有测试验证过介质界面上的波反射与透射系数。

## 类型

### `Solver`

```c
typedef struct Solver {
    const Model *model;  /* borrowed；必须比 solver 活得长 */
    int nx, ny;

    /* 场 */
    double *px, *py;     /* [nx*ny]      分裂压力，p = px + py */
    double *vx;          /* [(nx+1)*ny] */
    double *vy;          /* [nx*(ny+1)] */

    /* 预计算系数（实现细节，只读） */
    double *kdt;         /* [nx*ny]      流体单元为 K*dt，固体单元为 0 */
    double *bx;          /* [(nx+1)*ny]  有效面为 dt/rho_face，否则 0 */
    double *by;          /* [nx*(ny+1)] */
    double *ax_c, *cx_c; /* [nx]   单元中心处的 x 向阻尼系数 */
    double *ax_f, *cx_f; /* [nx+1] x 面处 */
    double *ay_c, *cy_c; /* [ny] */
    double *ay_f, *cy_f; /* [ny+1] */

    long step;           /* 已完成的步数；p 位于 time = step * dt */
    double time;         /* step * dt */
} Solver;
```

| 字段 | 性质 |
|---|---|
| `model`、`nx`、`ny` | 公开，只读。`nx`、`ny` 与 `model->grid` 相同，含 PML |
| `step`、`time` | 公开，只读。`step` 是已完成的步数，`time = step·dt` |
| `px`、`py`、`vx`、`vy` | 可读。没有速度访问函数，验证测试直接读 `vx`、`vy`。测试还直接写 `px` 来设置初始脉冲，这不属于头文件承诺的接口。若自行写入，固体单元的 p 和无效面的 v 必须保持 0 |
| `kdt`、`bx`、`by`、`ax_c` … `cy_f` | 实现细节，不要修改。测试读它们来检查系数，格式可能随实现变化 |

`Solver` 拥有所有数组，不拥有 `model`。

## 函数

### `solver_init`

```c
int solver_init(Solver *s, const Model *m, FdtdError *err);
```

| 参数 | 说明 |
|---|---|
| `s` | 输出。先整体清零，再填充 |
| `m` | 已构建的 `Model`，只借用，不复制。solver 释放之前不要释放它 |
| `err` | 错误信息，可为 `NULL` |

成功返回 0。所有场变量初值为 0，系数已算好，`step = 0`，`time = 0`。失败返回 -1，此时 `*s` 为全零（已分配的内存已释放），不需要再调用 `solver_free`。

**前置条件**（调用方保证，`solver_init` 不检查，违反时行为未定义）：

- `s`、`m` 不为 `NULL`。
- `*m` 由 `model_build` 生成，在 `solver_free` 之前保持有效（不修改、不释放）。因此网格尺寸、dx、dy、dt、介质表和 `medium_id`、四个 σ 数组（长度 `nx`、`nx+1`、`ny`、`ny+1`）、边界类型枚举、声源列表及其单元下标都视为合法，solver 不重新校验。
- CFL 条件（见上）、声源时间函数是否合法、声源权重是否为有限正数同样不检查。

**`solver_init` 自己检查的内容**，任何一项失败都返回 -1：

| 条件 | 错误信息 |
|---|---|
| 面数组大小会溢出 `size_t`（在分配前检查） | `solver_init: grid %d x %d is too large` |
| 流体单元的 ρ 或 K 不是有限正数（K = ρc² 可能因取值极端而上溢或下溢） | `solver_init: medium %d has unusable density %g or bulk modulus %g` |
| σ 不是有限值 | `solver_init: %s damping %g at index %d is not usable`（`%s` 为 `x cell`、`x face`、`y cell` 或 `y face`） |

内存不足时的错误信息来自 `fdtd_calloc`。`err` 可以为 `NULL`。固体单元的介质编号不被使用。后两项由 `test_init_rejects_unusable_coefficients` 覆盖：`c = 1e-200` 使 K 下溢为 0，PML `order = 1e306` 使 σ 上溢为无穷，这两组取值都能通过 config 和 model 的校验。

### `solver_free`

```c
void solver_free(Solver *s);
```

释放所有数组并把 `*s` 清零，不释放 `model`。`s == NULL`、全零结构体和重复调用都是安全的。应在 `model_free` 之前调用。释放（或清零）之后的 solver 不能再交给下面四个函数。

### `solver_step`

```c
void solver_step(Solver *s);
```

按上文顺序推进一步，并按当前的 `m->sources`、`m->n_sources` 注入声源。前置条件：`s` 已由成功的 `solver_init` 初始化且尚未释放，`s->model` 仍然有效。`solver_step`、`solver_get_pressure`、`solver_pressure_at`、`solver_acoustic_energy` 都不检查这一点。

### `solver_get_pressure`

```c
void solver_get_pressure(const Solver *s, double *out);
```

把全网格的总压力 `p = px + py` 写入 `out`，共 `nx*ny` 个值，下标 `j*nx + i`，包含 PML 单元，固体单元为 0。前置条件：`s` 已初始化，`out` 不为 `NULL` 且长度不小于 `nx*ny`。显示物理域时用 `grid.off_x`、`grid.off_y` 截取，见 grid.md。

### `solver_pressure_at`

```c
double solver_pressure_at(const Solver *s, size_t cell);
```

返回单元 `cell` 的总压力，`cell = j*nx + i`（即 `grid_cell_index`）。前置条件：`s` 已初始化，`cell < nx*ny`。越界下标不做检查，也不返回 0.0。

### `solver_acoustic_energy`

```c
double solver_acoustic_energy(const Solver *s);
```

返回整个网格（含 PML）的声能，单位 J/m（单位深度）：

```
E = Σ_流体单元 p²/(2K) · dx·dy  +  Σ_有效面 w · ρ_face · v² / 2 · dx·dy
```

- 势能：每个流体单元 `p²/(2K)`，其中 `p = px + py`。
- 动能：每个有效面（`bx > 0` 或 `by > 0`）`ρ_face·v²/2`，其中 `ρ_face = dt/bx`（y 面用 `by`）。
- 权重 w：内部面为 1。**压力释放侧的最外侧面为 ½**，因为这些面的控制体积有一半在网格外面，只有一半在网格内。取这个权重后离散能量平衡才能精确闭合：leapfrog 前后两个半步的 p·v 交叉项只有在这个权重下才会抵消。刚性侧和 PML 侧的最外侧面 `bx = by = 0`，不参与求和。
- 返回值按当前时刻的状态计算，p 在 `t_n`，v 在 `t_{n−½}`，两者差半步，所以 E 在无损封闭域中会小幅振荡，但不漂移。精确守恒的量需要用相邻两步的 p 构造（见验证一节），不是这个函数。
- 含 PML 时，E 随波被吸收而下降；PML 内的 E 只是诊断量，不是严格的物理能量。
- 前置条件：`s` 已初始化。

## 示例

用法与 `main.c` 相同：

```c
SimConfig cfg;
Model model;
Solver solver;
FdtdError err = {{0}};

if (config_load_file("case.toml", &cfg, &err) != 0 ||
    model_build(&cfg, &model, &err) != 0) {
    fprintf(stderr, "error: %s\n", err.msg);
    config_free(&cfg);
    return 1;
}
config_free(&cfg);                       /* model 不引用 cfg 的内存 */

if (solver_init(&solver, &model, &err) != 0) {
    fprintf(stderr, "error: %s\n", err.msg);
    model_free(&model);
    return 1;
}

const Grid *g = &model.grid;
double *pressure = fdtd_calloc(grid_cell_count(g), sizeof *pressure, &err);
if (!pressure) {
    fprintf(stderr, "error: %s\n", err.msg);
    solver_free(&solver);
    model_free(&model);
    return 1;
}

size_t probe = grid_cell_index(g, g->off_x + 10, g->off_y + 10);
for (long n = 1; n <= model.n_steps; n++) {
    solver_step(&solver);
    if (n % 100 == 0) {
        solver_get_pressure(&solver, pressure);   /* nx*ny 个值，含 PML */
        printf("t = %g s  p(probe) = %g Pa  E = %g J/m\n",
               solver.time, solver_pressure_at(&solver, probe),
               solver_acoustic_energy(&solver));
        /* ... 交给 render ... */
    }
}

free(pressure);
solver_free(&solver);                    /* 先于 model_free */
model_free(&model);
```

## 内存与性能

### 内存

`solver_init` 用 `fdtd_calloc` 分配下列 `double` 数组，全部清零：

| 数组 | 元素数 |
|---|---|
| `px`、`py`、`kdt` | 各 `nx*ny` |
| `vx`、`bx` | 各 `(nx+1)*ny` |
| `vy`、`by` | 各 `nx*(ny+1)` |
| `ax_c`、`cx_c` | 各 `nx` |
| `ax_f`、`cx_f` | 各 `nx+1` |
| `ay_c`、`cy_c` | 各 `ny` |
| `ay_f`、`cy_f` | 各 `ny+1` |

每个单元有 7 个二维 `double` 数组（px、py、vx、vy、kdt、bx、by），约 56 字节/单元；一维阻尼表很小，可忽略。以 440 × 340 网格（含 PML，149600 个单元）为例，总共 1051884 个 `double`，约 8.4 MB（8.0 MiB）。`Model` 另外占用每单元 3 字节（`medium_id` 2 字节、`solid` 1 字节）和一维 σ 表。`main.c` 还要为输出分配一个 `nx*ny` 个 `double` 的压力缓冲区。

### 速度

单线程。在 AMD Ryzen 9 7845HX、GCC 16.2、Release（`-O3`）下，440 × 340 网格（149600 个单元）跑 5544 步，总用时 1.37 s，约 6×10⁸ 单元·步/秒（149600 × 5544 ÷ 1.37 s）。这个用时包含模型构建和写 1 张 PNG，纯求解速度更高，未单独测量。`fdtd2d` 结束时在 stderr 打印的 `Mcell-updates/s` 按整个时间循环计，包含取压力和渲染的时间。

测试套件：`test_solver` 共 23 项，Release 下约 1.0 s。

## 验证

`tests/test_solver.c` 用 TOML 文本生成小模型（空气，ρ = 1.21 kg/m³，c = 343 m/s，dx = 1 cm，cfl = 0.7），共 23 项测试。物理类测试在输出中以 `[measure]` 开头的行打印实测数值。下面的实测值来自这些输出，阈值来自测试代码。

### 精确守恒量

在封闭无损域（刚性侧或压力释放侧，可含障碍物）里，leapfrog 格式精确守恒下面这个量，其中 `p^{n+1}` 是 solver 当前的压力，`p^n` 是上一步的压力，v 在 `n+½`：

```
I = ( Σ p^n·p^{n+1} / (2K) + Σ_面 w · ρ_face · v² / 2 ) · dx·dy
```

w 的取法同 `solver_acoustic_energy`。测试先以声源驱动 4 个周期，关闭声源后跟踪 3000 步（基本刚性盒为 4000 步），计算 `|I/I₀ − 1|` 的最大值：

| 算例 | 说明 |
|---|---|
| 刚性封闭盒 | 51 × 41 单元 |
| 刚性盒，dx ≠ dy | 61 × 29 单元，dy = 2dx |
| 压力释放盒 | 三种边界组合，其中两种 dy = 2dx；含四边全部压力释放 |
| 含障碍物的盒 | 圆和矩形；另有压力释放侧加障碍物贴着释放壁，dy = 2dx |

偏差实测约 **5×10⁻¹⁵**，阈值 1×10⁻¹¹。dy = 2dx 的算例能发现 `inv_dx` 与 `inv_dy` 用反的错误（这些盒内 σ = 0，σx 与 σy 的区分由下面的 PML 通道测试负责）。按测试代码的注释，压力释放最外侧面的权重不取 ½，或虚单元不取 `p_ghost = −p_inside`，都会破坏这个守恒量。

同时对 `solver_acoustic_energy` 与测试中独立写的参考实现比较，相对差小于 1×10⁻¹²。E 相对关闭声源时刻 E₀ 的范围：刚性盒 [0.88, 1.12]，其他盒 [0.85, 1.15]，压力释放盒的单独检查 [0.9, 1.1]（以上均为阈值）。刚性盒中，最后 500 步与最初 500 步的平均能量之差小于 1×10⁻³，即无漂移。两种介质的封闭盒（竖直和水平界面）的偏差阈值也是 1×10⁻¹¹。

### PML

四边全为 PML，厚度 20 个单元，R = 10⁻⁶，m = 2。驱动 3 个周期后关闭声源，共 1500 步，比较最终能量与峰值能量：

| 算例 | 网格（物理域 → 含 PML） | E_end / E_peak |
|---|---|---|
| 各向同性 | 60 × 60 → 100 × 100 | 约 **6.4×10⁻⁸** |
| 各向异性，dy = 2dx | 80 × 40 → 120 × 80 | 约 **8.5×10⁻⁸** |

阈值均为 10⁻³。这是封闭盒内总能量的衰减，不是单一入射角的反射系数。

另一个测试区分 σx 与 σy：在 PML 在 x 侧、刚性壁在 y 侧（或反过来）的窄通道里（dy = 2dx），沿 x 的脉冲只有 px、vx 非零，沿 y 的脉冲只有 py、vy 非零。沿 PML 方向的脉冲被吸收（E 比 < 10⁻³）；沿刚性方向的脉冲不受阻尼，E 比保持在 1 ± 0.1 内，精确守恒量偏差 < 10⁻¹¹。另一个分量全程精确为 0。

### 对称性

61 × 61 刚性盒，声源在中间单元中心，跑 300 步（多次壁面反射），比较 x 镜像、y 镜像和转置后的场，阈值是 10⁻¹²·max|p|。对称性精确成立。

### 相速度与波前

各向同性算例：1500 Hz，201 × 201 物理域。各向异性算例：1000 Hz，dy = 2dx，201 × 101 物理域。两者 PML 厚度均为 10，声源在中心，声源带 2 个周期的渐入。用稳态下两个同射线探针的相位延迟计算数值相速度，容许误差 1 步。

| 方向 | 探针 | 相位延迟相对误差 |
|---|---|---|
| 轴向（x） | 距声源 0.3 m 和 0.9 m | 0.3% |
| 对角线 | 距声源约 0.3 m 和 0.9 m | 0.14% |
| x（dy = 2dx，1000 Hz） | 0.3 m 和 0.9 m | 0.24% |
| y（dy = 2dx，1000 Hz） | 0.3 m 和 0.9 m | 0.64% |

dy = 2dx 的算例另与 1D leapfrog 色散关系 `sin(ω·dt/2) = (c·dt/h)·sin(k·h/2)` 的预测比较（h 取沿该方向的单元尺寸），残差不超过 0.5 步。按测试代码中的注释，该预测在 1 kHz 下沿 x 约 +0.1 步，沿 y 约 +0.5 步，所以 y 方向误差较大主要来自方案自身的色散。

波前到达时间测试（无渐入的声源，10% 波前）：轴向 30 和 90 个单元、对角线 85 个单元的探针，到达时间与 `r/(c·dt)` 之差不超过 4 步。

### 压力释放

- 压力释放侧在 x_min，刚性侧在 x_max，声源居中，y 侧用 PML。紧邻两个壁面的探针的 `max|p|` 之比应等于 `tan(k·dx/2)`（入射加反射平面波，第一个单元中心距 p = 0 平面 dx/2）。实测 0.1402，解析值 0.1383，相差 +1.4%，阈值 ±5%。虚单元若放在整格之外，比值约为 0.28，会被判错。
- 在 x_min、x_max、y_min、y_max 各开一侧压力释放，其余刚性，声源位置对应镜像或转置，各场彼此一致到 10⁻¹²·max|p|。

### 其他检查

| 测试 | 内容 |
|---|---|
| 无声源 | 100 步后 px、py、vx、vy 全部精确为 0，能量为 0，`step`、`time` 正确 |
| 首步幅值 | 从静止走一步后 `Σp·dx·dy = K·dt·s(dt/2)`（相对 1e-12），px 与 py 逐点相等 |
| 障碍物保持刚性 | 固体单元的 px、py，以及相邻面的 vx、vy 全程精确为 0 |
| 系数表 | a、c 与 σ 的关系误差 < 1e-15；压力释放侧最外侧面 a = c = 1（x_max 侧与混合边界，另有四边全为压力释放的算例逐面检查）；刚性和 PML 侧最外侧面不活动 |
| 退化尺寸 | 物理域 1×1、1×9、9×1，三种边界各一，共 9 种：要么运行且场有限、能量非负，要么返回错误并给出信息 |
| 释放 | `solver_free` 对零结构体、`NULL`、重复调用（含对已初始化的 solver 连续两次）的行为：不崩溃，结构体被清零 |

### 尚未覆盖

没有与解析解比较绝对幅值。`tests/test_validation.c` 目前是占位程序（返回 77，CTest 视为跳过）。计划的验证包括：自由场点源与 `(ωρQ/4)·H₀⁽²⁾(kr)` 比较，两侧刚性壁之间线源产生的平面波与解析幅值比较。

## 限制与扩展方式

**限制**

- 单线程。
- PML 是分裂场形式，不是 CPML。
- 没有速度读取函数，调用方只能直接读 `vx`、`vy`。
- 外边界只有刚性、压力释放和 PML，没有阻抗边界。
- `solver_init` 不检查 CFL，`solver_step` 不检查发散。
- solver 不校验 `Model` 的内容，也不检查 `solver_step` 等函数的参数，这些前置条件由 `model_build` 和调用方保证。
- 固体表面是阶梯状的。固体伸进 PML 会反射波，model 对此发出警告。
- `solver_acoustic_energy` 混用 `t_n` 的 p 和 `t_{n−½}` 的 v，不是严格守恒量。在 PML 中不代表物理能量。

**扩展方式**

- **多介质**：不需要改 solver，见“多介质预留”。
- **新的边界类型**（例如阻抗边界）：在 `compute_coefficients` 中为最外侧面设置 `bx`、`by` 和阻尼系数，在 `update_release_faces` 旁增加相应的更新，并在 `solver_acoustic_energy` 中给出该类最外侧面的权重。
- **速度源**：需要在 `ModelSource` 中记录源的种类和作用面，并在 `solver_step` 的第 1 步之后注入速度。
- **并行**：同一阶段内各行的写入互不冲突，按 j 分行并行是可行的，目前没有实现。
- **速度读取函数**：增加类似 `solver_get_vx`/`solver_get_vy` 的函数即可，场数组的布局见 grid.md。
