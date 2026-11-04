# source：声源空间离散

头文件：`src/source.h`　实现：`src/source.c`　依赖：geometry、grid、waveform、common

本模块负责把声源形状离散到网格上，得到声源覆盖的单元及每个单元的权重。声源由空间分布和时间函数相乘得到：空间分布由形状决定，时间函数由 waveform 模块提供。

| 形状 | 声源类型 | 版本 |
|---|---|---|
| point | 点源 | v1 |
| segment | 线源（直线段） | v1 |
| rect、circle、polygon | 面状辐射源 | 预留，v1 中报错 |

## 物理模型

v1 的声源都是单极子体积注入源（软源），以源项的形式加入压力方程：

```
∂p/∂t = −K ∇·v + K · q(x, t),      K = ρc²
q(x, t) = s(t) · D(x)              单位 1/s（单位时间单位面积注入的体积，按单位深度计）
```

s(t) 是时间函数，见 waveform 模块。D(x) 是空间分布：

| 类型 | D(x) | 幅值 A 的含义 | A 的单位 |
|---|---|---|---|
| 点源 | δ(x − x_s) | 单位深度的体积速度 Q | m²/s |
| 线源 | 沿线段的线 δ 分布 | 单位长度、单位深度的体积速度 Q′ | m/s |

软源的意思是把源项叠加到压力上，而不是强行设定压力值。因此散射波经过声源位置时不会被反射。

参考解：二维自由场中，简谐点源 `Q·e^{iωt}` 的远场为

```
p(r) = (ωρQ/4) · H₀⁽²⁾(kr)
```

其中 H₀⁽²⁾ 是第二类零阶汉克尔函数，对应 `e^{iωt}` 时间约定。验证测试将用这个式子校核幅值，但这项测试尚未实现，计划在求解器完成后补上。

由此可以估算点源在距离 r 处的声压幅值：`|p| ≈ (ωρQ/4)·√(2/(πkr))`。以 2 kHz 空气中的点源为例，Q = 2×10⁻³ m²/s 时，r = 1 m 处 |p| ≈ 1 Pa。

## 离散方式

网格上的 D(x) 用一组单元权重 w_c 表示，单位 1/m²，满足：

| 类型 | 归一化条件 |
|---|---|
| 点源 | Σ w_c·dx·dy = 1 |
| 线源 | Σ w_c·dx·dy = 线段（裁剪后、扣除固体部分后）的长度 [m] |

满足这些条件后，总源强度与网格尺寸无关，网格加密时结果收敛。solver 每一步对声源单元做如下更新：`p_c += K_c·dt·w_c·s(t_{n+½})`。

### 点源

1. **位置检查**：点必须位于闭物理域 `[x0, x1] × [y0, y1]` 内，否则报错。
2. **固体检查**：点所在的物理单元若是固体，报错（`point ... lies inside a rigid obstacle`）。
3. **双线性权重**：把源分配到最近的 4 个物理单元中心上。靠近物理域边缘时，下标截断到物理域范围内，所以权重不会落进 PML 单元，源强度与 PML 厚度无关。例如：
   - 点在物理域角点上时，只占 1 个单元；
   - 点在物理域边上时，占 2 个单元。
4. **剔除与归一化**：去掉权重小于 1e-12 的单元和固体单元，其余单元的权重重新归一化，使总和为 1。
5. **点恰在单元中心**：结果只含这一个单元。

### 线源

1. **裁剪**：把线段裁剪到物理域内。若长度因此变短，发出警告；若裁剪后什么都不剩，或只剩一个点，报错。线段长度必须大于 0，这一点由 `shape_validate` 保证，这里不再检查。
2. **分配长度**：逐列遍历线段经过的物理单元。每一列只检查线段实际穿过的那几行，所以计算量和内存都与线段长度成正比，不随包围盒面积增长。对每个单元，用 Liang–Barsky 算法计算线段落在其中的长度。即使线段恰好落在物理域的边界线上，也只分配给物理单元，不分给 PML 单元。
3. **归一化**：把各单元的长度按同一比例缩放，使总和等于裁剪后的线段长度。这样，恰好落在格线上的线段不会被相邻的两个单元重复计入。
4. **固体剔除**：去掉落在固体单元上的部分，并发出警告，说明去掉的长度占总长的百分比。若全部落在固体上，报错。
5. **辐射特性**：线段上各点等幅同相，作为单极子分布向两侧辐射。

## 类型

### `ModelSource`

```c
typedef struct ModelSource {
    ShapeType shape_type;   /* 来源形状类型 */
    Signal signal;          /* 时间函数（按值复制） */
    size_t n_cells;
    size_t *cells;          /* owned [n_cells]，单元下标（grid_cell_index），互不重复，均为流体单元 */
    double *weights;        /* owned [n_cells]，权重 [1/m²]，> 0 */
} ModelSource;
```

## 函数

### `source_discretize`

```c
int source_discretize(const Shape *shape, const Signal *signal, const Grid *g,
                      const uint8_t *solid, ModelSource *out, FdtdError *err);
```

| 参数 | 说明 |
|---|---|
| `shape` | 非 `NULL`，声源形状，已通过 `shape_validate`。v1 只接受 point 和 segment |
| `signal` | 非 `NULL`，时间函数，已通过 `signal_validate`，会被复制到 `out->signal` |
| `g` | 非 `NULL`，网格，要求 `dx > 0`、`dy > 0`、`nx ≥ 1`、`ny ≥ 1` |
| `solid` | 固体掩码 `[nx*ny]`，非 0 表示固体；可为 `NULL`，表示没有障碍物 |
| `out` | 非 `NULL`，输出，会被先清零 |

本函数不检查指针是否为 `NULL`，也不检查网格和形状本身是否合法。这些由调用方保证：`model_build()` 传入的网格是自己构造的，形状和信号来自 `config_load_*` 的校验结果。

成功返回 0。失败返回 -1，此时 `*out` 为全零。可能的错误（只涉及形状与网格的相对位置）：

| 情况 | 错误信息（示例） |
|---|---|
| 点在物理域外 | `point (5, 1) is outside the physical domain [0, 2] x [0, 1.5]` |
| 点在障碍物内 | `point (1.2, 0.75) lies inside a rigid obstacle` |
| 线段与物理域不相交 | `segment (...)-(...) does not cross the physical domain ...` |
| 线段完全落在固体上 | `segment (...)-(...) lies on rigid cells only` |
| 点、线段以外的形状（rect、circle、polygon） | `circle source is not supported in this version` |

model 调用时会在错误信息前加上 `source[k]: `。

### `model_source_free`

```c
void model_source_free(ModelSource *s);
```

释放并清零。可作用于全零结构体，可重复调用。

## 扩展方式

- **面状辐射源**：在 `source_discretize` 中为 rect、circle 和 polygon 实现离散。一种做法是体积分布，权重取单元面积分数除以面积；另一种是沿形状边界分布，相当于振动表面。此外，config 中关于面状声源的错误检查需要放开。
- **速度源（活塞，单侧辐射）**：需要在 `ModelSource` 中加入源的种类和作用的面，solver 中还要增加速度注入。
- **线源上的幅值或相位分布**（用于阵列或聚焦）：可以给每个单元加一个复数系数，或者为每个单元单独指定时间延迟。
