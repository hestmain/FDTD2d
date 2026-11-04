# boundary：外边界条件与 PML 参数

头文件：`src/boundary.h`　实现：`src/boundary.c`　依赖：common、libm

本模块定义计算域四条外边的边界类型和 PML 参数，只包含枚举、名称转换和参数校验。各边界条件的数值实现在 solver 中，PML 衰减剖面由 model 计算。计算域内部的刚性边界（障碍物）由 geometry 和 model 处理，不在本模块范围内。

## 边界类型

| 枚举 | TOML 名称 | 物理含义 | 数值实现 |
|---|---|---|---|
| `BC_RIGID` | `"rigid"` | 刚性壁：法向质点速度为 0 | 最外侧面上的法向速度恒为 0 |
| `BC_PRESSURE_RELEASE` | `"pressure_release"` | 压力释放：边界上 p = 0（如理想的水-空气界面） | 镜像虚单元取 p_ghost = −p_inside，再正常更新最外侧面的速度 |
| `BC_PML` | `"pml"` | 吸收边界，模拟无限远场 | 在物理域外附加 `thickness` 个单元的分裂场 PML，PML 最外侧为刚性 |

四条边分别设置。v1 中 TOML 里未写的边默认为 `"pml"`。

## 类型

### `BoundaryType`

```c
typedef enum BoundaryType { BC_RIGID = 0, BC_PRESSURE_RELEASE, BC_PML, BC_TYPE_COUNT } BoundaryType;
```

### `Side`

```c
typedef enum Side { SIDE_X_MIN = 0, SIDE_X_MAX, SIDE_Y_MIN, SIDE_Y_MAX, SIDE_COUNT } Side;
```

用作 `boundary[SIDE_COUNT]`、`pml_cells[SIDE_COUNT]` 等数组的下标。

### `PmlParams`

```c
typedef struct PmlParams {
    int thickness;     /* 单元数，≥ 1，默认 20 */
    double reflection; /* 理论法向入射反射系数 R，0 < R < 1，默认 1e-6 */
    double order;      /* 多项式剖面指数 m，≥ 0，默认 2 */
} PmlParams;
```

所有设为 PML 的边共用这一组参数。默认值由宏 `PML_DEFAULT_THICKNESS`、`PML_DEFAULT_REFLECTION`、`PML_DEFAULT_ORDER` 给出。

model 据此计算衰减剖面（详见 model 文档）：

```
σ(d) = σ_max · (d / L)^m,   σ_max = −(m + 1) · c_ref · ln(R) / (2L)
```

其中 d 是从物理域边缘向 PML 内部的深度，L = thickness × dx（x 方向）或 thickness × dy（y 方向）。

参数选择建议：
- `thickness` 取 10 到 30 层较常见。层数越多吸收越好，但计算量也越大。
- `reflection` 是连续介质中的理论值。离散网格上的实际反射会更大，主要来自剖面过陡造成的数值反射。
- `order` 取 2 到 3 较常见。

## 函数

### `boundary_type_name` / `boundary_type_from_name`

```c
const char *boundary_type_name(BoundaryType type);
int boundary_type_from_name(const char *name, BoundaryType *out);
```

在枚举和 TOML 名称之间转换，名称区分大小写。`boundary_type_name` 的 `type` 必须是有效枚举值（小于 `BC_TYPE_COUNT`）。`boundary_type_from_name` 的 `name` 不能为 `NULL`，用法与 `shape_type_from_name` 相同：成功返回 0，未知名称返回 -1。

### `side_name`

```c
const char *side_name(Side side);
```

返回 `"x_min"`、`"x_max"`、`"y_min"` 或 `"y_max"`，与 TOML `[boundary]` 表中的键名一致。`side` 必须是有效枚举值（小于 `SIDE_COUNT`）。

### `pml_params_validate`

```c
int pml_params_validate(const PmlParams *p, FdtdError *err);
```

校验规则：
- `thickness ≥ 1`
- `0 < reflection < 1`，且为有限值
- `order ≥ 0`，且为有限值

失败返回 -1，错误信息例如 `pml reflection must satisfy 0 < R < 1 (got 0)`。

## TOML 示例

```toml
[boundary]
x_min = "pml"
x_max = "pml"
y_min = "rigid"
y_max = "pressure_release"

[boundary.pml]
thickness  = 20
reflection = 1e-6
order      = 2
```

## 扩展方式

以后可以增加周期边界或阻抗边界（按反射系数或表面阻抗设置）。步骤：
1. 在 `BoundaryType` 和 `k_bc_names` 中加入新类型。
2. 在 solver 中实现最外侧面的更新规则。
3. 如需额外参数，按 `[boundary.pml]` 的方式增加一个子表。
