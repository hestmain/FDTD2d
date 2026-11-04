# grid：交错网格布局

头文件：`src/grid.h`（只有头文件，全部是 `static inline` 函数）　依赖：C 标准库、libm

本模块定义计算网格的尺寸、物理坐标与数组下标之间的换算，是 model、source、solver 和 main 共同遵守的布局约定。

## 网格结构

计算网格由物理域和 PML 层组成，PML 层加在物理域**外侧**，因此用户设置的物理域尺寸不会被 PML 占用。

```
 j
 ▲   ┌────────────── PML (y_max) ──────────────┐
 │   │ PML  ┌──────────────────────────┐   PML  │
 │   │(x_min)│       物理域              │(x_max) │
 │   │      │  nx_phys × ny_phys 单元    │        │
 │   │      │  左下角 = (x0, y0)         │        │
 │   │      └──────────────────────────┘        │
 │   └────────────── PML (y_min) ──────────────┘
 └──────────────────────────────────────────────► i
     off_x = x_min 侧 PML 厚度；off_y = y_min 侧 PML 厚度
```

不是 PML 的一侧厚度为 0，此时物理域边缘就是计算网格边缘。

交错网格（Yee 网格）上三类量的存放位置：

| 量     | 位置       | 数组长度        | 下标             | 说明                    |
| ----- | -------- | ----------- | -------------- | --------------------- |
| 声压 p  | 单元中心     | `nx*ny`     | `j*nx + i`     |                       |
| 速度 vx | x 面（竖直面） | `(nx+1)*ny` | `j*(nx+1) + i` | 面 i 位于单元 i−1 与单元 i 之间 |
| 速度 vy | y 面（水平面） | `nx*(ny+1)` | `j*nx + i`     | 面 j 位于单元 j−1 与单元 j 之间 |

以单元 (i, j) 为例：

```
              vy(i, j+1)
          ┌──────●──────┐
          │             │
 vx(i,j)  ●    p(i,j)   ●  vx(i+1,j)
          │             │
          └──────●──────┘
              vy(i, j)
```

所有数组都按行优先存储，x 方向变化最快。

## 类型

### `Grid`

```c
typedef struct Grid {
    int nx, ny;           /* 总单元数（含 PML） */
    int nx_phys, ny_phys; /* 物理域单元数 */
    int off_x, off_y;     /* 第一个物理单元的下标 = x_min / y_min 侧 PML 厚度 */
    double dx, dy;        /* 单元尺寸 [m] */
    double x0, y0;        /* 物理域左下角 [m] */
} Grid;
```

由 `model_build()` 填充，其他模块只读。不变量：`nx = off_x + nx_phys + （x_max 侧 PML 厚度）`，y 方向同理。

## 函数

### 数量与下标

| 函数 | 返回 |
|---|---|
| `size_t grid_cell_count(const Grid *g)` | `nx*ny` |
| `size_t grid_xface_count(const Grid *g)` | `(nx+1)*ny` |
| `size_t grid_yface_count(const Grid *g)` | `nx*(ny+1)` |
| `size_t grid_cell_index(const Grid *g, int i, int j)` | `j*nx + i` |
| `size_t grid_xface_index(const Grid *g, int i, int j)` | `j*(nx+1) + i` |
| `size_t grid_yface_index(const Grid *g, int i, int j)` | `j*nx + i` |

下标函数不检查越界，调用方必须保证 i、j 在合法范围内。乘法用 `size_t` 完成，不会发生 int 溢出。

### 坐标

| 函数                          | 返回（单位 m）                                       |
| --------------------------- | ---------------------------------------------- |
| `grid_cell_x(g, i)`         | 单元 i 中心的 x 坐标：`x0 + (i − off_x + 0.5)·dx`      |
| `grid_cell_y(g, j)`         | 单元 j 中心的 y 坐标                                  |
| `grid_face_x(g, i)`         | x 面 i 的 x 坐标：`x0 + (i − off_x)·dx`             |
| `grid_face_y(g, j)`         | y 面 j 的 y 坐标                                   |
| `grid_x1(g)` / `grid_y1(g)` | 物理域右上角坐标：`x0 + nx_phys·dx` / `y0 + ny_phys·dy` |

PML 单元的坐标落在物理域之外，因此 x0 左侧会出现小于 x0 的值。

### `grid_locate`

```c
bool grid_locate(const Grid *g, double x, double y, int *i, int *j);
```

找出包含物理点 (x, y) 的单元，结果总会被截断到 `[0, nx−1] × [0, ny−1]` 以内。点在整个计算网格（物理域加 PML）之内时返回 true。恰好落在面上的点归入右侧或上侧的单元。

### `grid_cell_is_physical`

```c
bool grid_cell_is_physical(const Grid *g, int i, int j);
```

单元 (i, j) 属于物理域时返回 true，属于 PML 时返回 false。

## 示例

```c
/* 遍历物理域内的单元，读取声压 */
for (int j = g->off_y; j < g->off_y + g->ny_phys; j++) {
    for (int i = g->off_x; i < g->off_x + g->nx_phys; i++) {
        double x = grid_cell_x(g, i), y = grid_cell_y(g, j);
        double p = pressure[grid_cell_index(g, i, j)];
        /* ... */
    }
}
```

## 限制

- 网格均匀；dx 和 dy 分开存储，允许不相等。dx ≠ dy 时，渲染结果中的单元仍画成正方形像素，图像会在一个方向上被拉伸。
- 单方向单元数受 `int` 范围限制，model 会检查 `nx+1` 和 `ny+1` 不溢出。
