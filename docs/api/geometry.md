# geometry：二维形状

头文件：`src/geometry.h`　实现：`src/geometry.c`　依赖：common

模型中所有几何对象共用同一个 `Shape` 类型。本模块只做纯几何计算，不涉及网格；把形状映射到网格单元的工作由 model（障碍物）和 source（声源）完成。

| 用途 | 允许的形状 | 版本 |
|---|---|---|
| 刚性障碍物（计算域内部的边界） | rect、circle、polygon | v1 |
| 点声源 | point | v1 |
| 线声源 | segment | v1 |
| 介质区域 | rect、circle、polygon | 预留 |
| 辐射面声源 | 面状形状 | 预留 |

## 类型

### `Vec2`

```c
typedef struct Vec2 { double x, y; } Vec2;
```

物理坐标，单位 m。

### `ShapeType`

```c
typedef enum ShapeType {
    SHAPE_POINT = 0, SHAPE_SEGMENT, SHAPE_RECT, SHAPE_CIRCLE, SHAPE_POLYGON,
    SHAPE_TYPE_COUNT
} ShapeType;
```

`SHAPE_TYPE_COUNT` 不是有效类型，只用于遍历和数组大小。

### `Shape`

```c
typedef struct Shape {
    ShapeType type;
    union {
        struct { Vec2 position; } point;
        struct { Vec2 start, end; } segment;
        struct { Vec2 min, max; } rect;             /* 轴对齐 */
        struct { Vec2 center; double radius; } circle;
        struct { Vec2 *vertices; int n; } polygon;  /* owned，n >= 3 */
    } u;
} Shape;
```

带类型标签的联合体，按 `type` 访问对应的成员。

| 类型 | TOML 写法 | 字段要求 |
|---|---|---|
| point | `{ type = "point", position = [x, y] }` | 坐标有限 |
| segment | `{ type = "segment", start = [x, y], end = [x, y] }` | 长度 > 0 |
| rect | `{ type = "rect", min = [x, y], max = [x, y] }` | 两个方向上都满足 max > min |
| circle | `{ type = "circle", center = [x, y], radius = r }` | r > 0 |
| polygon | `{ type = "polygon", vertices = [[x, y], ...] }` | 至少 3 个顶点，有向面积 ≠ 0；顶点顺序不限，首尾不必重复 |

**所有权**：只有 polygon 持有堆内存（`vertices`），必须用 `shape_free()` 释放。直接用 `=` 复制 polygon 是浅拷贝，会导致重复释放；需要副本时用 `shape_copy()`。

## 函数

### `shape_type_name` / `shape_type_from_name`

```c
const char *shape_type_name(ShapeType type);
int shape_type_from_name(const char *name, ShapeType *out);
```

在枚举和 TOML 类型名之间转换，类型名为 `"point" "segment" "rect" "circle" "polygon"`，区分大小写。`shape_type_name` 的 `type` 必须是有效枚举值（小于 `SHAPE_TYPE_COUNT`）。`shape_type_from_name` 的 `name` 不能为 `NULL`；匹配成功时返回 0 并写入 `*out`，未知名称返回 -1，此时不修改 `*out`。

### `shape_is_area`

```c
bool shape_is_area(const Shape *s);
```

rect、circle、polygon 返回 true，point 和 segment 返回 false。

### `shape_contains`

```c
bool shape_contains(const Shape *s, Vec2 p);
```

判断点 `p` 是否在面状形状内。

| 类型 | 规则 | 边界上的点 |
|---|---|---|
| rect | `min.x ≤ x ≤ max.x` 且 `min.y ≤ y ≤ max.y` | 算作内部 |
| circle | `|p − center|² ≤ r²` | 算作内部 |
| polygon | 奇偶规则（射线交点计数），支持凹多边形；自交多边形按奇偶规则处理 | 不确定 |
| point、segment | 恒为 false | — |

model 用它对网格单元中心做判断：单元中心在障碍物内部的单元即为固体单元。

### `shape_bbox`

```c
void shape_bbox(const Shape *s, Vec2 *min, Vec2 *max);
```

返回轴对齐包围盒。point 的包围盒退化为一个点。用于把栅格化限定在包围盒覆盖的单元范围内。

### `shape_validate`

```c
int shape_validate(const Shape *s, FdtdError *err);
```

检查所有坐标是有限值、形状不退化，规则见上方 `Shape` 表。失败时返回 -1，错误信息不含位置，例如 `circle radius must be > 0 (got -1)`。调用方（如 config）负责加上文件行号和键路径前缀。

### `shape_copy`

```c
int shape_copy(Shape *dst, const Shape *src, FdtdError *err);
```

深拷贝，polygon 的顶点会复制一份。`dst` 与 `src` 不能是同一对象。成功返回 0；分配失败返回 -1，此时 `dst` 中 polygon 的 `n == 0`、`vertices == NULL`，可以安全地 `shape_free`。

### `shape_free`

```c
void shape_free(Shape *s);
```

释放 polygon 顶点，并把 `*s` 重置为全零（即位于原点的 point）。可作用于全零结构体，可重复调用，`s == NULL` 时什么都不做。

### `geom_segment_length_in_box`

```c
double geom_segment_length_in_box(Vec2 a, Vec2 b, Vec2 bmin, Vec2 bmax);
```

用 Liang–Barsky 算法求线段 `[a, b]` 落在闭矩形 `[bmin, bmax]` 内的长度，没有重叠时返回 0。source 用它计算线声源在每个网格单元内的长度。

注意：矩形是闭区间，所以恰好落在两个相邻单元公共边上的线段会被两个单元**都**完整计入。需要把线段划分到各单元的调用方必须做归一化；source 的做法是按总长度缩放各单元的长度。

### `geom_clip_segment`

```c
bool geom_clip_segment(Vec2 *a, Vec2 *b, Vec2 bmin, Vec2 bmax);
```

把线段就地裁剪到闭矩形内。仍有剩余部分时返回 true，剩余部分可能退化为一个点；完全在矩形外时返回 false，且不修改 `*a`、`*b`。

## 示例

```c
Vec2 tri[3] = {{2.3, 0.0}, {2.7, 0.0}, {2.5, 0.5}};
Shape s = { .type = SHAPE_POLYGON };
s.u.polygon.vertices = malloc(sizeof tri);   // 由 s 持有
memcpy(s.u.polygon.vertices, tri, sizeof tri);
s.u.polygon.n = 3;

FdtdError err = {{0}};
if (shape_validate(&s, &err) == 0 && shape_contains(&s, (Vec2){2.5, 0.2})) {
    /* 点在三角形内 */
}
shape_free(&s);
```

## 扩展方式

新增形状（如旋转矩形、椭圆）的步骤：
1. 在 `ShapeType` 中加入新枚举值，在联合体中加入对应成员。
2. 在 `k_shape_names` 中加入 TOML 类型名。
3. 在 `shape_is_area`、`shape_contains`、`shape_bbox`、`shape_validate` 中加入对应分支；如有堆内存，同时修改 `shape_copy` 和 `shape_free`。
4. 在 config 中解析新形状的字段。

各 switch 语句都没有 `default` 分支，漏写某个枚举分支时编译器会给出 `-Wswitch` 警告。
