# render：热力图 PNG 输出

头文件：`src/render.h`　实现：`src/render.c`　依赖：common、libpng、libm

本模块把单元中心上的标量场（声压）离线绘成热力图，写成 PNG 文件。模块与网格无关，不认识 `Model` 和 `Grid`：调用方传入按行优先存储的数组、可选的固体掩码和一个裁剪窗口，模块只负责着色和写文件。v1 只做离线输出，没有实时显示。

模块不依赖 model 和 solver。实现中用到 POSIX 接口（`fstat`、`lstat`、`fileno`），因此只能在 POSIX 系统上构建。

## 图像几何

输入数组的布局与 [grid.md](grid.md) 一致：行优先，x 方向变化最快，单元 `(i, j)` 的下标为 `j*nx + i`。

图像的 y 轴向上：裁剪窗口中 `j = j0 + h - 1` 的单元行是图像最上面一行，`j = j0` 的单元行是最下面一行。x 轴向右。每个单元绘成 `pixel_scale × pixel_scale` 个像素。

```
图像宽度  = w · pixel_scale
图像高度  = h · pixel_scale

图像第 r 行（从上往下，r 从 0 起）  →  单元行 j = j0 + h − 1 − ⌊r / pixel_scale⌋
图像第 c 列（从左往右，c 从 0 起）  →  单元列 i = i0 + ⌊c / pixel_scale⌋
```

裁剪窗口 `[i0, i0+w) × [j0, j0+h)`（`w, h ≥ 1`）必须完全落在 `nx × ny` 数组内，这是调用方保证的前置条件，渲染器不检查（见“前置条件”）。窗口之外的单元不进入图像，也不参与统计。

图像宽度和高度都不能超过 65535（PNG 的上限），超过时报错，见“错误处理”。

### 示例

`nx = 4`、`ny = 3`，窗口 `i0 = 1, j0 = 1, w = 2, h = 2`，`pixel_scale = 3`。图像为 6 × 6 像素：

| 图像行 r | 单元行 j | 图像列 c | 单元列 i |
|---|---|---|---|
| 0, 1, 2 | 2 | 0, 1, 2 | 1 |
| 3, 4, 5 | 1 | 3, 4, 5 | 2 |

即左上 3 × 3 像素是单元 (1, 2)，右上是 (2, 2)，左下是 (1, 1)，右下是 (2, 1)。

## 色表

色表是蓝-白-红发散色表，以 0 为对称中心。调用方给出的数值先除以颜色上限 `limit`，得到 `t = v / limit`，再由 `render_colormap` 转成颜色。

| 单元值 | 颜色 |
|---|---|
| `v ≤ −limit` | 深蓝 (5, 48, 97) |
| `v = 0` | 近白 (247, 247, 247) |
| `v ≥ +limit` | 深红 (103, 0, 31) |

中间用分段线性插值。9 个控制点（`k_colormap`）如下：

| t | R | G | B |
|---|---|---|---|
| −1 | 5 | 48 | 97 |
| −0.75 | 33 | 102 | 172 |
| −0.5 | 67 | 147 | 195 |
| −0.25 | 146 | 197 | 222 |
| 0 | 247 | 247 | 247 |
| 0.25 | 244 | 165 | 130 |
| 0.5 | 214 | 96 | 77 |
| 0.75 | 178 | 24 | 43 |
| 1 | 103 | 0 | 31 |

`render_colormap(t, rgb)` 的计算步骤：

1. `t` 为 NaN 时取 0；`t < −1` 取 −1；`t > 1` 取 1。因此 ±∞ 会被截断到两端颜色，NaN 得到 0 对应的颜色。
2. `s = (t + 1) · 4`，段号 `k = min((int)s, 7)`，段内位置 `f = s − k`。
3. 每个通道 `rgb[c] = (uint8_t)(a + f·(b − a) + 0.5)`，其中 a、b 是第 k 个和第 k+1 个控制点的通道值。加 0.5 后截断，等于四舍五入。

例如 `t = 0.125`：`s = 4.5`，`k = 4`，`f = 0.5`，R = 247 + 0.5·(244 − 247) = 245.5，加 0.5 后取整得 246；G = 206；B = 189。

### 颜色上限

`RenderOptions.color_limit` 决定上限：

| `color_limit` | 实际使用的 `limit` |
|---|---|
| `> 0` | 固定上限，`limit = color_limit` |
| `≤ 0`（含 `−∞`） | 自动：`limit = max|v|`，范围是裁剪窗口内非固体、有限的单元；若该值为 0，或者没有这样的单元，则 `limit = 1` |
| NaN 或 `+∞` | 不允许（前置条件），渲染器不检查，结果未定义 |

固定上限下，超出 ±limit 的值显示为两端颜色（截断），统计值 `vmin`、`vmax` 仍是真实范围。自动上限只取决于当前这一帧，不同帧的颜色不能直接比较；要比较多帧，用固定上限。

### 特殊单元

每个像素的颜色按以下优先级决定：

1. 轮廓线（见下节）。
2. 固体单元：掩码 `mask[...]` 非 0 的单元，画成灰色 `(RENDER_SOLID_R, RENDER_SOLID_G, RENDER_SOLID_B) = (128, 128, 128)`。固体单元的值不参与颜色计算，也不参与统计和自动上限，即使值是 NaN 或很大的数。
3. 有限值：按 `t = v / limit` 查色表。
4. 非有限值（NaN、±∞）：画成 `t = 0` 的颜色 (247, 247, 247)，不参与统计和自动上限。

注意非有限值显示为 `t = 0` 的颜色，与 `render_colormap` 对 ±∞ 的处理（截断到端点颜色）不同：渲染时先用 `isfinite` 判断，不把 ±∞ 交给色表。

## 轮廓线

`RenderOptions.outline` 为 true 时，在一个单元矩形 `[outline_i0, outline_i0+outline_w) × [outline_j0, outline_j0+outline_h)` 的边界上画 1 像素宽的线，颜色为 `(RENDER_OUTLINE_R, RENDER_OUTLINE_G, RENDER_OUTLINE_B) = (0, 0, 0)`。矩形与数组使用同一套单元下标，不是相对裁剪窗口的下标。

语义：

- 线画在矩形边界**内侧**的一圈像素上，不会画到矩形外面。矩形的像素范围（含端点）为：

  ```
  x 方向  ox0 = (outline_i0 − i0) · s                         ox1 = (outline_i0 + outline_w − i0) · s − 1
  y 方向  oy0 = (h + j0 − outline_j0 − outline_h) · s          oy1 = (h + j0 − outline_j0) · s − 1
  ```

  其中 s 是 `pixel_scale`，`oy0` 是图像上边，`oy1` 是图像下边。四条边就是 `x = ox0`、`x = ox1`、`y = oy0`、`y = oy1`。
- 超出图像的部分被裁掉，只画落在图像内的边。例如矩形的右边和下边在窗口之外时，只会看到左边和上边。
- 矩形与裁剪窗口不相交，或者 `outline_w ≤ 0`、`outline_h ≤ 0` 时，轮廓线被忽略，不报错。
- 轮廓线盖在固体单元和场的颜色之上。
- 一个 1 × 1 单元的矩形在 `pixel_scale = 3` 时是 3 × 3 像素的方框，中心像素保持原色。若矩形恰好覆盖整个窗口，轮廓线画在图像最外圈。

### 命令行如何使用

`main.c` 里，`show_pml = true` 时，裁剪窗口取整个计算网格（含 PML），并把物理域画成轮廓线：

```c
field.i0 = 0;  field.j0 = 0;  field.w = g->nx;  field.h = g->ny;
ropt.outline = g->nx != g->nx_phys || g->ny != g->ny_phys;
ropt.outline_i0 = g->off_x;      ropt.outline_j0 = g->off_y;
ropt.outline_w  = g->nx_phys;    ropt.outline_h  = g->ny_phys;
```

只有当网格实际带有 PML 时（`nx != nx_phys` 或 `ny != ny_phys`）才画黑框。四边都不是 PML 时，`show_pml = true` 也不画框。`show_pml = false` 时，窗口取物理域 `(off_x, off_y, nx_phys, ny_phys)`，不画轮廓线。

## PNG 格式

| 项目 | 值 |
|---|---|
| 色深与类型 | 8 位 RGB（`PNG_COLOR_TYPE_RGB`），无 alpha |
| 隔行扫描 | 无（`PNG_INTERLACE_NONE`） |
| 压缩 | zlib 压缩级别 3（代码常量 `PNG_ZLIB_LEVEL`，不可配置） |
| 文本块 `Comment` | `RenderOptions.comment` 非 `NULL` 时写入（空字符串也会写入）；不压缩的 tEXt |
| 文本块 `Software` | 总是写入，内容为 `"fdtd2d " FDTD2D_VERSION`，当前是 `fdtd2d 0.1.0` |

写入按行流式进行：每个单元行着色一次，展开成 `pixel_scale` 个相同的图像行后逐行交给 libpng。渲染器自己只分配两个行缓冲，各 `width × 3` 字节（基础行和画轮廓线用的临时行），所以内存占用与图像宽度成正比，与高度无关。libpng 和 zlib 内部的缓冲不计入。

libpng 的警告被忽略，错误通过 `setjmp`/`longjmp` 转成返回值 -1，见下节。

## 前置条件

渲染器只在信任边界处校验一次：`render_field_png` 只由 `main.c` 调用，传入的裁剪窗口由网格算出，`[output]` 的值已由配置层校验（`pixel_scale` 在 1 到 16，`color_limit` 为有限正数或 `auto`）。因此下列条件由调用方保证，渲染器不再检查，违反时行为未定义（可能崩溃或越界读写）：

- `path`、`field`、`field->values`、`opt` 不为 `NULL`。`stats`、`err`、`field->mask`、`opt->comment` 可以为 `NULL`。
- `nx ≥ 1`、`ny ≥ 1`，裁剪窗口 `w ≥ 1`、`h ≥ 1`、`i0 ≥ 0`、`j0 ≥ 0`、`i0 + w ≤ nx`、`j0 + h ≤ ny`。
- `values` 和 `mask`（若给出）至少有 `nx × ny` 个元素。
- `pixel_scale ≥ 1`。
- `color_limit` 为有限值，或 `≤ 0`（含 `−∞`，表示自动上限）。NaN 和 `+∞` 不允许。

## 错误处理

成功返回 0，失败返回 -1，并把单行信息写入 `err`（`err` 可以为 `NULL`）。

渲染器自己负责检查的输入限制只有一项，在打开文件之前完成，此时不会碰 `path` 处的文件：

| 情况 | 错误信息 |
|---|---|
| 图像宽或高超过 65535 | `render: image size %lld x %lld pixels exceeds the PNG limit of 65535 (reduce pixel_scale or the crop window)` |

图像尺寸用 `long long` 计算，`w · pixel_scale` 不会因 `int` 溢出而漏检。`main.c` 在构建模型后有一份同样的提前检查（尽早失败，不写任何文件），渲染器这一份是它自己的限制，不依赖调用方。`color_limit = −∞` 不是错误，按 `≤ 0` 处理为自动上限。空路径不单独检查，会在 `fopen` 处失败。

打开文件之后的错误：

| 情况 | 错误信息 |
|---|---|
| 内存分配失败 | `out of memory allocating %zu x %zu bytes`（或 `allocation size overflow ...`），来自 `fdtd_calloc` |
| `fopen(path, "wb")` 失败（目录不存在、权限不足、`path` 是目录等） | `cannot write '<path>': <strerror(errno)>` |
| 创建 libpng 写入结构失败 | `cannot write '<path>': out of memory creating PNG writer` |
| libpng 报错（写入失败等） | `cannot write PNG '<path>': <libpng 的信息>`，libpng 的信息最长保留 255 个字符 |
| `fclose` 失败（缓冲数据在关闭时才写出，磁盘满时常见） | `cannot write PNG '<path>': <strerror(errno)>` |

libpng 的错误信息文字由所用的 libpng 版本决定，本文档不列出。`fclose` 的错误只在前面各步都成功时才报告，不会覆盖更早的错误信息。

### 失败时的文件处理

`fopen(path, "wb")` 一成功，`path` 处原有的文件就已被截断，所以“`path` 处原有文件在失败后丢失”无论如何都会发生。在此基础上，失败时的清理规则是：

- 只删除普通文件。打开后用 `fstat` 判断，是设备文件（如 `/dev/full`）、管道等则不删。
- 只在 `path` 现在仍指向被打开的那个文件时才删除：用 `lstat(path)` 检查它仍是普通文件，且 `st_dev`、`st_ino` 与打开时一致。`path` 是指向该文件的符号链接，或已被别的文件替换时，不删除（`lstat` 不跟随符号链接，因此链接本身不会被删，被截断的目标文件保留）。
- 成功时不做任何删除。

`stats` 只在成功时写入；失败时保持调用前的内容。

文件描述符不会泄漏：所有出错路径都会 `fclose`，测试中检查了 `/proc/self/fd` 的数量。

## 类型

### `RenderField`

```c
typedef struct RenderField {
    const double *values;  /* [nx*ny]，borrowed */
    const uint8_t *mask;   /* [nx*ny]，非 0 表示固体单元，可为 NULL */
    int nx, ny;            /* 数组尺寸（单元数），≥ 1 */
    int i0, j0, w, h;      /* 裁剪窗口（单元），w, h ≥ 1，必须落在数组内 */
} RenderField;
```

`values` 和 `mask` 都不归渲染器所有，只在 `render_field_png` 调用期间读取。`mask` 与 `values` 同样按行优先存储；`main.c` 直接传 `model.solid`。

### `RenderOptions`

```c
typedef struct RenderOptions {
    int pixel_scale;       /* 每个单元的像素边长，≥ 1 */
    double color_limit;    /* 有限且 > 0：固定的对称上限；≤ 0：自动 */
    bool outline;          /* 是否画轮廓线 */
    int outline_i0, outline_j0, outline_w, outline_h; /* 轮廓矩形（单元），与 values 同一套下标 */
    const char *comment;   /* PNG tEXt "Comment"，可为 NULL */
} RenderOptions;
```

`outline` 为 false 时，四个 `outline_*` 字段被忽略。`comment` 的内容原样写入文本块，不做转义。

### `RenderStats`

```c
typedef struct RenderStats {
    double vmin, vmax;     /* 裁剪窗口内非固体、有限单元的最小值和最大值；没有这样的单元时为 0, 0 */
    double color_limit;    /* 实际使用的颜色上限 */
} RenderStats;
```

`vmin`、`vmax` 是真实范围，不受颜色上限截断的影响。`color_limit` 在固定上限时等于 `RenderOptions.color_limit`，自动时是 `max|v|`（为 0 时是 1）。

### 宏

| 宏 | 值 | 含义 |
|---|---|---|
| `RENDER_SOLID_R` / `_G` / `_B` | 128 / 128 / 128 | 固体单元的灰色 |
| `RENDER_OUTLINE_R` / `_G` / `_B` | 0 / 0 / 0 | 轮廓线的黑色 |

## 函数

### `render_field_png`

```c
int render_field_png(const char *path, const RenderField *field,
                     const RenderOptions *opt, RenderStats *stats, FdtdError *err);
```

| 参数 | 说明 |
|---|---|
| `path` | 输出文件路径。已存在的文件被覆盖；所在目录必须已存在 |
| `field` | 场数据、掩码和裁剪窗口 |
| `opt` | 像素比例、颜色上限、轮廓线、PNG 注释 |
| `stats` | 输出统计量，可为 `NULL`；只在成功时写入 |
| `err` | 错误信息，可为 `NULL` |

前置条件见“前置条件”一节（指针非 `NULL`、窗口在数组内、`pixel_scale ≥ 1`、`color_limit` 有限或 `≤ 0`），渲染器不检查。成功返回 0，失败返回 -1。错误和失败时的文件处理见“错误处理”。`path` 是符号链接时，`fopen` 会跟随链接，写入的是链接指向的文件。

### `render_colormap`

```c
void render_colormap(double t, uint8_t rgb[3]);
```

把归一化的值 `t` 转成颜色，写入 `rgb[0..2]`。`t` 被截断到 [−1, 1]，NaN 按 0 处理。算法见“色表”。

## 用法示例

下面的写法与 `main.c` 一致：每隔若干步取一次压力场，渲染一帧并记录统计量。

```c
const Grid *g = &model.grid;
double *pressure = fdtd_calloc(grid_cell_count(g), sizeof *pressure, err);

RenderField field = {
    .values = pressure,
    .mask = model.solid,
    .nx = g->nx, .ny = g->ny,
    /* 只画物理域；要画 PML，窗口取 0, 0, nx, ny 并设置 outline */
    .i0 = g->off_x, .j0 = g->off_y, .w = g->nx_phys, .h = g->ny_phys,
};
RenderOptions ropt = {
    .pixel_scale = cfg.output.pixel_scale,
    .color_limit = cfg.output.color_auto ? 0.0 : cfg.output.color_limit,
};

/* 时间循环中 */
solver_get_pressure(&solver, pressure);

char comment[160];
snprintf(comment, sizeof comment, "fdtd2d pressure, step=%ld, time=%.9e s", n, solver.time);
ropt.comment = comment;

RenderStats stats;
if (render_field_png("out/p_000100.png", &field, &ropt, &stats, err) != 0) {
    /* err->msg 已填好 */
    return -1;
}
/* stats.color_limit、stats.vmin、stats.vmax 可用于写 frames.csv */
```

## 命令行的输出

`fdtd2d` 把 `[output]` 配置段的键（见 [config.md](config.md)）映射到渲染器参数：

| `[output]` 键 | 渲染器参数 |
|---|---|
| `directory` | 输出目录；不存在时按 `mkdir -p` 创建；命令行 `-o` 可覆盖 |
| `every` | 每 `every` 步输出一帧；最后一步总会输出 |
| `color_limit` | `"auto"` 时 `RenderOptions.color_limit = 0`；否则等于配置的数值（配置层保证 > 0） |
| `pixel_scale` | `RenderOptions.pixel_scale`；配置层限制为 1 到 16 |
| `show_pml` | 决定裁剪窗口和轮廓线，见“轮廓线”一节 |

输出目录中的文件：

**`p_XXXXXX.png`**：压力场热力图。`XXXXXX` 是步号，用 0 补齐到至少 6 位；总步数的位数超过 6 位时，补齐到总步数的位数。步号从 1 起（不输出初始时刻）。每张图的 `Comment` 文本块为 `fdtd2d pressure, step=<步号>, time=<时间, %.9e> s`。

**`frames.csv`**：每写出一帧追加一行，表头为：

```
step,time_s,file,color_limit_pa,p_min_pa,p_max_pa
```

| 列 | 内容 |
|---|---|
| `step` | 步号 |
| `time_s` | 求解器时间 `step · dt` [s]，格式 `%.9e` |
| `file` | PNG 文件名（不含目录） |
| `color_limit_pa` | 该帧实际使用的颜色上限 [Pa]，即 `RenderStats.color_limit`，格式 `%.6e` |
| `p_min_pa`、`p_max_pa` | 该帧渲染范围内非固体单元的最小、最大声压 [Pa]，即 `RenderStats.vmin`、`vmax`，格式 `%.6e` |

`show_pml = true` 时统计范围含 PML 单元，否则只含物理域。

**`summary.txt`**：在求解开始前写出。第一行是 `fdtd2d <版本>`，第二行是 `config: <配置文件路径>`，空一行后是 `model_print_summary` 的内容（网格、时间步、介质、边界、固体单元数、声源），与启动时打印到标准输出的摘要相同。`--check` 模式不创建输出目录，也不写这些文件。

写帧之前，`main.c` 会检查整个压力数组（含 PML）是否全为有限值；出现非有限值时中止并报错 `solution became non-finite at step ...`，因此命令行实际不会把 NaN 或 ±∞ 交给渲染器。渲染器自己仍按上文规则处理非有限值。`render_field_png` 失败时命令行也中止并返回错误。完整的命令行说明见后续的 CLI 文档。

## 性能

Release 构建，AMD Ryzen 9 7845HX：

| 场景 | 耗时 |
|---|---|
| 1000 × 1000 场（光滑），`pixel_scale = 1` | 约 71 ms |
| 1000 × 1000 场（随机噪声），`pixel_scale = 1` | 约 77 ms |
| 1000 × 1000 场，`pixel_scale = 2` | 约 124 ms |

输出文件约 1.8 MB。两个示例配置 `examples/point_source_free_field.toml`、`examples/line_source_obstacles.toml` 的完整运行（含求解和写文件）各输出 58 个文件，总用时 0.5 到 0.7 s。

## 测试

`tests/test_render.c` 含 21 个测试。每个用例都把 PNG 写到 `FDTD2D_TEST_TMPDIR`，再用 libpng 读回，逐像素与期望值比较，期望颜色由 `render_colormap` 或手工写出的几何给出。

| 方面 | 测试内容 |
|---|---|
| 色表 | 9 个控制点逐一核对；5 个中点的取整结果；`t < −1`、`t > 1`、±∞ 的截断；NaN 得到 0 的颜色 |
| 格式与尺寸 | 宽高、8 位 RGB、无隔行；1 × 1 场 |
| y 轴翻转 | 最上一行的单元出现在图像顶部，`j = 0` 的单元出现在底部；`pixel_scale = 2` 时每个单元是 2 × 2 块 |
| 裁剪窗口 | 窗口内的像素与单元值对应；统计量只取窗口内；窗口贴着最后一行和最后一列 |
| 固体掩码 | 固体单元为灰色；不参与统计和自动上限；全部是固体时 `vmin = vmax = 0`、`limit = 1`；窗口外的掩码不影响统计 |
| 颜色上限与统计 | 自动上限（正负极值都可定义上限）；`color_limit < 0` 和 `−∞` 等同自动；极小幅值仍占满色域；全零场；固定上限的截断和统计值；非有限值显示为 0 的颜色且不进统计 |
| 轮廓线 | 矩形边界的像素位置和数量；轮廓盖住固体和场颜色；1 × 1 矩形在 `pixel_scale = 3` 时中心镂空；覆盖整个窗口；窗口有偏移且 `pixel_scale = 3` 时的位置（期望像素手工写出）；部分在窗口外时的裁剪；关闭、不相交、`w = 0`、`h < 0` 时被忽略 |
| 文本块 | `Comment` 和 `Software` 的内容；不给注释时只有 `Software`；空注释仍写出 |
| 覆盖已有文件 | 第二次渲染得到新尺寸和新内容 |
| 尺寸上限 | 宽或高恰为 65535 可以写出，65536 报错；经 `pixel_scale` 超限（含 `INT_MAX` 溢出）；错误信息为单行，失败后不留下文件；`err` 为 `NULL` 时同样返回 -1 |
| 路径错误 | 目录不存在、路径是目录、空路径；`stats` 在失败时不变；不泄漏文件描述符 |
| 写入失败 | 写 `/dev/full`（每次写入都得到 ENOSPC）：返回 -1，信息含路径，设备文件仍在，不泄漏文件描述符；没有 `/dev/full` 时跳过 |
| 较大噪声图 | 150 × 150 场、`pixel_scale = 2`，重复 3 次，抽查像素，文件描述符数量不变 |

## 已知限制

- 图像里没有色标、坐标轴和文字标注。颜色对应的数值范围要从 `frames.csv` 的 `color_limit_pa` 列查，PNG 的 `Comment` 里只有步号和时间。
- 颜色上限只支持关于 0 对称的一个值，不能分别指定正负两侧。
- 部分写入后的删除路径没有被直接测试。`/dev/full` 测试覆盖的是“不删除设备文件”的分支，没有覆盖“删除普通文件”的分支。当初计划用 `RLIMIT_FSIZE` 触发写入失败，因为旧文件 `src/signal.h` 遮蔽了系统的 `<signal.h>` 而放弃；改名为 `waveform.h` 后这个障碍已经不存在，可以补上该测试。
- `path` 是符号链接时，`fopen` 跟随链接，写入并截断链接指向的文件。出错时因 `lstat` 看到的是符号链接，不会删除任何东西，被截断的目标文件留在原处。
- `lstat` 检查与 `remove` 之间不是原子操作。
- 单元总是绘成正方形像素。网格 `dx ≠ dy` 时图像在一个方向上被拉伸（见 [grid.md](grid.md)）。
- 图像宽高不能超过 65535 像素，配置层把 `pixel_scale` 限制在 16 以内。网格很大时帧尺寸仍可能超限，`fdtd2d` 会在模型构建后、写任何文件之前报错（`--check` 也会报），例如：`case.toml: output: frames would be 80000 x 60000 pixels, above the PNG limit of 65535; reduce pixel_scale`。
- 渲染器不检查 `values`、`mask` 的实际长度，调用方必须保证至少 `nx × ny` 个元素。
