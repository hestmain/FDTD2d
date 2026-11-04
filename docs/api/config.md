# config：解析器第一层（TOML → SimConfig）

头文件：`src/config.h`　实现：`src/config.c`　依赖：tomlc17（第三方）、geometry、signal、boundary、medium、common

config 读取 TOML 配置文件，经过校验后生成 `SimConfig` 纯数据结构。`SimConfig` 只记录用户写的内容和默认值，不涉及网格；凡是需要网格才能做的检查都交给 model，例如域长能否被 dx 整除、CFL 上限、声源是否在域内。

## 配置文件格式

TOML v1.0/1.1。所有数值都使用 SI 单位。凡是要求浮点数的字段也可以写整数，例如 `c = 343`。要求整数的字段写成浮点数会报错。

所有表都按白名单检查键名，出现未知键会报错，因此拼写错误不会被悄悄忽略。

### `[simulation]`（必需）

| 键 | 类型 | 默认 | 约束 | 说明 |
|---|---|---|---|---|
| `duration` | 数值 | 必需 | > 0 | 仿真时长 [s] |
| `cfl` | 数值 | 0.7 | 0 < cfl ≤ 1 | Courant 数，dt 由它推导 |
| `dt` | 数值 | — | > 0 | 直接指定时间步长 [s]，不能与 `cfl` 同时出现 |

### `[grid]`（必需）

| 键 | 类型 | 默认 | 约束 | 说明 |
|---|---|---|---|---|
| `x_range` | 2 元数组 | 必需 | [1] > [0] | 物理域 x 范围 [m] |
| `y_range` | 2 元数组 | 必需 | [1] > [0] | 物理域 y 范围 [m] |
| `dx` | 数值 | 必需 | > 0 | x 方向网格步长 [m] |
| `dy` | 数值 | 等于 `dx` | > 0 | y 方向网格步长 [m] |

PML 层加在物理域外侧，不占用 `x_range` 和 `y_range`。

### `[medium]`（必需）

背景介质。v1 只有这一种均匀介质。

| 键 | 类型 | 约束 | 说明 |
|---|---|---|---|
| `density` | 数值 | > 0 | 密度 ρ [kg/m³] |
| `c` | 数值 | > 0 | 声速 [m/s] |

### `[boundary]`（可选）

| 键 | 取值 | 默认 |
|---|---|---|
| `x_min` `x_max` `y_min` `y_max` | `"rigid"` \| `"pressure_release"` \| `"pml"` | `"pml"` |

子表 `[boundary.pml]`（可选，所有 PML 边共用）：

| 键 | 类型 | 默认 | 约束 |
|---|---|---|---|
| `thickness` | 整数 | 20 | ≥ 1，单位为单元数 |
| `reflection` | 数值 | 1e-6 | 0 < R < 1 |
| `order` | 数值 | 2 | ≥ 0 |

### `[[obstacle]]`（可选，可重复）

每一项是计算域内部的一个刚性障碍物。

| 键 | 说明 |
|---|---|
| `shape` | 面状形状：rect、circle 或 polygon。v1 不支持 point 和 segment 这类细障碍物 |

### `[[source]]`（可选，可重复）

| 键 | 说明 |
|---|---|
| `shape` | v1 只支持 `point`（点源）和 `segment`（线源） |
| `signal` | 时间函数，v1 只支持 `harmonic` |

可以没有声源，此时 model 会发出警告。

### 形状表 `shape`

既可以写成内联表，也可以写成普通子表。`type` 决定允许出现的其他键。

| type | 键 | 示例 |
|---|---|---|
| `point` | `position = [x, y]` | `{ type = "point", position = [0.5, 0.75] }` |
| `segment` | `start`, `end` | `{ type = "segment", start = [0.2, 0.3], end = [0.2, 1.2] }` |
| `rect` | `min`, `max` | `{ type = "rect", min = [1, 0], max = [1.2, 0.4] }` |
| `circle` | `center`, `radius` | `{ type = "circle", center = [1.2, 0.75], radius = 0.1 }` |
| `polygon` | `vertices = [[x, y], ...]`（至少 3 个顶点） | `{ type = "polygon", vertices = [[2.3, 0], [2.7, 0], [2.5, 0.5]] }` |

几何约束见 geometry.md 中的 `shape_validate`。

### 信号表 `signal`

| type | 键 | 默认 | 说明 |
|---|---|---|---|
| `harmonic` | `frequency` | 必需 | 频率 [Hz]，> 0 |
| | `amplitude` | 必需 | 点源：Q [m²/s]；线源：Q′ [m/s]，见 source.md |
| | `phase` | 0 | 初相 [rad] |
| | `ramp_cycles` | 3 | 起振渐变的周期数，≥ 0 |

### `[output]`（可选）

| 键 | 类型 | 默认 | 约束 | 说明 |
|---|---|---|---|---|
| `directory` | 字符串 | `"out"` | 非空 | 输出目录，不存在时自动创建；命令行 `-o` 可覆盖 |
| `every` | 整数 | 10 | ≥ 1 | 每 N 步输出一帧，最后一步总会输出 |
| `color_limit` | `"auto"` 或数值 | `"auto"` | 数值 > 0 | 色标范围 [Pa]；`"auto"` 表示每帧取 ±max\|p\| |
| `show_pml` | 布尔 | false | | 是否绘出 PML 区域，绘出时用黑框标出物理域 |
| `pixel_scale` | 整数 | 1 | 1..16 | 每个单元绘成的像素边长 |

完整示例见 `examples/point_source_free_field.toml` 和 `examples/line_source_obstacles.toml`。

## 类型

```c
typedef struct SourceConfig   { Shape shape; Signal signal; } SourceConfig;
typedef struct ObstacleConfig { Shape shape; } ObstacleConfig;

typedef struct OutputConfig {
    char *directory;      /* owned */
    int every;
    bool color_auto;      /* true 表示 "auto" */
    double color_limit;   /* color_auto == false 时有效 */
    bool show_pml;
    int pixel_scale;
} OutputConfig;

typedef struct SimConfig {
    double duration, cfl, dt;  bool dt_set;      /* [simulation] */
    double x_range[2], y_range[2], dx, dy;       /* [grid]，dy 已填默认值 */
    Medium medium;                               /* [medium] */
    BoundaryType boundary[SIDE_COUNT];           /* [boundary]，已填默认值 */
    PmlParams pml;                               /* 始终填好，无 PML 侧时也一样 */
    ObstacleConfig *obstacles; int n_obstacles;  /* owned */
    SourceConfig *sources;     int n_sources;    /* owned */
    OutputConfig output;
    char *source_name;                           /* owned，配置文件名 */
} SimConfig;
```

字段在加载时已全部填好默认值并通过校验，使用方不需要再判断字段是否缺省。

## 函数

### `config_load_file`

```c
int config_load_file(const char *path, SimConfig *cfg, FdtdError *err);
```

读取并校验 TOML 文件。成功返回 0，`cfg` 拥有全部内存。失败返回 -1，`cfg` 为全零，不需要释放。

错误信息为单行，格式是 `<文件>:<行号>: <键路径>: <问题>`。行号未知时省略。数组下标从 0 开始。以下是实际输出：

```
case.toml:12: grid.dxx: unknown key
case.toml:30: source[1].signal.frequency: must be > 0 (got -5)
case.toml:4: simulation.dt: cfl and dt are mutually exclusive; set only one of them
case.toml: cannot open file: No such file or directory
```

TOML 语法错误例外：它沿用 tomlc17 的原始信息，行号写在括号里，例如 `case.toml: (line 1) missing right-bracket`。

如果有多处错误，只报告第一处。检查顺序依次为：顶层未知键、`[simulation]`、`[grid]`、`[medium]`、`[boundary]`、`[[obstacle]]`、`[[source]]`、`[output]`。

需要网格才能做的检查由 model 负责，例如域长不能被 dx 整除、声源在域外。这类错误的信息里没有行号，`fdtd2d` 会在前面加上文件名，例如：

```
case.toml: grid: x_range length 1 is not a multiple of dx = 0.003 (nearest valid dx: 0.003003)
case.toml: source[0]: point (5, 1) is outside the physical domain [0, 1] x [0, 1]
```

### `config_load_string`

```c
int config_load_string(const char *text, const char *source_name, SimConfig *cfg, FdtdError *err);
```

功能同上，从内存字符串读取。`source_name` 只用于错误信息，为 `NULL` 时显示为 `<string>`。这个函数主要用于测试。

### `config_free`

```c
void config_free(SimConfig *cfg);
```

释放全部内存并清零。对全零结构体安全，可重复调用。

## 示例

```c
SimConfig cfg;
FdtdError err = {{0}};
if (config_load_file(argv[1], &cfg, &err) != 0) {
    fprintf(stderr, "error: %s\n", err.msg);
    return 1;
}
printf("%d sources, f0 = %g Hz\n", cfg.n_sources,
       cfg.n_sources ? cfg.sources[0].signal.u.harmonic.frequency : 0.0);
config_free(&cfg);
```

## 扩展方式

- **新形状或新信号**：先在 geometry 或 signal 中实现，再在本模块的形状、信号解析函数中加一个分支，写明该类型允许的键和读取方式。
- **介质区域**：增加 `[[medium_region]]` 数组表，每项包含 shape、density 和 c；在 `SimConfig` 中加上对应的数组；然后由 model 把区域写入 `medium_id`。
- **新的顶层表**（如接收点 `[[probe]]`）：在顶层键白名单中登记，再增加对应的解析函数。
