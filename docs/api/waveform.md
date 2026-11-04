# waveform：声源时间函数

头文件：`src/waveform.h`　实现：`src/waveform.c`　依赖：common、libm

本模块描述声源随时间的变化 s(t)。声源 = 空间分布（由形状决定，见 source）× 时间函数（本模块）。两者相互独立，因此新增一种信号不影响点源、线源的实现。

**v1 范围**：只实现简谐信号 `harmonic`。

**命名**：文件原名 `signal.[ch]`，与系统头文件 `<signal.h>` 同名，而 `src` 是 PUBLIC include 目录，会遮蔽系统头文件，所以改为 `waveform.[ch]`。类型和函数仍叫 `Signal`、`signal_*`，TOML 键仍是 `signal`；这些名字不与系统头文件冲突。

## 简谐信号定义

```
s(t) = A · ramp(t) · sin(2π f t + φ)          t ≥ 0
s(t) = 0                                       t < 0

ramp(t) = ½ · (1 − cos(π t / T_r))             t < T_r
ramp(t) = 1                                    t ≥ T_r
T_r = ramp_cycles / f                          ramp_cycles = 0 表示不渐变
```

起振渐变用升余弦包络，在前 `ramp_cycles` 个周期内把幅值从 0 平滑提升到 A。这样可以抑制突然起振带来的宽带瞬态，使场更快进入稳态。代价是渐变期间的场不是纯单频，比较稳态结果时应避开这段时间。

幅值 A 的物理含义由声源类型决定（见 source 文档）：
- 点源：A 为单位深度体积速度 Q，单位 m²/s。
- 线源：A 为单位长度、单位深度的体积速度 Q′，单位 m/s。

## 类型

### `SignalType`

```c
typedef enum SignalType { SIGNAL_HARMONIC = 0, SIGNAL_TYPE_COUNT } SignalType;
```

### `HarmonicSignal`

```c
typedef struct HarmonicSignal {
    double frequency;   /* f [Hz]，> 0 */
    double amplitude;   /* A，有限值（可为 0 或负数） */
    double phase;       /* φ [rad]，有限值 */
    double ramp_cycles; /* ≥ 0，可为小数 */
} HarmonicSignal;
```

### `Signal`

```c
typedef struct Signal {
    SignalType type;
    union { HarmonicSignal harmonic; } u;
} Signal;
```

不持有堆内存，可以直接按值复制。

对应的 TOML 写法（`phase` 默认 0，`ramp_cycles` 默认 3）：

```toml
signal = { type = "harmonic", frequency = 2000.0, amplitude = 2e-3, phase = 0.0, ramp_cycles = 3 }
```

## 函数

### `signal_type_name` / `signal_type_from_name`

```c
const char *signal_type_name(SignalType type);
int signal_type_from_name(const char *name, SignalType *out);
```

在枚举和 TOML 名称之间转换，v1 只有 `"harmonic"`。`signal_type_name` 的 `type` 必须是有效枚举值（小于 `SIGNAL_TYPE_COUNT`）。`signal_type_from_name` 的 `name` 不能为 `NULL`，用法与 `shape_type_from_name` 相同：成功返回 0，未知名称返回 -1。

### `signal_validate`

```c
int signal_validate(const Signal *s, FdtdError *err);
```

harmonic 的校验规则：
- `frequency` 为大于 0 的有限值。
- `amplitude` 和 `phase` 为有限值。
- `ramp_cycles` 为大于等于 0 的有限值。

失败返回 -1，错误信息不含位置，由调用方补充前缀。

### `signal_eval`

```c
double signal_eval(const Signal *s, double t);
```

返回 s(t)；t < 0 时返回 0。solver 每一步在半步时刻 `t = (n + ½)·dt` 调用它。

### `signal_max_frequency`

```c
double signal_max_frequency(const Signal *s);
```

返回信号中显著成分的最高频率，harmonic 返回 f。model 用它检查每波长网格点数。

## 示例

```c
Signal s = {.type = SIGNAL_HARMONIC,
            .u.harmonic = {.frequency = 1000.0, .amplitude = 1.0,
                           .phase = 0.0, .ramp_cycles = 2.0}};
double v0 = signal_eval(&s, 0.0);        /* 0 */
double v1 = signal_eval(&s, 2.25e-3);    /* 渐变已结束：sin(2π·2.25) = 1 */
```

## 扩展方式

新增时间函数（如高斯脉冲、Ricker 子波、正弦脉冲串、从文件读入的波形）的步骤：
1. 在 `SignalType` 中加入新枚举值，在联合体中加入参数结构体，并在 `k_signal_names` 中加入名称。
2. 在 `signal_validate`、`signal_eval`、`signal_max_frequency` 中加入对应分支。宽带信号的 `signal_max_frequency` 应返回有效带宽的上限，例如 Ricker 子波约取 2.5 倍中心频率。
3. 在 config 中解析新信号的参数。

如果信号持有堆内存（例如从文件读入的波形），需要另加 `signal_free()`，并在 `ModelSource` 和 `SourceConfig` 的释放路径中调用它。
