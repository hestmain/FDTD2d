# medium：介质

头文件：`src/medium.h`　实现：`src/medium.c`　依赖：common

本模块描述无黏性理想流体介质，并管理介质表。model 为每个网格单元存一个介质编号，solver 根据介质表计算更新系数。

**v1 范围**：只有一种均匀背景介质，编号为 0。

**扩展方式**：以后增加介质区域（`[[medium_region]]`）时，先用 `medium_table_add()` 追加介质，再在 model 中把区域形状覆盖到的单元写成新编号。solver 已经按单元读取 ρ 和 K，并在面上对 ρ 取算术平均，所以不需要修改。

## 类型

### `Medium`

```c
typedef struct Medium {
    double density; /* ρ [kg/m³] */
    double c;       /* 声速 [m/s] */
} Medium;
```

### `MediumId`

```c
typedef uint16_t MediumId;
#define MEDIUM_MAX_COUNT 65535
```

单元介质编号的存储类型。每个单元占 2 字节，最多支持 65535 种介质。

### `MediumTable`

```c
typedef struct MediumTable {
    Medium *items;   /* owned */
    int count;
} MediumTable;
```

`items[0]` 始终是背景介质。

## 函数

### `medium_validate`

```c
int medium_validate(const Medium *m, FdtdError *err);
```

要求 `density` 和 `c` 都是大于 0 的有限值。失败返回 -1，错误信息例如 `density must be a finite value > 0 (got -1)`。介质只在这里校验一次，由 `config_load_*` 调用；介质表的函数不会再校验。

### `medium_bulk_modulus`

```c
double medium_bulk_modulus(const Medium *m);
```

返回体积模量 K = ρc²，单位 Pa。例如 20 °C 空气（ρ = 1.21 kg/m³，c = 343 m/s）的 K ≈ 1.424×10⁵ Pa。

### `medium_table_init`

```c
int medium_table_init(MediumTable *t, const Medium *background, FdtdError *err);
```

先清零 `*t`，再把背景介质放入第 0 项，`count = 1`。`background` 必须已通过 `medium_validate`（`config_load_*` 已经校验），本函数不再检查。内存不足时返回 -1，此时 `*t` 为全零。

### `medium_table_add`

```c
int medium_table_add(MediumTable *t, const Medium *m, FdtdError *err);
```

追加一种介质，返回它的编号（≥ 1）。`m` 必须已通过 `medium_validate`，本函数不再检查。超过 `MEDIUM_MAX_COUNT` 或内存不足时返回 -1，表保持原样。v1 没有调用方，为介质区域预留。

### `medium_table_c_max` / `medium_table_c_min`

```c
double medium_table_c_max(const MediumTable *t);
double medium_table_c_min(const MediumTable *t);
```

返回表中所有介质的最大、最小声速。表必须由 `medium_table_init` 初始化过（`count ≥ 1`）。用途：
- 最大声速 c_max 决定 CFL 时间步长上限，也作为 PML 衰减剖面的参考声速。
- 最小声速 c_min 决定最短波长，用于检查每波长网格点数。

### `medium_table_free`

```c
void medium_table_free(MediumTable *t);
```

释放并清零。`t == NULL`、全零结构体和重复调用都是安全的。

## 示例

```c
MediumTable media;
FdtdError err = {{0}};
Medium air = {.density = 1.21, .c = 343.0};
if (medium_table_init(&media, &air, &err) != 0) { /* 报错 */ }
double K = medium_bulk_modulus(&media.items[0]);   /* ≈ 1.424e5 Pa */
medium_table_free(&media);
```
