# common：错误、警告与内存辅助

头文件：`src/common.h`　实现：`src/common.c`　依赖：C 标准库

所有模块共用的基础设施，包括错误信息结构、警告输出和带溢出检查的内存分配。

## 宏

| 宏 | 值 | 说明 |
|---|---|---|
| `FDTD2D_VERSION` | `"0.1.0"` | 程序版本字符串，用于 `--version` 和 PNG 元数据 |
| `FDTD_PRINTF(fmt_idx, arg_idx)` | GCC/Clang 下为 `__attribute__((format(printf, …)))` | 让编译器检查格式串参数 |

## 类型

### `FdtdError`

```c
typedef struct FdtdError {
    char msg[1024];
} FdtdError;
```

单行错误信息缓冲区。调用方在栈上分配，以指针传给可能失败的函数。消息超过 1023 字节时截断，不会溢出。

```c
FdtdError err = {{0}};
if (model_build(&cfg, &model, &err) != 0) {
    fprintf(stderr, "error: %s\n", err.msg);
}
```

## 函数

### `fdtd_error_set`

```c
void fdtd_error_set(FdtdError *err, const char *fmt, ...);
```

用 printf 风格的格式化结果覆盖 `err->msg`。

| 参数 | 说明 |
|---|---|
| `err` | 目标错误对象；为 `NULL` 时什么都不做 |
| `fmt, ...` | printf 格式串与参数，编译期检查 |

### `fdtd_error_prefix`

```c
void fdtd_error_prefix(FdtdError *err, const char *fmt, ...);
```

在已有消息前加上 `"<格式化结果>: "`，用于外层补充上下文。原消息为空时只写入前缀，不带冒号。总长超出缓冲区时截断。`err` 为 `NULL` 时什么都不做。

```c
if (source_discretize(..., err) != 0) {
    fdtd_error_prefix(err, "source[%d]", k);   // "source[2]: segment lies outside ..."
    return -1;
}
```

### `fdtd_warn`

```c
void fdtd_warn(const char *fmt, ...);
```

向警告流输出 `warning: <消息>\n`，并把全局警告计数加 1。警告流默认是 `stderr`。

### `fdtd_set_warning_stream`

```c
void fdtd_set_warning_stream(FILE *stream);
```

重定向警告输出。传入 `NULL` 时静默，计数照常累加。测试中常用它屏蔽输出，再用计数断言有没有产生警告。

### `fdtd_warning_count` / `fdtd_reset_warning_count`

```c
int  fdtd_warning_count(void);
void fdtd_reset_warning_count(void);
```

读取或清零自进程启动以来（或上次清零以来）的警告次数。

```c
fdtd_set_warning_stream(NULL);
fdtd_reset_warning_count();
model_build(&cfg, &m, &err);
CHECK(fdtd_warning_count() == 1);     // 例如：期望恰好一条“每波长点数不足”警告
```

### `fdtd_calloc`

```c
void *fdtd_calloc(size_t n, size_t size, FdtdError *err);
```

带检查的 `calloc`，内存清零。

| 情况 | 结果 |
|---|---|
| 成功 | 返回清零的内存，用 `free()` 释放 |
| `n * size` 溢出 `size_t` | 返回 `NULL`，`err` 为 `allocation size overflow (...)` |
| 内存不足 | 返回 `NULL`，`err` 为 `out of memory allocating ...` |
| `n == 0` 或 `size == 0` | 返回一个可 `free()` 的有效指针（按 1 字节分配） |

### `fdtd_strdup`

```c
char *fdtd_strdup(const char *s, FdtdError *err);
```

ISO C11 没有 `strdup`，这里提供替代。返回新分配的副本，用 `free()` 释放。分配失败时返回 `NULL` 并设置 `err`。`s` 不能为 `NULL`。

## 注意事项

- 警告计数与警告流是进程级全局变量，非线程安全。v1 是单线程，不受影响。
- 本模块不提供日志级别；常规进度信息由 `main.c` 直接打印。
