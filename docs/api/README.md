# fdtd2d API 文档总览

fdtd2d 是一个 C11 编写的二维声场 FDTD 仿真工具：无黏性理想流体、静止介质、线性声学。本目录按模块逐一说明公开 API。各模块的头文件（`src/*.h`）是接口契约；本文档与头文件不一致时以头文件为准，并应当修正文档。

## 模块分层

```
                 main.c (命令行)
                     │
  ┌──────────────────┼──────────────────────┐
  │ 解析器            │ 求解器               │ 渲染器
  │ config  → model  │ solver               │ render
  │  (TOML)  (网格化) │ (时间推进)           │ (PNG 热力图)
  └────┬─────────┬───┴──────────┬───────────┘
       │         │              │
       │      source (声源离散)  │
       │         │              │
  geometry  medium  signal  boundary  grid      ← 基础模块
                     │
                  common (错误、警告、内存)
```

| 模块 | 文件 | 作用 | 文档 |
|---|---|---|---|
| common | `src/common.[ch]` | 错误信息、警告、内存分配辅助 | [common.md](common.md) |
| geometry | `src/geometry.[ch]` | 与网格无关的二维形状 | [geometry.md](geometry.md) |
| grid | `src/grid.h` | 交错网格布局与索引换算（仅头文件） | [grid.md](grid.md) |
| medium | `src/medium.[ch]` | 介质参数与介质表 | [medium.md](medium.md) |
| signal | `src/signal.[ch]` | 声源时间函数（v1：简谐） | [signal.md](signal.md) |
| boundary | `src/boundary.[ch]` | 外边界类型与 PML 参数 | [boundary.md](boundary.md) |
| config | `src/config.[ch]` | 解析器第一层：TOML → `SimConfig` | [config.md](config.md) |
| source | `src/source.[ch]` | 声源空间离散 | [source.md](source.md) |
| model | `src/model.[ch]` | 解析器第二层：`SimConfig` → 网格模型 | [model.md](model.md) |
| solver | `src/solver.[ch]` | FDTD 时间推进 | 待模块完成后提供 |
| render | `src/render.[ch]` | 热力图 PNG 输出 | 待模块完成后提供 |

依赖只能自上而下：基础模块互不依赖（只依赖 common），renderer 不依赖 model，也不依赖 solver。

## 全局约定

**单位**：全部使用 SI 单位，包括 m、s、Pa、kg/m³、m/s、Hz、rad。二维问题中的“体积”量均按单位深度（每米深度）计。

**错误处理**：可能失败的函数返回 `int`，`0` 表示成功，`-1` 表示失败。失败时把单行可读信息写入调用方传入的 `FdtdError *err`。`err` 可以为 `NULL`，此时只返回错误码。外层调用用 `fdtd_error_prefix()` 补充上下文，最终信息形如 `source[1]: point (5, 1) is outside the physical domain`。

**警告**：可继续运行、但用户应当知道的情况（如每波长网格点数偏少），通过 `fdtd_warn()` 输出到 stderr，并计数。

**内存所有权**：
- 结构体中标注 *owned* 的指针由该结构体负责释放，并配有对应的 `*_free()` 函数。
- 所有 `*_free()` 都可以安全地作用于全零结构体，也可以重复调用。
- 构建函数（如 `model_build`）失败时保证输出结构体为全零，调用方无需清理。
- 标注 *borrowed* 的指针不归该结构体所有，被指对象的生命周期必须更长。

**数组布局**：二维场一律按行优先存储，x 方向变化最快。单元 `(i, j)` 的下标为 `j*nx + i`。详见 [grid.md](grid.md)。

**线程安全**：v1 为单线程。`common` 中的警告计数和警告输出流是进程级全局状态。

## 构建

本机运行在 AppImage 环境中，会泄露 `LD_LIBRARY_PATH` 等变量，导致系统 cmake 加载到错误的 libarchive。构建前先清除这些变量（fish）：

```fish
set -e LD_LIBRARY_PATH; set -e PYTHONHOME; set -e PYTHONPATH
cmake -S . -B build -DFDTD2D_WERROR=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

CMake 选项：

| 选项 | 默认 | 作用 |
|---|---|---|
| `FDTD2D_BUILD_TESTS` | ON | 构建单元测试与验证测试 |
| `FDTD2D_WERROR` | OFF | 把编译警告视为错误 |
| `FDTD2D_SANITIZE` | OFF | 启用 AddressSanitizer 与 UBSan |

链接目标：`fdtd2d_core`（静态库，包含全部模块）、`fdtd2d`（命令行程序）、`tomlc17`（第三方 TOML 解析库，见 `third_party/tomlc17/VERSION.txt`）。
