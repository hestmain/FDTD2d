# fdtd2d 开发交接文档（状态与进度）

更新时间：2026-11-04 15:50。本文档用于上下文压缩或中断后接续开发，记录截至当前的需求决定、完成情况、未决事项和下一步。

## 1. 项目与已确认的需求

二维声场 FDTD 仿真工具，介质为无黏性理想流体，按静止介质中的线性声学处理。

| 项 | 决定 |
|---|---|
| 语言 / 构建 | C11 + CMake（≥ 3.20）。不用 Makefile |
| 控制方程 | 一阶 p-v 方程；交错（Yee）网格；二阶中心差分；leapfrog 时间推进；double 精度 |
| 配置格式 | TOML。解析库 tomlc17（MIT，R261003，原样拷入 `third_party/tomlc17/`，见 `VERSION.txt`） |
| 网格 | 均匀网格，SI 单位，支持 dx ≠ dy。PML 加在物理域**外侧** |
| dt | 默认 `cfl = 0.7`，`dt = cfl/(c_max·√(1/dx²+1/dy²))`；也可直接给 `dt`，但 cfl > 1 时报错；`cfl` 与 `dt` 不能同时给 |
| 介质 | v1 只有均匀背景介质。用“单元介质编号 + 介质表”的结构，方便以后扩展多介质区域 |
| 几何 | 统一的 Shape 类型：point / segment / rect / circle / polygon。用于刚性障碍物（v1）、介质区域（预留）和辐射面声源（预留） |
| 边界 | 每条边独立设置：rigid / pressure_release / pml（分裂场 PML，多项式剖面） |
| 声源 | 点源和线源（直线段），单极子软源；v1 只做简谐信号（升余弦起振渐变） |
| 声源幅值单位 | 点源为 Q [m²/s]，线源为 Q′ [m/s]，都是体积速度、按单位深度计 |
| 输出 | 只做离线输出：声压场热力图 PNG（libpng，蓝-白-红发散色表，固体灰色）+ `frames.csv` + `summary.txt`。不做实时显示 |
| 命令行 | `fdtd2d [--check] [-o DIR] [-q] CONFIG.toml` |
| 文档 | 每个模块都要有详细的 API 文档（`docs/api/*.md`，中文；代码注释用英文） |
| 流程 | **每个模块完成后交给用户检查**，用户确认后再进入下一步 |
| 子代理 | 模型 `model_id = "fast"`（用户已设为 Sonnet）。用户要求“极高挡位”，但 Task 工具没有设置推理挡位的参数，目前只在提示词里要求 “work very thoroughly” |

还没有得到用户答复的事项：
- 是否执行 `git init`；
- 用户是否认可 `examples/*.toml` 中的配置格式；
- 文档用中文、代码注释用英文，是否统一成一种语言。

## 2. 构建与测试（重要）

本机运行在 AppImage 环境中，泄露的 `LD_LIBRARY_PATH` 会让系统 cmake 加载错误的 libarchive。shell 是 fish，构建前先执行：

```fish
set -e LD_LIBRARY_PATH; set -e PYTHONHOME; set -e PYTHONPATH
cmake -S . -B build -DFDTD2D_WERROR=ON && cmake --build build -j && ctest --test-dir build
cmake -S . -B build-asan -DFDTD2D_SANITIZE=ON -DFDTD2D_WERROR=ON -DCMAKE_BUILD_TYPE=Debug
```

CTest 中的测试：
- 单元测试：geometry、config、model、source、render、solver。
- 验证测试：validation。
- 命令行测试：`cli_check_<example>`，对每个示例执行 `fdtd2d --check`。
- 测试程序返回码 77 表示跳过（用于占位测试）。

已知工具问题：子代理用 Write 写文件时曾在每行开头多写一个 `+`。每次交付后都要运行 `grep -c '^+' src/*.c tests/*.c` 检查，结果应全为 0。

## 3. 模块进度

| 模块 | 文件 | 状态 | API 文档 | 用户检查 |
|---|---|---|---|---|
| common | `src/common.[ch]` | ✅ 完成 | `docs/api/common.md` | 已提交（检查点 1），待确认 |
| geometry | `src/geometry.[ch]` | ✅ 完成，`test_geometry` 17/17 | `geometry.md` | 检查点 1，待确认 |
| grid | `src/grid.h` | ✅ 完成 | `grid.md` | 检查点 1，待确认 |
| medium | `src/medium.[ch]` | ✅ 完成 | `medium.md` | 检查点 1，待确认 |
| signal | `src/signal.[ch]` | ✅ 完成 | `signal.md` | 检查点 1，待确认 |
| boundary | `src/boundary.[ch]` | ✅ 完成 | `boundary.md` | 检查点 1，待确认 |
| **解析器** config | `src/config.[ch]` | ✅ 完成，`test_config` 42/42 | `config.md` | **检查点 2：待复审通过后提交** |
| **解析器** source | `src/source.[ch]` | ✅ 完成，`test_source` 20/20 | `source.md` | 检查点 2 |
| **解析器** model | `src/model.[ch]` | ✅ 完成，`test_model` 17/17 | `model.md` | 检查点 2 |
| 求解器 solver | `src/solver.[ch]` | ✅ 实现完成（原子代理被取消，但代码已写完）。`test_solver` 15/15，Release 和 ASan 均通过。🔄 独立审查 `review-solver1` 进行中 | 未写 | 未提交 |
| 渲染器 render | `src/render.[ch]` | 🔄 重新派出 `impl-render4`（fast）。`render.c` 仍是 STUB | 未写 | 未提交 |
| 命令行 | `src/main.c` | ✅ 已写完（主循环、PNG 帧、frames.csv、summary.txt、NaN 检测），等求解器和渲染器完成后才能端到端运行 | 待写（可放在 README 或 cli.md） | 未提交 |
| 验证测试 | `tests/test_validation.c` | ⏳ 占位，还没写 | — | — |

所有模块的头文件（接口契约）都已写好。实现须严格遵守头文件的注释；文档与头文件不一致时以头文件为准，并修正文档。

## 4. 解析器模块（检查点 2）当前情况

- **第一轮独立审查**：结论 REQUEST_CHANGES。没有 Critical 或 Major 问题，Minor 问题 M1–M9 都已由我修复：
  - M1–M6：文档中的错误信息示例和摘要示例与实际输出不一致、README 表格过时、提到了尚不存在的验证测试。已全部修正。
  - M7：`model_build` 现在会对 PML 侧调用 `pml_params_validate`，非法的 R 不会再让 PML 悄悄退化成刚性壁。
  - M8：点源和线源的权重只分配到**物理域内**的单元，不再落进 PML。
  - M9：线源离散改为逐列遍历，计算量与内存随线段长度增长，不再随包围盒面积增长。在 4000×4000 网格上测试，耗时为毫秒级。
  - N1、N2、N4：已修。另外两项新增：障碍物伸进 PML 时发出警告；model 层的错误信息由 main 加上文件名前缀。
- **修复后的验证**：Release 和 ASan 构建下，geometry、config、source、model 四个测试全部通过，没有编译警告，也没有行首的 `+`。
- **增量复审**（`review-parser2`）：APPROVE，附带一个条件 Minor-1：`model.md` 中 nearest valid dx 的示例数值错误。已改为实测值 0.00499501。
- 复审提出的 Minor-2 测试缺口已补上：
  - 上边缘点源的包含单元为固体时报错；
  - 上边缘线段（x = x1 和 y = y1）只用物理单元；
  - 流体与固体交界格线上的竖直线段丢弃一半长度，并发出 1 条警告。
- 仍未覆盖：PML 障碍物警告的文字内容。原因是 common 模块没有捕获警告文本的 API。
- 复审中的 Nit 留待以后处理：
  - Nit-2：`source.c` 中 `n == 0` 是死分支；
  - Nit-3：PML 参数错误信息带侧名，但参数是全局的；
  - 精度：边界坐标 `x1` 有 ulp 级差异，可考虑加容差。
- 2026-11-04 再次验证：Release 和 ASan 下 ctest 全部通过（render 和 validation 为跳过）。
- **检查点 2 已提交给用户检查**，等待反馈。

## 5. 正在运行或待收集的后台任务

| bg_task_id | 角色 | 内容 | 独占写入范围 |
|---|---|---|---|
| `review-solver1_bg1` | independent-reviewer（fast） | 求解器独立审查 | 只读 |
| `impl-render4_bg1` | implementation-engineer（fast） | 渲染器加单元测试。完整规格见压缩记录 8-8493.txt 第 4344 行的 T3 提示词 | `src/render.c`、`tests/test_render.c`；构建目录 `/tmp/fdtd2d-T3*` |

如果这些任务已经丢失，就重新派出。重新派出前先看文件的当前内容，已有的实现可以接着做。旧任务 `review-parser2`、`impl-solver3`、`impl-render3` 已结束或已取消。

## 6. 下一步

1. 收集上面三个后台任务的结果，**亲自检查**代码和测试结果，不只看子代理的总结。
2. 解析器复审通过后，提交给用户检查（检查点 2）。
3. 求解器：独立审查，确认没有 Critical 或 Major 问题。然后写 `docs/api/solver.md`，内容包括离散格式、边界实现、PML 系数、能量函数和性能数据，再提交给用户检查。
4. 渲染器：同样的流程，文档为 `docs/api/render.md`。
5. 集成：
   - 端到端运行两个示例，查看 PNG；
   - 写 `tests/test_validation.c`，验证项包括：
     - 自由场点源与 `(ωρQ/4)·H₀⁽²⁾(kr)` 比较，用 glibc 的 `j0`/`y0`；
     - 两侧刚性壁之间的线源产生平面波，幅值为 `ρcQ′/2`；
     - 刚性壁和压力释放壁的镜像源解；
     - 小域加 PML 与大域参考解比较，检验 PML 反射。
   - 写命令行文档；
   - 做最终验证和审查。
6. 还要问用户：是否 `git init`；配置格式是否认可；文档语言是否统一。

## 7. 关键设计约定（速查）

- **错误处理**：失败返回 `-1`，错误写入 `FdtdError`。外层用 `fdtd_error_prefix` 补充上下文。构建类函数失败时输出结构体为全零；`*_free` 可以作用于全零结构体，也可以重复调用。
- **数组布局**：
  - 单元 `j*nx+i`；
  - x 面 `j*(nx+1)+i`，面 i 位于单元 i−1 与 i 之间；
  - y 面 `j*nx+i`，面 j 位于单元 j−1 与 j 之间。
- **PML 剖面**：`σ(d) = σ_max (d/L)^m`，`σ_max = −(m+1)·c_max·ln R/(2L)`；σ 在交界面上为 0，物理域内为 0。
- **声源注入**：在 t = (n+½)dt 时刻，`px += ½·K·dt·w·s`，`py` 加同样的量。点源满足 `Σ w·dx·dy = 1`；线源满足 `Σ w·dx·dy = 线源有效长度`。
- **求解器的边界规则**：
  - 刚性面和 PML 最外侧面：v = 0；
  - 压力释放边：镜像虚单元取 `p_ghost = −p_inside`；
  - 固体单元：p 恒为 0；接触固体单元的面：v = 0。
