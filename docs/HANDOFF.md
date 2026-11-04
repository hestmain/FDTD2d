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

Git：仓库由用户的 IDE（CLion）初始化，分支为 `master`，没有远程仓库。
- 首次提交 `3ea6b88`：核心模块、解析器、求解器、命令行、测试、示例和文档；渲染器为占位代码。
- `.gitignore` 忽略 `build*/`、`cmake-build-*/`、`.idea/` 和 `output/`。
- 用户没要求时不提交，也不推送。

用户已确认：`examples/*.toml` 的配置格式认可；文档中文、代码注释英文保持不变。

用户决定：`src/signal.[ch]` 改名为 `src/waveform.[ch]`，已完成，`docs/api/signal.md` 同步改为 `waveform.md`。类型、函数名（`Signal`、`signal_*`）和 TOML 键 `signal` 保持不变。现在用 `-Isrc` 也能拿到系统的 `<signal.h>`，已用实际编译验证。

用户要求（✅ 已完成，等待用户检查）：检查不必要的防御性编程并安全去除。
- 结果：src 由 4793 行减到 4547 行，测试只删除了针对被删检查的用例。
- 复审 `review-defclean` 结论为 APPROVE。它的 m1 已修：派生出的 dt 可能上溢，`model.c` 已补上 `isfinite(dt)`，并加了测试。m3 也已修：solver 保留下来的 K 和 σ 检查补了测试 `test_init_rejects_unusable_coefficients`。两项都做过变异检查，确认测试能抓到问题。
- 验证：
  - Release 和 ASan 构建下 ctest 全部通过：geometry 17、config 42、model 17、source 18、solver 23、render 21；
  - 两个示例的 `--check` 输出、frames.csv 和全部 PNG 与清理前的二进制逐字节一致。
- 未处理：
  - 复审的 m4：main.c 提前做的 65535 检查没有 CLI 测试；
  - Nit：`cell_is_solid` 可以内联，test_model 里有一处重复的 CHECK。只读审查分两路：
- `audit-def-a`：config、main、common、boundary、medium、waveform、geometry；
- `audit-def-b`：model、source、solver、render、grid。

判定标准：
- 保留：信任边界检查、真实运行时条件、已文档化并被清理路径依赖的约定。
- 可删：不可达代码（D1）、对上游已保证数据的重复校验（D2）、内部 API 的编程错误检查（D3）、冗余覆盖（D4）、类型上不可能的条件（D5）。

去除前的快照在 `/tmp/fdtd2d-snapshot-cp34`，内容是检查点 3/4 的未提交状态，可以用来把求解器/渲染器和本次清理拆成两个提交。去除前的二进制和参考输出在 `/tmp/fdtd2d-ref/`。

我的决策（审计 ID 见两份审计报告）：
- **原则**：只在信任边界校验一次。
  - config.c 校验全部用户输入；
  - model 只校验与网格相关的事实；
  - solver 和 render 依赖文档写明的前置条件。
- **保留**：
  - 各 `*_validate` 函数；
  - `err` 可为 NULL、`*_free` 幂等、构建失败时把输出清零这三条约定；
  - 分配、I/O 和尺寸溢出检查；
  - solver 的 V6 溢出、V10（K 上溢/下溢）、V11（σ 有限）；
  - render 的 65535 上限、轮廓裁剪、colormap 的 NaN 处理；
  - config 的 F4、F5、F13、F16；
  - `shape_validate` 的有限性检查（G1–G3），W1–W3；
  - main 的 M2、CLI 检查和 NaN 扫描。
- **去除**：
  - main：M1、M3；
  - config：F1–F3、F6–F12、F14、F15；
  - boundary/waveform/geometry：B2、B3、G4；
  - model：M1、M2、M4–M19、M21、M22（M8 需先证明）；
  - source：S1–S5，S6 合并；
  - medium：init/add 里的重复 validate、count 守卫；
  - solver：V1–V5、V7–V9、V12、V13、V15–V18；
  - render：R1–R5、R7。
- **已知风险**：去掉 V7、V9、R4 以后，生产者如果出错，会造成静默越界。现在依靠生产者自身的保证、`check_source_sane` 测试和 ASan 构建兜底。

实施分三路并行，文件互不重叠：
- `impl-def-input`：config、main、geometry、boundary、waveform 及其测试和文档；
- `impl-def-model`：model、source、medium 及其测试和文档；
- `impl-def-solver`：solver、render 及其测试和文档。

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
| waveform（原 signal） | `src/waveform.[ch]` | ✅ 完成 | `waveform.md` | 检查点 1，待确认 |
| boundary | `src/boundary.[ch]` | ✅ 完成 | `boundary.md` | 检查点 1，待确认 |
| **解析器** config | `src/config.[ch]` | ✅ 完成，`test_config` 42/42 | `config.md` | **检查点 2：待复审通过后提交** |
| **解析器** source | `src/source.[ch]` | ✅ 完成，`test_source` 18/18（防御清理后） | `source.md` | 检查点 2 |
| **解析器** model | `src/model.[ch]` | ✅ 完成，`test_model` 17/17 | `model.md` | 检查点 2 |
| 求解器 solver | `src/solver.[ch]` | ✅ 审查意见已修复，`test_solver` 22/22，ASan 通过 | `solver.md` | **检查点 3，已提交给用户** |
| 渲染器 render | `src/render.[ch]` | ✅ 审查 APPROVE，Minor 已修，`test_render` 21/21（防御清理后），ASan 通过 | `render.md` | **检查点 4，已提交给用户** |
| 命令行 | `src/main.c` | ✅ 两个示例端到端跑通（Release 和 ASan）。新增：模型构建后、写任何文件前检查帧尺寸是否超过 65535 | 待写（README 或 cli.md） | 未提交 |
| 验证测试 | `tests/test_validation.c` | ⏳ 占位，还没写 | — | — |

求解器审查 `review-solver1` 的问题：
- **Major M1**：测试全部使用方形单元，dx≠dy 以及 x/y 方向互换都没有覆盖。
- **Minor m1**：压力释放边界外层面的能量应取半权。我已手推确认。
- **Minor m2–m4**：需要加强相关测试。

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
当前没有后台任务。`solver.md`、`render.md` 已写完，我已抽查并修正。同时修正了几处注释：
- `source.h`：线源权重单位改为 1/m；
- `render.h`：宏名改为 `RENDER_SOLID_R/G/B`，零值颜色写明为 247；
- `test_solver.c`：“121 步”改为约 96 步；
- `config.md`：`show_pml` 的说明。

已完成的任务：
- **`impl-solver4`**：M1、m1–m4 和 nits 全部处理。`test_solver` 共 22 项，Release 下约 1.05 s，ASan 下约 11 s。变异检查确认新测试能发现 dx/dy 互换和半权错误。我已检查 diff 并复跑通过。
- **`review-render1`**：结论 APPROVE，有 Minor 问题。我已修复：
  - 失败时删除文件前先 `lstat`，并比对 dev/ino，不会误删符号链接；
  - 新增 crop 偏移 + scale 3 的轮廓测试；
  - 新增 `-inf` 视为自动范围的测试；
  - `render.h` 补充了非有限值和清理规则的说明。

  `test_render` 改为 22 项，防御清理后为 21 项。未处理的问题：`/dev/full` 以外的写失败路径没有测试（signal.h 遮蔽已因改名解决，可补测试），fd 检查偏弱，错误信息缺少 errno。

性能（Release -O3，Ryzen 9 7845HX）：149600 个单元跑 5544 步用时 1.37 s，约 6e8 单元·步/秒。

端到端运行：两个示例都已跑通，每个约 0.5–0.7 s，各输出 58 个文件，PNG 目视正确。点源示例设了 `color_limit = 1.0`，r < 1 m 的区域颜色饱和。这是显示设置造成的，不是缺陷：实测幅值与 ωρQ/4·√(2/πkr) 的估算一致。

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
