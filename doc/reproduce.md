# 从 RTL 到产品的位流（复现手册）

本文回答一个问题：**拿到 `release/` 这棵树，怎么一步步做出和交付/签核时一致的位流，并且用可验证的数字证明它是对的。**

结论先给：**时序可以逐位复现，位流可以做到"除头部生成时间戳外逐字节相同"。**

---

## 1. 两种"复现"，目标不同

先想清楚要复现什么，两者的判据不一样：

| 目的 | 用什么 RTL | 判据 | 结果 |
|---|---|---|---|
| **A. 验证配方可信**（能不能重放出签核那份硬件） | 签核候选 RTL / 其网表 `fc403490…` | 时序逐位相同 **且** 码流除时间戳外逐字节相同 | ✅ 已通过（§1.1） |
| **B. 验证交付树达标**（这棵树自己能不能过） | `release/rtl` 原样 | 只有 `All Constraints Met.`，码流内容本就该不同 | ✅ 已通过（§1.2） |

**为什么 B 的码流不该和 A 一样**：交付树改了 RTL（预取 4→16、剥调试网络、
`RAM_ADDR_W` 16、固件内嵌），逻辑变了，布局布线自然不同。B 的正确判据是
时序达标 + 功能回归通过，不是 md5 相同。

### 1.1 A：同源复现签核码流（已验证）

用 `timing_opt/runs/v2_menu/facerec_top_syn.vm`（md5 `fc403490…`）走 `v2_flow.py` 任务流：

```
Design Summary : All Constraints Met.
clk_sys  +0.459 ns      clk_cnn  +0.119 ns
```

与 `timing_opt/out/facerec_top_202p5_signoff.rtr` / `v2_menu` 的报告**逐位相同**。

位流 `release/build/pnr/v2_repro_menu_line/prj_tasks/pnr_1/generate_bitstream/facerec_top.sbit`：

```
差异字节 6 个；其中落在生成时间戳字段内 6 个，字段外 0 个
RESULT 除生成时间戳外逐字节相同（设计内容等价）
```

→ 结论：**配方（RTL + 约束 + 任务流 + 工具版本）完全可重放，且布局布线是确定性的。**

### 1.2 B：交付树自身签核（已验证）

`release/rtl` 原样 → 综合 → `v2_flow.py`：

```
Design Summary : All Constraints Met.
clk_sys  101.25 → 103.99 MHz   WNS +0.260 ns   0/40,542 失败
clk_cnn  202.50 → 207.80 MHz   WNS +0.126 ns   0/144,617 失败
```

产物：`release/build/pnr/v3_final/`（报告 `report_timing/facerec_top.rtr`，
码流 `generate_bitstream/facerec_top.sbit`，9,188,552 B）。

---

## 2. 复现目标（判据速查表）

| 判据 | A（同源复现） | B（交付树达标） |
|---|---|---|
| `Design Summary` | `All Constraints Met.` | `All Constraints Met.` |
| `clk_sys` WNS | +0.459 ns | +0.260 ns（±0.3 抖动内） |
| `clk_cnn` WNS | +0.119 ns | +0.126 ns（±0.3 抖动内） |
| 码流尺寸 | 9,188,552 B | 9,188,552 B |
| 码流内容 | 除时间戳外与 `692e2dc7…` 逐字节相同 | 与 A 不同（预期） |

> 参考位流：`timing_opt/out/facerec_top_menu_line_202p5.sbit`（md5 `692e2dc7…`）。
> 它由 `timing_opt/runs/v2_menu` 产出。

---

## 3. 三条硬性事实（搞错任何一条，数字都是假的）

### 3.1 必须两段式，引擎不同

| 阶段 | 引擎 | 为什么 |
|---|---|---|
| 综合 | `E:/PDS_2022.2-SP6.4` | 2025.2 合成器在本设计上**段错误**（`rtl-data-opt` 阶段 `signal 11`） |
| 布局 + 布线 + 出码流 + 报告 | `E:/pango/PDS_2025.2-ads` | 只有 2025.2 的数字能互相比较（跨引擎排名会**反转**） |

### 3.2 综合时工作目录里必须有 `rtl/`

RTL 里 ROM 初始化写的是**相对路径** `rtl/accel/mem/*.mem`，PDS 按**当前工作目录**解析。
工作目录里没有 `rtl/` 时它**静默**把权重/描述符/帧 ROM 读成空（只报 `C: Verilog-3001`），
综合照样"成功"，但 DRM/FF 腰斩 —— 网表是假的。

另外 `soc_top.v` 用 `#include "rv32i_defs.v"`，PDS 在 `work_dir` 里找这个文件。
`tools/synth_vm.py` 已把这两件事都处理了（拷贝 `rtl/` 到运行目录 + 拷 `rv32i_defs.v` 到根）。

**自检**：综合完看 `synth.log` 里 `Cannot open specified data file` 必须为 0，
且网表里 `grep -c shc_q_r` 应 ≈770（这是 menu 候选相对旧 pairflat 候选的关键改动）。

### 3.3 SDC 必须在 `add_design` 之前

2025.2 的 `create_project -project_type post_syn` 里先 `add_constraint`（SDC）
再 `add_design`（.vm），否则时钟定义不生效。`v2_flow.py` 已按此顺序。

### 3.4 许可

| 变量 | 用于 | 本机有效文件 |
|---|---|---|
| `LM_LICENSE_FILE`（`27001@LAPTOP-JRJMGR56`） | 2022.2 | 浮动许可服务器 |
| `PANGO_LICENSE_FILE` | 2025.2 | 指向你自己的 PDS 许可证（与 hostid 绑定，不随仓库提供） |

> 注意：`E:/downloads/new_pds_F4A80DA47D72.lic` 已于 2026-11-03 到期，**不要再指向它**。
> 想确认许可活着，必须跑一条真正 checkout 的命令（如 `create_project`），
> 并看日志首行 `License checkout: ... from <路径>`；只跑 `exit` 什么也不 checkout。

---

## 4. 一键流程（推荐）

```bash
cd release

# 全流程：综合 → 布局布线 → 出码流 → 报告 + 可选码流比对
python tools/build_flow.py rel_final \
       --ref-sbit ../timing_opt/out/facerec_top_menu_line_202p5.sbit
```

耗时：综合 ≈14 min，布局 ≈13 min，布线 ≈25 min，报告 ≈2 min，
**合计约 55 min**。产物全部在 `build/` 下：

```
build/
├── syn_rel_final/
│   ├── synth.tcl / synth.log          综合脚本与日志
│   └── synthesize/facerec_top_syn.vm  门级网表（约 50 MB）
└── pnr/rel_final/
    ├── build.tcl  place.tcl  route.tcl  rt.tcl
    ├── build.log  place.log  route.log  rt.log
    ├── timing_summary.json            ← 解析后的结论
    └── prj_tasks/pnr_1/
        ├── report_timing/facerec_top.rtr   ← 签核报告
        └── generate_bitstream/facerec_top.sbit
```

---

## 5. 分步流程（要改中间产物时用）

```bash
cd release

# ---- 步骤 1：综合（2022.2）→ build/syn_<n>/synthesize/facerec_top_syn.vm
python tools/synth_vm.py rel_final

# ---- 步骤 2：布局布线（2025.2，任务流，确定性）→ build/pnr/rel_final/
PANGO_LICENSE_FILE="<你的 PDS 许可证路径>" \
  python tools/v2_flow.py rel_final \
         --netlist build/syn_rel_final/synthesize/facerec_top_syn.vm

# ---- 步骤 3：读结论（只认 Slow Corner WNS）
grep -A6 "Performance Summary" \
  build/pnr/rel_final/prj_tasks/pnr_1/report_timing/facerec_top.rtr

# ---- 步骤 4：与参考码流比对（忽略时间戳）
python tools/cmp_sbit.py --verbose \
  ../timing_opt/out/facerec_top_menu_line_202p5.sbit \
  build/pnr/rel_final/prj_tasks/pnr_1/generate_bitstream/facerec_top.sbit
```

`v2_flow.py` 的四种会话对应四个 tcl：

| tcl | 内容 |
|---|---|
| `build.tcl` | `create_project -project_type post_syn` → `add_constraint`(SDC) → `add_design`(.vm) → `dev_map` |
| `place.tcl` | `add_constraint`(PCF 15 条钉位) → `launch_tasks -to_action place` |
| `route.tcl` | `launch_tasks -to_action route` → `-to_action gen_bit_stream` |
| `rt.tcl` | `clean -task [get_tasks {pnr_1}]` → `launch_tasks ... report_timing` |

> `--direct-pnr N --seed S` 会换成 `pnr -gplace_seed S -gplace_times N` 直连路径。
> 那条路径**不能**复现历史码流（它是另一种布局算法），只用来多种子找更好的 slack。

---

## 6. 为什么 md5 不同而结论"等价"

`.sbit` 头部有一段 TLV，两个字段记录**生成时刻**：

```
offset 80:  00 63 00 09 "11/02/26"     ← c = 生成日期 MM/DD/YY
offset 92:  00 64 00 09 "01:28:08"     ← d = 生成时间 HH:MM:SS
```

同一份网表 + 同一套约束 + 同一条任务流，换个时刻重跑，**只有这 18 字节里的 6 个数字不同**，
其余 **9,188,546 字节逐字节相同**。`tools/cmp_sbit.py` 正是按这个规则判定：

```
差异字节 6 个；其中落在生成时间戳字段内 6 个，字段外 0 个
RESULT 除生成时间戳外逐字节相同（设计内容等价）
```

这比"md5 相等"更强也更有意义 —— md5 相等只能说明"同一时刻跑过两次"，
而"除时间戳外逐字节相同"证明的是**布局布线结果完全确定性**。

> 注意 `facerec_top_menu_min_202p5.sbit` 的 md5 与 line 版**不同**（`bf901588…`），
> 这是另一次跑（不同 BRAM 初值/种子），但时序逐位相同。台账里
> "时序逐位相同"指的就是这个，不是"逐字节相同"。

---

## 7. 目录即输入：什么变了就得重跑什么

| 改动 | 要重跑 |
|---|---|
| `rtl/**/*.v`（含 `simple_ram` 里的固件初值） | 综合 + P&R（全流程） |
| `sw/*` 固件源码 | `build_menu_fw.py` 重新内嵌 → 再全流程 |
| `constraints/product/board.fdc` | 综合（fdc 是综合约束）+ P&R（SDC 同源，需同步 `signoff/timing_clean.sdc`） |
| `constraints/product/signoff/*.sdc / *.pcf` | 只 P&R |
| `accel/mem/*.mem` | 综合 + P&R（BRAM 初值变了） |

**约束是两份，必须同源**：`board.fdc` 给综合（含 IO 位置/标准），
`signoff/timing_clean.sdc` 给 2025.2 的时钟定义（同一组 `create_clock` 周期）。
当前两者周期一致：9.876543 / 4.938272 ns。

---

## 8. 常见坑

1. **`report_timing` 报 "already been run"** —— 需先重置。2025.2 没有 `reset_tasks`，
   且 `clean` 的 `-task` / `-action` / `-all` **只能给其中一个**，写成
   `clean -task X -action Y` 会报 `E: Flow-0153`。正确写法：
   `clean -task [get_tasks {pnr_1}]`（实测 `Executing : clean -task pnr_1 successfully.`）。
2. **只看 placer 自估的 `worst slack`** —— 那是布局估计，不是签核数字。
   只认 `report_timing` 的 `Setup Summary(Slow Corner)` 的 WNS。
3. **拿 2022.2 的 slack 和 2025.2 比** —— 不可比，排名会反转。
   历史上"CNN 天花板 193 MHz"就是 2022.2 的结论，2025.2 上是 207.5。
4. **网表血统搞错** —— `accel/cnn_engine.v` / `cnn_outacc.v` 必须用
   `timing_opt/candidates/cpu101_cnn202_menu/` 的版本（含 `shc_q_r` 打拍）。
   用旧的 pairflat 版本会让 `clk_cnn` 从 +0.119 掉到 **−0.356**（实测差 0.47 ns）。
5. **固件没重编译就重跑综合** —— BRAM 初值不变，等于没改。
