# 时序证据

本工程两套约束，**都已闭环到布线级**，但签核深度不同：

| 约束集 | 时钟 | 状态 |
|---|---|---|
| `constraints/product/board.fdc` | clk_sys 9.876543 ns / clk_cnn 4.938272 ns | ✅ **全整机布线签核** `All Constraints Met.`（2025.2 引擎；基线 WNS +0.395 / +0.113 ns，交付主码流 +0.459 / +0.119 ns） |
| `constraints/27mhz/board.fdc` | 27 MHz 单时钟（PLL 旁路） | ⚠ 仅 CPU-only 工程有过布线签核（WNS +14.080 ns / TNS 0）；**整机 27 MHz 未单独跑过** |

> 早期文档里"产品频率只有布局级证据、CNN 域欠 80 ps"的说法已被推翻：
> 那是 **2022.2 引擎**在一次候选筛选（`soc101_cnn202_pairflat_seed1`，
> `-place_only`）上的数字。换到 2025.2 引擎做完整布线后，同一份 RTL
> 是 **+0.395 / +0.113 ns**。跨引擎的 slack 不可比（见 §4），
> 2022.2 的数字只作历史对照，不再当作结论。

---

## 1. 产品约束：完整布线签核（唯一可信阶段）

判据只认**布线后** `report_timing` 的 Slow Corner WNS（不是 placer 自估的 ps 级
`worst slack`，也不是布局估计）。

### 1.1 基线（`v2_netlist_d1pin3`，PDS 2025.2）

```
Performance Summary:
  clk_sys   requested 101.2500 MHz   estimated 105.4683 MHz   9.8765 → 9.4815   slack +0.395
  clk_cnn   requested 202.5000 MHz   estimated 207.2425 MHz   4.9383 → 4.8253   slack +0.113
Design Summary : All Constraints Met.
```

| 检查项 | clk_sys | clk_cnn |
|---|---|---|
| Setup WNS (slow) | **+0.395 ns**，0/39,310 失败 | **+0.113 ns**，0/144,776 失败 |
| Setup WNS (fast) | +3.966 ns | +1.633 ns |
| Hold WHS (slow / fast) | +0.100 / +0.040 ns | +0.100 / +0.040 ns |
| Recovery / Removal | +2.673 / +3.710 ns | +1.791 / +0.510 ns |
| Min pulse width | +3.978 ns | +1.509 ns |
| 跨域 cnn→sys / sys→cnn | +1.337 / +1.628 ns | — |

报告：`timing_opt/out/facerec_top_202p5_signoff.rtr`（Oct 31 22:16，tool 2025.2）。

### 1.2 交付主码流（`facerec_top_menu_line_202p5.sbit`）

固件 11,357 词 / 44.4 KB，DRM 360/415：
**`All Constraints Met.`（sys +0.459 / cnn +0.119）**。
九个码流同尺寸 9,188,552 B，同器件同流程。

---

## 2. 本次交付树的复核

交付树 = 基线 RTL **剥掉 ILA/PMU/dbg_pc 观测网络** + **预取队列 8→16**，
其余一行未改（`core/` 与 `accel/` 与签核网表逐文件相同）。

复核运行 `release/build/pnr/v3_final`（`tools/build_flow.py`）：RTL = `release/rtl` 原样，
约束 = `timing_clean.sdc` + `d1pin6.pcf`（与基线完全一致），
工具 = 2022.2 综合 + 2025.2 布局布线，任务流（确定性）。

```
Design Summary : All Constraints Met.

时钟      请求        实测        WNS      失败端点
clk_sys   101.25 MHz  103.99 MHz  +0.260   0 / 40,542
clk_cnn   202.50 MHz  207.80 MHz  +0.126   0 / 144,617
```

| 检查项 | clk_sys | clk_cnn |
|---|---|---|
| Setup WNS (slow) | **+0.260 ns**，0/40,542 | **+0.126 ns**，0/144,617 |
| Hold WHS (slow) | +0.098 ns | +0.101 ns |
| 跨域 cnn→sys / sys→cnn | +1.962 / +1.952 ns | — |

报告：`release/build/pnr/v3_final/prj_tasks/pnr_1/report_timing/facerec_top.rtr`
（Nov 3 17:39，tool 2025.2 build 211867）。位流同目录
`generate_bitstream/facerec_top.sbit`，9,188,552 B。

**怎么读这组数**：cnn 从基线的 +0.113 变成 +0.126（+13 ps）、sys 从 +0.395 变成
+0.260（−135 ps），两个都在布局随机抖动范围内（同一份网表不同种子的实测摆动
约 ±0.3 ns），且**都为正、都 All Constraints Met**。变化方向不构成"预取加深伤了
时序"的证据 —— sys 的 −135 ps 与剥掉 `pm_events`（原扇出 112，被刻意打拍）
本应带来的改善方向相反，说明这是布局落点的噪声，不是逻辑改动的影响。

### 2.1 同源复现：位流可以做到"除时间戳外逐字节相同"

复核运行只能证明"当前交付树的时序达标"，不能证明"配方是对的"。
为此单独做了一次**同源复现**：用签核候选的网表 `fc403490…`
（`timing_opt/runs/v2_menu/facerec_top_syn.vm`）走同一条任务流，结果

```
Design Summary : All Constraints Met.
clk_sys  +0.459 ns      clk_cnn  +0.119 ns      ← 与签核报告逐位相同
```

位流比对（`tools/cmp_sbit.py`）：

```
差异字节 6 个；其中落在生成时间戳字段内 6 个，字段外 0 个
RESULT 除生成时间戳外逐字节相同（设计内容等价）
```

即 9,188,552 B 里有 **9,188,546 B 与签核位流完全一致**，只有头部
`c`（日期）/`d`（时刻）两个 TLV 字段的 6 个 ASCII 数字不同。
这证明**布局布线是确定性的**：同一份网表 + 同一套约束 + 同一条任务流，
换任何时刻重跑都得到同一份硬件。详见 `doc/reproduce.md` §5。

### 为什么剥离调试逻辑不可能让时序变差

被删的每一根线都只驱动观测端口，不参与任何控制/数据判决。其中
`pm_events` 当初正是因为**扇出 112** 被刻意打了一拍流水寄存器
（`pm_events_q` / `pm_events2_q`）—— 删掉它等于把这级寄存器的负载整个拿掉，
路径只会更短。`soc_top` 里整块删掉的 47 个 `pm_cnt` 计数器是一个独立状态机
（含 47 位比较、清零/冻结逻辑），与 CPU、cache、CNN 的数据通路无关。

唯一**新增**的硬件是预取队列 8→16：多 8 组 `{addr, data}` 寄存器（512 个 FF）。
关键路径上 `pf_buffer` 比较的是编译期常量 `DEPTH-2`，位宽（5 位指针）
不随深度增长，因此不产生新的关键路径。

---

## 3. 27 MHz 单时钟（保底配置）

产物来源：`timing_opt/runs/cpu27_pnr2`（`cpu27_dbg2`）与 `timing_opt/runs/cml_pnr`
（`cpu27_cmloop`），两次都是 `dev_map → pnr → bit` 全流程跑完：

```
All Constraints Met.
  sys_clk → sys_clk    WNS +14.080 ns    TNS 0    WHS +41 ps
```

两次结果**完全一致**（只换了 BRAM 初值，理应如此）。

**范围必须说清**：这两个 P&R 的顶层是 **CPU-only**（CNN 整块不在），
所以它证明的是「`core/` + `soc/` 在 27 MHz 下裕量极大」，**不能**当作
「CPU+CNN 整机 27 MHz 已签核」。本交付树提供 `FACEREC_NO_PLL`（PLL 旁路，
27 MHz 直驱两域）+ `constraints/27mhz/board.fdc`，需要时按 §5 跑一次即可；
按周期放大 3.75×/7.5× 推断通过没有悬念，但**本文不把推断当证据**。

---

## 4. 工具链事实（会改变结论，务必按这条读）

1. **综合必须用 2022.2，布局布线用 2025.2。**
   2025.2 的综合器在本设计上段错误（`rtl-data-opt` 阶段 `signal 11`），
   所以流程是：2022.2 `synthesize` 出 `facerec_top_syn.vm`（门级 Verilog，51 MB）
   → 2025.2 以 `-project_type post_syn` 导入 → `dev_map` → `place` → `route` → `report_timing`。
2. **跨引擎 slack 不可比。** 同一批 RTL 变体，2022.2 与 2025.2 的排名会**反转**
   （变体间差异 0.06 ns 小于引擎差异 0.35 ns）。所以：
   - 只有 2025.2 的数字能互相比较；
   - 2022.2 时代的历史最好（CPU 104.8878 MHz / CNN 193.1248 MHz，CNN 仍 −0.240）
     **只作历史**，不要再用"CNN 天花板 193 MHz"这个结论 —— 2025.2 上是 207.24。
3. 一轮完整流程约 **26 min**（dev_map 1m19s + place 331 s + route 12 min + report）；
   2022.2 走完整流程要 **2h30m**，慢 6 倍，只用来补综合。

---

## 5. 怎么复现

```bash
cd timing_opt

# 1) 2022.2 综合（约 8 min）→ runs/syn_<name>/synthesize/facerec_top_syn.vm
python tools/v2_synth.py release_clean

# 2) 2025.2 布局+布线+出码流+报告（约 26 min）
python tools/v2_batch.py v2_release

# 3) 读报告（只认 Slow Corner WNS）
python tools/paths.py runs/v2_release/prj_tasks/pnr_1/report_timing/facerec_top.rtr
```

`v2_batch.py` 的四个会话：`build.tcl`（post_syn 建工程 + SDC + dev_map）、
`place.tcl`（+ pcf 钉位 + place）、`route.tcl`（route + gen_bit_stream）、
`rt.tcl`（report_timing）。

> 2025.2 的 license 变量是 `PANGO_LICENSE_FILE`（2022.2 是 `LM_LICENSE_FILE`），
> 两者不通用；本机 2025.2 许可 2026-11-03 到期，之后再出码流需先续许可。
