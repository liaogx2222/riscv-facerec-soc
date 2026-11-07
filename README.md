# FaceRec SoC — RISC-V 人脸识别视觉 SoC

在紫光同创 **PG2L200H-6-FBB676** FPGA 上实现的异构片上系统：双发射 RV32I 处理器
（101.25 MHz）与人脸识别 CNN 加速器（202.5 MHz）经保持总线与 toggle 握手跨域协作，
完成滑窗检测、ROI 缩放、64 维特征提取与门禁判定。整机在实板实测 CoreMark
**5.169 CoreMark/MHz**、人脸计算吞吐约 **60 帧/秒**，三份码流均 All Constraints Met。

[技术报告](docs/技术报告_RISC-V人脸识别视觉SoC_最终提交版.pdf) · [三分钟展示](docs/RISC-V人脸识别视觉SoC_三分钟展示_精修版.pptx) · [复现手册](doc/reproduce.md)

本仓库是「睿识芯 RISC-V 人脸识别视觉 SoC」的**完整功能**源码树：
双发射 RV32I CPU + FaceRec CNN 加速器 + ROM 帧源 + UART 菜单固件，
**已剥离全部 ILA / PMU / dbg_pc 观测逻辑**，保留产品运行所需的全部功能。

```
riscv-facerec-soc/
├── rtl/                  综合用 RTL（唯一真源）
│   ├── core/             CPU：5 级顺序双发射 RV32I + Zmmul（17 个文件含 rv32i_defs.v）
│   ├── soc/              SoC：dcache/icache/pf_buffer/intc/uart/simple_ram/soc_top/CDC 桥
│   ├── accel/            FaceRec CNN 加速器（27 个 .v + 15 个 ROM .mem）
│   ├── board/            facerec_top（板级顶层）+ pll（27→101.25/202.5 MHz）
│   └── files.f           综合文件清单（路径相对本目录）
├── constraints/
│   ├── product/board.fdc 产品约束：clk_sys 9.876543 ns + clk_cnn 4.938272 ns
│   ├── product/signoff/  2025.2 签核用 timing_clean.sdc + d1pin6.pcf（15 条钉位）
│   └── 27mhz/board.fdc   27 MHz 单时钟约束（保底配置，见 doc/timing.md §3）
├── tb/                   自检测试台 + 固件镜像 + CNN 黄金向量
├── sw/                   固件源码（菜单固件 + CoreMark + 门禁识别）
├── tools/                构建 / 综合 / 布局布线 / 位流比对脚本
├── doc/                  reproduce.md（RTL→位流的复现手册）、timing.md（时序证据）、
│                         optimization.md（逐项优化实测）、tradeoff.md（频率↔性能的账）
├── MANIFEST.md           输入指纹清单（RTL/约束/固件/ROM 的 sha256）
├── bitstreams/           三份最终码流 + MD5S.txt
├── signoff/              各配置最终 STA（.rtr）与布线资源（.rur）报告
├── configs/stable_menu/  稳定演示配置的 6 个数据文件（按 README.txt 覆盖即切换）
└── docs/                 技术报告 PDF、三分钟展示 PPT
```

**只想复现位流，看 [`doc/reproduce.md`](doc/reproduce.md)，一条命令：**

```bash
python tools/build_flow.py rel_final \
    --ref-sbit bitstreams/facerec_top_menu_line_202p5.sbit
```

---

## 1. 包含的功能

| 模块 | 说明 |
|---|---|
| CPU | 5 级顺序、**双发射**（每拍最多 2 条退休）、RV32I + Zmmul；I-cache 8 KB 2-way、D-cache 2 KB（32 组×2 路×32 B）、16 项预取缓冲、BTB 256 项、RAS、4 项 store buffer、双乘法器 |
| CNN 加速器 | 描述符驱动序列器、滑窗/行缓冲、分页 PE 阵列、输出累加、ROI 裁剪、权重/移位 ROM |
| 帧源 | ROM 帧源（`accel/mem/framesrc.mem`，需用 `tools/pack_frames.py` 生成，见 §3.4），CPU 经 `0x6000_0000` 读取 |
| SoC | RAM 64 KB（真双端口）、UART、GPIO、64 位周期计数器 + INTC、CPU/CNN 异步域 hold-until-ack 握手桥 |
| 固件 | 串口菜单：`1` 跑 CoreMark 6,000 次迭代（≈11.6 s，满足赛题 ≥10 s）；`2` 跑门禁人脸识别 |

MMIO 映射：RAM `0x0000_0000`、UART `0x1000_0000`、GPIO `0x2000_0000`、
定时器 `0x3000_0000`、INTC `0x3000_0040`、CNN `0x4000_0000`、帧源 `0x6000_0000`。

---

## 2. 已验证结论

### 2.1 CoreMark 2K（功能 + 性能）

`tb/tb_coremark.v` 跑完整一轮 2K 工作负载，UART 输出的四行 CRC 必须等于
CoreMark 官方参考值：

| 项 | 值 |
|---|---|
| crclist / crcmatrix / crcstate / crcfinal | `0xe714` / `0x1fd7` / `0x8e3a` / `0xe714` ✅ |
| Total ticks（优化前 → 优化后） | 194,583 → **190,553**（−2.07%） |
| 指令数 | 274,696 → 274,697（不变） |
| IPC（指令数 / ticks） | 1.412 → **1.442**（+2.1%） |

> 仿真里 CoreMark 会打印 `ERROR! Must execute for at least 10 secs`。这是
`core_main.c` 用**实际墙钟秒数**判定跑分时长导致的：仿真不消耗真实时间，
`time_in_secs()` 恒为 0。上板 101.25 MHz、6,000 次迭代约 11.6 s，不会误报。
CRC 四行与官方一致，才代表结果正确。

### 2.2 整机集成（CPU + CNN 端到端）

`tb/tb_system_integration.v` 跑完整识别链路：CPU 配描述符 → 帧源/ROI →
滑窗 → PE 阵列 → 输出累加 → 64 B embedding → CPU 突发回读比对：

```
CPU seek/fsrc/ACT readback passed at cycle 12507; D1 fixture loaded
SYSTEM_INTEGRATION_PASS cycles=17402 embedding=64/64 cpu_readback=16/16
```

即 64 字节 embedding 与黄金向量逐字节相等、CPU 侧 16 拍突发回读全部一致。

### 2.3 时序（布线级签核）

产品约束（clk_sys 9.876543 ns / clk_cnn 4.938272 ns）**已完整布线签核**：

| 运行 | 对象 | clk_sys WNS | clk_cnn WNS | 结论 |
|---|---|---|---|---|
| 基线 `v2_netlist_d1pin3` | 签核候选 RTL | +0.395 ns | +0.113 ns | All Constraints Met |
| 主码流 `menu_line_202p5` | 签核候选 RTL | +0.459 ns | +0.119 ns | All Constraints Met |
| **本交付树 `v3_final`** | **`release/rtl` 原样** | **+0.260 ns** | **+0.126 ns** | **All Constraints Met** |

交付树实测：sys 103.99 MHz（0/40,542 失败）、cnn 207.80 MHz（0/144,617 失败），
hold +0.098 / +0.101 ns。位流 `9,188,552 B`。

**同源复现已验证**：用签核网表 `fc403490…` 走同一条任务流，复现出
`+0.459 / +0.119`（与签核报告逐位相同），位流 **除头部生成时间戳外逐字节相同**
（9,188,546 / 9,188,552 B 一致）。即布局布线是确定性的，配方可重放。

详见 `doc/timing.md`（时序证据）与 `doc/reproduce.md`（复现手册）。

---

## 3. 怎么跑

以下命令都在**仓库根目录**下执行。

### 3.1 功能仿真（Icarus Verilog）

```bash
# CoreMark 回归（约 21 万拍 / 5 分钟），末行打印 TB_PASS + 四行 CRC
iverilog -g2012 -s tb_coremark -I rtl/core -I rtl/soc \
  -o /tmp/cm.vvp rtl/core/*.v rtl/soc/*.v tb/tb_coremark.v
vvp /tmp/cm.vvp

# 整机 CNN 端到端回归，末行打印 SYSTEM_INTEGRATION_PASS
# 注意：FACEREC_EN=1 会拉进整个 accel/，且 sim_write/sim_read 钩子在
#       `ifdef FACEREC_SIM 内，因此必须带这个宏、并编译 files.f 全量。
iverilog -g2012 -DFACEREC_SIM -s tb_system_integration \
  -I rtl/core -I rtl/soc -I rtl/accel \
  -o /tmp/sys.vvp $(cat rtl/files.f) tb/tb_system_integration.v
vvp /tmp/sys.vvp
```

> 板级顶层 `facerec_top` 例化了 Pango 原语 `GTP_GPLL/GTP_CLKBUFG`，
> Icarus 里没有这些模型，因此仿真直接例化 `soc_top`（等价于
> `FACEREC_NO_PLL` / `FACEREC_SIM` 的时钟连接）。
>
> 想只检查「整棵树是否完整」（不做仿真）可以旁路 PLL 展开一次：
> ```bash
> iverilog -g2012 -DFACEREC_NO_PLL -s facerec_top \
>   -I rtl/core -I rtl/soc -I rtl/accel -o /tmp/all.vvp $(cat rtl/files.f)
> ```
> 只会剩下 `cnn_soc.v` / `cnn_engine.v` 里几条既有的位宽告警（PDS 同样会报）。

### 3.2 综合 + 布局布线（Pango PDS）

**签核流程是两段式的，引擎不同，别混用**：

| 阶段 | 引擎 | 原因 |
|---|---|---|
| 综合 | **2022.2-SP6.4** | 2025.2 综合器在本设计上段错误（`rtl-data-opt` 阶段 `signal 11`） |
| 布局 + 布线 + 出码流 | **2025.2** | 只有 2025.2 的数字互相可比（跨引擎排名会反转） |

**推荐用法 —— 一条命令走完全程**（含码流比对，约 55 min）：

```bash
python tools/build_flow.py rel_final \
       --ref-sbit bitstreams/facerec_top_menu_line_202p5.sbit
```

它依次调 `synth_vm.py` → `v2_flow.py` → `cmp_sbit.py`，最后打印
`Design Summary`、两个时钟的 slack、位流 md5 和"除时间戳外是否逐字节相同"。

分步用法（要单独改某一段时）：

```bash
# 1) 2022.2 综合 → build/syn_<n>/synthesize/facerec_top_syn.vm
python tools/synth_vm.py rel_final

# 2) 2025.2 布局布线（把 .vm 按 post_syn 导入）→ build/pnr/rel_final/
#    默认 SDC/PCF 就是签核用的那一套，位于 constraints/product/signoff/
PANGO_LICENSE_FILE=<你的 PDS 许可证路径> \
  python tools/v2_flow.py rel_final \
         --netlist build/syn_rel_final/synthesize/facerec_top_syn.vm

# 3) 与参考码流比对（忽略头部生成时间戳）
python tools/cmp_sbit.py --verbose \
  ../timing_opt/out/facerec_top_menu_line_202p5.sbit \
  build/pnr/rel_final/prj_tasks/pnr_1/generate_bitstream/facerec_top.sbit
```

> `synth_vm.py` 负责两件容易踩的事：把 RTL 覆盖式拷到运行目录（ROM 的相对路径
> `rtl/accel/mem/*.mem` 才解析得到），以及把 `rv32i_defs.v` 拷到工作根目录
> （`soc_top.v` 的 `include` 才找得到）。缺任一件，综合会"成功"但网表是假的
> —— 详见 `doc/reproduce.md` §3.2。

`v2_flow.py` 默认走**任务流**（place → route → route_optimize → report_timing →
gen_bit_stream），这条路径是确定性的：同一份 `.vm` + 同一套约束，重复跑出来的
`.sbit` 逐字节相同。想要多种子找更好 slack 时加 `--direct-pnr N --seed S`
（换成 `pnr -gplace_seed` 直连，但那条路径**不能**复现历史码流）。

只想看**布局级**趋势（30 分钟、不签核）时，`run_pds.py` + `execute_pds.py
--through place` + `report_placement.py` 仍然可用，但那种数字**只能用来筛候选**，
不能当结论（见 `doc/timing.md` §4）。

27 MHz 单时钟（保底配置）：`--single-clock 27` 会自动旁路 PLL
（`-compiler_directives "+define+FACEREC_NO_PLL"`）、把 `CLK_HZ`/`CNN_CLK_HZ`
改成 27 MHz、并按 37.037037 ns 校验 FDC：

```bash
python tools/run_pds.py my_27m --profile gate --single-clock 27 \
  --constraint-file constraints/27mhz/board.fdc --bitstream --prepare-only
python tools/execute_pds.py my_27m          # 全流程 dev_map → pnr → bit
```

`run_pds.py` 会把 RTL、约束、固件的 sha256 一起冻结进 `build/pds/<run>/`，
之后任何一次读数都能追溯到确切的输入。

### 3.3 重编固件

```bash
# 需要 xpack-riscv-none-elf-gcc 15.2.0；用 RISCV_TOOLCHAIN 环境变量指向其 bin 目录
python tools/build_menu_fw.py             # 菜单固件：编译 + 内嵌进 rtl/soc/simple_ram.v
python tools/build_menu_fw.py --no-ram    # 只编译到 build/fw/

# 整机回归镜像（tb/integration.mem，158 词补齐到 8192 词）
python tools/build_integration_fw.py --check   # 重建并逐字节比对现有镜像
```

镜像以 `initial` 逐字写进 `simple_ram` 的 `mem`，由 `EMBED_INIT` 参数门控
（板级=1，仿真=0 并改用 `INIT_FILE`）。改完固件必须重跑综合，否则 BRAM 初值不变。

### 3.4 生成帧 ROM（可选，仓库不附带人脸图像）

`rtl/accel/mem/framesrc.mem` 为四帧人脸图像的 ROM 转储，原数据来自 ChokePoint
研究数据集，**不随仓库分发**。仿真与重建码流前请自行生成等价 ROM：

```bash
# 用自己的图片（任意尺寸，脚本会缩放到 128×96 并转 24b RGB）
python tools/pack_frames.py --out rtl/accel/mem/framesrc.mem img1.png img2.png img3.png img4.png

# 没有图片时生成测试图案（梯度+运动方块），可用于验证流水线是否跑通
python tools/pack_frames.py --synthetic --out rtl/accel/mem/framesrc.mem
```

ROM 格式与 `rtl/accel/cnn_framesrc.v` 一致：每行一个 24b 字（`%06X`，R 在高字节），
按 `y*FW+x` 排列，每帧占 `STRIDE=16384` 个字的对齐跨度。

---

## 4. 与「带 ILA 的调试版」的差异

剥离的**只有观测网络**，没有触碰任何功能路径：

| 文件 | 剥离内容 |
|---|---|
| `core/cpu_rv32i.v` | `dbg_pc`、`pm_events`、`pm_events2`、`pm_dual`、`pm_dual2/3`、`pm_pf3`、`pm_retire2` 端口与驱动；`pm_events_q/pm_events2_q` 打拍寄存器 |
| `soc/soc_top.v` | `dbg_pc` 端口与连线、全部 `pm_*` 观测线、`pmhi_sel` 译码、`pm_cnt[0:46]` 47 个计数器整块、`pm_clear/pm_stop/pm_frozen`、ICache busy 观测、预取器观测口、dcache 统计口 |
| `board/facerec_top.v` | `.dbg_pc()` 连线 |

**为什么这不影响时序、也不会变差**：被删的每一根线都只驱动观测端口，
不参与任何控制/数据判决；其中 `pm_events` 当初正是因为**扇出 112**才被刻意
打了一拍流水 —— 删掉它只会缩短路径，不可能拉长。

保留：64 位周期计数器（`0x3000_0000/0004`，固件要用）、UART、GPIO、INTC、
cache、预取器、BTB，以及 `cpu_stall` 状态输出。

---

## 5. 优化（详见 `doc/optimization.md`）

按分析报告第 8 章的方向做**逐项实测**，只有测出正收益的才进交付树：

| 项 | 结论 | 实测 |
|---|---|---|
| 预取队列 8 → 16 | **采纳** | ticks −2.07%，取指未就绪 10,144→7,844 拍，IPC +2.1% |
| 放宽"双内存操作"配对（报告 §8.1 的 P2 放宽） | **否决** | 13–14 k 次拒绝里 ld+ld 占 76%、st+ld（报告指的正是这条）只占 12%，上限 ≤0.9% IPC，且要动数据端口仲裁 |
| 加宽取指消耗宽度 | 未实施 | 需重构 ICache 读口，风险与收益不匹配（见 §5.3） |
| 数据通路单周期化（长期） | 超出本次范围 | — |

预取深度 8→16 的代价只是 8 组 `{addr,data}` 寄存器；关键路径上比较的是
**常量** `DEPTH-2`，与深度无关，因此不进入新的关键路径。

**为什么把钱花在 IPC 而不是频率**：见 `doc/tradeoff.md`。一句话 —— 签核余量
是 CPU +4.16% / CNN +2.34%，而 PLL 的下一档是 **+6.67%**（VCO 810→864），
一步就跨过全部余量；频率这条路上没有 2% 的增量可买，而 IPC 上刚好有。

---

## 5.5 两套数据配置与三份码流的对应

RTL 逻辑只有一个版本；换模型只改数据（4 个模型 ROM + `simple_ram.v` 固件初值 +
`tb/flow_emb.hex` 仿真金标），网表结构与布局不变。本仓库 `configs/stable_menu/`
存放稳定演示配置的 6 个数据文件，按其中 README.txt 覆盖即切换：

| 配置 | 模型 | 对应码流（bitstreams/） | 实测时序 |
|---|---|---|---|
| 默认（包内 rtl/ 现状） | 五人 QAT（0013/0014/0020/0022/0001） | `facerec_top_qat5_202p5.sbit`（`b2c404f4…`） | sys +0.260 / cnn +0.126 |
| `configs/stable_menu` 切换 | 稳定单人模板（演示 ROM） | 菜单配置交付树版本（STA 见 `signoff/facerec_top_release_stable.rtr`） | sys +0.260 / cnn +0.126 |

两配置的整机仿真均为 `SYSTEM_INTEGRATION_PASS embedding=64/64 cpu_readback=16/16`
（cycles=17402）。另两份码流（菜单演示版 / ILA 调试版）由开发期工作流生成，本仓库以码流与 STA 报告的形式提供。

---

## 6. 已知限制

1. **产品约束已布线签核**（交付树 +0.260 / +0.126 ns，见 §2.3），但余量不厚：
   两个域都只剩 ~0.1–0.3 ns。任何往 `clk_cnn` 上加逻辑的改动，都必须重跑一次
   完整 P&R 才能确认。布局落点有 ±0.3 ns 级抖动，同一份网表重跑 slack 会浮动，
   所以判断"改动有没有伤时序"必须看多次运行，不能只比一次。
2. **27 MHz 单时钟只在 CPU-only 工程上签核过**（+14.080 ns / TNS 0），
   整机 27 MHz 没有单独跑过。本树给了 `FACEREC_NO_PLL` + 27 MHz FDC，
   需要时按 §3.2 跑一次；周期放大 3.75×/7.5×，通过没有悬念，但推断不算证据。
3. **2025.2 许可**：PDS 许可证与机器 hostid 绑定，本仓库不附带任何许可文件；
   请通过 `PANGO_LICENSE_FILE` 环境变量或 Makefile 的 `LIC_2025` 指向你自己的许可证。
4. `soc_top` 里保留了若干历史遗留参数（`QLN_EN`、`ACCEL_EN`、`QLN_*`）。
   它们默认 0，对应模块（`qln_top`、`accel/acc_top*.v`）不参与综合，
   也不在 `files.f` 里，属于惰性配置项；未做删除以免影响已验证网表。
5. `accel/mem/*.mem` 是 CNN 权重/描述符 ROM，必须随 RTL 一起保留；
   `files.f` 中已包含，缺一个都会导致 CNN 输出错误。

---

## 7. 仓库内容与文档

| 路径 | 内容 |
|---|---|
| `bitstreams/` | 三份最终码流（菜单版 / ILA 调试版 / 五人 QAT 版）与 MD5 清单 |
| `signoff/` | 各配置最终 STA（.rtr）与完整布线资源（.rur）报告 |
| `configs/stable_menu/` | 稳定演示配置的 6 个数据文件，按其中 README.txt 覆盖即切换 |
| `docs/` | 技术报告 PDF、三分钟展示 PPT |
| `doc/` | 复现手册、时序证据、逐项优化、频率/性能权衡 |

## 8. 许可证与第三方组件

本项目以 **MIT 许可证**开源，详见 [LICENSE](LICENSE)。

- `sw/coremark/` 为 EEMBC CoreMark 基准源码，遵循其原始许可（保留文件头版权声明）。
- 人脸图像 ROM **不随仓库提供**：原始四帧来自 ChokePoint 研究数据集，不允许再分发；
  `tools/pack_frames.py` 可用你自己的图像生成等价 ROM。
- 模型权重为本项目基于公开数据自行训练并量化的结果，随仓库提供的 `.mem` 为权重/描述符 ROM。
- 紫光同创 PDS 工具链不随仓库提供，需自行安装并配置与本机 hostid 绑定的许可证。

## 9. 引用

如本项目对你的工作有帮助，欢迎引用 `docs/` 中的技术报告。
