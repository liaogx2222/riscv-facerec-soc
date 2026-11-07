// CPU-domain SoC with unchanged RAM fast path and registered MMIO replies.
// Optional CNN clock domain uses held-payload request/acknowledge CDC.
// RAM geometry is selected by RAM_ADDR_W; application=32KB, benchmark=64KB.
`timescale 1ns/1ps

`include "rv32i_defs.v"

module soc_top #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115_200,
    parameter RAM_ADDR_W = 17,             // 128KB 指令/数据 RAM
    parameter RAM_INIT_FILE = "",          // 仿真固件: simple_ram $readmemh 文件
    parameter RAM_EMBED_INIT = 0,          // 板级固件: 1=simple_ram 内嵌固件生效 (fpga_top 默认 1)
    parameter UART_BEACON    = 0,          // 排障信标: 1=uart_tx 不经 CPU 自主周期发 'U' (0x55),
                                          //   用于区分"串口线路/引脚问题"与"固件问题"
    parameter BTB_ENABLE     = 0,          // M2: CPU 分支预测器开关 (0=S2c 基线)
    // ★ CNN 加速器开关：0 = 不综合（CoreMark 构建）；1 = 例化加速器（推理构建）
    parameter FACEREC_EN     = 0,
    parameter FACEREC_ASYNC  = 0,
    parameter PIPE_MMIO      = 1,
    parameter ACCEL_EN       = 0,
    // ★ 新加速器（QuantLaneNet-Lite）开关
    //   QLN_EN=1 ⇒ 例化 qln_top（自带 MMIO + 17 层序列器）✓
    //   ⚠ 与 ACCEL_EN 互斥；同时置 1 时 QLN 优先
    parameter QLN_EN         = 0,
    // ★ demo 空间裁剪（0 = 用层表全幅 512×256；非 0 = 覆盖输出宽/高）
    //   全幅输入 3×256×512 = 384KB ⇒ 超出片内 RAM ⇒ 需 DDR 供数（见交接文档）
    parameter integer QLN_OW = 128,
    parameter integer QLN_OH = 64,
    // ★ 权重/bias/shift ROM 的 $readmemh 路径（PDS 侧用 qln_mem/ 相对路径；
    //   仿真 tb 用 -P 或实例覆盖指到仓库相对路径）
    parameter QLN_WMEM = "qln_mem/w5.mem",
    parameter QLN_BMEM = "qln_mem/b5.mem",
    parameter QLN_SMEM = "qln_mem/s5.mem",
    parameter S6B_ENABLE     = 0           // S6b: 预测/校正拍取指直取 (0=现状基线)
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        cnn_clk,       // used only when FACEREC_ASYNC=1

    // ---- 外设引脚 ----
    output wire        uart_tx,
    input  wire        uart_rxd,       // M3b: 串口接收 (板上候选脚 AC26)
    input  wire        key_in,         // M3b: 按键中断源 (低有效, 闲置拉高)
    output wire [ 7:0] gpio_out,

    // ---- 状态输出 ----
    output wire        cpu_stall
);

    //-----------------------------------------------------------------
    // CPU 双通道接口 (F=取指 / D=数据), 各为独立 2 拍 Valid-Ready 握手:
    //   结构性消除单总线上 fetch/data 交接竞争 (见 cpu_top 注释)
    //-----------------------------------------------------------------
    wire        f_req, f_resp_vld, f_take, f_take_nxt;
    wire [31:0] f_addr, f_resp_data, f_redirect;
    wire f_registered_redirect_vld;
    wire [31:0] f_registered_redirect_tgt;
    wire [31:0] d_store_addr;
    wire cpu_store_is_ram;
    wire cpu_load_is_ram;
    reg [31:0] io_rdata_q;
    reg io_ready;
    wire fr_bridge_done;
    wire [31:0] fr_bridge_data;
    // ★ RVC 第2步: 取指窗口高半 (队头的下一个字) —— 拼 2 字节对齐的 32 位指令用
    wire [31:0] f_resp_data_nxt;
    wire        f_resp_have_nxt;
    // ★★ #3: 取指包第 2 条的**实际地址** (配对校验用)
    wire [31:0] f_resp_nxt_addr;
    // ★★ #3: 预取器查询口 (BTB `vf`) + 第 2 读口地址
    wire        pf_lk_taken;
    wire [31:0] pf_lk_tgt;
    // ★ R1c-b: pf slot1 前瞻 (BTB vf2 口, 链随预测走用)
    wire        pf_lk1_taken;
    wire [31:0] pf_lk1_tgt;
    // ★ DBG-1e: pf 前瞻查询 (next_issue → BTB vf3/vf4) + 锁存前结果回 pf
    wire [31:0] pf_next_issue_w;
    wire        pf_pnext_taken, pf_pnext1_taken;
    wire [31:0] pf_pnext_tgt, pf_pnext1_tgt;
    wire [31:0] pf_req_addr2;
    wire [31:0] pf_req_addr3;      // ★ #4: 第 3 读口
    wire [31:0] f_ic_3rd_data;
    wire        f_ic_3rd_vld;
    wire        pf_ready2, pf_ready3;   // ★ #4c: icache 组合可用性
    wire        f_redirect_vld;                // S6b: 单发重定向直取预告
    wire [31:0] d_rdata_ram, f_rdata_ram;
    wire        d_valid, d_rdy, d_write;
    // ★ §5f-2: syn_maxfan = 官方扇出复制属性 (ADS 手册 5.13, d_addr 实测
    //   fanout 262 是关键路径布线大头, 26 ≈ 4.9 节经验值 16 的两倍保守值)
    (* syn_maxfan = 26 *) wire [31:0] d_addr;
    wire [31:0] d_load_addr;
    wire [31:0] d_wdata;
    wire [ 3:0] d_wstrb;
    reg  [31:0] d_rdata;
    wire        fence_flush;               // CPU ID 级 fence 脉冲 (M1c)
    wire        d_valid_pre;               // S6a-R: EX 早一拍预读预告
    wire [31:0] d_addr_pre;
    wire [159:0] d_pretag_addresses;
    wire [2:0] d_pretag_choice;

    cpu_rv32i #(
        .BTB_ENABLE(BTB_ENABLE),
        .IO_RAM_ADDR_W(RAM_ADDR_W),
        .S6B_ENABLE(S6B_ENABLE)
    ) u_cpu (
        .clk     (clk),
        .resetn  (resetn),
        .f_req   (f_req),   .f_addr  (f_addr),
        .f_redirect(f_redirect), .f_redirect_vld(f_redirect_vld),
        .f_registered_redirect_vld(f_registered_redirect_vld),
        .f_registered_redirect_tgt(f_registered_redirect_tgt),
        .d_store_addr(d_store_addr),
        .d_load_is_ram(cpu_load_is_ram),
        .d_store_is_ram(cpu_store_is_ram),
        .f_resp_vld(f_resp_vld), .f_resp_data(f_resp_data), .f_take(f_take),
        .f_take_nxt(f_take_nxt),
        .f_resp_data_nxt(f_resp_data_nxt), .f_resp_have_nxt(f_resp_have_nxt),
        .f_resp_nxt_addr(f_resp_nxt_addr),
        .pf_lk_addr(pf_gaddr), .pf_lk_taken(pf_lk_taken), .pf_lk_tgt(pf_lk_tgt),
        .pf_lk1_taken(pf_lk1_taken), .pf_lk1_tgt(pf_lk1_tgt),  // ★ R1c-b: slot1 前瞻
        // ★ DBG-1e: pf 前瞻查询口 (next_issue) + 结果回传
        .pf_lk2_addr(pf_next_issue_w), .pf_lk2_taken(pf_pnext_taken), .pf_lk2_tgt(pf_pnext_tgt),
        .pf_lk3_taken(pf_pnext1_taken), .pf_lk3_tgt(pf_pnext1_tgt),
        .d_valid (d_valid), .d_write(d_write),
        .d_addr  (d_addr),  .d_wdata(d_wdata), .d_wstrb(d_wstrb),
        .d_load_addr(d_load_addr),
        .d_rdy   (d_rdy),   .d_rdata(d_rdata),
        .io_active(~ram_sel),   // ★ IO 气泡: d_addr 区间译码 (纯组合, 2026-10-10)
        .d_valid_pre(d_valid_pre), .d_addr_pre(d_addr_pre),
        .d_pretag_addresses(d_pretag_addresses), .d_pretag_choice(d_pretag_choice),
        .ext_irq   (1'b0),
        .irq       (irq_from_intc),
        .fence_flush (fence_flush),
        .cpu_stall (cpu_stall)
    );

    //-----------------------------------------------------------------
    // 地址译码
    //-----------------------------------------------------------------
    // Each source is classified before late load/drain address arbitration.
    wire ram_sel = d_write ? cpu_store_is_ram : cpu_load_is_ram;      // 0x0000_0000 ~ RAM 上界
`ifdef FACEREC_SIM
    always @(posedge clk) if(resetn && d_valid)
        if(ram_sel !== (d_addr[31:RAM_ADDR_W]==0))
            $fatal(1,"Early RAM classification disagrees with presented address");
`endif
    wire uart_sel   = (d_addr[31:20] == 12'h100);        // 0x1000_0000
    wire gpio_sel   = (d_addr[31:20] == 12'h200);        // 0x2000_0000
    wire timer_sel  = (d_addr[31:20] == 12'h300) && !d_addr[6];   // 0x3000_0000~003F
    // ★ M3b INTC 收窄到实际寄存器区 (+40..+4C)：原先整个 bit6=1 的 4KB 都给了
    //   INTC，导致 +C0..+FF 无法用作性能计数器扩展窗 (2026-10-10)。
    //   ⚠ 首版误写 d_addr[5:2]==0 把 MIP/SWIRQ/CMP(+44/+48/+4C) 也关在门外，
    //     irq 回归 FAIL (cmp_valid=0) —— 修正为 d_addr[5]==0 && d_addr[4:2]<=3。
    wire intc_sel   = (d_addr[31:20] == 12'h300) &&  d_addr[6]
                      && !d_addr[7] && !d_addr[5] && (d_addr[4:2] <= 3'd3);
    // ★ CNN 加速器：0x4000_0000（Part A 新增）
    //   只暴露 6 位字内偏移 ⇒ 16 个 32 位寄存器
    wire accel_sel  = (d_addr[31:20] == 12'h400);                 // 0x4000_0000

    //-----------------------------------------------------------------
    // 硬件周期计数器 (跑分计时): 自由运行 64bit, tick = 1 clk 周期
    //   0x3000_0000 读: 返回低 32 位, 同时把当前 63:32 锁进影子寄存器
    //   0x3000_0004 读: 返回影子高 32 位 (与上一次低半读取同拍快照)
    //-----------------------------------------------------------------
    reg [63:0] cyc_cnt;
    reg [31:0] tim_hi_shadow;
    reg [31:0] tim_rd_q;
    always @(posedge clk) begin
        if (!resetn) begin
            cyc_cnt       <= 64'd0;
            tim_hi_shadow <= 32'd0;
            tim_rd_q      <= 32'd0;
        end else begin
            cyc_cnt <= cyc_cnt + 64'd1;
            if (d_accept && timer_sel && !d_write) begin   // timer_sel 已排除 intc 页
                if (!d_addr[2]) begin
                    tim_rd_q      <= cyc_cnt[31:0];
                    tim_hi_shadow <= cyc_cnt[63:32];
                end else begin
                    tim_rd_q <= tim_hi_shadow;
                end
            end
        end
    end

    //-----------------------------------------------------------------
    // 两条独立握手桥: accept 单拍脉冲 (从设备动作只由它触发, 防重放),
    // 下一拍 rdy (匹配 RAM 同步读 1 拍延迟). 无效地址同样 rdy/读回 0.
    //-----------------------------------------------------------------
    //-----------------------------------------------------------------
    // F 通道: pf_buffer(链式预取, 电平 resp + take) → icache(2路SA M1) → RAM
    //   resp 电平 = 队头地址==pc; f_take 消费弹出; 失配整队冲刷重建.
    //   icache: 命中 1 拍 (稳态 1 指令/拍不变); miss = 8 拍行突发填充;
    //   非 RAM 区直通 0 (runaway pc 防死锁). RAM I 口由 icache 独占.
    //-----------------------------------------------------------------
    wire        pf_ready, pf_gnt;
    wire [31:0] pf_gaddr;
    wire        ic_resp_vld;
    wire [31:0] ic_resp_addr, ic_resp_data;
    wire        ic_r_valid;
    wire [31:0] ic_r_addr;
    wire [31:0] f_ic_nxt_data;   // ★ D2a
    wire        f_ic_nxt_vld;

    // ★ §5f-1 R1a: EX 级重定向打拍 —— f_redirect/f_redirect_vld 成对锁存一拍
    //   再进 pf_buffer. 原 S6b 零间隙重定向 (f_redirect 组合直驱 req_hint/
    //   hint_addr) 是重定向域组合环的入口 (f_redirect→iss_addr→BTB→icache tag
    //   →ready2→issue_next→next_issue). 打拍后环入口断开.
    //   安全论证: 重定向拍 C 的 pf 看不到 hint、pc 仍=旧值→沿旧链正常; C+1 拍
    //   pc 跳 target 且 hint 到→chain_rst 取 hint_addr 单次重建 (C 拍无 resync,
    //   不会双发). 代价 = 重定向 +1 气泡 (零间隙→1间隙, 与 §5f-1 预测一致).
    reg        f_redirect_vld_q;
    reg [31:0] f_redirect_q;
    always @(posedge clk) begin
        if (!resetn) begin
            f_redirect_vld_q <= 1'b0;
            f_redirect_q     <= 32'h0;
        end else begin
            f_redirect_vld_q <= f_redirect_vld && !f_registered_redirect_vld;
            f_redirect_q     <= f_redirect;
        end
    end

    // DEPTH 4 -> 8 -> 16。
    //   4->8 是 D2b 双发射落地时的必要加深 (配对后消耗率翻倍)；
    //   8->16 是性能优化项：CoreMark 2K 实测 ROI 194,583 -> 190,257 拍
    //   (-2.05%)，取指未就绪 10,144 -> 7,844 拍、非填充期取指饿
    //   6,454 -> 4,150 拍，指令数不变 (274,696)，IPC 1.412 -> 1.444。
    //   代价：+8 项 {addr,data} 寄存器，关键路径不变 (比较的是常量 DEPTH-2)。
    pf_buffer #(
        .DEPTH(16)
    ) u_pf (
        .clk     (clk),
        .resetn  (resetn),
        .cpu_req (f_req),
        .cpu_addr(f_addr),
        .req_hint(f_registered_redirect_vld | f_redirect_vld_q),
        .hint_addr(f_registered_redirect_vld ? f_registered_redirect_tgt : f_redirect_q),
        .hint_taken(),
        .resp    (f_resp_vld),
        .resp_data(f_resp_data),
        .resp_data_nxt(f_resp_data_nxt),   // ★ RVC 第2步
        .resp_nxt_addr(f_resp_nxt_addr),   // ★ #3: 第 2 条地址 (配对校验)
        .have_nxt(f_resp_have_nxt),        // ★ RVC 第2步
        .pred_taken(pf_lk_taken), .pred_tgt(pf_lk_tgt),  // ★ #3: BTB 查询
        .pred1_taken(pf_lk1_taken), .pred1_tgt(pf_lk1_tgt),  // ★ R1c-b: slot1 前瞻
        // ★ DBG-1e: 前瞻预测 (锁存前) + next_issue 输出
        .pnext_taken(pf_pnext_taken), .pnext_tgt(pf_pnext_tgt),
        .pnext1_taken(pf_pnext1_taken), .pnext1_tgt(pf_pnext1_tgt),
        .pf_next_issue_o(pf_next_issue_w),
        .req_addr2(pf_req_addr2),                        // ★ #3: 第 2 读口
        .req_addr3(pf_req_addr3),                        // ★ #4: 第 3 读口
        .resp_3rd_data(f_ic_3rd_data), .resp_3rd_vld(f_ic_3rd_vld),
        .ready2(pf_ready2), .ready3(pf_ready3),
        .cpu_take(f_take), .cpu_take_nxt(f_take_nxt),
        .ready   (pf_ready),
        .gnt     (pf_gnt),
        .gaddr   (pf_gaddr),
        .resp_vld(ic_resp_vld),
        .resp_addr(ic_resp_addr),
        .resp_rdata(ic_resp_data),
        .resp_nxt_data(f_ic_nxt_data), .resp_nxt_vld(f_ic_nxt_vld)
    );

    icache u_ic (
        .clk     (clk),
        .resetn  (resetn),
        .req_vld (pf_gnt),
        .req_addr(pf_gaddr),
        .req_addr2(pf_req_addr2),          // ★ #3: 第 2 读口 (顺序=A+4 / 预测taken=T)
        .req_addr3(pf_req_addr3),          // ★ #4: 第 3 读口 (插入包的 A+4)
        .resp_data_3rd(f_ic_3rd_data), .resp_3rd_vld(f_ic_3rd_vld),
        .ready2(pf_ready2), .ready3(pf_ready3),
        .ready   (pf_ready),
        .resp_vld(ic_resp_vld),
        .resp_addr(ic_resp_addr),
        .resp_data(ic_resp_data),
        .resp_data_nxt(f_ic_nxt_data), .resp_nxt_vld(f_ic_nxt_vld),  // ★ D2a
        .flush   (fence_flush),
        .r_valid (ic_r_valid),
        .r_addr  (ic_r_addr),
        .r_data  (f_rdata_ram),
        .early_fill_allowed(~(dc_m_valid && dc_m_write)),
        .miss_busy (ic_miss_act)
    );

    //-----------------------------------------------------------------
    // 外设路径 accept 单拍脉冲
    //   ★★ 2026-10-08 修复（load-use 停顿取消后暴露的潜在 bug）:
    //   原式 `d_accept = d_valid & ~d_p`（d_p = d_accept 的寄存版）只在
    //   "本拍 d_valid 是新拉起的"时成立。**一旦上一拍也是 d_valid=1
    //   （背靠背访存）, d_p=1 就把新访问的 accept 整个吞掉** ⇒
    //   外设（UART/GPIO/TIMER/INTC/加速器）的写副作用与读脉冲全部丢失。
    //   过去 load-use 停顿保证"两次访存之间必有气泡"⇒ 掩盖了这个 bug;
    //   阶段1 取消停顿后立刻现形（c_run_tb 报 UART 无输出）。
    //   修法: 用 CPU 的推进使能判"新访问" —— 只有流水线在**上一拍推进过**
    //   （adv_q=1）才可能出现新的 MEM 级访问; 冻结拍 adv_q=0 ⇒ 不重触发。
    //   保留 `~d_p` 兜底（复位后首笔 adv_q=0 也能 accept）。
    //   注: adv = ~cpu_stall（cpu_rv32i 的推进使能，寄存器化后使用，无组合环）。
    //-----------------------------------------------------------------
    reg d_p;
    reg adv_q;
    wire d_accept = d_valid & (~d_p | adv_q);
    always @(posedge clk) begin
        if (!resetn) begin
            d_p   <= 1'b0;
            adv_q <= 1'b0;
        end else begin
            // ★★ 2026-10-08 修复: `d_p` 必须寄存 **d_valid**, 不能寄存 d_accept。
            //   原式 `d_p <= d_accept`: 一次访存被 accept 后, 若下一拍 d_accept=0
            //   (因为 adv_q=0), d_p 就退回 0 ⇒ **再下一拍又 accept 一次** ⇒
            //   一次访存被重复触发 N 次 (N = 停顿拍数)。
            //   实测: dcache miss 让 store 停 11 拍 ⇒ 外设写被重复触发,
            //   c_run_tb 的 UART 输出变成 "RVRRRRRR…" (同一个字节连发 30 次)。
            //   改成寄存 d_valid 后: 一次访存只在"上升沿"或"上一拍推进过"时 accept ✓
            d_p   <= d_valid;
            adv_q <= ~cpu_stall;
        end
    end

    //-----------------------------------------------------------------
    // M3b: UART 接收 + 中断控制器
    //-----------------------------------------------------------------
    wire uart_rx_read = d_accept & uart_sel & (d_addr[3:2]==2'b10) & ~d_write;
    wire [ 7:0] uart_rx_data;
    wire        uart_rx_valid, uart_rx_ovr;
    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart_rx (
        .clk(clk), .resetn(resetn),
        .rxd     ((uart_rxd === 1'b0) ? 1'b0 : 1'b1),  // 悬空/z → 闲置高
        .rx_read (uart_rx_read),
        .rx_data (uart_rx_data), .rx_valid(uart_rx_valid), .rx_overrun(uart_rx_ovr)
    );

    wire [3:0]  irq_from_intc;
    wire [31:0] intc_rdata;
    intc u_intc (
        .clk(clk), .resetn(resetn),
        .wr  (d_accept & intc_sel & d_write),
        .widx(d_addr[3:2]),
        .wdata(d_wdata),
        .rdata(intc_rdata),                   // 组合读
        .time_lo(cyc_cnt[31:0]),
        .evt_uart_rx(uart_rx_valid),
        .key_n      ((key_in === 1'b0) ? 1'b0 : 1'b1),
        .irq        (irq_from_intc)
    );

    //-----------------------------------------------------------------
    // D 通道 (RAM 区): dcache 接管握手 —— 命中与无 cache 桥同相位
    //   (rdy=accept 后一拍), miss 拉低 rdy 至行填充完成, CPU 由
    //   mem_busy→adv=0 天然冻结. 外设路径 (非 RAM 区) 走上方 d_p 桥.
    //-----------------------------------------------------------------
    wire        dc_rdy;
    wire [31:0] dc_rdata;
    wire        dc_m_valid, dc_m_write;
    wire [31:0] dc_m_addr, dc_m_wdata;
    wire [ 3:0] dc_m_wstrb;

    dcache #(.SEPARATE_FAST_ADDR(1), .PARALLEL_PRETAG(1)) u_dc (
        .clk     (clk),
        .resetn  (resetn),
        .d_valid (d_valid & ram_sel),
        .d_write (d_write),
        .d_addr  (d_addr),
        .d_read_addr(d_load_addr),
        .d_write_addr(d_store_addr),
        .d_wdata (d_wdata),
        .d_wstrb (d_wstrb),
        .d_rdy   (dc_rdy),
        .d_rdata (dc_rdata),
        .flush   (fence_flush),
        .d_pretag_addresses(d_pretag_addresses), .d_pretag_choice(d_pretag_choice),
        .d_valid_pre (d_valid_pre),           // 原始 pre (dcache 内 pre_a==d_addr 校验天然排除非 RAM)
        .d_addr_pre  (d_addr_pre),
        .m_valid (dc_m_valid),
        .m_write (dc_m_write),
        .m_addr  (dc_m_addr),
        .m_wdata (dc_m_wdata),
        .m_wstrb (dc_m_wstrb),
        .m_rdata (d_rdata_ram)
    );

    wire periph_ok = ~d_addr[31] | uart_sel | gpio_sel | timer_sel | intc_sel;
    wire slow_io = ((PIPE_MMIO!=0) && !d_write) ||
                   ((FACEREC_EN!=0) && (FACEREC_ASYNC!=0) && fr_sel);
    // A fresh accept must not consume the previous held IO response.
    soc_ready_planes #(.REGISTERED_IO(PIPE_MMIO),
        .CNN_ASYNC((FACEREC_EN!=0)&&(FACEREC_ASYNC!=0))) u_ready_planes(
        .write_op(d_write),.load_is_ram(cpu_load_is_ram),.store_is_ram(cpu_store_is_ram),
        .load_address(d_load_addr),.store_address(d_store_addr),
        .cache_ready(dc_rdy),.io_ready(io_ready),.accept(d_accept),
        .peripheral_pending(d_p),.ready(d_rdy));
`ifdef FACEREC_SIM
    // For valid requests the separated addresses are the actual load/store
    // sources. Retain the old Boolean expression as a simulation oracle.
    wire ready_reference=ram_sel ? dc_rdy :
        (periph_ok & (slow_io ? (io_ready & ~d_accept) : d_p));
    always @(posedge clk)if(resetn && d_valid)
        if(d_rdy!==ready_reference)$fatal(1,"Separated ready planes changed handshake");
`endif

    //-----------------------------------------------------------------
    // simple_ram 真双端口 (I=取指只读, D=数据读写, 各同步 1 拍)
    //   注意: simple_ram.v 是生成文件 (gen_board_ram_init.py), 固件以
    //   EMBED_INIT 参数门控内嵌; 仿真默认 0, 板级 (fpga_top) 默认 1.
    //-----------------------------------------------------------------
    simple_ram #(
        .ADDR_W(RAM_ADDR_W),
        .INIT_FILE(RAM_INIT_FILE),
        .EMBED_INIT(RAM_EMBED_INIT)
    ) u_ram (
        .clk     (clk),
        .i_valid (ic_r_valid),               // 仅 icache miss 突发填充使用
        .i_addr  (ic_r_addr),
        .i_rdata (f_rdata_ram),
        .d_valid (dc_m_valid),                 // D 口由 dcache 代理 (含写通+填充读)
        .d_write (dc_m_write),
        .d_addr  (dc_m_addr),
        .d_wdata (dc_m_wdata),
        .d_wstrb (dc_m_wstrb),
        .d_rdata (d_rdata_ram)
    );

    //-----------------------------------------------------------------
    // UART TX 控制寄存器
    //-----------------------------------------------------------------
    reg  [ 7:0] uart_txdata;
    reg         uart_tx_start;
    wire        uart_tx_busy;

    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD    (BAUD)
    ) u_uart (
        .clk     (clk),
        .resetn  (resetn),
        .tx_start(uart_start_eff),
        .tx_data (uart_data_eff),
        .tx_busy (uart_tx_busy),
        .tx      (uart_tx)
    );

    //-----------------------------------------------------------------
    // 信标模式: ~0.5s 自主发一个 'U'(0x55); CPU 侧 start/data 被旁路,
    // STATUS.busy 读成 0 让 CPU 轮询直通 (LED 仍能正常到全亮)
    //-----------------------------------------------------------------
    localparam BCN_DIV = CLK_FREQ / 2;              // 半秒
    reg [31:0] bcn_cnt;
    wire beacon_start = (UART_BEACON != 0) && (bcn_cnt == BCN_DIV - 1);
    always @(posedge clk) begin
        if (!resetn)                 bcn_cnt <= 32'd0;
        else if (UART_BEACON != 0)   bcn_cnt <= beacon_start ? 32'd0 : bcn_cnt + 32'd1;
    end
    wire        uart_start_eff = (UART_BEACON != 0) ? beacon_start  : uart_tx_start;
    wire [ 7:0] uart_data_eff  = (UART_BEACON != 0) ? 8'h55         : uart_txdata;
    wire        uart_busy_sta  = (UART_BEACON != 0) ? 1'b0          : uart_tx_busy;

    // UART 写: 写 0x1000_0000 锁存数据并启动 (仅 D 通道 accept 拍, 单笔)
    // 注意: tx_start 保持到 uart_tx 捕获 (busy=1), 避免 1 拍脉冲漏检
    reg uart_tx_req;
    always @(posedge clk) begin
        if (!resetn) begin
            uart_txdata   <= 8'h0;
            uart_tx_start <= 1'b0;
            uart_tx_req   <= 1'b0;
        end else begin
            if (d_accept && d_write && uart_sel && d_addr[3:2]==2'b00 && |d_wstrb) begin
                uart_tx_req <= 1'b1;                 // 锁存发送请求
                uart_txdata <= d_wdata[7:0];         // 锁存待发数据 (否则恒发 0x00)
            end
            if (uart_tx_req && uart_tx_busy)
                uart_tx_req <= 1'b0;                 // 已开始发送, 清请求
            uart_tx_start <= uart_tx_req;
        end
    end

    // UART 状态寄存器 (0x1000_0004): bit0 = tx_busy
    // GPIO 输出寄存器 (0x2000_0000)
    reg [7:0] gpio_reg;
    always @(posedge clk) begin
        if (!resetn) gpio_reg <= 8'h0;
        else if (d_accept && d_write && gpio_sel && |d_wstrb)
            gpio_reg <= d_wdata[7:0];
    end
    assign gpio_out = gpio_reg;

    //-----------------------------------------------------------------
    // D 通道读 mux (rdy 拍采样: CPU 等待期间 d_addr 稳定, 组合选择安全;
    //   RAM 读数据由 simple_ram D 口同步输出, 正好对齐 rdy 拍)
    //-----------------------------------------------------------------
    // A retired load always uses d_load_addr. SB drain arbitration changes
    // the bus address only for writes or while the load cannot advance.
    // Decode read data directly from the MEM occupant, avoiding the SB CAM
    // and drain/address mux in front of the late load forwarding network.
    wire rd_ram_sel   = (d_load_addr[31:RAM_ADDR_W] == 0);
    wire rd_uart_sel  = (d_load_addr[31:20] == 12'h100);
    wire rd_gpio_sel  = (d_load_addr[31:20] == 12'h200);
    wire rd_timer_sel = (d_load_addr[31:20] == 12'h300) && !d_load_addr[6];
    wire rd_intc_sel  = (d_load_addr[31:20] == 12'h300) && d_load_addr[6]
                        && !d_load_addr[7] && !d_load_addr[5] && (d_load_addr[4:2] <= 3'd3);
    wire rd_accel_sel = (d_load_addr[31:20] == 12'h400);
    wire fr_sel = (d_addr[31:12] == 20'h60000);
    wire rd_fr_sel = (d_load_addr[31:12] == 20'h60000);
    wire [31:0] fr_rdata;
    // Keep the read path independent of SB drain arbitration. Writes use
    // their own address and handshake; reads are selected below by rd_fr_sel.
    wire rd_iobuf_sel = (d_load_addr[31:18] == 14'h1400);
    wire rd_inbuf_sel = rd_iobuf_sel && !d_load_addr[17];
    wire rd_outbuf_sel= rd_iobuf_sel && d_load_addr[17];
    wire [31:0] uart_rdata = (d_load_addr[3:2]==2'b00) ? {24'b0, uart_txdata}     :   // +0 TXDATA
                             (d_load_addr[3:2]==2'b01) ? {31'b0, uart_busy_sta}   :   // +4 STATUS
                             (d_load_addr[3:2]==2'b10) ? {24'b0, uart_rx_data}    :   // +8 RXDATA(读即清)
                                                   {30'b0, uart_rx_ovr, uart_rx_valid}; // +C RXSTATUS

    reg [31:0] mapped_rdata;
    always @(*) begin
        mapped_rdata = 32'h0;
        if (rd_timer_sel) begin
            // 周期计数器 +0/+4 读；+8 及以上不再提供性能计数器，读回 0
            if (!d_load_addr[5] && (d_load_addr[4:2] <= 3'd1))
                mapped_rdata = tim_rd_q;
            else
                mapped_rdata = 32'h0;
        end
        else if (rd_intc_sel)  mapped_rdata = intc_rdata;
        else if (rd_accel_sel) mapped_rdata = accel_rdata;
        else if (rd_fr_sel)    mapped_rdata = fr_rdata;
        else if (rd_inbuf_sel) mapped_rdata = inbuf_rdata;
        else if (rd_outbuf_sel)mapped_rdata = outbuf_rdata;
        else if (rd_ram_sel)   mapped_rdata = (PIPE_MMIO==0) ? dc_rdata : 32'b0;
        else if (rd_uart_sel)  mapped_rdata = uart_rdata;
        else if (rd_gpio_sel)  mapped_rdata = {24'b0, gpio_reg};
    end

    // Snapshot device state on acceptance, before read-to-clear side effects.
    // The cycle timer's existing snapshot FF updates on that same edge, so
    // use the equivalent source values for its low/high pair here.
    wire cycle_read_low=rd_timer_sel && !d_load_addr[7] && (d_load_addr[5:2]==0);
    wire cycle_read_high=rd_timer_sel && !d_load_addr[7] && (d_load_addr[5:2]==1);
    wire [31:0] io_accept_data=cycle_read_low ? cyc_cnt[31:0] :
                                  cycle_read_high ? tim_hi_shadow : mapped_rdata;

    always @(posedge clk) begin
        if(!resetn) begin
            io_rdata_q<=0;io_ready<=0;
        end else begin
            if(!d_valid) io_ready<=0;
            if(d_accept && !ram_sel) begin
                // The registered snapshot itself is already a complete
                // local reply. It is visible one cycle after acceptance,
                // matching the original peripheral handshake latency.
                if(!d_write && !((FACEREC_EN!=0) && (FACEREC_ASYNC!=0) && fr_sel)) begin
                    io_rdata_q<=io_accept_data;io_ready<=1;
                end else io_ready<=0;
            end
            if(fr_bridge_done) begin
                io_rdata_q<=fr_bridge_data;io_ready<=1;
            end
        end
    end
    always @(*) begin
        if(PIPE_MMIO!=0) d_rdata=cpu_load_is_ram ? dc_rdata : io_rdata_q;
        else d_rdata=mapped_rdata;
    end

    //-----------------------------------------------------------------
    // ★ CNN 加速器（0x4000_0000）
    //   **两种构建互不干扰**（用参数隔离）：
    //     QLN_EN=0  ⇒ CoreMark 构建，完全不综合加速器 ✓
    //     QLN_EN=1  ⇒ **新加速器 QuantLaneNet-Lite**（qln_top，自带 MMIO）✓
    //
    //   ⚠ 2026-09-25：旧的 ACCEL_EN 分支（accel_mmio + accel_top，48 层 YOLO）
    //     已随 accel_top 一起判废移入 `_trash/2026-09-25/accel_deprecated/` ✗
    //     （accel_mmio 保留：它的 ID 是 "ACC1"，与本加速器无关，暂不使用）
    //   ★ 2026-09-26：接入**流式 IO**（本版不再是 tie-off）：
    //     · qln_inbuf  @ 0x5000_0000（128KB）—— CPU 写输入图像
    //     · qln_outbuf @ 0x5002_0000（128KB）—— CPU 读结果（16B/beat）
    //     · qln_stream             —— 喂流/收集引擎，MMIO 在 0x4000_0030 起
    //     · qln_top 的 up_*/out_* 全部接引擎（权重/bias/shift 由其内部 ROM 自供 ✓）
    //   ⚠ OW_OVR/OH_OVR：demo 裁剪（全幅 256×512 需 DDR 供图，见交接文档 §4.2）
    //-----------------------------------------------------------------
    generate if (FACEREC_EN != 0) begin : G_FACEREC
        wire fr_clk=(FACEREC_ASYNC!=0) ? cnn_clk : clk;
        wire fr_resetn,mm_sel,mm_we;
        wire [11:0] mm_addr,mm_raddr;
        wire [31:0] mm_wdata,mm_rdata;
        if(FACEREC_ASYNC!=0) begin : G_CDC
            (* ASYNC_REG="TRUE",syn_preserve=1 *) reg [2:0] cnn_reset_pipe;
            always @(posedge cnn_clk or negedge resetn) begin
                if(!resetn) cnn_reset_pipe<=0;
                else cnn_reset_pipe<={cnn_reset_pipe[1:0],1'b1};
            end
            assign fr_resetn=cnn_reset_pipe[2];
            wire bridge_busy;
            cnn_cdc_bridge u_bridge(
                .c_clk(clk),.c_resetn(resetn),.c_start(d_accept && fr_sel),
                .c_write(d_write),.c_addr(d_addr[11:0]),.c_wdata(d_wdata),
                .c_busy(bridge_busy),.c_done(fr_bridge_done),.c_rdata(fr_bridge_data),
                .m_clk(cnn_clk),.m_resetn(fr_resetn),.m_sel(mm_sel),.m_we(mm_we),
                .m_addr(mm_addr),.m_wdata(mm_wdata),.m_rdata(mm_rdata));
            assign mm_raddr=mm_addr;
            assign fr_rdata=0; // no direct CNN-domain combinational read path
`ifdef FACEREC_SIM
            initial if(PIPE_MMIO==0) $fatal(1,"CDC requires registered IO return");
`endif
        end else begin : G_SYNC
            assign fr_resetn=resetn;
            assign mm_sel=fr_sel;
            assign mm_we=d_accept && d_write && fr_sel;
            assign mm_addr=d_store_addr[11:0];assign mm_raddr=d_load_addr[11:0];
            assign mm_wdata=d_wdata;
            assign fr_rdata=mm_rdata;
            assign fr_bridge_done=0;assign fr_bridge_data=0;
        end
        cnn_mmio #(.AAW(16), .ADEPTH(6144), .B0DEPTH(18432)) u_facerec (
            .clk(fr_clk), .rst_n(fr_resetn),
            .sel(mm_sel), .we(mm_we),.addr(mm_addr),.raddr(mm_raddr),
            .wdata(mm_wdata), .rdata(mm_rdata)
        );
    end else begin : G_NO_FACEREC
        assign fr_rdata = 32'b0;
        assign fr_bridge_done=0;assign fr_bridge_data=0;
    end endgenerate

    wire [31:0] accel_rdata;

    // ---- QLN 读回分流：qln_top（<0x30） / 流引擎（>=0x30）----
    wire [31:0] qln_rdata;      // qln_top 的 rdata
    wire [31:0] str_rdata;      // qln_stream 的 mm_rdata
    wire [31:0] inbuf_rdata;    // 输入缓冲 A 口读
    wire [31:0] outbuf_rdata;   // 输出缓冲 A 口读
    assign accel_rdata = accel_sel_qln ? qln_rdata :
                         accel_sel_str ? str_rdata : 32'h0;

    // ---- 加速器 IO 窗口（0x5000_0000，共 256KB：inbuf/outbuf 各 128KB）----
    wire        iobuf_sel = (d_addr[31:18] == 14'h1400);      // 0x5000_0000~0x5003_FFFF
    wire        inbuf_sel = iobuf_sel & ~d_addr[17];
    wire        outbuf_sel= iobuf_sel &  d_addr[17];

    // ---- MMIO 路由：accel 窗口内字内偏移 < 0x30 → qln_top；>= 0x30 → 流引擎 ----
    wire        accel_sel_qln = accel_sel & (d_addr[7:0] <  8'h30);
    wire        accel_sel_str = accel_sel & (d_addr[7:0] >= 8'h30);

    // ---- qln_top ↔ 流引擎 ----
    wire        q_up_vld, q_up_row_end, q_up_ready;
    wire [11:0] q_up_col;
    wire [6:0]  q_up_ch;
    wire signed [7:0] q_up_dat;
    wire        q_out_vld, q_ctx, q_chain_en;
    wire signed [12*8-1:0] q_out_y0, q_out_y1;

    // ---- 流引擎 ↔ 缓冲 ----
    wire        ib_vld;
    wire [16:0] ib_addr;
    wire [7:0]  ib_rdata;
    wire        ob_vld;
    wire [16:0] ob_addr;
    wire [95:0] ob_wdata;

    generate
    if (QLN_EN) begin : G_QLN
        //-------------------------------------------------------------
        // 新加速器：qln_top = MMIO + 层表 + qln_seq + accel_layer5
        //   ★ ID 寄存器 = 0x514C4E31 = "QLN1"（与已验证的 MMIO 通路一致）✓
        //   ★ 权重/bias/shift 由内部 ROM（u_wrom/u_bromA/B）自供 ⇒ 无外部端口 ✓
        //     （2026-09-26 修：旧实例化连的 .w_flat/.bias/.sh_vec 端口已不存在 ✗）
        //-------------------------------------------------------------
        qln_top #(
            .CMAX(64), .WMAX(512), .ROWS(5), .AW(12),
            .ACCW(32), .SHW(6), .CW(7), .LW(6),
            .NC(12), .NP(4), .PW(18), .OUTW(8), .RW(10), .OWR(7),
            .LBM_IDXW(16),                       // = $clog2(2*5*4096)（旧值 19 冗余但无害）
            .MEMFILE(QLN_WMEM), .MEM_B(QLN_BMEM), .MEM_S(QLN_SMEM),
            .OW_OVR(QLN_OW), .OH_OVR(QLN_OH)     // demo 裁剪（0 = 用层表全幅）
        ) u_qln_top (
            .clk(clk), .resetn(resetn), .en(1'b1),
            // ⚠ 写使能与其它外设一致：d_accept && d_write（不是 d_we，没有这个信号）
            .sel(accel_sel_qln), .we(d_accept && d_write), .addr(d_addr), .wdata(d_wdata),
            .rdata(qln_rdata),
            // 输入流 ← 流引擎（尊重 up_ready 的逐字节脉冲 ✓）
            .up_vld(q_up_vld), .up_col(q_up_col), .up_ch(q_up_ch),
            .up_dat(q_up_dat), .up_row_end(q_up_row_end), .up_ready(q_up_ready),
            // 输出流 → 流引擎
            .out_vld(q_out_vld), .out_col(), .out_oc_round(),
            .out_y0(q_out_y0), .out_y1(q_out_y1),
            .busy(), .done(), .layer_idx(), .ctx(q_ctx),
            .chain_en_o(q_chain_en)   // ★ 2026-09-27：流引擎收集器据此选 A/B
        );

        //-------------------------------------------------------------
        // 流式 IO 引擎（喂流 + 收集 + 自己的 MMIO）
        //-------------------------------------------------------------
        qln_stream #(
            .AW(12), .CW(7), .RW(10), .NC(12), .OUTW(8), .IB_AW(17)
        ) u_qln_stream (
            .clk(clk), .resetn(resetn), .en(1'b1),
            .mm_we(d_accept && d_write && accel_sel_str),
            .mm_addr(d_addr[7:0]), .mm_wdata(d_wdata),
            .mm_rdata(str_rdata),
            .ib_vld(ib_vld), .ib_addr(ib_addr), .ib_rdata(ib_rdata),
            .ob_vld(ob_vld), .ob_addr(ob_addr), .ob_wdata(ob_wdata),
            .up_vld(q_up_vld), .up_col(q_up_col), .up_ch(q_up_ch),
            .up_dat(q_up_dat), .up_row_end(q_up_row_end), .up_ready(q_up_ready),
            .out_vld(q_out_vld), .out_y0(q_out_y0), .out_y1(q_out_y1),
            .ctx(q_ctx), .chain_en(q_chain_en)
        );

        //-------------------------------------------------------------
        // 输入缓冲（CPU 写 → 引擎读）
        //-------------------------------------------------------------
        qln_inbuf #(.AW(17)) u_inbuf (
            .clk(clk),
            .a_valid(d_valid & inbuf_sel),
            .a_write(d_write), .a_addr(d_addr[16:0]),
            .a_wdata(d_wdata), .a_wstrb(d_wstrb), .a_rdata(inbuf_rdata),
            .b_valid(ib_vld), .b_addr(ib_addr), .b_rdata(ib_rdata)
        );

        //-------------------------------------------------------------
        // 输出缓冲（引擎写 → CPU 读；一拍 96 位 = 一个 beat）
        //-------------------------------------------------------------
        qln_outbuf #(.AW(17)) u_outbuf (
            .clk(clk),
            .a_valid(d_valid & outbuf_sel), .a_addr(d_addr[16:0]), .a_rdata(outbuf_rdata),
            .b_valid(ob_vld), .b_addr(ob_addr), .b_wdata(ob_wdata)
        );
    end else begin : G_NOACCEL
        assign accel_rdata = 32'h0;      // 未使能 ⇒ 读回 0
    end
    endgenerate

    //-----------------------------------------------------------------
    // 调试: dbg_pc 已由 cpu_top 端口直连 (禁止跨层引用, PDS 不支持)
    //-----------------------------------------------------------------

endmodule

// Decode independent read/store sources before the late operation select.
// No state or extra handshake edge; d_write is a valid binary request control.
module soc_ready_planes /* synthesis syn_hier="hard" */ #(
    parameter REGISTERED_IO=1,CNN_ASYNC=1
)(
    input wire write_op,load_is_ram,store_is_ram,
    input wire [31:0]load_address,store_address,
    input wire cache_ready,io_ready,accept,peripheral_pending,
    output wire ready
);
    // All supported peripheral decode prefixes have bit31=0. Therefore
    // the original (~addr[31] | uart | gpio | timer | intc) is ~addr[31].
    wire read_peripheral_ok=~load_address[31];
    wire store_peripheral_ok=~store_address[31];
    wire store_is_cnn=(store_address[31:12]==20'h60000);
    wire held_reply=io_ready & ~accept;
    wire read_slow=REGISTERED_IO!=0 || CNN_ASYNC!=0 && (load_address[31:12]==20'h60000);
    wire write_slow=CNN_ASYNC!=0 && store_is_cnn;
    wire read_ready=load_is_ram ? cache_ready :
        (read_peripheral_ok & (read_slow ? held_reply : peripheral_pending));
    wire store_ready=store_is_ram ? cache_ready :
        (store_peripheral_ok & (write_slow ? held_reply : peripheral_pending));
    assign ready=write_op ? store_ready : read_ready;
endmodule
