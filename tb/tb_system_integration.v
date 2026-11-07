//=====================================================================
// tb_system_integration.v : 交付树整机（CPU + CNN）端到端回归
//
//   固件 sw/*.c（integration 镜像）跑完整人脸识别链路：
//     CPU 配置 CNN 描述符 → 帧源/ROI → 滑窗 → PE 阵列 → 输出累加
//     → embedding 64 字节 → CPU 回读比对
//   判定：
//     1) GPIO[7:4] 不得出现 4'he（固件自检失败）
//     2) GPIO==8'hA5 时，CNN 产出的 64 字节 embedding 与黄金向量
//        flow_emb.hex 逐字节相等
//     3) CPU 侧 16 拍突发回读与 embedding 拼字一致
//   本台与 build/cpu101_cnn202_pairflat_candidate/tb/system_integration_tb.v
//   同源，仅去掉已剥离的 dbg_pc 观察口（交付树不含调试端口）。
//
//   运行（在 release/ 目录下，ROM 的 $readmemh 路径以 rtl/ 为基准）:
//     iverilog -g2012 -DFACEREC_SIM -s tb_system_integration \
//       -I rtl/core -I rtl/soc -I rtl/accel \
//       -o /tmp/rel_sys.vvp $(cat rtl/files.f) tb/tb_system_integration.v
//     vvp /tmp/rel_sys.vvp
//=====================================================================
`timescale 1ns/1ps
`define CNN u_soc.G_FACEREC.u_facerec.u_soc

module tb_system_integration;

    reg clk = 0, resetn = 0;
    always #4.938272 clk = ~clk;                 // CPU 101.25 MHz

    reg cnn_clk = 0;
    initial begin #1.37; forever #2.469136 cnn_clk = ~cnn_clk; end   // CNN 202.5 MHz

    wire [7:0] gpio;

    soc_top #(
        .CLK_FREQ       (101_250_000),
        .BAUD           (10_000_000),
        .RAM_ADDR_W     (15),
        .RAM_INIT_FILE  ("tb/integration.mem"),
        .RAM_EMBED_INIT (0),
        .BTB_ENABLE     (1),
        .S6B_ENABLE     (1),
        .FACEREC_EN     (1),
        .FACEREC_ASYNC  (1)
    ) u_soc (
        .clk      (clk),
        .cnn_clk  (cnn_clk),
        .resetn   (resetn),
        .uart_tx  (),
        .uart_rxd (1'b1),
        .key_in   (1'b1),
        .gpio_out (gpio),
        .cpu_stall()
    );

    reg [63:0] expected [0:96639];
    reg [31:0] lp       [0:1363];
    reg [7:0]  emb      [0:63];

    integer cycles = 0, i, j, k, base, off, errors = 0;
    reg fixture = 0;
    reg [63:0] got;

    initial begin
        $readmemh("tb/flow_exp_all.hex",   expected);
        $readmemh("tb/flow_layers.txt.hex", lp);
        $readmemh("tb/flow_emb.hex",       emb);
        // 未用到的激活区按上电 BRAM 值显式清零，保证可复现
        for (i = 0; i < 18432; i = i + 1) `CNN.u_act.g_bank[0].u_bank.sim_write(i, 0);
        for (i = 0; i < 6144; i = i + 1) begin
            `CNN.u_act.g_bank[1].u_bank.sim_write(i, 0);
            `CNN.u_act.g_bank[2].u_bank.sim_write(i, 0);
            `CNN.u_act.g_bank[3].u_bank.sim_write(i, 0);
        end
        repeat (8) @(negedge clk);
        resetn = 1;
    end

    always @(negedge clk) if (resetn) begin
        cycles = cycles + 1;

        if (gpio[7:4] === 4'he) $fatal(1, "firmware failure GPIO=%h", gpio);

        if (gpio === 8'd2 && !fixture) begin
            base = lp[61*22+15]; off = lp[61*22+20];
            // M10 是 D1 边界的对外输入夹具，不是桩。
            for (j = 0; j < 128; j = j + 1) begin
                k = base + j;
                case (k & 3)
                    0: `CNN.u_act.g_bank[0].u_bank.sim_write(k>>2, expected[off+j]);
                    1: `CNN.u_act.g_bank[1].u_bank.sim_write(k>>2, expected[off+j]);
                    2: `CNN.u_act.g_bank[2].u_bank.sim_write(k>>2, expected[off+j]);
                    3: `CNN.u_act.g_bank[3].u_bank.sim_write(k>>2, expected[off+j]);
                endcase
            end
            fixture = 1;
            $display("CPU seek/fsrc/ACT readback passed at cycle %0d; D1 fixture loaded", cycles);
        end

        if (gpio === 8'ha5) begin
            if (!fixture) $fatal(1, "missing fixture");
            for (j = 0; j < 64; j = j + 1) begin
                case ((j/8) & 3)
                    0: got = `CNN.u_act.g_bank[0].u_bank.sim_read(j/32);
                    1: got = `CNN.u_act.g_bank[1].u_bank.sim_read(j/32);
                    2: got = `CNN.u_act.g_bank[2].u_bank.sim_read(j/32);
                    3: got = `CNN.u_act.g_bank[3].u_bank.sim_read(j/32);
                endcase
                if (got[(j%8)*8+:8] !== emb[j]) begin
                    errors = errors + 1;
                    $display("embedding[%0d] got=%h expected=%h", j, got[(j%8)*8+:8], emb[j]);
                end
            end
            for (j = 0; j < 16; j = j + 1) begin
                got = 0;
                for (k = 0; k < 4; k = k + 1) got[k*8+:8] = emb[j*4+k];
                if (u_soc.u_ram.mem['h1c00+j] !== got[31:0]) begin
                    errors = errors + 1;
                    $display("CPU burst readback[%0d] got=%h expected=%h",
                             j, u_soc.u_ram.mem['h1c00+j], got[31:0]);
                end
            end
            if (errors) $fatal(1, "%0d embedding/readback mismatches", errors);
            $display("SYSTEM_INTEGRATION_PASS cycles=%0d embedding=64/64 cpu_readback=16/16", cycles);
            $finish;
        end

        if (cycles > 150000) $fatal(1, "timeout gpio=%h", gpio);
    end

endmodule
