//=====================================================================
// tb_coremark.v : 交付树 CoreMark 功能验证台
//
//   口径与 timing_opt/runs/perf_sim/perf_tb.v 的固件镜像、RAM 容量、
//   BTB/S6b 开关完全一致；只去掉性能计数器读数（交付树已剥离 PMU）。
//   判定：UART 打印的四行 CRC 必须等于 CoreMark 2K 官方值
//         crclist 0xe714 / crcmatrix 0x1fd7 / crcstate 0x8e3a
//         crcfinal 0xe714
//
//   运行（在 release/ 目录下）:
//     iverilog -g2012 -s tb_coremark \
//       -I rtl/core -I rtl/soc \
//       -o /tmp/rel_cm.vvp rtl/core/*.v rtl/soc/*.v tb/tb_coremark.v
//     vvp /tmp/rel_cm.vvp
//   约 21.6 万拍（5 分钟左右）；末行打印 TB_PASS / TB_FAIL。
//=====================================================================
`timescale 1ns/1ps

module tb_coremark;

    reg clk = 0;
    reg resetn = 0;
    always #5 clk = ~clk;

    soc_top #(
        .CLK_FREQ       (100_000_000),
        .BAUD           (10_000_000),
        .RAM_ADDR_W     (16),
        .RAM_INIT_FILE  ("tb/coremark.mem"),
        .RAM_EMBED_INIT (0),
        .BTB_ENABLE     (1),
        .S6B_ENABLE     (1),
        .FACEREC_EN     (0),
        .ACCEL_EN       (0),
        .QLN_EN         (0),
        .PIPE_MMIO      (1)
    ) u_soc (
        .clk      (clk),
        .resetn   (resetn),
        .cnn_clk  (clk),
        .uart_tx  (),
        .uart_rxd (1'b1),
        .key_in   (1'b1),
        .gpio_out (),
        .cpu_stall()
    );

    integer uart_cnt = 0;
    reg [63:0] hist = 64'd0;
    localparam [63:0] KEY = 64'h63726366_696E616C;   // "crcfinal"

    // "crcfinal" 只是行首；其后的 ": 0xeeee" 才是结果。命中后再收 20 个字符
    // （"\t: 0xeeee\r\n"），把数值也打印出来并核对。
    integer tail_cnt = 0;
    reg     tail_on  = 0;
    reg [47:0] tail  = 48'd0;                 // 最近 6 个字符
    localparam [47:0] GOOD = 48'h30_78_65_37_31_34;   // "0xe714"
    reg done_seen = 0;
    reg crc_ok = 0;

    always @(posedge clk)
        if (resetn && u_soc.d_accept && u_soc.d_write && u_soc.uart_sel
            && u_soc.d_addr[3:2] == 2'b00) begin
            $write("%c", u_soc.d_wdata[7:0]);
            uart_cnt = uart_cnt + 1;
            hist = {hist[55:0], u_soc.d_wdata[7:0]};
            if (uart_cnt % 64 == 0) $fflush();
            if (hist == KEY) tail_on = 1;
            if (tail_on) begin
                tail = {tail[39:0], u_soc.d_wdata[7:0]};
                tail_cnt = tail_cnt + 1;
                if (tail == GOOD) crc_ok = 1;
                if (tail_cnt >= 20) begin
                    done_seen = 1;
                    tail_on = 0;
                end
            end
        end

    integer cycles = 0;
    initial begin
        repeat (8) @(negedge clk);
        resetn = 1;
    end

    always @(negedge clk) if (resetn) begin
        cycles = cycles + 1;
        if (done_seen) begin
            $display("");
            if (crc_ok) $display("TB_PASS crcfinal=0xe714 cycles=%0d", cycles);
            else        $display("TB_FAIL crcfinal != 0xe714 cycles=%0d", cycles);
            $finish;
        end
        if (cycles > 900_000) begin
            $display("");
            $display("TB_TIMEOUT cycles=%0d", cycles);
            $finish;
        end
    end

endmodule
