# MES2L676-200HP: 27MHz input; CPU101.25MHz and CNN202.5MHz PLL outputs.
# Async external controls; only external I/O endpoints are excluded.
create_clock -name clk_sys -period 9.876543 [get_pins {u_pll/clkout0}]
create_clock -name clk_cnn -period 4.938272 [get_pins {u_pll/clkout1}]
set_clock_uncertainty 0.100 [get_clocks {clk_sys clk_cnn}]
set_false_path -from [get_ports {key_n uart_rxd}]
set_false_path -to [get_ports {led_out[*] uart_txd}]

# Handshake CDC only: first synchronizer stages receive asynchronous toggles.
# The second stages and ALL intra-domain logic remain timed.
set_false_path -to [get_cells -hierarchical {*u_bridge/m_req_sync1*}]
set_false_path -to [get_cells -hierarchical {*u_bridge/c_ack_sync1*}]
# Reset assertion crosses into a three-stage CNN reset-release synchronizer.
# This does not cut the CNN-clocked shift/release paths between its stages.
set_false_path -from [get_clocks {clk_sys}] -to [get_cells -hierarchical {*G_CDC/cnn_reset_pipe*}]
# Bundled payloads are held until the returned synchronized acknowledgement.
# Bound physical data delay rather than excluding these data paths.
set_max_delay -datapath_only -from [get_cells -hierarchical {*u_bridge/c_req_hold*}] -to [get_cells -hierarchical {*u_bridge/m_req_hold*}] 3.500
set_min_delay -from [get_cells -hierarchical {*u_bridge/c_req_hold*}] -to [get_cells -hierarchical {*u_bridge/m_req_hold*}] 0.000
set_max_delay -datapath_only -from [get_cells -hierarchical {*u_bridge/m_rsp_hold*}] -to [get_cells -hierarchical {*u_bridge/c_rdata*}] 3.500
set_min_delay -from [get_cells -hierarchical {*u_bridge/m_rsp_hold*}] -to [get_cells -hierarchical {*u_bridge/c_rdata*}] 0.000

# key_n (C22, 低有效)。平台 rst_n 可能长期为低 → 顶层仅作辅助复位(3 级同步)

# 调试串口: AB26 已实测确认为 FPGA→仪器 TXD

# UART RX 预留脚 (AC26 = TXD AB26 对侧)

#---------------------------------------------------------------------
# S3 指示灯 LED[0..3] (低电平点亮)
#   [0] 出复位常亮 / [1] 心跳闪烁 / [2] 门状态 / [3] uart 活动
#---------------------------------------------------------------------




# Keep the CPU, fetch queue and caches close; no timing exceptions added.
