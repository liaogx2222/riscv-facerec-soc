# 输入指纹清单

由 `tools/manifest.py --write` 生成。任何一次 P&R 的结论都应该能对回这里。

| 组 | 文件数 | 整体指纹（sha256 前 16 位） |
|---|---|---|
| RTL (core) | 17 | `5676fa1532f5c81b` |
| RTL (soc) | 9 | `bbfd8e52418d4492` |
| RTL (accel) | 27 | `a4064777be139138` |
| RTL (board) | 2 | `c6e2068b8cb06de8` |
| CNN ROM (.mem) | 15 | `823dfa0fb9c97f3a` |
| 约束 | 2 | `4fb9c121ae5d059b` |
| 约束 (签核) | 1 | `70e868ac796ef635` |
| 约束 (钉位) | 1 | `7fe9c9dfacdeba8b` |
| 固件源码 | 16 | `4a4844149824ec01` |
| 测试台 | 2 | `fae9ada59ecd8d29` |

## 明细

### RTL (core)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `rtl/core/alu.v` | 7578 | `8657ba3c79f68a72` |
| `rtl/core/btb.v` | 18841 | `894fec268af101fb` |
| `rtl/core/cpu_rv32i.v` | 168791 | `208799dd1d523a90` |
| `rtl/core/csr.v` | 5803 | `6074f54517ca72f5` |
| `rtl/core/ctrl.v` | 14178 | `fd9e162d255ac8cf` |
| `rtl/core/forwarding_unit.v` | 9905 | `43ff4a859aeb0221` |
| `rtl/core/hazard_unit.v` | 4048 | `bd078e9a73c4d841` |
| `rtl/core/imm_gen.v` | 1639 | `6fe9336c210c07b5` |
| `rtl/core/load_format.v` | 592 | `56005e75f3f1cc93` |
| `rtl/core/mul_unit.v` | 5006 | `d4ee3b586dd8b85b` |
| `rtl/core/pair_value_reg.v` | 498 | `31e653e62d86b1a7` |
| `rtl/core/pc_inc4.v` | 236 | `fada3bc3d9deb73e` |
| `rtl/core/pc_step.v` | 269 | `6034a8d8f7b01cf8` |
| `rtl/core/ras.v` | 2554 | `7984df84189acfbe` |
| `rtl/core/regfile.v` | 3547 | `dd5e51bdefaee1c7` |
| `rtl/core/rv32i_defs.v` | 2342 | `f30fb61458ac5123` |
| `rtl/core/sb_cam_match.v` | 1117 | `ef79c57b57349634` |

### RTL (soc)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `rtl/soc/cnn_cdc_bridge.v` | 2740 | `9cebd31322e10cdf` |
| `rtl/soc/dcache.v` | 19039 | `c1747ed7d414cc05` |
| `rtl/soc/icache.v` | 16212 | `1c5f4e299e08d88e` |
| `rtl/soc/intc.v` | 3252 | `ea075ec1d795441d` |
| `rtl/soc/pf_buffer.v` | 31812 | `a74c44f5cd7cb72d` |
| `rtl/soc/simple_ram.v` | 449068 | `e25cecf4f315c8d0` |
| `rtl/soc/soc_top.v` | 37858 | `c073cfcee94dbfaf` |
| `rtl/soc/uart_rx.v` | 3650 | `f491a5c043d6e374` |
| `rtl/soc/uart_tx.v` | 3359 | `afdfebf7aee868c2` |

### RTL (accel)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `rtl/accel/acc_top.v` | 6411 | `9bb85736a0468dfa` |
| `rtl/accel/acc_top_slide.v` | 7345 | `6d199687cde784f1` |
| `rtl/accel/cnn_act.v` | 4884 | `118093c398dee202` |
| `rtl/accel/cnn_bl.v` | 1094 | `6fccd4fbeb1296cd` |
| `rtl/accel/cnn_ctrl.v` | 11118 | `423b1cb195a8b1f2` |
| `rtl/accel/cnn_d1.v` | 15880 | `70fc7f49228c4e71` |
| `rtl/accel/cnn_desc_dec.v` | 4510 | `89d6fc3e5d5acb6f` |
| `rtl/accel/cnn_dman.v` | 2772 | `9cb7fdd4665dcedb` |
| `rtl/accel/cnn_dwrom.v` | 1748 | `787a70884e450aa4` |
| `rtl/accel/cnn_engine.v` | 26471 | `522b2b698a76c50b` |
| `rtl/accel/cnn_framesrc.v` | 6190 | `57491c638b1c846c` |
| `rtl/accel/cnn_fsrc.v` | 5788 | `43588863be4dcdef` |
| `rtl/accel/cnn_fuse_fifo.v` | 3181 | `dbf34602a82c5cb9` |
| `rtl/accel/cnn_linebuf.v` | 73010 | `7b91a6c2b74a4693` |
| `rtl/accel/cnn_mmio.v` | 14016 | `c4ffb8089e224ddd` |
| `rtl/accel/cnn_outacc.v` | 13995 | `73e02ec95f12c6c6` |
| `rtl/accel/cnn_pe.v` | 2334 | `5a91aae86831b9ba` |
| `rtl/accel/cnn_pe_col.v` | 2487 | `019ef9e0bd0474a7` |
| `rtl/accel/cnn_pe_page.v` | 6112 | `57a454301473700e` |
| `rtl/accel/cnn_ram.v` | 2060 | `0485b822da74f412` |
| `rtl/accel/cnn_roi.v` | 21387 | `21755a2a6f82cffe` |
| `rtl/accel/cnn_seq.v` | 25124 | `7604a871b22d4934` |
| `rtl/accel/cnn_shcrom.v` | 1496 | `7b3db89f91470f59` |
| `rtl/accel/cnn_slidewin.v` | 14347 | `118702714f2b39a4` |
| `rtl/accel/cnn_slidewin_rom4_banked.v` | 1396 | `6bd84ce0737222bf` |
| `rtl/accel/cnn_soc.v` | 13682 | `a6f19aceb2cb3a2e` |
| `rtl/accel/cnn_wrom.v` | 3023 | `226a334d9a35af5e` |

### RTL (board)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `rtl/board/facerec_top.v` | 4042 | `11f26a27fd071cce` |
| `rtl/board/pll.v` | 1293 | `880df5a316645dea` |

### CNN ROM (.mem)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `rtl/accel/mem/cnn_d1shc.mem` | 256 | `b8f11bd32ea92bf4` |
| `rtl/accel/mem/cnn_desc.mem` | 5120 | `1f88ab3af66b6450` |
| `rtl/accel/mem/cnn_dwrom.mem` | 147456 | `b61c96c1f01e5aba` |
| `rtl/accel/mem/cnn_shcrom.mem` | 11748 | `ab0976e63c41e603` |
| `rtl/accel/mem/cnn_shcrom_tb.mem` | 132 | `0038d6eecd726d45` |
| `rtl/accel/mem/cnn_wrom.mem` | 838678 | `38e7cd9da5af9e07` |
| `rtl/accel/mem/cnn_wrom_tb.mem` | 6936 | `1f15dff55bf3d8f2` |
| `rtl/accel/mem/framesrc.mem` | 917504 | `300a179e8f9790dc` |
| `rtl/accel/mem/slidewin.mem` | 921600 | `71eb8d5753aa204a` |
| `rtl/accel/mem/slidewin4.mem` | 57600 | `53e1cf72245a721b` |
| `rtl/accel/mem/slidewin_bank0.mem` | 196608 | `80b8d969b304103f` |
| `rtl/accel/mem/slidewin_bank1.mem` | 196608 | `a61545717f7a36ee` |
| `rtl/accel/mem/slidewin_bank2.mem` | 196608 | `d7a16b606dc20be0` |
| `rtl/accel/mem/slidewin_bank3.mem` | 196608 | `6ff11ad0ab2f056f` |
| `rtl/accel/mem/slidewin_bank4.mem` | 135168 | `3416c42093ac980d` |

### 约束

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `constraints/27mhz/board.fdc` | 4581 | `3ef513c2db19007d` |
| `constraints/product/board.fdc` | 4699 | `f4e3c9afe50efc0c` |

### 约束 (签核)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `constraints/product/signoff/timing_clean.sdc` | 2215 | `05597c1d623ee1aa` |

### 约束 (钉位)

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `constraints/product/signoff/d1pin6.pcf` | 1131 | `f878bb895e799773` |

### 固件源码

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `sw/_test_vec.h` | 363895 | `8d8ca822728ffc46` |
| `sw/cm_loop_main.c` | 2604 | `7b1774e436f92a65` |
| `sw/gate_api.h` | 1627 | `3d3c4160ed194eec` |
| `sw/gate_board.c` | 15461 | `2e6d1ec083cf2be8` |
| `sw/gate_core.c` | 11443 | `9b23cabca37a1189` |
| `sw/gate_core.h` | 6498 | `e47c76159e0964e2` |
| `sw/gate_main.c` | 18486 | `7e5a0299972f5115` |
| `sw/gate_map.h` | 5572 | `3a2c4ba38c293e29` |
| `sw/gate_regs.h` | 2940 | `aef997745f0fa9d1` |
| `sw/integration.c` | 2017 | `3dccac134f59a1ce` |
| `sw/menu_main.c` | 16591 | `cfecdffb3286a387` |
| `sw/start.S` | 1298 | `dd9a3738e0e9d783` |
| `sw/start_cmloop.S` | 1307 | `5956be0bb2ec2f1f` |
| `sw/test_gate_core.c` | 8817 | `1e5e59b577f78616` |
| `sw/uart_out.c` | 4450 | `e3f89c545db4bfd0` |
| `sw/uart_out.h` | 3603 | `a5d64acb4d50e5c7` |

### 测试台

| 文件 | 字节 | sha256/16 |
|---|---|---|
| `tb/tb_coremark.v` | 3351 | `fa5950286cfe7187` |
| `tb/tb_system_integration.v` | 5255 | `ea5ed2d888661135` |
