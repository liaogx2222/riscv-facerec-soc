# FaceRec SoC 交付树 —— 顶层构建入口
#
# 常用目标：
#   make sim-cm       CoreMark 回归（Icarus，约 5 min）
#   make sim-sys      整机 CNN 端到端回归（Icarus，约 2 min）
#   make sim          sim-cm + sim-sys
#   make syn          只综合（2022.2，约 14 min）→ build/syn_$(RUN)/synthesize/
#   make pnr          只布局布线（2025.2，约 40 min），需先 make syn
#   make bit          全流程：综合 + 布线 + 位流 + 报告（约 55 min）
#   make manifest     刷新 MANIFEST.md（输入指纹）
#   make clean        删除 build/（保留 rtl/sw/constraints）
#
# 变量：
#   RUN=名字          运行名（默认取当天时间），产物落在 build/syn_$(RUN) / build/pnr/$(RUN)
#                     注意 P&R 目录没有 pnr_ 前缀（v2_flow.py 只接受 <name>）
#   REF_SBIT=路径     给 bit 目标指定参考码流，做"除时间戳外逐字节"比对
#   LIC_2025         2025.2 许可文件路径
#
# 注意：综合用 2022.2，布局布线用 2025.2，两段不能合并（见 doc/reproduce.md §3）。

RUN       ?= $(shell date +%m%d_%H%M)
LIC_2025  ?= $(PANGO_LICENSE_FILE)
REF_SBIT  ?=
PY        ?= python
IV        ?= iverilog
VVP       ?= vvp

.PHONY: all sim sim-cm sim-sys syn pnr bit manifest help clean

all: sim

help:
	@grep -E '^#   ' Makefile | sed 's/^#   //'

# ---------------------------------------------------------------- 仿真
sim-cm:
	$(IV) -g2012 -s tb_coremark -I rtl/core -I rtl/soc \
	  -o build/cm.vvp rtl/core/*.v rtl/soc/*.v tb/tb_coremark.v
	$(VVP) build/cm.vvp | tail -3

sim-sys:
	$(IV) -g2012 -DFACEREC_SIM -s tb_system_integration \
	  -I rtl/core -I rtl/soc -I rtl/accel \
	  -o build/sys.vvp $$(cat rtl/files.f) tb/tb_system_integration.v
	$(VVP) build/sys.vvp | tail -3

sim: sim-cm sim-sys

# ---------------------------------------------------------------- 实现
syn:
	$(PY) tools/synth_vm.py $(RUN)

pnr:
	PANGO_LICENSE_FILE="$(LIC_2025)" $(PY) tools/v2_flow.py $(RUN) \
	  --netlist build/syn_$(RUN)/synthesize/facerec_top_syn.vm

bit:
	$(PY) tools/build_flow.py $(RUN) $(if $(REF_SBIT),--ref-sbit $(REF_SBIT),)

manifest:
	$(PY) tools/manifest.py --write

clean:
	rm -rf build/*.vvp

.PRECIOUS: build/%.vvp
