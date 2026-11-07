稳定演示配置（单人模板，对应技术报告表4"菜单版/调试版"的模型数据）。
切换方法：将本目录 6 个文件按原路径覆盖到 rtl/ 与 tb/：
  cnn_*.mem -> rtl/accel/mem/
  simple_ram.v -> rtl/soc/
  flow_emb.hex -> tb/
覆盖后用 tools/build_flow.py 或 Makefile 正常构建；仿真期望 SYSTEM_INTEGRATION_PASS。
包内默认配置为五人QAT模型（对应表4"五人QAT模型版"码流）。
