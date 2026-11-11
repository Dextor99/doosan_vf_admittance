# 旧柔顺程序与迁移范围

本记录依据 `legacy-baseline/doosan_ft_calibration_cpp/` 中已经冻结的文件。四个历史程序分别是 `ft_admittance_x`、`ft_admittance_xyz`、`ft_admittance_xyz_return`、`ft_admittance_y_trajectory`。旧工程仍是行为基线；新工程按 G1–G4 的准入条件逐步迁移。

## 源码对应关系

| 当前功能 | 旧工程位置 | 新工程目标位置和阶段 |
|---|---|---|
| XJC Modbus 读取、CRC、六个 float 候选解码 | `include/xjc_serial.hpp` | G2 `hardware/xjc_driver`；先验证准确型号协议 |
| Doosan 监控快照、姿态换算、采样元数据 | `include/ft_acquisition.hpp`、`include/pose_math.hpp` | G2 机器人只读适配与数据契约 |
| 静态零偏与重力补偿 | `include/ft_compensator.hpp`、`output/ft20_run02_model.txt` | G2 `estimation/ft_compensator`；沿用模型并做回归 |
| X 单轴与 XYZ 自由导纳 | `include/admittance_x.hpp`、`include/admittance_xyz.hpp` | G3 数学对照；G4 以一个控制器加轴使能配置迁移 |
| XYZ 定点回归及轨迹偏移 | `include/admittance_xyz_return.hpp` | G3 数学对照；G4 以固定参考或独立 Y 轨迹迁移 |
| 连接、控制权、`servol()`、停止 | 四个 `src/ft_admittance_*.cpp` 各自实现 | G2/G5 统一驱动、状态机与监督器；唯一命令发送者 |

四个入口均包含近似相同的传感器轮询、机器人监控、启动检查和控制循环。迁移时提取行为与数据语义，不直接把四份完整入口合并到新运行时。旧程序的具体参数和停止行为见 `docs/legacy_modes.md`。

## 旧工程构建和依赖

- 构建工具：GNU Make、g++ C++17、pthread。目标程序使用上级 `BINARY_GL010118_20230324A` 的 Ubuntu 20.04 `libDRFL.a`、Poco 库和 DRFL 头文件。
- 冻结快照里的四个柔顺程序均能在独立临时目录从源码编译；旧工程 `make check` 的离线检查通过。构建记录见 `docs/evidence/G0_offline_build.md`。
- `make all` 仍因缺失 `src/doosan_movej_autoinit.cpp` 而失败；Makefile 和 CMakeLists 还引用了缺失的 `src/doosan_movejx_python_match.cpp`。这两个运动示例不属于四个柔顺程序，但使旧工程整体构建无法通过。
- SDK 文件和标定资产的 SHA-256 在 `legacy-baseline/manifest.json`。目录名不是控制器当前固件/SDK 兼容性的证明。

## 功能边界

- `--monitor` 是只读机器人和力数据监视；`--run` 申请控制权并发送运动目标。四个程序均使用普通 TCP `servol()`，不使用实时 UDP。
- 目前只做平移柔顺；末端姿态保持固定。旧标定模型只补偿静态重力与零偏，不覆盖运动惯性、线缆受力或温漂。
- 四个模式的历史原始运行日志不存在于已提供目录。20 姿态标定数据不能充当柔顺实验回放。详见 `docs/legacy_data_inventory.md`。
