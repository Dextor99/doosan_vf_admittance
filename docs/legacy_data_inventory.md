# 旧工程数据盘点

盘点范围：冻结快照 `legacy-baseline/doosan_ft_calibration_cpp/`。原目录与快照的 63 个非构建文件 SHA-256 一致；具体哈希见 `legacy-baseline/manifest.json`。

| 文件 | 实际内容 | 可用于 |
|---|---|---|
| `data/ft20_run02.csv` | 2026-11-08 的 20 姿态静态数据，15 行训练、5 行验证；列含 `stamp_s`、传感器相对基座姿态 `qx,qy,qz,qw`、六轴原始读数 | 标定模型复算、静态补偿回归 |
| `data/ft20_run02.csv.meta`、`.status` | 串口与安装假设；状态为 `COMPLETE`，20/20 姿态 | 对照采集条件，尚不能证明厂家协议正确 |
| `output/ft20_run02_model.txt`、`.meta` | 静态零偏/重力模型及匹配元数据 | 新补偿实现的回放基线 |
| `output/ft20_run02_model.txt.compensated.csv` | 20 行标定数据的补偿输出 | 静态补偿数值对照 |
| `data/taught_poses.csv` | 20 行示教姿态记录 | 核对标定采集姿态，不是柔顺运动轨迹 |
| `data/ft20_run01.csv`、`data/auto_samples_*.csv` | 只有表头，没有样本行 | 保存原始会话痕迹；不能做回放 |
| `data/synthetic.csv`、`output/compensated.csv`、`output/calibration.txt`、`output/demo_test_log.txt` | 合成示例及其输出 | 离线工具演示，不是实机证据 |

`ft20_run02.csv` 的 `stamp_s` 是 Unix 时间，样本范围按 Asia/Shanghai 为 2026-11-08 15:24:43–15:27:24。它是静止姿态采集记录，不含连续控制循环的参考位置、命令目标或跟踪误差。旧程序控制线程使用单调时钟；这两种时间不能直接混为同一时钟域。

在冻结快照的独立构建目录运行 `ft_calibration validate ... valid`，五个留出姿态的 Fx/Fy/Fz RMSE 为 0.112500/0.115416/0.147341 N，Mx/My/Mz RMSE 为 0.002919/0.006308/0.009828 N·m；这只确认当前静态模型与保存数据一致。详细输出见 `docs/evidence/G0_offline_build.md`。

## 四种模式运行日志缺口

| 计划数据集 | 当前真实运行日志 | 仍需取得的关键证据 |
|---|---|---|
| `free_x` | 缺失 | X 受力、释放后的实际/目标位置与速度 |
| `free_xyz` | 缺失 | 三轴方向、实际/目标位置与交叉响应 |
| `point_return` | 缺失 | 固定参考点、偏移和释放后的恢复过程 |
| `trajectory_y` | 缺失 | Y 参考轨迹、导纳偏移、命令和实际轨迹 |

后续记录至少要有单调时间戳或可对齐的采样时间、机器人实际 TCP/法兰位姿与关节状态、XJC 原始/补偿 wrench、模式、参考/目标/实际位置、实际 `dt`、设备数据年龄、控制权/保护状态、错误码和程序版本。当前旧程序约每十个周期向终端打印一次摘要；它没有保存上述字段完整的逐周期日志。若旧日志无法找回，可新增独立监视与日志采集；应把新采集数据标明日期，不将其称为过去实验的原始日志，也不修改旧控制算法补录历史。

本地有型号图纸与软件连接设置 PDF：`../XJC.InstrumentSet(V1.0.21.5)/CustomSheet/`。准确通讯协议、轴序、符号和单位仍需针对实物铭牌及厂家资料确认。
