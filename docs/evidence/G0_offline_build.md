# G0 旧工程离线构建记录

日期：2026-11-11。环境：Ubuntu 20.04.6 LTS、g++ 9.4.0、GNU Make 4.2.1。输入是 `legacy-baseline/doosan_ft_calibration_cpp/` 的只读快照，先复制到被 Git 忽略的 `build/g0_legacy/`，原目录没有修改。

使用 `SDK_ROOT=/home/hzy/Code/Doosan/BINARY_GL010118_20230324A`。仓库快照不包含 SDK 二进制；版本标签与文件哈希见 `legacy-baseline/manifest.json`。

| 执行项 | 结果 |
|---|---|
| `make -C build/g0_legacy SDK_ROOT=... -j2 build/ft_admittance_x build/ft_admittance_xyz build/ft_admittance_xyz_return build/ft_admittance_y_trajectory` | 成功；四个柔顺程序从快照源码编译并链接 |
| `make -C build/g0_legacy SDK_ROOT=... -j2 check` | 成功；旧工程已有的离线测试程序和标定 CLI 检查通过 |
| `./build/ft_calibration validate data/ft20_run02.csv output/ft20_run02_model.txt valid`（在临时旧工程目录内） | 成功；五个留出姿态的力 RMSE 为 0.112500/0.115416/0.147341 N，力矩 RMSE 为 0.002919/0.006308/0.009828 N·m |
| `make -C build/g0_legacy SDK_ROOT=... -j2 all` | 失败：`src/doosan_movej_autoinit.cpp` 缺失；`src/doosan_movejx_python_match.cpp` 也被构建配置引用但不在原目录 |

这些步骤只编译和运行旧工程的离线检查，没有连接机器人或传感器，也没有运行四个 `--run` 程序。构建终端记录保存在本地 `build/g0_build.log`、`build/g0_check.log`、`build/g0_all.log`；该目录为临时构建目录，不纳入版本库。
