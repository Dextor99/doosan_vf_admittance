# G0 原型冻结记录

**记录日期：** 2026-11-11

**状态：** 部分完成；不满足 G0 验收条件。

## 已完成

- 将相邻目录 `../doosan_ft_calibration_cpp` 中的源码、Makefile、README、研究文档、配置模板、原始数据、标定模型、模型元数据和已有输出复制到 `legacy-baseline/doosan_ft_calibration_cpp/`。
- 快照共 63 个文件、580,694 字节；`build/` 生成物未复制。每个纳入文件的 SHA-256 在 `legacy-baseline/manifest.json` 中。
- 快照文件设置为只读；原工程目录没有修改。
- 从源码提取的四种运行模式、参数、数据条件和停止行为见 `docs/legacy_modes.md`。
- 原目录与快照中的 63 个非构建文件逐一比对哈希一致；新仓库的 `legacy-baseline` tag 指向快照首次纳入 Git 的提交 `a1a45dd9e59ee68fc60738e336cc84048fe884f1`。该 tag 标记导入快照，不代表旧目录原有 Git 历史。可用 `python3 tools/verify_legacy_snapshot.py` 检查快照完整性。
- 在独立临时目录成功编译四个柔顺程序，并运行旧工程 `make check` 离线检查。详细命令和结果见 `docs/evidence/G0_offline_build.md`。
- 使用旧 `ft_calibration validate` 对五个留出姿态复核了模型：力通道 RMSE 为 0.112500/0.115416/0.147341 N，力矩通道 RMSE 为 0.002919/0.006308/0.009828 N·m。这是静态标定回归结果，不是动态柔顺控制精度。
- 已对标定数据、输出、合成示例和四种模式的历史日志缺口分类，见 `docs/legacy_data_inventory.md`；旧源码迁移范围见 `docs/legacy_capabilities.md`。

## 未完成 / 缺口

- 原目录没有 `.git` 元数据，无法记录旧工程自己的源 commit。新仓库的只读快照与初始导入提交只能证明复制时的文件内容。
- 没有找到 `free_x`、`free_xyz`、`point_return`、`trajectory_y` 四种模式的真实运行日志。`data/synthetic.csv` 是合成数据，不能代替实机基线；标定数据和示例输出也不能当作导纳运行日志。
- `make all` 在独立目录失败：旧 Makefile 引用的 `src/doosan_movej_autoinit.cpp` 不存在，另一个 `src/doosan_movejx_python_match.cpp` 也缺失。四个柔顺程序本身可以单独构建。
- 标定文件和本地 Doosan SDK 文件哈希已保存；控制器实际 DRCF/DRFL 兼容版本与准确 XJC 协议仍未核验。新工程的 CMake 仍因本机缺少 `cmake` 命令而未构建。

## Gate 结论

G0 已完成可用旧文件保护、模式盘点、四程序独立构建和现有离线检查；仍缺四模式历史日志，且旧工程整体 `make all` 的两个源码文件缺失，因此不标记为通过。下一步先查找原始运行记录或明确记录其不可恢复，再确定无改动旧控制算法的新采集办法。
