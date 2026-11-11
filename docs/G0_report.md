# G0 原型冻结记录

**记录日期：** 2026-11-11

**状态：** 部分完成；不满足 G0 验收条件。

## 已完成

- 将相邻目录 `../doosan_ft_calibration_cpp` 中的源码、Makefile、README、研究文档、配置模板、原始数据、标定模型、模型元数据和已有输出复制到 `legacy-baseline/doosan_ft_calibration_cpp/`。
- 快照共 63 个文件、580,694 字节；`build/` 生成物未复制。每个纳入文件的 SHA-256 在 `legacy-baseline/manifest.json` 中。
- 快照文件设置为只读；原工程目录没有修改。
- 从源码提取的四种运行模式、参数、数据条件和停止行为见 `docs/legacy_modes.md`。

## 未完成 / 缺口

- 原目录没有 `.git` 元数据，不能创建 `legacy-baseline` Git tag，也无法记录源 commit。当前以只读文件快照和哈希清单替代。
- 没有找到 `free_x`、`free_xyz`、`point_return`、`trajectory_y` 四种模式的真实运行日志。`data/synthetic.csv` 是合成数据，不能代替实机基线；标定数据和示例输出也不能当作导纳运行日志。
- 当前环境没有 CMake，因此未确认旧工程能从干净目录重新编译。旧目录已有的 `build/` 二进制不构成可复现构建证明。
- 标定文件哈希已保存，但文件来源日期、控制器/DRFL SDK 版本与准确 XJC 型号仍未全部从原始资料核验。

## Gate 结论

G0 只完成旧文件保护和模式盘点，不应标记为通过。下一步需补齐四模式历史日志（若不存在则明确记录缺失）、运行清洁构建，并将源工程纳入独立 Git 仓库或生成外部归档版本号。
