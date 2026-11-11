# 依赖与本地开源快照记录

本地三个参考项目由压缩包解压得到；目前没有上游 commit 元数据，因此下面的 SHA-256 只固定用户本地压缩包字节，不等于上游 commit。源码目录和 ZIP 保持本地参考状态，不作为本工程编译依赖，也未复制进 Git。

| 项目 | 上游 | 本地 ZIP SHA-256 | 许可证（本地材料） | 初步边界 |
|---|---|---|---|---|
| `sdu_controllers-main` | <https://github.com/SDU-Robotics/sdu_controllers> | `ae4736d20837293f3f8b9722529b1915e95384d91bc43f5fea9b539cfc239bc4` | MIT (`LICENSE.md`) | 唯一候选导纳内核；先锁 commit 并完成离线评估。当前没有集成。 |
| `force_control-mainline` | <https://github.com/yifan-hou/force_control> | `449a5d3d6823069676ff35fc0c98456a64fc3b8c5a61efcf441f0e015476f312` | MIT (`LICENSE`) | 六维 wrench/twist 与混合柔顺的参考；依赖 cpplibrary/RobotUtilities、Eigen 3.4 和 yaml-cpp，不作为并行运动控制器。 |
| `API-DRFL-main` | <https://github.com/DoosanRobotics/API-DRFL> | `4b3917e38071c235b32bc85ac09b87bdbd486259ac4dfa76360ea1315b304d93` | BSD-3-Clause (`LICENSE`) | 只作官方 SDK 头文件/API 与样例参考；具体版本必须匹配机器人控制器的 DRCF/DRFL 版本。 |

## 环境盘点

- 编译器：`c++ (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0`。
- CMake：当前不可用（`cmake` 命令未找到）。
- `API-DRFL-main` 包含 amd64 Ubuntu 20.04 的静态库和 Poco 动态库，但上游 README 将仓库内库文件说明为示例版本，并要求核对/下载匹配的正式 SDK。未尝试链接该版本。
- 旧工程实际构建依赖相邻目录 `../BINARY_GL010118_20230324A` 的 Ubuntu 20.04 DRFL/Poco 库；已记录 `DRFL.h`、`DRFLEx.h`、`libDRFL.a` 和两个 Poco 库的 SHA-256 于 `legacy-baseline/manifest.json`。文件夹标签不能代替现场控制器版本确认。
- 力传感器确切型号、寄存器映射、单位、通道顺序及符号仍待铭牌和厂家协议确认。

## 锁版规则

进入集成前须从上游 Git 获取源码和完整 commit hash，记录所选 branch/tag、许可证、依赖版本及本地修改。不得把当前无 commit 的压缩包快照写成已锁定的上游版本。Doosan SDK 必须以现场控制器上的 DRFL/DRCF 版本为准。
