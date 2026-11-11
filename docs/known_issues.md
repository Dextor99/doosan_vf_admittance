# 当前未解决事项

- `doosan_ft_calibration_cpp` 原目录无 Git 元数据；四种导纳模式真实运行日志缺失。新仓库的 `legacy-baseline` tag、初始导入提交和哈希清单标记了复制快照。
- 四个柔顺程序可单独编译，现有离线 `make check` 通过；但旧工程 `make all` 因缺 `src/doosan_movej_autoinit.cpp` 而失败，`src/doosan_movejx_python_match.cpp` 也缺失。
- 旧程序运行中分别检查机器人/XJC 数据年龄，但未对两者采样时刻差设置独立的控制门限；XYZ 三模式只打印跟踪误差。
- 本地三个开源项目由 ZIP 解压，没有上游 commit；SDU 尚未构建或数值评估。SDU 的固定步长实现与旧工程实际 `dt` 的处理不同。
- 当前主机没有 CMake，新工程的初版骨架未通过 CMake 构建；旧工程四个柔顺程序已由 Make 单独构建。
- 新工程还没有 Doosan/XJC 驱动、校准补偿代码、日志回放或运行监督器。
- XJC 候选 Modbus 定义尚未用准确型号厂家协议和逐轴静态动作核实。
- Doosan 控制器固件/DRCF/DRFL 版本及本地 API-DRFL 库版本对应关系未知。
- 控制器安全配置、现场工作区/工装、硬件急停和停止行为没有在本仓库验证。

因此当前产物只用于代码结构与数据契约的离线起步，不是可以连接或驱动实机的发布版本。
