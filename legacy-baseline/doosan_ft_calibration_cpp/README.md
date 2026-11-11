# Doosan 与 XJC 六维力：静态零偏标定与重力补偿

本工程采集机械臂法兰姿态和外接 XJC 传感器的六维读数，利用多个静止、无接触姿态拟合传感器的恒定零偏和末端负载的静态重力，再计算补偿后的数据。`ft_acquire` 是只读采集程序；**`ft_auto run` 和 `run-recorded` 会控制真实机械臂运动**，使用前需要审核当前起点和各段路径。

**使用前必须核对 XJC 数据。** 目前按照本机的只读通信结果，将站号 1、功能码 04、地址 0 起的 12 个寄存器按大端浮点数解释成 `Fx,Fy,Fz,Mx,My,Mz`。该解释尚未由传感器准确型号的协议或厂家软件确认，轴顺序、正负、单位、原厂标定与量程都需要核对。未经核对及独立验证的结果不能用于机器人力控。命令中的 `--assume-xjc-format` 表示采用这个待核实的解释，并不证明它正确。

## 1. 编译

以 `Doosan` 为工作区，终端执行：

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
make
```

生成离线工具、采集/标定程序，以及 `build/doosan_movej_minimal`、`build/doosan_movej_autoinit`、`build/doosan_movejx_python_match` 等单次运动测试程序。当前 Linux 的 SDK 在上一级 `BINARY_GL010118_20230324A` 下，设备程序使用其中的 Ubuntu 20.04 版本及 g++/pthread。无需安装 CMake、ROS、Eigen 或 libmodbus。若安装了 CMake，也可使用 `cmake -S . -B build` 和 `cmake --build build`；没有 SDK 时 CMake 可只构建离线工具。

目录原有的 `data/synthetic.csv`、`output/calibration.txt`、`output/compensated.csv` 是**模拟数据和模拟标定结果**，不能当作这台设备的标定结果。

## 单方向 X 自由导纳实验

`ft_admittance_x` 使用真实标定模型 `output/ft20_run02_model.txt`，只开放**机器人基坐标系 X 方向**，锁定当前 TCP 的 Y、Z 和姿态。作用力必须通过传感器之后的末端把手传递；推动机械臂其他连杆不会正确反映到外置传感器。先只读查看 X 向补偿力，并在受控条件下确认人为向机器人基坐标系 `+X` 施力时输出的正负号：

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
make build/ft_admittance_x
./build/ft_admittance_x 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --monitor
```

如果向 `+X` 施力时 `Fx_base` 为正，运行时传 `--run 1`；若为负，传 `--run -1`。以下命令中的 `1` 只是格式示例，必须按实际方向检查结果选择：

```bash
./build/ft_admittance_x 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --run 1
```

程序检查模型元数据、机器人和 XJC 新鲜度及初始无接触状态。TCP X 不设置程序内的行程边界，由机器人控制器的工作空间和安全限位管理；运行前需在示教器确认这些限位已配置。开始时以无接触的基坐标系 X 力作为零偏参考；施力时按 `M=1.0 kg、D=12 N·s/m、K=0` 的一维模型生成目标。当前死区 `0.25 N`、低通时间常数 `0.08 s`、导纳目标最大速度 `30 mm/s`、加速度 `150 mm/s²`；`servol` 跟随上限设为 `50 mm/s` 和 `300 mm/s²`，为机械臂跟随目标及往返反向留出余量。**三轴外力合力**超过软件阈值、传感器或机器人数据过期、控制权/安全状态异常、关节安全范围或 TCP 目标跟踪误差超过 `12 mm` 时退出并请求停止。日志同时打印基坐标系 `Fx,Fy,Fz`、合力 `|F|` 和扣除零偏后的 `Fx_control`。按回车或 Ctrl+C 正常停止。

若已确认所用末端把手、传感器量程和工作区允许更大的交互力，可在 `--run 1` 或 `--run -1` 后加 `--force-limit-n 25`，将合力停机阈值显式设为 `25 N`；参数可设为 `1～200 N`。例如 `--force-limit-n 200` 仅把本程序的软件超力停机阈值提高到 `200 N`，**不会提高运动速度，也不会修改机器人控制器中的安全力限值**。先根据日志确认施力方向和其他两轴分量，再选择阈值。

这版在普通 Ubuntu 上使用 SDK 的连续位置接口 `servol()`，不是 `servol_rt()`。普通 `servol()` 用于非实时外环验证；程序设置自己的速度和加速度上限。现场仍需保持独立急停可用，程序的 `STOP_TYPE_QUICK` 请求不能代替硬件急停。导纳模型使用静态标定，运动中的惯性力仍会影响补偿值。`ft_admittance_x` 只开放 X 轴，XYZ 平移由下面的独立程序实现；转动方向导纳和完整六维导纳尚未实现。

## XYZ 三方向自由导纳

`ft_admittance_xyz` 在机器人基坐标系中开放 TCP 的 X/Y/Z 平移，末端姿态保持不变。程序以启动前的无接触力作为三轴零偏，力按基坐标系分量计算。速度 `30 mm/s` 和加速度 `150 mm/s²` 是 XYZ 合速度、合加速度上限；质量、阻尼、死区、滤波和 `servol` 跟随参数沿用 X 程序当前值。程序不设置 TCP 绝对行程边界，由控制器安全工作空间限制行程。

先只读监视，并逐个方向轻推末端把手，确认基坐标系各轴的力符号：

```bash
make build/ft_admittance_xyz
./build/ft_admittance_xyz 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --monitor
```

确认向每个正方向施力时对应分量为正，就用 `1 1 1`；某个方向读数为负时，该轴符号传 `-1`。先用较低的合力停机阈值逐轴验证，再尝试同时多轴拖动：

```bash
./build/ft_admittance_xyz 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --run 1 1 1 --force-limit-n 20
```

运行时会输出 `Fbase`、扣除启动零偏后的 `Fcontrol`、实际 TCP、目标 TCP 和 XYZ 跟踪误差；跟踪误差仅作观察，不会单独触发停机。三轴外力合力超过阈值、关节安全范围或机器人/传感器状态异常时程序会停止并请求快速停止。多方向运动前确认控制器的三维工作空间、传感器安装和线缆范围；此程序只控制末端平移，不控制姿态，也不支持通过机械臂其他连杆拖动。

## XYZ 定点回归导纳

`ft_admittance_xyz_return` 同样开放基坐标系 X/Y/Z 平移并保持姿态不变，但目标点固定为控制开始时的 TCP 位置。持续施力时，导纳目标随力移动；松手后弹簧项将目标拉回起始点。模型为 `M ẍ + D ẋ + K(x-x₀) = F`，当前 `M=1.0 kg、D=12 N·s/m、K=36 N/m`，三轴合速度上限 `30 mm/s`、合加速度上限 `150 mm/s²`。当前阻尼与刚度接近临界阻尼，可减少松手回归时的振荡。`servol` 的平移跟随加速度上限设为 `1000 mm/s²`；这是 SDK 跟随限值，虚拟导纳目标仍受 `150 mm/s²` 限制。

先用 `--monitor` 确认基坐标系各方向力的符号，再分别设置三个轴的符号（每个为 `1` 或 `-1`）。例如三个方向的正向施力读数均为正时：

```bash
make build/ft_admittance_xyz_return
./build/ft_admittance_xyz_return 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --run 1 1 1 --force-limit-n 20
```

启动时记录无接触的 XYZ 力零偏和当前 TCP 回归点。程序不设置 TCP 绝对行程边界；请确认控制器工作空间和线缆范围允许回归点周围的 XYZ 运动。运行中超出合力阈值、关节安全范围、传感器/机器人数据超时或安全状态异常时会停止。第一次试验先单轴轻推，确认符号和松手回归方向，再尝试多轴。

## Y 轴往返轨迹抗扰恢复

`ft_admittance_y_trajectory` 以启动时 TCP 为起点，沿基坐标系 `+Y` 走 `200 mm`，再沿 `-Y` 回到起点，并持续循环；每轮总路程 `400 mm`，约 `20.8 s`。只有按回车或 Ctrl+C 才会正常结束。名义轨迹速度 `20 mm/s`、加速度 `50 mm/s²`。外力产生 XYZ 偏移，松手后按 `M=1 kg、D=12 N·s/m、K=36 N/m` 回到**当时的名义轨迹**，轨迹时钟不因扰动暂停。

先用 `--monitor` 确认基坐标系各方向力的符号：

```bash
make build/ft_admittance_y_trajectory
./build/ft_admittance_y_trajectory 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --monitor
```

正向 `+X/+Y/+Z` 推力读数均为正时传 `1 1 1`；对应方向为负则传 `-1`。开始前确认反复运行的 `+Y 200 mm` 往返路径有足够空间，再用较低合力阈值轻力验证：

```bash
./build/ft_admittance_y_trajectory 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  output/ft20_run02_model.txt --run 1 1 1 --force-limit-n 20
```

程序保持姿态不变，不设置额外 TCP 行程边界；运行前确认控制器三维安全工作空间和线缆范围覆盖完整路径及扰动区域。力超过阈值、关节安全范围、传感器/机器人反馈超时或控制器告警时会停止。往返循环不会在完成一轮后自动退出。

## 手动拖动并记录姿态

`ft_record_poses` 只连接机器人并读取监控数据，不申请控制权、不切换模式，也不发送运动指令。按机器人和示教器的规定操作手动引导功能；程序不会替操作人员开启拖动模式。

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
make
./build/ft_record_poses 192.168.123.156 data/taught_poses.csv
```

也可以在 VS Code 的“运行和调试”中选择 **`FT: 手动拖动记录机械臂姿态`** 后按 F5。将焦点放在 VS Code **终端**，以便接收按键；选择的输出文件必须不存在。默认文件已存在时，在 `.vscode/launch.json` 中给该配置换一个文件名，避免覆盖先前记录。

1. 先手动将机械臂放到**预期自动复现时的起始姿态**。保持不动，按一次空格，程序检查约 0.5 秒内六轴关节是否静止后保存第 1 个姿态。
2. 人手拖动到下一个姿态，等待稳定，再按空格。程序每次保存一行并立即写盘。若采集期间关节变化超过 0.2°、监控数据过期或与已有姿态每轴差值均小于 0.1°，不会保存该次姿态，屏幕会提示原因。
3. 按上一步继续记录；计划使用本工程的自动标定时请记录**恰好 20 组**。完成后按回车退出；Ctrl+C 也可以结束，已保存的行会保留。

CSV 保存序号、时间、机器人 IP、J1～J6 实际关节角（度）、法兰实际位置 XYZ（mm）和法兰姿态 ABC（ZYZ 欧拉角，度）。它**不采集力传感器数据**；后续自动复现到每一姿态后才会读取六维力。退出时打印已记录关节的实际范围、相对第 1 点最大偏移及相邻点最大变化；这些只是观察值，不能作为无碰撞证明。

## 单次关节运动测试

`ft_movej_test` 是独立于自动标定流程的测试程序。它读取当前六轴关节角，检查目标关节限位并显示角度变化；按回车后申请普通控制权，需要时最多尝试一次伺服上电，然后切换到 `AUTONOMOUS` 并请求 `EVENT_MOVE`。只发送一条同步 `MoveJ`，速度固定为 2°/s、加速度 4°/s²，不采集传感器数据，也不执行 20 组姿态。

参数是目标 **J1～J6 关节角度（度）**，不是 XYZ/RX/RY/RZ 笛卡尔坐标。先编译并查看用法：

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
make build/ft_movej_test
./build/ft_movej_test --help
```

例如，传入六个目标角度；可省略 IP（默认 `192.168.123.156`）：

```bash
sg dialout -c './build/ft_movej_test 89.2 -1.0 86.1 -4.0 99.5 0.0'
```

该示例接近此前读到的姿态，实际运行前仍要检查程序打印的当前角度、目标角度及每段路径。只有控制器接受自动模式和运动安全事件，并报告 `SW_RUN` 或 `CW_RUN`，程序才会发送运动；否则会退出。急停或保护停机不会自动复位。`ft_movej_test_debug` 可用 `make build/ft_movej_test_debug` 编译，适合用 GDB 单步调试程序逻辑；真实机械臂仍会响应运动命令。

### 对照 Python 封装的 MANUAL + MoveJX 测试

`doosan_movejx_python_match` 对照原 Python `cd.tmain(50)` 与 `cd.movejx(x, 30, 20, 2)` 的控制流程：按一次 Enter 后申请控制权；若控制器仍为 `SAFE_OFF`，发送一次 `CONTROL_SERVO_ON` 并等待 `STANDBY`；随后设置 REAL / MANUAL，再按 Python 顺序发送伺服开启、全局运行速度覆盖 50%，并直接发送一次绝对 `movejx`。位姿查询使用与 `cdrflex.get_current_posx()` 相同的同步 DRFL API，不依赖监控数据回调缓存。目标格式为 `[X,Y,Z,Rx,Ry,Rz]`（mm、度），MoveJX 速度 30、加速度 20、构型 2。程序不切到 AUTONOMOUS、不调用 `set_safety_mode`、不改控制箱 DO10，也不自动恢复 `SAFE_STOP`、`SAFE_STOP2` 或 `SAFE_OFF2` 状态；若伺服准备后仍报告 `VD_STO`，不会发送运动指令。

编译并先只检查初始化、不发送 MoveJX（仍会申请控制权、设置 REAL / MANUAL 并进行伺服准备）：

```bash
make build/doosan_movejx_python_match
sg dialout -c './build/doosan_movejx_python_match --init-only'
```

完整 MoveJX 使用指定的笛卡尔任务位姿 `[X,Y,Z,Rx,Ry,Rz]`：

```bash
sg dialout -c './build/doosan_movejx_python_match 0 360 430 90 -180 90'
```

程序先显示控制器与 DRFL 库版本、初始状态、目标和参数。`FORCE_REQUEST` 可能从其他客户端接管控制权；检查当前路径、工具/负载、线缆和工作区，确认现场监护与独立急停可用后按回车。之后程序自动执行 Python 对应的控制权、伺服、REAL / MANUAL、50% 全局速度倍率流程；初始化完成后再调用 `get_current_posx()` 读取并显示当前末端位姿，然后发送一条 MoveJX，不再要求输入 `SERVO_ON` 或 `MOVEJX`。`VD_STO` 若未随伺服准备解除，程序会退出且不发送 MoveJX；安全停止和恢复状态仍需按现场安全流程处理。MoveJX 在关节空间插补，不保证末端走直线。`--init-only` 会执行初始化但不发送运动。Python 封装中的控制箱 DO10 操作仍然省略，因为该输出连接用途尚未确认。

### 自动设置模式的一次 MoveJ 测试

`doosan_movej_autoinit` 单独测试控制权、伺服准备、自动模式设置和一次关节位置运动。正常 MoveJ 流程要求连接真实机器人且控制器初始为 `STANDBY`，默认申请普通控制权；按 Enter 确认后，如果移交后状态仍为 `STANDBY`，程序直接继续；如果变为 `SAFE_OFF`，仅当安全状态为 `JH_SOS`、本次没有新报警、控制权仍有效且现场人员已在示教器确认无活动报警时，才发送一次 `CONTROL_SERVO_ON` 并最多等待 15 秒确认 `STANDBY`。不会重试，也不会对 `VD_STO`、`VD_SOS`、急停、保护停止或其他未知状态伺服上电。随后程序请求 `AUTONOMOUS`、`SAFETY_MODE_EVENT_MOVE`，只有 SDK 接受设置且监控状态报告 `SW_RUN` 或 `CW_RUN` 才发送同步 `MoveJ`。目标为绝对 J1～J6 角度，不再设置人为的每轴变化角度上限，但仍检查控制器普通与减速模式下的关节范围。MoveJ 命令参数为 `2°/s`、`4°/s²`，并另行设置控制器全局运行速度覆盖为 `50%`。程序使用带 250 ms 新鲜度检查的监控关节反馈；运动结束后会核对反馈是否到达目标。监控状态变化会带时间打印，连接及控制权移交后会输出模式、安全状态、控制权及 SDK 报警详情。

编译：

```bash
make build/doosan_movej_autoinit
```

若要先复现并记录控制权移交后的状态变化，不发送伺服、模式、速度或运动指令，可在六轴目标参数后加 `--diagnose-only`。机器人初始状态不是 `STANDBY` 时，它只打印诊断并正常退出，不申请控制权；初始为 `STANDBY` 时才会申请控制权，以便记录移交后的状态。示教器若出现移交请求，按现场流程处理。

```bash
sg dialout -c './build/doosan_movej_autoinit 89.2 -1.0 88.0 -2.0 94.5 56.0 --diagnose-only'
```

若要只测试一次伺服准备、不发送 MoveJ，运行 `--servo-init-only`。仅在状态为 `SAFE_OFF` 且安全状态为 `JH_SOS`、没有本次会话报警回调，并由现场人员检查示教器确认无活动报警后，才会在第二次明确输入 `SERVO_ON` 后发送一次 `CONTROL_SERVO_ON`，最多等待 15 秒进入 `STANDBY`，不会重试。若安全状态为 `VD_STO`、`VD_SOS` 或其他状态，则拒绝请求；须先在示教器处理原因。若已是 `STANDBY`，不会发送伺服命令。正常 MoveJ 流程则使用最初的路径/现场确认作为许可，在上述严格状态条件满足时自动尝试一次伺服准备。

```bash
sg dialout -c './build/doosan_movej_autoinit --servo-init-only'
```

正常单次运动命令（仅在路径已现场复核后使用）：

```bash
sg dialout -c './build/doosan_movej_autoinit 89.2 -1.0 88.0 -2.0 94.5 56.0'
```

程序会先显示实时关节角、目标、差值和运行参数，再等待回车；上面的角度只是示例，每次都要按实时起点重新核对。回车前请审核完整路径、负载、末端设备和线缆，并确保现场监护及独立急停可用。目标在控制器关节范围内不等于路径无碰撞。50% 是控制器的全局速度覆盖，不是绝对速度保证；它与 MoveJ 命令速度、加速度及控制器安全配置共同决定实际运动。`--force-access` 可显式选择一次 `FORCE_REQUEST`，但它可能从其他客户端接管控制权；仅在确认没有其他控制客户端且当前工作站独占控制时使用：

```bash
sg dialout -c './build/doosan_movej_autoinit 89.2 -1.0 88.0 -2.0 94.5 56.0 --force-access'
```

调试版可用 `make build/doosan_movej_autoinit_debug` 编译。不要在 gdb 中直接放行实际运动，除非现场已再次确认路径与急停措施。

下面的 `plan-recorded` 小幅计划要求相对第 1 点的配置偏移和相邻点变化最多 5°；直接回放 `run-recorded` 使用下节说明的 360° 单段上限。即使端点都由手动拖动记录，后续 MoveJ 在两点间采用的路径也不保证与手动拖动路径一致，必须单独审核每段路径。

要把记录用于后续自动标定，先复制 `config/recorded_settings.template` 为 `config/recorded_settings.txt`，由现场人员填写确认过的六轴限位、每轴相对第 1 点允许偏移、速度、加速度和等待时间，然后运行：

```bash
./build/ft_auto plan-recorded config/recorded_settings.txt \
  data/taught_poses.csv data/auto_plan.txt
```

程序按 CSV 原顺序生成 20 个目标，并将第 4、8、12、16、20 点留作验证。计划生成时只读取机械臂，并检查数据格式、机器人 IP、配置范围和控制器关节限制；需要机器人已处于 `STANDBY`。执行前必须安全回到**第 1 个记录姿态**并按下节逐段审核；程序不会自动回到第 1 点。

## 自动运动标定：必须先审核路径

### 直接执行记录的姿态

`run-recorded` 读取 `data/taught_poses.csv` 的 20 组关节角，不需要计划文件或校验码。确认路径后按回车，程序申请控制权，然后按当前姿态→关节角 `90,0,90,0,90,90`→20 组记录关节角→相同 Home 的顺序执行并计算标定。使用异步关节运动 `amovej`，指令速度 100°/s、加速度 200°/s²，运行速度覆盖为 50%；50% 是相对指令速度的倍率，名义速度上限约 50°/s，实际速度可能受控制器限制。取得控制权并确认伺服就绪后，程序请求 `ROBOT_MODE_AUTONOMOUS`，再请求 `SAFETY_MODE_AUTONOMOUS / SAFETY_MODE_EVENT_MOVE`；只有收到控制器成功响应，且安全状态回报为 `SW_RUN` 或 `CW_RUN`，才会发送运动。运动前会打印运行模式、机器人状态、安全状态、控制权和关节角。若模式切换失败或安全状态未进入允许运动状态，程序退出而不发送运动指令。Python 示例中的笛卡尔目标是另一种位姿表示，与这里的 Home 关节角并不等同。

程序在运动前检查当前姿态→Home、各采集点以及返回 Home 的单轴变化是否超过用户指定的统一上限 360°，并显示其中最大的一段；360° 不是碰撞检测。每段异步运动需要观察到运动开始，确认控制器空闲且目标附近的连续反馈稳定后，再等待 3 秒进行采样。采样时也持续检查控制权、机器人状态和目标位置；若发现机器人正在运动或状态无法确认，会尝试请求 QUICK 停止。审核与启动期间如果起点变化超过 0.2°，程序退出。

普通 `SAFE_OFF` 时，请先在示教器确认没有急停或保护报警、可正常上伺服。按程序提示回车并获得普通控制权后，程序最多发送一次 `CONTROL_SERVO_ON`，并等待 `STANDBY` 后继续。若控制权移交后才变为 `SAFE_OFF`，程序会要求重新确认；收到新报警、控制权丢失、请求失败或等待超时则停止，不会自动重试。伺服就绪后程序切换到 `AUTONOMOUS` 并请求 `EVENT_MOVE`；若控制器不接受，或回报状态不是 `SW_RUN/CW_RUN`，程序不会运动。`SAFE_OFF2`、急停及保护停止不会自动恢复。

之前的运行记录出现过报警 `7056`，同一段回调中记录到安全状态 `JH_SOS`。本次改动将 `AUTONOMOUS / EVENT_MOVE` 配置放在首条运动之前，并要求控制器报告 `SW_RUN` 或 `CW_RUN`。这按报警分析处理了一个可疑的运行模式/安全事件不匹配，但还没有在真实机械臂上验证，也不代表已确认或消除了报警根因；如果示教器仍显示报警，或状态切换不成功，应停止并根据示教器报警记录处理，不能通过屏蔽安全监控来继续运行。

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
./build/ft_auto run-recorded data/taught_poses.csv data/ft20_run02.csv output/ft20_run02_model.txt
```

先前的 `data/ft20_run01.csv.status` 为 `INCOMPLETE`（0/20 点），已有的 `data/ft20_run01.csv` 会被保留；重跑时使用上面的 `run02` 新文件名。

上述命令假定传感器坐标轴与法兰坐标轴对齐。若实际安装有旋转，用 `--mount-q qx qy qz qw` 指定**传感器坐标系到法兰坐标系**的单位四元数，例如 `--mount-q 0 0 0 1` 为对齐。这个安装关系会写入采样与模型的 `.meta`，后续实时补偿须使用同一参数；安装关系错误会影响重力补偿。

标定完成后，可在机械臂静止且无接触时用下列命令查看实时原始值和补偿值；若标定时指定了其他 `--mount-q`，这里也要传入相同的四元数：

```bash
./build/ft_acquire monitor output/ft20_run02_model.txt 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  0 0 0 1 --assume-xjc-format
```

该命令会在回车后申请控制权，出现请求时需在示教器同意。当前 `data/taught_poses.csv` 已有 20 组记录；程序先运动到 `90,0,90,0,90,90`，随后依次运动到第 1～20 组关节角，每点静止等待 3 秒并采集 30 次六维力读数的均值。第 4、8、12、16、20 组留作独立验证，其余 15 组用于拟合零偏、负载质量和重心。最后返回初始关节角，输出原始均值 CSV、模型文本和补偿 CSV。补偿值等于原始六维读数减去该姿态下预测的恒定零偏和负载重力。补偿文件名由模型路径加 `.compensated.csv` 得到，例如 `output/ft20_run02_model.txt.compensated.csv`。

CSV、模型、补偿文件及相应 `.meta`、CSV 的 `.status` 必须是不存在的新文件；重复采集时换新文件名。`.status` 中 `IN_PROGRESS` 表示正在采集，`FITTING` 表示采集和返回流程结束、正在计算，`INCOMPLETE` 表示异常退出并尽可能记录错误原因，`COMPLETE` 表示采集、拟合、验证报告和补偿文件已生成；`COMPLETE` 不代表残差已经达到现场验收标准。`ft_calibration calibrate` 会拒绝状态未就绪或实际行数与状态不符的自动采集文件。进程被强制终止时可能停留在 `IN_PROGRESS` 或 `FITTING`，应先核查数据和现场状态。VS Code 也可选择 **FT: 直接执行记录姿态并标定**。下面保留基于计划文件的其他用法供参考。

`ft_auto` 可使用上节的手动记录姿态，也可通过 `plan` 围绕启动时的实际关节角生成仅改变 J4、J5 的 20 个候选姿态。审核后程序按顺序移动、等待稳定、采集并拟合。其中 15 组是训练数据，5 组是未参与拟合的验证数据。`plan-home` 生成的计划在完成采集后返回固定初始姿态；其他计划停在最后一个姿态。关节的小幅变化可能仍导致末端或线缆发生较大位移，也可能不足以提供可靠的重力方向变化。

**自动运动前必须由现场人员确认实际安装、负载、线缆、起始点、所有目标及每一段完整路径。** 控制器关节限位、程序的角度与速度上限都不能保证无碰撞；程序在运动阶段不使用外部力传感器进行碰撞检测，现场要依靠机械臂的安全功能并保证独立急停装置可用。`Ctrl+C` 时程序会尝试通过 SDK 请求 QUICK 停止，但网络断开、控制权丢失或进程异常时不保证停止成功，它不能替代急停装置。

### 20 点计划

#### 固定初始姿态流程

`plan-home` 只读记录当前关节角，新生成的计划使用固定初始姿态 `90,0,90,0,90,90`。执行时先从计划记录的实际起点到初始姿态，再按顺序采集 20 点，最后从第 20 点返回初始姿态，再计算标定。任何运动、采集或返回异常时立即中止，不会自动继续返回或拟合；已经保存的采样文件保留。速度为 2°/s、加速度为 4°/s²。

上次发生 `7056` 安全运行停止监控违规，原因尚未查明。排查处理前不要启动真实运动。固定初始姿态不能保证从任意姿态到达的路径安全；在现场逐段审核当前起点→初始姿态、初始姿态→第 1 点、所有采集点和第 20 点→初始姿态。已生成 `data/auto_plan_home_v1.txt`，校验码 `2a7872188878da1d`，初始姿态→第 1 点的 J6 变化约 +88.79°，第 20 点→初始姿态的 J6 变化约 −61.51°。

机械臂起点发生变化时，以只读方式重新生成新计划并审核路径，不能删除起点检查：

```bash
./build/ft_auto plan-home data/taught_poses.csv data/新的计划文件.txt
```

确认 7056 问题已处理且计划全部路径已审核后，用新计划路径及其输出的校验码执行。采样 CSV、标定结果和对应 `.meta` 文件须使用不存在的新文件名。

现场已确认记录的 20 点顺序路径，速度为 2°/s、加速度为 4°/s²。原计划运行中曾意外停止，原因尚未查明；起点也发生了变化。新生成的 `data/auto_plan_manual_v2.txt` 以生成时的实际关节角为起点，依次到达记录的 20 个目标，新起点到第 1 点的路径须重新审核。第 14→15 点的 J6 变化约 83.09°；此计划使用单独的大范围计划格式，不受下文普通小幅计划的 5° 限制。程序中根据这批端点加 0.5° 得到的关节范围只是异常监测范围，不能证明路径无碰撞。姿态、工具、线缆、环境或计划发生变化时，应重新生成计划并重新确认路径。

先确认示教器无急停或保护报警、独立急停可用且现场有人监护。运动程序要求 `REAL` 系统，并会在确认、控制权移交和伺服就绪后请求 `AUTONOMOUS` 与 `EVENT_MOVE`。这会改变控制器的运行模式；示教器或安全互锁可能拒绝切换，发生拒绝时程序中止，不发运动。出现过意外停机时，应先查示教器报警记录并按现场安全流程处理。下面是旧计划的示例，它不包含返回固定初始姿态；如果起点已变化，必须生成和审核新计划：

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
sg dialout -c './build/ft_auto run data/auto_plan_manual_v2.txt 89c2c0e60250fb8f data/auto_samples_manual.csv output/auto_calibration.txt'
```

目标 CSV、模型及其 `.meta` 文件必须不存在。按回车后，若示教器出现控制权移交请求，请在 60 秒内选择同意；拒绝或超时则退出，不发送运动命令。控制权获得及起点检查完成后才创建采样文件。每点运动后等待 3 秒，采集 30 帧均值；第 4、8、12、16、20 点留作验证。结束后机械臂停在第 20 点。按当前速度，仅按每段最大关节位移估算的运动时间下限约 7 分钟，还需计入加减速、等待和采样时间。

程序等待控制器初始化。执行运动时，普通 `SAFE_OFF` 在操作人员确认后可请求一次伺服上电；急停、保护停止或 `SAFE_OFF2` 会退出，不自动恢复。获得普通控制权并确认 `STANDBY` 后才会继续。

如果当前关节角与计划起点相差超过 0.2°，程序会拒绝运行；应重新生成计划，并重新确认新的起始路径。重新生成时为输出指定新文件名，再把新的校验码和路径同步到运行配置：

```bash
./build/ft_auto plan-confirmed data/taught_poses.csv data/new_auto_plan.txt
```

自动运动流程已用模拟设备检查；真实设备曾在前往第 1 点时意外停止，尚未完成 20 点运动。传感器通道顺序、符号、单位和原厂标定仍需核对；拟合完成后务必检查验证残差。

1. 只读检查当前状态和控制器限位：

   ```bash
   ./build/ft_auto inspect 192.168.123.156
   ```

   输出包含当前关节角以及控制器普通/减速模式关节范围。它们是控制器配置，**不是现场无碰撞范围**。计划生成阶段不发伺服或运动指令；执行阶段按上面的确认流程处理普通 `SAFE_OFF`，伺服就绪后请求自动运行模式和运动安全事件。

2. 如果采用 J4/J5 网格候选姿态，复制 `config/auto_settings.template` 为 `config/auto_settings.txt`，填入**现场审核过的**六轴绝对关节最小值、最大值，J4 和 J5 的允许偏移幅度、速度、加速度和稳定等待时间；如果采用手动记录姿态，请使用上节的 `recorded_settings.template` 和 `plan-recorded` 命令。模板中的 `REQUIRED` 不是有效数值，会阻止计划生成。程序只接受速度大于 0 且不超过 5°/s、加速度大于 0 且不超过 10°/s²、等待时间 2～30 秒；这些只是软件上限，不能证明安全。

3. 如果采用 J4/J5 网格，在确认的起始姿态以只读方式生成计划：

   ```bash
   ./build/ft_auto plan config/auto_settings.txt data/auto_plan.txt
   ```

   对于手动记录姿态，使用上节的 `plan-recorded` 命令生成计划。无论采用哪种方式，`data/auto_plan.txt` 必须是新文件。由现场人员审核 **从起始姿态到第 1 点及后续每一段路径**，包括机械臂本体、传感器、工具、线缆与周围设备。校验码只用于发现计划内容变化，不具备安全认证功能。现场或设置变化后，应生成新计划并重新审核。

4. 审核通过后，在集成终端运行下面命令，将 `审核后的校验码` 替换为计划输出的 16 位码；输出文件必须不存在：

   ```bash
   ./build/ft_auto run data/auto_plan.txt 审核后的校验码 \
     data/auto_samples.csv output/auto_calibration.txt
   ```

   程序会再次显示计划、检查当前角度是否与计划起点相差不超过 0.2°、读取传感器并等待回车。**回车后会申请普通控制权并开始运动**；拒绝/超时/状态异常则退出。程序不强制抢占控制权。每次运动后等待设定时间，采集 30 帧均值，再执行下一点。

   也可在 VS Code 中选择 **`FT: 执行已审核的自动标定计划`**，输入审核后的校验码并在终端按回车。默认路径是上述计划、采样和模型文件；需要新文件名时修改 `.vscode/launch.json` 对应配置。

程序会检查计划与校验码、现场设定限位、控制器普通及减速模式关节范围和速度限制、起点、控制权、监控数据新鲜度、机器人状态，以及自动运行模式和安全运行状态。命令失败、超时、监控异常或中断时停止后续采样及拟合；若运动可能正在进行，会尽力请求 QUICK 停止，并记录连接期间接收到的状态、安全状态和报警回调。软件检查**不包含空间碰撞检测**。本次模式和安全状态门槛尚未在真实运动中验证；首次运行需要现场人员逐段确认路径并监护。

20 组采集成功后程序自动调用拟合并打印独立验证残差。拟合失败会保留已采集 CSV；模型文件生成不代表精度合格，须核对负载质量、质心和独立验证残差。采集前后不得改变传感器清零、负载或安装；数据轴顺序和单位仍需与厂家资料核对。

## 2. 坐标、单位与连接

采集使用 DRFL 监控数据中的法兰实际姿态，按 ZYZ 欧拉角（度）转换为 `q_BF`。安装旋转 `q_FS` 表示**传感器 S 到法兰 F**，最终 `q_BS = q_BF * q_FS`。本次确认传感器 XYZ 与法兰 XYZ 同向，所以 `q_FS = (0,0,0,1)`，顺序为 `qx qy qz qw`。如果更换安装方式，需要重新设置并重新采样。

启动采集时，程序打印传感器 X、Y、Z 轴在基座坐标系中的方向。你提供的 `-X、Y、-Z` 是当前某个姿态下的方向；在该姿态核对输出应分别接近 `[-1,0,0]`、`[0,1,0]`、`[0,0,-1]`。机械臂运动时，相对基座的方向会变化。若方向不符，先核对示教器、法兰轴和传感器轴，不要继续标定。

力和力矩需要分别为传感器坐标系中的 **N**、传感器测量原点的 **N·m**。模型假设基座 Z 轴向上、重力 `[0,0,-9.80665] m/s²`。如果机器人基座不水平、传感器单位或力矩参考点不同，须先正确转换；当前程序不会自动修正这些情况。

机械臂 IP 曾设置为 `192.168.123.156`，电脑有线地址曾为 `192.168.123.94/24`。连接前用以下命令确认路由实际走机械臂网线，避免连接到同网段的其他网络：

```bash
ip -br -4 address
ip route get 192.168.123.156
```

当前已识别的 USB 转 RS485 设备稳定路径为 `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0`。通信采用 115200 8N1、Modbus RTU，只发送读取请求。`Permission denied` 时检查用户是否在 `dialout` 组；加入后须注销桌面并重新登录才能让新权限生效。关闭其他占用串口的软件。

## 3. 采集真实数据

确认机械臂静止、无接触、传感器供电且接线和方向已核实后：

```bash
cd /home/hzy/Code/Doosan/doosan_ft_calibration_cpp
./build/ft_acquire record data/real_samples.csv 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  0 0 0 1 --assume-xjc-format
```

程序进入交互提示后：

1. 用示教器将机器人调整到一个姿态，停止并等待振动衰减，保持无接触。
2. 在终端输入 `train` 并回车。程序按 100 ms 间隔读取 30 帧，检查监控回调期间关节及姿态变化是否超过 0.2°；通过后保存一行均值，打印六个通道的均值和标准差。该阈值只是辅助检查，不能代替确认静止、无接触、无饱和及线缆无干扰。
3. 改变法兰的倾斜方向，重复采集至少 10 个 `train` 姿态，建议 15–30 个。只绕重力方向旋转不能提供足够的姿态变化。
4. 再采集至少约 5 个没有参与拟合的不同姿态，输入 `valid` 保存。输入 `quit` 退出，或按 Ctrl+C。

程序以主机接收时间配对传感器与机械臂数据，要求估算的接收时间差不超过 250 ms；它不是硬件同步。只应在静止状态下采集，不能因此推断运动过程中的同步精度。串口回复错误、断线、姿态过期或检测到运动时该姿态不会被接受。

采集会新建 CSV 和同名 `.meta` 文件。继续追加时，程序检查串口路径、安装四元数和协议设置；两文件需要一起保留。此检查不能发现传感器被重新清零、更换负载、单位错误或实际安装方向变化。**整个采样及使用期间不要更改软件清零状态**，更换负载或安装后重新标定。

VS Code 中选择“运行和调试”里的 **`FT: 采集六维力标定数据`**，按 F5；在集成终端输入 `train`、`valid`、`quit`。参数位于工作区 `.vscode/launch.json`；配置会生成不优化的调试程序。运行前同样需要确认网络、传感器协议及坐标方向。

## 4. 拟合、验证与离线补偿

```bash
./build/ft_calibration calibrate data/real_samples.csv output/real_calibration.txt
./build/ft_calibration validate data/real_samples.csv output/real_calibration.txt valid
./build/ft_calibration apply data/real_samples.csv output/real_calibration.txt output/real_compensated.csv
```

拟合只使用 `train` 行，输出带符号等效质量、传感器坐标系下质心、六通道恒定偏置、QR 对角比及训练和验证残差。姿态不足、设计矩阵秩不足或严重病态时会报错。保存的模型和 `output/real_calibration.txt.meta` 要一起保留。为保护已有数据，`demo`、`calibrate` 和 `apply` 的输出文件必须使用尚不存在的文件名，重复运行时请换一个名字；验证和离线补偿时若采样与模型的元数据都存在，程序会检查它们是否一致。

**检查独立 `valid` 姿态的六通道残差**，结合传感器噪声、量程和预期接触力判断是否可用；同时核对质量是否与实际下游负载相符、质心是否合理，并重复标定比较稳定性。没有 `valid` 行时程序会提示，但仍会保存模型；保存不等于通过验证。离线工具也接受手工准备的 CSV，不过缺少采集元数据的模型不能用于下面的在线监视。

CSV 严格表头：

```text
split,stamp_s,qx,qy,qz,qw,fx,fy,fz,mx,my,mz
```

`split` 是 `train`、`valid` 或 `contact`；`stamp_s` 是秒，采集程序使用 Unix 时间；四元数是 S 到基座 B 的 `qx,qy,qz,qw`。训练和验证数据必须是无接触、静止姿态的均值；`contact` 行不会参与拟合。

## 5. 查看实时补偿值

完成真实采样和独立验证后，可用同样的串口路径和安装四元数查看数据：

```bash
./build/ft_acquire monitor output/real_calibration.txt 192.168.123.156 \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00TFOX-if00-port0 \
  0 0 0 1 --assume-xjc-format
```

每行显示 `raw` 和 `compensated`，顺序都是 `Fx Fy Fz Mx My Mz`，按 Ctrl+C 停止。程序检查模型旁的采集元数据与当前串口路径、安装旋转是否一致。它不会自动验证模型质量，也无法检测软件清零或换装。

该补偿模型只包括常值偏置与静态重力。运动时的惯性、线缆力、温漂、饱和及真实时间同步误差不会被消除；因此在线显示不能直接当作接触力闭环控制输入。显示间隔为软件轮询间隔，不是硬实时控制周期。

## 6. 模型说明与模拟数据

```text
g_S     = R_BS^T * [0,0,-9.80665]
F_raw   = m_signed * g_S + bias_F + F_external
Tau_raw = h_signed × g_S + bias_T + Tau_external
h_signed = m_signed * r_COM
```

联合拟合 10 个未知数（带符号质量、三个质量一阶矩和六个偏置），用列主元 Householder QR 求解；程序输出秩、QR 对角比和未缩放设计矩阵的 2 范数条件数。条件数受参数单位影响，不能单独作为标定验收依据。质量符号取决于传感器力方向约定。多姿态恒定偏置估计与传感器厂家原厂标定及单姿态软件清零是不同操作，不能替代它们。

无需连接设备即可验证离线流程（使用新文件名，避免覆盖真实数据）：

```bash
./build/ft_calibration demo /tmp/ft_synthetic.csv
./build/ft_calibration calibrate /tmp/ft_synthetic.csv /tmp/ft_synthetic_model.txt
./build/ft_calibration validate /tmp/ft_synthetic.csv /tmp/ft_synthetic_model.txt valid
./build/ft_compensate_example /tmp/ft_synthetic_model.txt
```

`include/ft_compensator.hpp` 可供其他 C++ 程序使用，输入为同步的传感器坐标系六维读数和 `q_BS`。调用者负责核对实际单位、时间、姿态、模型适用条件及异常状态。
