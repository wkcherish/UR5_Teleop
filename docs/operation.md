# 网络、标定与运行

## 网络和机器人

电脑与 UR5 建议有线连接，Quest 使用同一电脑可访问的 Wi-Fi。电脑可有两个网卡；填入 Quest 可达的电脑地址，不能填写 UR5 地址、`127.0.0.1` 或仅机器人网卡可见的地址。

| 方向 | 端口 | 用途 |
| --- | --- | --- |
| Quest → 电脑 | UDP 8080 | 原始右手柄输入 |
| 电脑 → Quest | UDP 8081 | 实测关节角和夹爪命令状态 |
| 电脑 → UR5 | TCP 30004 | RTDE 收发 |
| 电脑 → UR5 | TCP 30002 | ur_rtde 自动上传控制脚本 |
| 电脑 → UR5 | TCP 63352（可选） | Robotiq URCap ASCII 服务 |

根据实际运行配置调整防火墙。UDP 没有身份认证，仅在可信局域网运行，不要暴露到公网。真机必须指定 `quest_ip`，程序只接受该 IP 的数据；虚拟模式首次合法输入锁定发送 IP，切换设备需重启电脑程序。

使用正确的 PolyScope 版本和远程控制配置；不得同时运行其他运动程序、旧通信桥或 Unity 的 legacy URScript 输出。启动日志报端口占用时先查旧进程。保护停止或急停由操作者在示教器处理，程序不会自动解锁、上电、回零或移动到初始姿态。

## 坐标与工作区

控制是相对位姿：Grip 起始位置可以任意，按下时锚定当前实测 TCP。位置单位为米、四元数为 xyzw；UR5 目标姿态转换为弧度旋转向量。

默认坐标转换为 `robot_xyz = [quest_z, -quest_x, quest_y]`，再绕机器人 +Z 应用 `input_yaw_degrees`。保持固定佩戴朝向，每次修改航向后先松开 Grip，以很小位移逐轴核对。原点平移由离合锚点抵消，方向和比例仍必须验证。Grip+A 的下抓参考为机器人基座坐标系绕 X 旋转 π，仅映射绕基座竖直轴的手柄转动，忽略倾斜；Grip+B 才映射完整旋转。夹具 TCP 不同可能需要调整下抓参考。

`workspace_min/max` 表示 TCP 位置边界，不是整条机械臂、夹爪、线缆的碰撞检测。边界应考虑桌面、夹具和实际安装位置。AR 模型摆放不会改变真机坐标；本项目没有自动识别或对齐真实基座的空间标定功能，反馈姿态不能替代基座视觉配准。模型关节轴和零位通过 `Ur5RtdeJointStateReceiver.jointSigns/jointOffsetsDegrees` 校准，默认使用 URDF 原轴。

开始前在示教器上设置实际 TCP、负载、重心与 UR 安全限制。初次用默认 `0.035 m/s`、`14 deg/s`，逐轴验证映射、松手停止、Wi-Fi 断开、追踪失效后的停止和恢复离合。软件限速不是硬件安全系统，操作时保留急停和示教器安全边界。

## Robotiq

目前支持 Robotiq URCap ASCII TCP 服务的二态开合，`trigger < 0.5` 打开，`>= 0.5` 闭合。不是通用 Modbus、串口或数字 IO 驱动；只有在控制器已提供该服务时才能使用。

配置 `gripper: "robotiq"`，设置对应的端口、速度、力和可选 `gripper_slave_id`。优先在示教器上先激活夹爪。若需要程序激活，显式添加 `--activate-gripper`，激活可能移动夹爪。默认不启动夹爪硬件。

夹爪 IO 在独立线程执行，避免 socket 阻塞占用伺服循环。松开 Grip、失踪、超时或关闭程序会取消待执行开合并请求 `GTO=0`。反馈 JSON 中的 `gripper_command` 是已请求的开合输入，不是实测指距；抓取是否完成需要机械夹爪反馈和现场检查。

## 日志与排错

日志默认写入运行目录的 `logs/teleop-*.jsonl`；可用 `--log` 指定新的文件路径，不覆盖已有文件。主要字段：

- `state`：`ready`、`moving`、`input_timeout`、`invalid_tracking_or_stop`、`controller_pose_jump_or_gap`、`out_of_order`、`robot_safety_state` 等。
- `actual_tcp/target_tcp`：米与弧度旋转向量，便于比较实测和目标。
- `limits`：`linear_speed`、`angular_speed`、`workspace`、`command_lead` 等限幅或拒绝原因。
- `armed`：是否已经观察到有效 Grip 松开；为 true 也不代表当前正在运动。

完全没有 `ready`：检查电脑 IP、局域网隔离、防火墙、启动面板是否关闭、右手柄是否正常追踪。虚拟模式不显示模型是预期行为。

真机没有模型：检查电脑 UDP 8080 收包、Quest 的 UDP 8081 可达性、RTDE 状态和模型关节映射。模型会在状态陈旧时隐藏，不显示预测运动来代替实测状态。

程序重启或网络恢复后按住 Grip 不动：先松开再按下。该恢复门控防止旧目标自动重放。RTDE 连接故障会退出进程，排除问题后手动重启；不会自动恢复运动。

`ur_rtde` 无法安装：推荐 CPython 3.11，Windows/Linux x86_64 有对应发行包；macOS 可能需要按上游说明安装编译工具并从源码构建。头显 APK 可以在 Mac 构建、控制程序在 Linux/Windows 运行。不要为了安装驱动变更机器人固件。
