# Quest 3 AR → UR5 Teleoperation

使用 Quest 3 透视画面和右手柄遥操作 UR5，并在头显中显示实测关节状态。Unity 应用、电脑控制程序、协议和构建入口均在本仓库内；无需其他训练或采集仓库、ROS、模型权重或额外编译通信桥。

```text
Quest 3 APK ── UDP :8080 ──→ 电脑 Python 控制程序 ── RTDE ──→ UR5
            ←─ UDP :8081 ── 实测关节状态                ←──────
```

电脑需要同时访问 Quest 和 UR5，推荐电脑通过有线网连接 UR5、Quest 通过局域网 Wi-Fi 访问电脑。电脑端程序需要在操作期间持续运行。

## 环境要求

**仓库根目录是团结工程。Windows、macOS、Linux 的标准 Unity 用户需要先导出标准 Unity 工程，再打开导出目录。** 导出工具不需要安装团结，也不修改源工程。

| 环境 | 编辑器版本 | 打开的目录 |
| --- | --- | --- |
| macOS / 团结 | `2022.3.62t11 / 1.9.3` | 仓库根目录 |
| macOS / 标准 Unity | `2022.3.62f1` | 导出的 `quest-ur5-unity` |
| Windows / 标准 Unity | `2022.3.62f1` | 导出的 `quest-ur5-unity` |
| Linux / 标准 Unity | `2022.3.62f1` | 导出的 `quest-ur5-unity` |

请使用该 Unity 版本支持的操作系统环境。编辑器安装、导出和构建步骤见 [跨平台构建说明](docs/building.md)。

- 头显：Quest 3、Touch Plus 手柄，开启开发者模式。
- 电脑：Python ≥ 3.10，控制程序建议使用 **CPython 3.11**。真机依赖固定为 `ur-rtde==1.6.2`。
- 真机：可通过网络访问的 UR5；Robotiq 夹爪可选。先使用虚拟后端检查通信，再接入真机。

## 1. 构建 Quest 应用

### 现有团结工程

用团结 `2022.3.62t11` 打开仓库根目录，等待包恢复，打开 `Assets/Scenes/SampleScene.scene`。选择 **Quest UR5 → Validate Project**，通过后选择 **Quest UR5 → Build Quest APK**。

### 标准 Unity（macOS / Linux / Windows）

先安装 Python 3.10 或以上版本（电脑控制程序建议使用 3.11）。在仓库根目录运行；输出目录必须不存在且位于仓库外。

macOS / Linux：

```bash
python3 tools/prepare_unity_project.py --output ../quest-ur5-unity
```

Windows PowerShell（已安装 Python 3.11）：

```powershell
py -3.11 tools/prepare_unity_project.py --output ../quest-ur5-unity
```

在 Unity Hub 中安装 `2022.3.62f1` 和 **Android Build Support、Android SDK & NDK Tools、OpenJDK**。在 Projects 中选择从磁盘添加项目，选择导出的 `quest-ur5-unity` 整个目录，等待包恢复。脚本转换团结的场景扩展名和序列化标签，使用固定的标准 Unity 包清单，保留资源 GUID 和主场景引用。不要直接用标准 Unity 打开未经转换的团结根工程。

首次恢复包需要访问 Unity 和 Meta 包注册表。导出目录只包含 Unity 构建工程；电脑控制程序仍从原仓库的 `robot/` 安装和运行。后续 Unity 源码修改应在原仓库进行，再导出到新的目录。

包恢复完成、Console 无编译错误后，打开 `Assets/Scenes/SampleScene.unity`，将构建平台设为 **Android**。选择 **Quest UR5 → Validate Project**，确认主场景脚本和 Android OpenXR 配置有效，再选择 **Quest UR5 → Build Quest APK**。构建入口会自动执行相同检查。首次导入后按 [跨平台构建说明](docs/building.md) 核对配置。

### 安装 APK

APK 输出到**当前打开的 Unity 工程**的 `Builds/quest-ur5.apk`，应用名称为 **Quest UR5 AR**，包名为 `com.questur5.teleop`。通过 USB 连接 Quest，在头显中允许 USB 调试，再使用 Android SDK 的 `adb` 检查设备：

```bash
adb devices
```

根据采用的工程执行对应安装命令。下列路径以**原仓库根目录**为工作目录，并假设使用上述默认导出目录：

| APK 来源 | 安装命令 |
| --- | --- |
| 团结原工程 | `adb install -r Builds/quest-ur5.apk` |
| 标准 Unity 导出工程 | `adb install -r ../quest-ur5-unity/Builds/quest-ur5.apk` |

构建机和运行控制程序的电脑可以不同；后续修改网络 IP 不需要重建 APK。

## 2. 安装电脑控制程序

以下命令从本仓库根目录执行。

macOS / Linux（已安装 Python 3.11）：

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e ./robot
```

Windows PowerShell（不依赖虚拟环境激活脚本）：

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ./robot
```

下文的 `python` 在 Windows 下可替换为 `.\.venv\Scripts\python.exe`。

## 3. 检查 Quest → 电脑通信

macOS / Linux（已激活上述虚拟环境）：

```bash
python -m quest_ur5 --backend virtual
```

Windows PowerShell：

```powershell
.\.venv\Scripts\python.exe -m quest_ur5 --backend virtual
```

默认后端不连接真实机器人。保持电脑程序运行，启动头显中的 **Quest UR5 AR**。局域网应允许电脑接收 UDP `8080`、Quest 接收 UDP `8081`；详细网络要求见 [运行与标定](docs/operation.md)。

戴上 Quest，在启动面板填写**运行该命令的电脑局域网 IPv4 地址**，默认端口 `8080`：

- 左 **X**：编辑 IP；右摇杆左右选择数字、上下修改。
- 右 **B**：保存并关闭面板；下次启动按左 **Y** 使用保存地址。
- 左菜单键：打开/关闭面板。面板显示期间会禁止运动输入。

先松开右 Grip，再按住并移动手柄。终端显示 `ready`、`moving`，松开后回到 `ready`。`logs/` 中的 JSONL 记录 TCP 目标与停止原因。

虚拟后端仅验证 TCP 控制与通信，没有关节 IK，**不会伪造实测关节回传，头显中的机械臂模型保持隐藏**。查看终端与日志判断收包。真实 UR5 反馈到达后才显示模型，反馈超时再次隐藏。

## 4. 连接真实 UR5

macOS / Linux：安装 RTDE 依赖并复制配置。

```bash
python -m pip install -e "./robot[robot]"
cp robot/config.example.json robot/config.local.json
```

Windows PowerShell：

```powershell
.\.venv\Scripts\python.exe -m pip install -e "./robot[robot]"
Copy-Item robot/config.example.json robot/config.local.json
```

配置文件首次创建时复制一次；已有本地配置时直接编辑。

编辑 `robot/config.local.json`：

- `robot_ip`：UR5 控制器地址；`quest_ip`：头显 Wi-Fi 地址。
- `workspace_min/max`：按实际环境重新测定的基座坐标 TCP 边界，单位米。示例不是已标定安全区域。
- `input_yaw_degrees`：佩戴者朝向与基座的航向偏差。默认 Quest 前方 → 机器人 +X、右方 → −Y、上方 → +Z。
- 使用 Robotiq URCap TCP 服务时设 `gripper` 为 `robotiq`；否则保持 `disabled`。

示教器上确认机器人已上电、解除保护停止并允许远程控制，TCP 和负载符合夹具，工作区无障碍且急停可触达。RTDE 自动上传控制脚本，不需要构建外部桥。网络与夹爪要求见 [运行与标定](docs/operation.md)。

macOS / Linux：

```bash
python -m quest_ur5 --config robot/config.local.json --check-config
python -m quest_ur5 --config robot/config.local.json --backend rtde --confirm-real-robot
```

Windows PowerShell：

```powershell
.\.venv\Scripts\python.exe -m quest_ur5 --config robot/config.local.json --check-config
.\.venv\Scripts\python.exe -m quest_ur5 --config robot/config.local.json --backend rtde --confirm-real-robot
```

先以默认低速验证各方向，再扩大范围。启动、程序重启、追踪丢失、超时或故障后，都需要**松开 Grip 再重新按下**。`Ctrl+C` 停止控制并释放连接。

## 手柄操作

| 右手柄输入 | 行为 |
| --- | --- |
| 按住 Grip | 相对平移 TCP，锁定姿态 |
| Grip + A | 固定 TCP 位置，保持向下抓取参考，仅调整水平航向 |
| Grip + B | 固定 TCP 位置，自由调整相对姿态 |
| Trigger（同时按 Grip） | Robotiq 开/合，默认阈值 0.5；需要启用夹爪后端 |
| 松开 Grip | 停止机械臂和夹爪，下一次从实测 TCP 重新锚定 |

## 结构与维护

```text
Assets/             Quest 输入、AR 显示、反馈模型、场景和构建入口
Packages/           团结开发包配置与内嵌 URDF Importer
ProjectSettings/    当前开发工程设置
robot/              独立 Python 控制程序、配置示例和测试
tools/              标准 Unity 导出工具和固定包清单
docs/               构建、标定、协议与 development.md 开发说明
third_party/        复用代码的来源说明和许可证
```

日志、APK、缓存、虚拟环境、私有网络配置和签名密钥均不应提交。源码与 `.meta` 必须一起维护；保留内嵌 URDF Importer。

## 测试与文档

已在开发 Mac 上通过 **52 项 Python 控制程序测试、18 项导出工具测试、113 项 Unity EditMode 测试**，以及本机 UDP 虚拟后端检查、标准 Unity 源工程导出检查和团结 Android ARM64 / IL2CPP APK 构建。

[自动化工作流](.github/workflows/host-tests.yml) 配置了 Windows、macOS、Linux 的 Python 测试与源工程导出检查，不执行 Unity APK 构建。

安装开发依赖后可运行以下检查；Windows 使用上述虚拟环境的 Python 可执行文件替代 `python`：

```bash
python -m pip install -e "./robot[dev]"
python -m pytest robot/tests tools/tests
python -m ruff check robot
python -m ruff check --config robot/pyproject.toml tools
```

- [跨平台构建](docs/building.md)：导出、编辑器安装、配置检查和命令行构建。
- [运行与标定](docs/operation.md)：网络、机器人与夹爪配置、坐标标定、故障恢复。
- [架构与协议](docs/architecture.md)：输入、控制程序、反馈和维护边界。
- [开发与维护](docs/development.md)：资源组织、GUID 规范和本地生成文件。

`third_party/teleoperation/` 保存部分复用算法的 [来源与改动说明](third_party/teleoperation/NOTICE) 和 [Apache 2.0 许可证](third_party/teleoperation/LICENSE)，不参与程序运行。Meta XR 和 URDF Importer 遵循各自许可。
