# 跨平台构建

## 两条编辑器路径

根目录是团结 `2022.3.62t11` 工程；继续用该版本开发。

标准 Unity 用户先安装 Python 3.10 或以上版本，建议使用 3.11。在仓库根目录运行以下对应命令。

macOS / Linux：

```bash
python3 tools/prepare_unity_project.py --output ../quest-ur5-unity
```

Windows PowerShell（已安装 Python 3.11）：

```powershell
py -3.11 tools/prepare_unity_project.py --output ../quest-ur5-unity
```

脚本只复制 `Assets/`、`ProjectSettings/`、内嵌 URDF Importer，转换 `.scene` 和团结 YAML 标签，替换标准 Unity 包配置。导出时检查大小写冲突、Windows 非法/保留文件名、包相对路径、GUID 格式及重复 GUID。文本按 UTF-8 读取，避免 Windows 系统编码影响。不会复制缓存、密钥、日志、Python 虚拟环境或 `.git`；已有目标目录会拒绝覆盖。后续更新时导出到新的目录，避免覆盖其他人的编辑。

资源 `.meta` 使用标准的 32 位 GUID，克隆者无需安装团结来处理元数据。后续在团结中新增或重新导入资源，若导出提示 `nonstandard asset GUID`，开发者应在源工程执行 **Quest UR5 → Normalize Metadata for Unity Export**，将规范化后的 `.meta` 和资源一起维护；不要重新生成 GUID。该菜单也兼容标准 Unity。

导出项目仍从原仓库运行 Python 控制程序。导出目录只用于 Unity 构建，修改 Unity 源码应回到原仓库，以免下次导出丢失修改。

## 编辑器安装

Unity Hub 安装 `2022.3.62f1`，勾选 Android Build Support、SDK/NDK Tools、OpenJDK。三个桌面系统使用相同的项目文件和构建菜单，不能使用 iOS 或桌面平台替代 Android 输出。

在 Unity Hub 的 Projects 中从磁盘添加项目，选择导出的 `quest-ur5-unity` 整个目录，再使用上述版本打开。选择的是包含 `Assets/`、`Packages/`、`ProjectSettings/` 的项目目录；原仓库根目录继续由团结编辑器打开。

使用 Unity 该版本支持的 Windows、macOS 或 Linux 桌面环境；Linux 应按官方列出的发行版与图形环境部署，不能泛指所有 Linux。系统要求及 Android 模块安装参见 [Unity 2022.3 系统要求](https://docs.unity3d.com/2022.3/Documentation/Manual/system-requirements.html) 与 [Android 环境配置](https://docs.unity3d.com/2022.3/Documentation/Manual/android-sdksetup.html)。使用 Hub 随该编辑器安装的 Android 工具，不复制开发者 Mac 上的 SDK/NDK/JDK 路径。

标准包版本固定在 `tools/unity-manifest.json` 与 `tools/unity-packages-lock.json`。Meta Core 为 `69.0.0`，OpenXR 为 `1.14.3`，XRI 为 `3.2.2`；不自动升级到要求更高编辑器版本的 Meta SDK。Meta 的公共包源已随工程配置，首次恢复需要网络访问 Unity 和 Meta 注册表。

## 首次导入核对

1. 打开 `Assets/Scenes/SampleScene.unity`（团结工程为 `.scene`）。
2. 切换 Android；Project Settings → XR Plug-in Management → Android 使用 OpenXR，Initialize XR on Startup 开启。
3. OpenXR 的 Android 设置保留 Meta Quest Support、Oculus Touch Controller Profile；运行项目验证并检查错误。
4. 场景 XR Origin 的 OVRManager / OVRPassthroughLayer 存在且启用；主相机清屏 Alpha 为 0。反馈模型只有收到实测关节状态后显示。
5. Player 使用 ARM64、IL2CPP、OpenGLES3，网络权限开启。菜单构建入口会固定 ARM64、IL2CPP、最小 API 29、自动目标 API 和 Internet 权限。

包恢复完成后执行 **Quest UR5 → Validate Project**：实际加载主场景检查丢失脚本，并检查 Android OpenXR Loader 和启动初始化配置。APK 构建入口自动执行相同检查；不会自动保存或覆盖用户正在编辑的场景。若出现 Missing Script，先检查包恢复与编辑器版本，不要直接删组件。

现有机械臂网格和材质已随资源保存，打开场景与构建 APK 无需重新执行 URDF 导入。第三方 Assimp 的桌面原生库与“重新导入新的 URDF”有关；此操作不属于 APK 构建步骤，也不应作为首次打开项目的要求。

## 构建和安装

菜单：**Quest UR5 → Build Quest APK**，输出 `Builds/quest-ur5.apk`。同一份导出工程在三个系统使用该菜单；每台电脑在本机恢复包与生成 `Library/`，不要跨系统复制 Unity 缓存。

命令行使用已安装编辑器的可执行文件；设置 `UNITY_EDITOR` 为本机路径。关闭同一项目的其他编辑器实例：

```bash
"$UNITY_EDITOR" -batchmode -nographics -quit -buildTarget Android -projectPath /path/to/quest-ur5-unity \
  -executeMethod QuestUr5Build.BuildApk -logFile /path/to/unity-build.log
```

PowerShell：

```powershell
& $env:UNITY_EDITOR -batchmode -nographics -quit -buildTarget Android -projectPath C:\path\quest-ur5-unity `
  -executeMethod QuestUr5Build.BuildApk -logFile C:\path\unity-build.log
```

命令行首次导入也必须指定 `-buildTarget Android`：批处理不支持在执行方法中切换编译目标后重新加载程序集。缺少此参数且目标不是 Android 时，构建入口会给出明确错误。GUI 菜单可自动切换目标。命令行参数依据 [Unity 2022.3 构建目标切换说明](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/EditorUserBuildSettings.SwitchActiveBuildTarget.html)。

编辑器许可证和 Android 模块必须可用。本仓库不会修改系统 Android SDK 路径，也不包含个人签名密钥。开发设备可使用编辑器默认 debug 签名安装；面向公开商店发布需要另行维护自己的签名和商店配置。

```bash
adb devices
adb install -r Builds/quest-ur5.apk
```

应用名称为 **Quest UR5 AR**，Android 包名为 `com.questur5.teleop`，偏好设置使用 `QuestUr5_` 前缀。若设备安装过旧包名的应用，新版会作为独立应用安装；首次启动需重新填写电脑 IP 和端口，确认新版正常后可卸载旧应用。

## 验证记录

- 已在开发 Mac 上验证：C# 运行时/编辑器程序集编译；113 项 Unity EditMode 测试；52 项 Python 控制程序测试、18 项导出工具测试及本机 UDP；标准 Unity 导出、GUID 和固定包清单；团结 Android APK 构建成功（ARM64、IL2CPP、网络权限、应用名与包名、包完整性已检查）。

| 编辑器与系统 | 导入 / 场景检查 | Android APK 构建 |
| --- | --- | --- |
| macOS / 团结 2022.3.62t11 | 已验证 | 已验证 |
| macOS / 标准 Unity 2022.3.62f1 | 待实际编辑器验证 | 待验证 |
| Windows / 标准 Unity 2022.3.62f1 | 待实际编辑器验证 | 待验证 |
| Linux / 标准 Unity 2022.3.62f1 | 待实际编辑器验证 | 待验证 |

标准 Unity 的各系统验收都必须从全新导出工程开始：包恢复无编译错误、Validate Project 通过、Build Quest APK 成功；APK 应用名 `Quest UR5 AR`、包名 `com.questur5.teleop`、ABI `arm64-v8a`。仅导出脚本通过或 Python CI 通过不能替代 Unity 构建验收。

- 未验证：上述标准 Unity 环境、Quest 穿戴测试、真实 UR5 和 Robotiq。当前电脑仅安装团结，无法声称三个系统的标准 Unity 已全部实测通过。
- CI 配置检查 macOS、Windows、Linux 的 Python 3.10/3.11/3.12 与源工程导出；本次没有提交，因此 CI 尚未运行。CI 不包含需要许可证和设备的 Unity/UR5 测试。
