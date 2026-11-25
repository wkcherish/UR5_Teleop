# 开发与维护

## 当前入口

- 主场景：`Assets/Scenes/SampleScene.scene`；标准 Unity 导出后为同名 `.unity`。
- Quest 输入、网络与显示：`Assets/Scripts/UR5/`。
- 独立控制程序及测试：`robot/`。
- 编辑器检查与 APK 构建：`Assets/Editor/QuestUr5Build.cs`。
- 标准 Unity 导出与固定包版本：`tools/`。

主场景使用 UDP 独立控制模式，由电脑端处理机器人运动。Unity 中保留的本地 IK、位姿预览、CSV 诊断和对应测试用于开发调试，默认运行链及协议见 [架构说明](architecture.md)。这些组件存在代码和测试依赖，清理目录时应一并检查调用关系。

## 资源维护

`Assets/Robots/ur5/` 保留 URDF、原始网格、导入后的网格和材质。URDF 用相对文件路径引用原始网格，即使某个 STL 没有 Unity GUID 引用，也不能据此判断它无用。

`Assets/Environment/DemoObject/` 是主场景使用的示例物体。FBX 与纹理文件保留导入时的名称，以维持模型内部的纹理查找；它们不依赖模型生成插件。

移动 Unity 资源时同时移动 `.meta`，保留 GUID。新增团结资源后，若导出检查发现编码 GUID，先执行 **Quest UR5 → Normalize Metadata for Unity Export**，再按 [构建说明](building.md) 导出。

## 验证

```bash
python -m pip install -e "./robot[dev]"
python -m pytest robot/tests tools/tests
python -m ruff check robot
python -m ruff check --config robot/pyproject.toml tools
```

Unity 中执行 **Quest UR5 → Validate Project** 和 EditMode 测试；资源、包或场景调整后再构建 APK。Windows、Linux、macOS 的实际 Unity 构建验收状态见 [构建说明](building.md)。

## 本地生成文件

`Library/`、`UserSettings/`、`Logs/`、`Builds/`、IDE 工程文件和 Python 缓存由工具生成，已加入忽略规则。`ProjectSettings/PackageManagerSettings.asset` 是编辑器包管理器的界面和注册表缓存；共享包源与版本由 `Packages/manifest.json` 和锁文件维护。

Meta XR 会在构建时生成根目录及 `Assets/StreamingAssets/` 下的 `RuntimeActionBindings.json`，这些文件由 SDK 重新生成，不作为源文件维护。本项目目前没有其他 StreamingAssets 资源。
