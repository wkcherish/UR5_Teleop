# 控制架构与通信契约

## 代码边界

| 模块 | 职责 |
| --- | --- |
| `Quest3UdpTeleopSender` | 原始 tracking-frame 输入、按键、真实 tracking flags、会话 ID |
| `Quest3UdpEndpointRuntimePanel` | 保存电脑 IP；面板可见时禁止运动输入 |
| `Ur5RtdeJointStateReceiver` | 实测关节渲染；校验来源 IP、会话、序号、有限数；拒绝部分更新 |
| `robot/.../protocol.py` | 严格 v2 输入边界，不把字符串转换为布尔 |
| `session.py` | Grip 状态机、恢复门控、相对映射、模式切换与执行安全过滤 |
| `teleop.py` / `safety.py` | 坐标和姿态数学、OpenTeach 平滑、工作区/速度/目标超前限制 |
| `runtime.py` | 有界 UDP drain、按控制周期执行、日志与反馈 |
| `backend.py` | 虚拟后端或 RTDE 连接、状态有效性、IK/控制器安全检查、watchdog 和清理 |
| `robotiq.py` / `gripper.py` | 异步夹爪服务与无硬件默认实现 |

Unity 的本地 IK、预览和 CSV 诊断仍保留，开发资料位于 `docs/development.md`。主场景使用 UDP 独立控制模式，关闭本地运动写入，Python 是真实机械臂运动的唯一所有者。修改旧预览参数不会修改 Python 控制参数。

## Quest → 电脑（v2）

每包 UTF-8 JSON，最大 4096 bytes，默认发送频率 100 Hz（实际受头显帧率限制）：

```json
{
  "protocol_version": 2,
  "session_id": "quest-process-unique-id",
  "sequence_id": 1,
  "source_timestamp_ns": 1234567890,
  "source_timestamp_clock": "quest_realtime_since_startup",
  "source": "quest3_unity",
  "stream": "right_controller",
  "mode": "translate",
  "valid": {"controller_pose": true, "buttons": true, "tracking": true},
  "controller": {"position": [0, 1, 0], "quaternion_xyzw": [0, 0, 0, 1], "frame": "quest_tracking"},
  "buttons": {"clutch": true, "grip": 1, "trigger": 0, "primary": false, "secondary": false,
              "rotation_adjust": false, "free_wrist": false, "recenter": false, "stop_episode": false},
  "heartbeat": true
}
```

`mode` 为 `idle / translate / default_grasp / free_wrist`。无 clutch 必须为 idle；有 clutch 必须是另外三者。布尔值必须是真布尔值。位置/姿态、trigger、序号、时钟和来源接受前均校验。

`session_id` 区分 Quest 进程重启；暂兼容没有此字段的旧 v2（标为 legacy，会话恢复能力有限，建议重建 APK）。每个会话内序号和源时间必须递增；旧会话重放会触发停止。头显与电脑单调时钟不直接相减；接收时间用于超时，源时间差用于连续性校验，不能据此宣称测得绝对单向延迟。

Unity 检查 `isTracked` 和 position/rotation tracking bits；暂停/失焦、按键特征不可读、网络面板可见时发停止/无效输入。PC 不把 UDP 发送成功或设备仍连接视作追踪有效。

## 电脑 → Quest（v1）

仅 RTDE 后端发布 `protocol_version=1`、`stream=ur5_joint_state`、`source=quest_ur5_host`。字段含电脑进程 `session_id`、`sequence`、单调时间、六个标准关节名、`joint_positions_rad`、`gripper_command` 和 `control_state`。

关节顺序：shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3。夹爪值 −1 表示没有命令反馈，其余为 [0,1] 请求值。Unity 只接受当前设置的电脑 IP；整包验证成功后一次性替换待显示状态。电脑重启可使用新会话从 sequence=0 开始，已退休会话不再接受。超过 0.25 s 没有反馈就隐藏机器人。

虚拟后端没有 IK，不发假关节反馈。若后续需要模拟整臂，应增加独立仿真后端和明确的 simulation stream。

## 停止和恢复

启动必须先观察到一次有效 Grip 松开。运动中失踪、跳变、乱序、数据饱和、超时、机器人状态不允许运动或夹爪异常都会停止并撤销 armed 状态；恢复后再次松开 Grip 才能重新锚定实测 TCP。同一批 UDP 包里的停止事件不能被后面的运动包覆盖。

主机默认最多维持最新位置输入 0.15 s；不外推速度。每次控制循环只执行一个受限命令，迟到后不累积完整周期的运动预算。RTDE 状态时间戳停止变化、回退或断线会退出；UR 控制器另外设置 5 Hz watchdog 并在每个控制周期喂狗，防止主机卡死无限维持命令。

反馈模式由 bootstrap 明确关闭本地夹爪输入，不依赖组件 Awake 的先后顺序。

`Ctrl+C`、SIGTERM、异常和初始化失败都执行连接清理；停止夹爪与机械臂分别尝试，关闭一个连接失败不跳过其他资源。硬件 watchdog 和软件急停仍需真实 UR5 实测验证，不能替代示教器急停。

## 开发验证

`python -m pytest robot/tests` 覆盖协议、离合恢复、限速/边界、RTDE mock、初始化失败清理、真实 UDP 子进程和反馈契约。原参考算法来源和改动说明保留在 `third_party/teleoperation/NOTICE`。
