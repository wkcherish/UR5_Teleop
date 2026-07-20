using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// 将 Quest 右手控制器的原始 OpenXR 位姿发送到采集电脑。
/// 
/// 该组件只发送 UDP 遥测数据，不会创建 URScript 连接，也不会控制 UR5 或夹爪。
/// 真实机器人动作必须由 PC 端完成坐标映射、RTDE 状态校验和安全过滤后才可执行。
/// </summary>
[DefaultExecutionOrder(-80)]
public class Quest3UdpTeleopSender : MonoBehaviour
{
    [Serializable]
    private class ControllerPayload
    {
        public float[] position;
        public float[] quaternion_xyzw;
    }

    [Serializable]
    private class ButtonsPayload
    {
        public bool clutch;
        public float trigger;
        public bool recenter;
        public bool stop_episode;
    }

    [Serializable]
    private class QuestPacketPayload
    {
        public int protocol_version = 1;
        public long sequence;
        public long quest_timestamp_ns;
        public ControllerPayload controller;
        public ButtonsPayload buttons;
    }

    [Header("Quest To PC Shadow Telemetry")]
    [Tooltip("Default is off. Enabling this sends only controller telemetry to the PC; it never enables real-robot output.")]
    public bool sendPackets;
    [Tooltip("IP address of the DG-VLA capture computer running shadow_teleop_ur5_quest3.py.")]
    public string receiverHost = "";
    public int receiverPort = 8080;
    public float sendRateHz = 72.0f;
    public bool logStatus = true;

    [Header("Quest Input Mapping")]
    public XRNode controllerNode = XRNode.RightHand;
    [Tooltip("Grip is the PC-side clutch/deadman signal. It uses analog grip when available, otherwise gripButton.")]
    public bool useGripAsClutch = true;
    [Range(0.0f, 1.0f)] public float clutchGripThreshold = 0.65f;
    [Tooltip("Disabled by default because A/B may already be used by Unity grasp assist. Enable only after assigning a non-conflicting PC-side recenter action.")]
    public bool sendPrimaryButtonAsRecenter;
    [Tooltip("Disabled by default because A/B may already be used by Unity grasp assist. Enable only after assigning a non-conflicting PC-side episode-stop action.")]
    public bool sendSecondaryButtonAsStopEpisode;

    private InputDevice controllerDevice;
    private UdpClient udpClient;
    private long nextSequence;
    private float nextSendTime;
    private bool hasLoggedMissingDevice;
    private bool hasLoggedInvalidEndpoint;

    public bool IsSending => sendPackets && udpClient != null && controllerDevice.isValid;
    public long LastSentSequence { get; private set; } = -1;

    private void Start()
    {
        RefreshDeviceIfNeeded();
    }

    private void Update()
    {
        if (!sendPackets)
        {
            return;
        }

        if (!HasValidEndpoint())
        {
            if (logStatus && !hasLoggedInvalidEndpoint)
            {
                hasLoggedInvalidEndpoint = true;
                Debug.LogWarning("Quest UDP telemetry is enabled but receiverHost is empty or receiverPort is invalid.");
            }

            return;
        }

        hasLoggedInvalidEndpoint = false;
        RefreshDeviceIfNeeded();
        if (!controllerDevice.isValid)
        {
            if (logStatus && !hasLoggedMissingDevice)
            {
                hasLoggedMissingDevice = true;
                Debug.LogWarning("Quest UDP telemetry is waiting for the configured controller.");
            }

            return;
        }

        hasLoggedMissingDevice = false;
        if (Time.unscaledTime < nextSendTime)
        {
            return;
        }

        float interval = 1.0f / Mathf.Max(1.0f, sendRateHz);
        nextSendTime = Time.unscaledTime + interval;
        TrySendLatestControllerPacket();
    }

    private void OnDisable()
    {
        CloseSocket();
    }

    private void OnApplicationQuit()
    {
        CloseSocket();
    }

    private void TrySendLatestControllerPacket()
    {
        if (!controllerDevice.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position)
            || !controllerDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation))
        {
            return;
        }

        // 故意不经过 XR Origin：PC 端需要的是 Quest tracking frame 的原始数据，
        // 再以 clutch 相对位姿方式映射到 UR5 base frame。
        QuestPacketPayload packet = new QuestPacketPayload
        {
            sequence = nextSequence,
            quest_timestamp_ns = GetQuestMonotonicTimestampNanoseconds(),
            controller = new ControllerPayload
            {
                position = new[] { position.x, position.y, position.z },
                quaternion_xyzw = new[] { rotation.x, rotation.y, rotation.z, rotation.w }
            },
            buttons = new ButtonsPayload
            {
                clutch = ReadClutch(),
                trigger = ReadTrigger(),
                recenter = sendPrimaryButtonAsRecenter && ReadButton(CommonUsages.primaryButton),
                stop_episode = sendSecondaryButtonAsStopEpisode && ReadButton(CommonUsages.secondaryButton)
            }
        };

        try
        {
            EnsureSocket();
            byte[] payload = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
            udpClient.Send(payload, payload.Length, receiverHost.Trim(), receiverPort);
            LastSentSequence = nextSequence;
            nextSequence++;
        }
        catch (Exception exception)
        {
            CloseSocket();
            if (logStatus)
            {
                Debug.LogWarning("Quest UDP telemetry send failed: " + exception.Message);
            }
        }
    }

    private bool ReadClutch()
    {
        if (!useGripAsClutch)
        {
            return true;
        }

        if (controllerDevice.TryGetFeatureValue(CommonUsages.grip, out float gripAmount))
        {
            return gripAmount >= Mathf.Clamp01(clutchGripThreshold);
        }

        return controllerDevice.TryGetFeatureValue(CommonUsages.gripButton, out bool gripPressed)
            && gripPressed;
    }

    private float ReadTrigger()
    {
        return controllerDevice.TryGetFeatureValue(CommonUsages.trigger, out float trigger)
            ? Mathf.Clamp01(trigger)
            : 0.0f;
    }

    private bool ReadButton(InputFeatureUsage<bool> usage)
    {
        return controllerDevice.TryGetFeatureValue(usage, out bool pressed) && pressed;
    }

    private void RefreshDeviceIfNeeded()
    {
        if (!controllerDevice.isValid)
        {
            controllerDevice = InputDevices.GetDeviceAtXRNode(controllerNode);
        }
    }

    private void EnsureSocket()
    {
        if (udpClient == null)
        {
            udpClient = new UdpClient();
        }
    }

    private bool HasValidEndpoint()
    {
        return !string.IsNullOrWhiteSpace(receiverHost)
            && receiverPort >= 1
            && receiverPort <= 65535;
    }

    private long GetQuestMonotonicTimestampNanoseconds()
    {
        return (long)(Time.realtimeSinceStartupAsDouble * 1_000_000_000.0);
    }

    private void CloseSocket()
    {
        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }
    }

    private void OnValidate()
    {
        receiverPort = Mathf.Clamp(receiverPort, 1, 65535);
        sendRateHz = Mathf.Clamp(sendRateHz, 1.0f, 120.0f);
        clutchGripThreshold = Mathf.Clamp01(clutchGripThreshold);
    }
}
