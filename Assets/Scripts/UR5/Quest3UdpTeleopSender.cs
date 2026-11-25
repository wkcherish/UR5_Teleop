using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// 将 Quest 右手控制器的原始 OpenXR 位姿发送到独立控制程序。
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
        public string frame;
    }

    [Serializable]
    private class ValidPayload
    {
        public bool controller_pose;
        public bool buttons;
        public bool tracking;
    }

    [Serializable]
    private class ButtonsPayload
    {
        public bool clutch;
        public float grip;
        public float trigger;
        public bool primary;
        public bool secondary;
        public bool rotation_adjust;
        public bool free_wrist;
        public bool recenter;
        public bool stop_episode;
    }

    [Serializable]
    private class QuestPacketPayload
    {
        public int protocol_version = 2;
        public string session_id;
        public long sequence_id;
        public long source_timestamp_ns;
        public string source_timestamp_clock;
        public string source;
        public string stream;
        public string mode;
        public ValidPayload valid;
        public ControllerPayload controller;
        public ButtonsPayload buttons;
        public bool heartbeat;
    }

    [Header("Quest To Standalone Control Host")]
    [Tooltip("Default is off. Enabling this sends only controller telemetry to the PC; it never enables real-robot output.")]
    public bool sendPackets;
    [Tooltip("IP address of the computer running python -m quest_ur5 from this repository.")]
    public string receiverHost = "";
    public int receiverPort = 8080;
    public float sendRateHz = 72.0f;
    public bool logStatus = true;

    [Header("Quest Input Mapping")]
    public XRNode controllerNode = XRNode.RightHand;
    [Tooltip("Grip is the PC-side clutch/deadman signal. It uses analog grip when available, otherwise gripButton.")]
    public bool useGripAsClutch = true;
    [Range(0.0f, 1.0f)] public float clutchGripThreshold = 0.65f;
    [Range(0.0f, 1.0f)] public float clutchGripReleaseThreshold = 0.40f;
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
    private bool clutchLatched;
    private readonly string sessionId = Guid.NewGuid().ToString("N");
    private bool applicationPaused;
    private bool applicationFocused = true;

    // The endpoint panel owns this gate while the user edits connection settings.
    public bool InputBlocked { get; set; }

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
        }

        hasLoggedMissingDevice = !controllerDevice.isValid;
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
        bool hasPosition = controllerDevice.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
        bool hasRotation = controllerDevice.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);
        bool hasPrimary = controllerDevice.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryPressed);
        bool hasSecondary = controllerDevice.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryPressed);
        bool hasTrigger = controllerDevice.TryGetFeatureValue(CommonUsages.trigger, out float trigger);
        bool hasGrip = controllerDevice.TryGetFeatureValue(CommonUsages.grip, out float unusedGrip)
            || controllerDevice.TryGetFeatureValue(CommonUsages.gripButton, out bool unusedGripButton);
        bool tracked = controllerDevice.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked) && isTracked;
        bool hasTrackingState = controllerDevice.TryGetFeatureValue(CommonUsages.trackingState, out InputTrackingState trackingState);
        InputDevice headset = InputDevices.GetDeviceAtXRNode(XRNode.Head);
        bool headsetPresent = !headset.TryGetFeatureValue(CommonUsages.userPresence, out bool userPresent) || userPresent;
        InputTrackingState required = InputTrackingState.Position | InputTrackingState.Rotation;
        bool trackingValid = tracked && hasTrackingState && (trackingState & required) == required
            && !applicationPaused && applicationFocused && headsetPresent;
        bool buttonsValid = hasPrimary && hasSecondary && hasTrigger && hasGrip;
        bool clutch = ReadClutch(out float gripAmount);
        if (!hasPosition || !hasRotation || !trackingValid || !buttonsValid || InputBlocked)
        {
            clutch = false;
            clutchLatched = false;
        }
        if (!hasRotation)
        {
            rotation = Quaternion.identity;
        }
        string mode = ResolveMode(clutch, primaryPressed, secondaryPressed);

        // 故意不经过 XR Origin：PC 端需要的是 Quest tracking frame 的原始数据，
        // 再以 clutch 相对位姿方式映射到 UR5 base frame。
        QuestPacketPayload packet = new QuestPacketPayload
        {
            session_id = sessionId,
            sequence_id = nextSequence,
            source_timestamp_ns = GetQuestMonotonicTimestampNanoseconds(),
            source_timestamp_clock = "quest_realtime_since_startup",
            source = "quest3_unity",
            stream = controllerNode == XRNode.LeftHand ? "left_controller" : "right_controller",
            mode = mode,
            valid = new ValidPayload
            {
                controller_pose = hasPosition && hasRotation,
                buttons = buttonsValid && !InputBlocked,
                tracking = trackingValid
            },
            controller = new ControllerPayload
            {
                position = new[] { position.x, position.y, position.z },
                quaternion_xyzw = new[] { rotation.x, rotation.y, rotation.z, rotation.w },
                frame = "quest_tracking"
            },
            buttons = new ButtonsPayload
            {
                clutch = clutch,
                grip = gripAmount,
                trigger = Mathf.Clamp01(trigger),
                primary = primaryPressed,
                secondary = secondaryPressed,
                rotation_adjust = mode == "default_grasp",
                free_wrist = mode == "free_wrist",
                recenter = sendPrimaryButtonAsRecenter && primaryPressed && !clutch,
                stop_episode = sendSecondaryButtonAsStopEpisode && secondaryPressed && !clutch
            },
            heartbeat = true
        };

        try
        {
            EnsureSocket();
            byte[] payload = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
            udpClient.Send(payload, payload.Length, receiverHost.Trim(), receiverPort);
            LastSentSequence = nextSequence;
            if (logStatus && (nextSequence == 0 || nextSequence % 300 == 0))
            {
                Debug.Log("Quest UDP telemetry sent seq "
                    + nextSequence
                    + " to "
                    + receiverHost.Trim()
                    + ":"
                    + receiverPort);
            }

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

    private bool ReadClutch(out float gripAmount)
    {
        gripAmount = 0.0f;
        bool hasAnalogGrip = controllerDevice.TryGetFeatureValue(CommonUsages.grip, out float rawGripAmount);
        bool hasGripButton = controllerDevice.TryGetFeatureValue(CommonUsages.gripButton, out bool gripPressed);
        if (hasAnalogGrip)
        {
            gripAmount = Mathf.Clamp01(rawGripAmount);
        }
        else if (hasGripButton && gripPressed)
        {
            gripAmount = 1.0f;
        }

        if (!useGripAsClutch)
        {
            clutchLatched = false;
            return false;
        }

        if (hasAnalogGrip)
        {
            float pressThreshold = Mathf.Clamp01(clutchGripThreshold);
            float releaseThreshold = Mathf.Min(pressThreshold, Mathf.Clamp01(clutchGripReleaseThreshold));
            clutchLatched = clutchLatched
                ? gripAmount >= releaseThreshold || (hasGripButton && gripPressed)
                : gripAmount >= pressThreshold || (hasGripButton && gripPressed);
            return clutchLatched;
        }

        clutchLatched = hasGripButton && gripPressed;
        return clutchLatched;
    }

    private void OnApplicationPause(bool paused)
    {
        applicationPaused = paused;
        clutchLatched = false;
    }

    private void OnApplicationFocus(bool focused)
    {
        applicationFocused = focused;
        clutchLatched = false;
    }

    private static string ResolveMode(bool clutch, bool primaryPressed, bool secondaryPressed)
    {
        if (!clutch)
        {
            return "idle";
        }

        if (secondaryPressed)
        {
            return "free_wrist";
        }

        return primaryPressed ? "default_grasp" : "translate";
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
        clutchGripReleaseThreshold = Mathf.Clamp(clutchGripReleaseThreshold, 0.0f, clutchGripThreshold);
    }
}
