using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

/// <summary>
/// Drives the Unity UR5 digital twin from measured RTDE joint angles sent by
/// the robot-side bridge. This is a receive-only display path: it never sends
/// a command to UR5 and it does not change the Quest-to-PC command channel.
/// </summary>
[DefaultExecutionOrder(1000)]
public class Ur5RtdeJointStateReceiver : MonoBehaviour
{
    [Serializable]
    private class JointStatePacket
    {
        public int protocol_version;
        public string stream;
        public string source;
        public string session_id;
        public long source_timestamp_ns;
        public long sequence;
        public string[] joint_names;
        public float[] joint_positions_rad;
        public float gripper_command = -1.0f;
    }

    public const int ProtocolVersion = 1;
    public const string StreamName = "ur5_joint_state";

    [Header("References")]
    public Ur5ArticulationJointController jointController;
    public Quest3RobotiqGripperController gripperController;
    public Quest3UdpTeleopSender commandSender;

    [Header("UDP Read-Only Feedback")]
    [Tooltip("Must equal the PC bridge --quest-feedback-port. This component only listens for state packets.")]
    public int listenPort = 8081;
    public bool receiveFeedback = true;
    [Tooltip("After this timeout, no stale RTDE pose is applied.")]
    public float staleTimeoutSeconds = 0.25f;
    public bool logStatus = true;

    [Header("Feedback-First Startup Gate")]
    [Tooltip("Hide the virtual robot until the first valid RTDE joint state arrives, so Unity's imported URDF pose is never presented as the initial robot pose.")]
    public bool hideRobotUntilFirstFeedback = true;
    [Tooltip("Disables Unity-local IK/trajectory writers and its legacy URScript output. Quest telemetry continues to reach the PC bridge, which is the only allowed motion owner.")]
    public bool disableUnityLocalMotionWriters = true;

    [Header("UR RTDE rad -> Unity articulation deg")]
    [Tooltip("One entry per UR5 joint. Keep +1 when URDF axes match RTDE; set an entry to -1 only after a single-joint verification.")]
    public float[] jointSigns = { 1f, 1f, 1f, 1f, 1f, 1f };
    [Tooltip("Static URDF/importer zero offsets in degrees. Leave at 0 until a zero-pose calibration proves an offset.")]
    public float[] jointOffsetsDegrees = { 0f, 0f, 0f, 0f, 0f, 0f };

    private UdpClient client;
    private int boundPort;
    private IPEndPoint receiveEndpoint = new IPEndPoint(IPAddress.Any, 0);
    private float[] pendingJointDegrees;
    private float pendingGripperCommand = -1.0f;
    private float latestArrivalTime = float.NegativeInfinity;
    private long lastSequence = -1;
    private string currentSession;
    private readonly HashSet<string> retiredSessions = new HashSet<string>();
    private string lastError = "Waiting for RTDE feedback";
    private Renderer[] robotRenderers = new Renderer[0];
    private bool hasReceivedFirstFeedback;
    private bool revealAfterNextFixedUpdate;

    public bool HasFreshFeedback => Time.unscaledTime - latestArrivalTime <= staleTimeoutSeconds;
    public long LastSequence => lastSequence;
    public string LastError => lastError;

    private void Awake()
    {
        ResolveJointController();
        ConfigureFeedbackFirstStartup();
    }

    private void OnEnable()
    {
        OpenSocket();
    }

    private void OnDisable()
    {
        CloseSocket();
    }

    private void Update()
    {
        if (!receiveFeedback)
        {
            return;
        }

        OpenSocket();
        if (client == null)
        {
            return;
        }

        try
        {
            int budget = 64;
            while (client.Available > 0 && budget-- > 0)
            {
                byte[] bytes = client.Receive(ref receiveEndpoint);
                if (bytes.Length <= 4096
                    && commandSender != null
                    && IPAddress.TryParse(commandSender.receiverHost, out IPAddress expectedHost)
                    && receiveEndpoint.Address.Equals(expectedHost))
                {
                    TryAcceptPacket(Encoding.UTF8.GetString(bytes));
                }
            }
            if (!HasFreshFeedback && hideRobotUntilFirstFeedback)
            {
                SetRobotVisible(false);
                lastError = "Waiting for fresh UR5 feedback";
            }
        }
        catch (SocketException exception)
        {
            lastError = "UDP receive failed: " + exception.Message;
            if (logStatus)
            {
                Debug.LogWarning(lastError);
            }
            CloseSocket();
        }
    }

    private void FixedUpdate()
    {
        if (!HasFreshFeedback || pendingJointDegrees == null)
        {
            return;
        }

        ResolveJointController();
        if (jointController == null || jointController.JointCount < 6)
        {
            return;
        }

        // Direct state assignment prevents Unity's local IK/drive lag from
        // making the displayed arm lead or trail the physical arm.
        jointController.SetMeasuredStateTeleopTargetsDegrees(
            pendingJointDegrees,
            pendingJointDegrees.Length,
            false,
            true);
        if (gripperController != null && pendingGripperCommand >= 0.0f)
        {
            gripperController.SetFeedbackCloseAmount(pendingGripperCommand);
            pendingGripperCommand = -1.0f;
        }
        if (revealAfterNextFixedUpdate)
        {
            revealAfterNextFixedUpdate = false;
            SetRobotVisible(true);
        }
    }

    private void TryAcceptPacket(string json)
    {
        JointStatePacket packet;
        try
        {
            packet = JsonUtility.FromJson<JointStatePacket>(json);
        }
        catch (ArgumentException)
        {
            return;
        }

        if (packet == null
            || packet.protocol_version != ProtocolVersion
            || packet.stream != StreamName
            || packet.source != "quest_ur5_host"
            || string.IsNullOrEmpty(packet.session_id)
            || packet.session_id.Length > 64
            || retiredSessions.Contains(packet.session_id)
            || packet.joint_positions_rad == null
            || packet.joint_positions_rad.Length != 6
            || packet.sequence < 0
            || (packet.session_id == currentSession && packet.sequence <= lastSequence))
        {
            return;
        }

        EnsureMappingArrays();
        float[] nextJointDegrees = new float[6];
        for (int index = 0; index < nextJointDegrees.Length; index++)
        {
            float radians = packet.joint_positions_rad[index];
            if (float.IsNaN(radians) || float.IsInfinity(radians))
            {
                lastError = "Discarded non-finite RTDE joint state";
                return;
            }
            nextJointDegrees[index] = radians * Mathf.Rad2Deg * jointSigns[index] + jointOffsetsDegrees[index];
        }

        if (float.IsNaN(packet.gripper_command) || float.IsInfinity(packet.gripper_command))
        {
            lastError = "Discarded non-finite gripper feedback";
            return;
        }

        // Commit only after validating the entire datagram. Rejected packets
        // must never partially overwrite the last accepted measured pose.
        if (currentSession != null && currentSession != packet.session_id)
        {
            retiredSessions.Add(currentSession);
        }
        currentSession = packet.session_id;
        pendingJointDegrees = nextJointDegrees;
        pendingGripperCommand = packet.gripper_command >= 0.0f ? Mathf.Clamp01(packet.gripper_command) : -1.0f;
        lastSequence = packet.sequence;
        latestArrivalTime = Time.unscaledTime;
        lastError = "";
        revealAfterNextFixedUpdate = true;
        if (!hasReceivedFirstFeedback)
        {
            hasReceivedFirstFeedback = true;
            revealAfterNextFixedUpdate = true;
            if (logStatus)
            {
                Debug.Log("UR5 digital twin initialized from first RTDE joint-state feedback.");
            }
        }
    }

    private void ResolveJointController()
    {
        if (jointController == null)
        {
            jointController = GetComponent<Ur5ArticulationJointController>();
        }
        if (jointController == null)
        {
            jointController = GetComponentInChildren<Ur5ArticulationJointController>();
        }
        if (gripperController == null)
        {
            gripperController = GetComponent<Quest3RobotiqGripperController>();
        }
        if (gripperController == null)
        {
            gripperController = GetComponentInChildren<Quest3RobotiqGripperController>();
        }
    }

    private void OpenSocket()
    {
        if (client != null && boundPort != listenPort)
        {
            CloseSocket();
        }
        if (!receiveFeedback || client != null || listenPort < 1 || listenPort > 65535)
        {
            return;
        }
        try
        {
            client = new UdpClient(listenPort);
            boundPort = listenPort;
            client.Client.Blocking = false;
            lastError = "Waiting for RTDE feedback";
        }
        catch (SocketException exception)
        {
            lastError = "Cannot listen on UDP " + listenPort + ": " + exception.Message;
            if (logStatus)
            {
                Debug.LogWarning(lastError);
            }
        }
    }

    private void CloseSocket()
    {
        if (client != null)
        {
            client.Close();
            client = null;
        }
    }

    private void EnsureMappingArrays()
    {
        if (jointSigns == null || jointSigns.Length != 6)
        {
            jointSigns = new[] { 1f, 1f, 1f, 1f, 1f, 1f };
        }
        if (jointOffsetsDegrees == null || jointOffsetsDegrees.Length != 6)
        {
            jointOffsetsDegrees = new[] { 0f, 0f, 0f, 0f, 0f, 0f };
        }
    }

    private void ConfigureFeedbackFirstStartup()
    {
        robotRenderers = GetComponentsInChildren<Renderer>(true);
        if (hideRobotUntilFirstFeedback)
        {
            SetRobotVisible(false);
        }

        if (!disableUnityLocalMotionWriters)
        {
            return;
        }

        foreach (Ur5TcpTargetFollower follower in GetComponentsInChildren<Ur5TcpTargetFollower>(true))
        {
            follower.followTarget = false;
        }
        foreach (Ur5JointTrajectoryPlayer trajectory in GetComponentsInChildren<Ur5JointTrajectoryPlayer>(true))
        {
            trajectory.play = false;
        }
        foreach (Ur5UrScriptSpeedlClient speedlClient in GetComponentsInChildren<Ur5UrScriptSpeedlClient>(true))
        {
            speedlClient.enableRealRobotOutput = false;
            speedlClient.SetMotionArmed(false);
        }
        foreach (Quest3RobotiqGripperController gripper in GetComponentsInChildren<Quest3RobotiqGripperController>(true))
        {
            // Keep FixedUpdate available for feedback rendering, but stop local Quest trigger writes.
            gripper.useTrigger = false;
        }
    }

    private void SetRobotVisible(bool visible)
    {
        foreach (Renderer renderer in robotRenderers)
        {
            if (renderer != null)
            {
                renderer.enabled = visible;
            }
        }
    }
}
