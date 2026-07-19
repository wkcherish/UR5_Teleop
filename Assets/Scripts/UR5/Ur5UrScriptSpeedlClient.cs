using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

[DefaultExecutionOrder(-40)]
public class Ur5UrScriptSpeedlClient : MonoBehaviour
{
    [Header("Robot Connection")]
    public string robotHost = "192.168.0.10";
    public int robotPort = 30002;
    public bool enableRealRobotOutput;
    public bool connectOnStart;
    public float connectTimeoutSeconds = 1.0f;
    public float reconnectIntervalSeconds = 1.0f;
    public int socketWriteTimeoutMilliseconds = 50;

    [Header("Arming")]
    [Tooltip("When enabled, non-zero motion is blocked until SetMotionArmed(true) or armOnStart is used.")]
    public bool requireMotionArmed = true;
    public bool armOnStart;
    public bool allowKeyboardArming;
    public KeyCode armKey = KeyCode.F9;
    public KeyCode disarmKey = KeyCode.F10;

    [Header("URScript speedl")]
    public float sendRateHz = 50.0f;
    public float commandDurationSeconds = 0.08f;
    public float acceleration = 0.25f;
    public float stopAcceleration = 0.50f;
    public float maxLinearSpeed = 0.05f;
    public float maxAngularSpeedRadiansPerSecond = 0.35f;
    public float commandTimeoutSeconds = 0.20f;

    [Header("Output Safety")]
    public bool limitOutputAcceleration = true;
    public float maxOutputLinearAcceleration = 0.15f;
    public float maxOutputAngularAcceleration = 1.00f;
    public bool sendStoplOnStop = true;

    [Header("Debug")]
    public bool logConnectionStatus = true;
    public bool logOutgoingScript;

    private TcpClient client;
    private NetworkStream stream;
    private Vector3 commandedLinearVelocity;
    private Vector3 commandedAngularVelocity;
    private Vector3 lastSentLinearVelocity;
    private Vector3 lastSentAngularVelocity;
    private bool commandActive;
    private bool robotMotionArmed;
    private bool sentStopForCurrentStop = true;
    private float lastCommandTime = -999.0f;
    private float nextSendTime;
    private float nextReconnectTime;

    public bool IsConnected => client != null && client.Connected && stream != null;
    public bool IsMotionArmed => !requireMotionArmed || robotMotionArmed;
    public bool IsCommandFresh => Time.time - lastCommandTime <= commandTimeoutSeconds;
    public bool IsMovingCommand => commandActive && IsCommandFresh && IsMotionArmed;
    public Vector3 CommandedLinearVelocity => commandedLinearVelocity;
    public Vector3 CommandedAngularVelocity => commandedAngularVelocity;
    public Vector3 LastSentLinearVelocity => lastSentLinearVelocity;
    public Vector3 LastSentAngularVelocity => lastSentAngularVelocity;

    private void Start()
    {
        robotMotionArmed = armOnStart && requireMotionArmed;
        if (enableRealRobotOutput && connectOnStart)
        {
            TryEnsureConnected();
        }
    }

    private void Update()
    {
        if (!allowKeyboardArming)
        {
            return;
        }

        if (Input.GetKeyDown(armKey))
        {
            SetMotionArmed(true);
        }

        if (Input.GetKeyDown(disarmKey))
        {
            SetMotionArmed(false);
        }
    }

    private void FixedUpdate()
    {
        if (!enableRealRobotOutput)
        {
            return;
        }

        float interval = sendRateHz > 0.0f ? 1.0f / sendRateHz : Time.fixedDeltaTime;
        if (Time.time < nextSendTime)
        {
            return;
        }

        nextSendTime = Time.time + Mathf.Max(0.001f, interval);
        if (!TryEnsureConnected())
        {
            return;
        }

        bool shouldMove = IsMovingCommand;
        Vector3 targetLinearVelocity = shouldMove ? commandedLinearVelocity : Vector3.zero;
        Vector3 targetAngularVelocity = shouldMove ? commandedAngularVelocity : Vector3.zero;
        float deltaTime = Mathf.Max(interval, Time.fixedDeltaTime);
        Vector3 linearVelocity = LimitOutputAcceleration(
            lastSentLinearVelocity,
            targetLinearVelocity,
            maxOutputLinearAcceleration,
            deltaTime);
        Vector3 angularVelocity = LimitOutputAcceleration(
            lastSentAngularVelocity,
            targetAngularVelocity,
            maxOutputAngularAcceleration,
            deltaTime);

        if (!shouldMove && sendStoplOnStop && !sentStopForCurrentStop)
        {
            SendStopl(stopAcceleration);
            sentStopForCurrentStop = true;
        }

        SendSpeedl(linearVelocity, angularVelocity, shouldMove ? acceleration : stopAcceleration);
        lastSentLinearVelocity = linearVelocity;
        lastSentAngularVelocity = angularVelocity;

        if (shouldMove)
        {
            sentStopForCurrentStop = false;
        }
    }

    public void SetCommand(
        Vector3 baseLinearVelocity,
        Vector3 baseAngularVelocity,
        bool active)
    {
        if (!IsFinite(baseLinearVelocity) || !IsFinite(baseAngularVelocity))
        {
            commandedLinearVelocity = Vector3.zero;
            commandedAngularVelocity = Vector3.zero;
            commandActive = false;
            lastCommandTime = Time.time;
            return;
        }

        commandedLinearVelocity = Vector3.ClampMagnitude(baseLinearVelocity, Mathf.Max(0.0f, maxLinearSpeed));
        commandedAngularVelocity = Vector3.ClampMagnitude(
            baseAngularVelocity,
            Mathf.Max(0.0f, maxAngularSpeedRadiansPerSecond));
        commandActive = active;
        lastCommandTime = Time.time;

        if (!active)
        {
            commandedLinearVelocity = Vector3.zero;
            commandedAngularVelocity = Vector3.zero;
        }
    }

    public void SetMotionArmed(bool armed)
    {
        robotMotionArmed = armed;
        if (!armed)
        {
            StopRobot();
        }

        if (logConnectionStatus)
        {
            Debug.Log("UR speedl motion armed=" + IsMotionArmed);
        }
    }

    public void StopRobot()
    {
        commandedLinearVelocity = Vector3.zero;
        commandedAngularVelocity = Vector3.zero;
        commandActive = false;
        lastCommandTime = Time.time;
        lastSentLinearVelocity = Vector3.zero;
        lastSentAngularVelocity = Vector3.zero;
        sentStopForCurrentStop = true;

        if (IsConnected)
        {
            SendStopl(stopAcceleration);
            SendSpeedl(Vector3.zero, Vector3.zero, stopAcceleration);
        }
    }

    private Vector3 LimitOutputAcceleration(
        Vector3 currentVelocity,
        Vector3 targetVelocity,
        float maxAcceleration,
        float deltaTime)
    {
        if (!limitOutputAcceleration)
        {
            return targetVelocity;
        }

        float accelerationLimit = Mathf.Max(0.0f, maxAcceleration);
        if (accelerationLimit <= 0.0f)
        {
            return targetVelocity;
        }

        return Vector3.MoveTowards(currentVelocity, targetVelocity, accelerationLimit * Mathf.Max(0.0001f, deltaTime));
    }

    private bool TryEnsureConnected()
    {
        if (IsConnected)
        {
            return true;
        }

        if (Time.time < nextReconnectTime)
        {
            return false;
        }

        nextReconnectTime = Time.time + Mathf.Max(0.1f, reconnectIntervalSeconds);
        return TryConnect();
    }

    private bool TryConnect()
    {
        Disconnect();
        if (string.IsNullOrEmpty(robotHost) || robotHost.Trim().Length == 0)
        {
            Debug.LogWarning("UR speedl client robotHost is empty.");
            return false;
        }

        try
        {
            client = new TcpClient();
            IAsyncResult connectResult = client.BeginConnect(robotHost, robotPort, null, null);
            bool connected = connectResult.AsyncWaitHandle.WaitOne(
                TimeSpan.FromSeconds(Mathf.Max(0.05f, connectTimeoutSeconds)));
            if (!connected)
            {
                Disconnect();
                Debug.LogWarning("UR speedl client connection timed out: " + robotHost + ":" + robotPort);
                return false;
            }

            client.EndConnect(connectResult);
            client.NoDelay = true;
            client.SendTimeout = Mathf.Max(1, socketWriteTimeoutMilliseconds);
            stream = client.GetStream();
            stream.WriteTimeout = Mathf.Max(1, socketWriteTimeoutMilliseconds);

            if (logConnectionStatus)
            {
                Debug.Log("UR speedl client connected: " + robotHost + ":" + robotPort);
            }

            return true;
        }
        catch (Exception exception)
        {
            Disconnect();
            Debug.LogWarning("UR speedl client failed to connect: " + exception.Message);
            return false;
        }
    }

    private void SendSpeedl(Vector3 linearVelocity, Vector3 angularVelocity, float commandAcceleration)
    {
        string script = string.Format(
            CultureInfo.InvariantCulture,
            "speedl([{0:F5},{1:F5},{2:F5},{3:F5},{4:F5},{5:F5}], {6:F3}, {7:F3})\n",
            linearVelocity.x,
            linearVelocity.y,
            linearVelocity.z,
            angularVelocity.x,
            angularVelocity.y,
            angularVelocity.z,
            Mathf.Max(0.0f, commandAcceleration),
            Mathf.Max(0.008f, commandDurationSeconds));
        WriteScript(script);
    }

    private void SendStopl(float commandAcceleration)
    {
        string script = string.Format(
            CultureInfo.InvariantCulture,
            "stopl({0:F3})\n",
            Mathf.Max(0.0f, commandAcceleration));
        WriteScript(script);
    }

    private void WriteScript(string script)
    {
        if (!IsConnected)
        {
            return;
        }

        try
        {
            byte[] payload = Encoding.ASCII.GetBytes(script);
            stream.Write(payload, 0, payload.Length);
            stream.Flush();

            if (logOutgoingScript)
            {
                Debug.Log("URScript: " + script.TrimEnd());
            }
        }
        catch (Exception exception)
        {
            Debug.LogWarning("UR speedl client write failed: " + exception.Message);
            Disconnect();
        }
    }

    private bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private void Disconnect()
    {
        if (stream != null)
        {
            stream.Close();
            stream = null;
        }

        if (client != null)
        {
            client.Close();
            client = null;
        }
    }

    private void OnDisable()
    {
        StopRobot();
        Disconnect();
    }

    private void OnApplicationQuit()
    {
        StopRobot();
        Disconnect();
    }

    private void OnValidate()
    {
        robotPort = Mathf.Clamp(robotPort, 1, 65535);
        connectTimeoutSeconds = Mathf.Max(0.05f, connectTimeoutSeconds);
        reconnectIntervalSeconds = Mathf.Max(0.1f, reconnectIntervalSeconds);
        socketWriteTimeoutMilliseconds = Mathf.Max(1, socketWriteTimeoutMilliseconds);
        sendRateHz = Mathf.Clamp(sendRateHz, 1.0f, 125.0f);
        commandDurationSeconds = Mathf.Max(0.008f, commandDurationSeconds);
        commandTimeoutSeconds = Mathf.Max(0.02f, commandTimeoutSeconds);
        acceleration = Mathf.Max(0.0f, acceleration);
        stopAcceleration = Mathf.Max(0.0f, stopAcceleration);
        maxLinearSpeed = Mathf.Max(0.0f, maxLinearSpeed);
        maxAngularSpeedRadiansPerSecond = Mathf.Max(0.0f, maxAngularSpeedRadiansPerSecond);
        maxOutputLinearAcceleration = Mathf.Max(0.0f, maxOutputLinearAcceleration);
        maxOutputAngularAcceleration = Mathf.Max(0.0f, maxOutputAngularAcceleration);
    }
}
