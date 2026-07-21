using UnityEngine;
using UnityEngine.XR;

public class Quest3RobotiqGripperController : MonoBehaviour
{
    [Header("Robot")]
    public Transform robotRoot;
    public string leftDriverName = "left_driver_link";
    public string rightDriverName = "right_driver_link";
    public string leftSpringName = "left_spring_link";
    public string rightSpringName = "right_spring_link";
    public string leftFollowerName = "left_follower_link";
    public string rightFollowerName = "right_follower_link";

    [Header("Quest Input")]
    public XRNode controllerNode = XRNode.RightHand;
    public bool useTrigger = true;
    [Tooltip("右手 Grip 是夹爪 Trigger 的死手开关。未握住 Grip 时，扳机输入不会改变夹爪目标。")]
    public bool requireGripDeadman = true;
    [Range(0.0f, 1.0f)] public float gripPressThreshold = 0.65f;
    [Range(0.0f, 1.0f)] public float gripReleaseThreshold = 0.40f;
    [Range(0.0f, 0.25f)] public float triggerDeadband = 0.04f;
    public float triggerSmoothingSharpness = 22.0f;

    [Header("Gripper Motion")]
    [Range(0.0f, 1.0f)] public float targetCloseAmount;
    public float closedAngleDegrees = 51.5662f;
    public float closeSpeedPerSecond = 2.60f;
    public float openSpeedPerSecond = 3.20f;
    public float stiffness = 8000.0f;
    public float damping = 450.0f;
    public float forceLimit = 160.0f;

    private ArticulationBody leftDriver;
    private ArticulationBody rightDriver;
    private ArticulationBody leftSpring;
    private ArticulationBody rightSpring;
    private ArticulationBody leftFollower;
    private ArticulationBody rightFollower;
    private InputDevice controllerDevice;
    private float currentCloseAmount;
    private float filteredTriggerAmount;
    private bool gripLatched;

    public float CloseAmount => currentCloseAmount;

    private void Awake()
    {
        ResolveJoints();
        ConfigureDrives();
    }

    private void Update()
    {
        if (useTrigger)
        {
            RefreshDeviceIfNeeded();
            if (controllerDevice.isValid
                && (!requireGripDeadman || ReadGripDeadman())
                && controllerDevice.TryGetFeatureValue(CommonUsages.trigger, out float triggerAmount))
            {
                float desiredTriggerAmount = ApplyTriggerDeadband(triggerAmount);
                filteredTriggerAmount = SmoothScalar(
                    filteredTriggerAmount,
                    desiredTriggerAmount,
                    triggerSmoothingSharpness,
                    Time.deltaTime);
                targetCloseAmount = filteredTriggerAmount;
            }
        }

        if (Input.GetKey(KeyCode.O)) targetCloseAmount = 0.0f;
        if (Input.GetKey(KeyCode.P)) targetCloseAmount = 1.0f;
    }

    private void FixedUpdate()
    {
        float speed = targetCloseAmount >= currentCloseAmount
            ? closeSpeedPerSecond
            : openSpeedPerSecond;
        currentCloseAmount = Mathf.MoveTowards(
            currentCloseAmount,
            targetCloseAmount,
            Mathf.Max(0.0f, speed) * Time.fixedDeltaTime);
        ApplyCloseAmount(currentCloseAmount);
    }

    public void SetTargetCloseAmount(float closeAmount)
    {
        targetCloseAmount = Mathf.Clamp01(closeAmount);
        filteredTriggerAmount = targetCloseAmount;
    }

    public void OpenGripper()
    {
        SetTargetCloseAmount(0.0f);
    }

    public void CloseGripper()
    {
        SetTargetCloseAmount(1.0f);
    }

    public void ApplyConfiguredDriveSettings()
    {
        ResolveJoints();
        ConfigureDrives();
    }

    private void ResolveJoints()
    {
        if (robotRoot == null)
        {
            robotRoot = transform;
        }

        if (leftDriver == null)
        {
            leftDriver = FindArticulationBody(leftDriverName);
        }

        if (rightDriver == null)
        {
            rightDriver = FindArticulationBody(rightDriverName);
        }

        if (leftSpring == null)
        {
            leftSpring = FindArticulationBody(leftSpringName);
        }

        if (rightSpring == null)
        {
            rightSpring = FindArticulationBody(rightSpringName);
        }

        if (leftFollower == null)
        {
            leftFollower = FindArticulationBody(leftFollowerName);
        }

        if (rightFollower == null)
        {
            rightFollower = FindArticulationBody(rightFollowerName);
        }
    }

    private void ConfigureDrives()
    {
        ConfigureDrive(leftDriver);
        ConfigureDrive(rightDriver);
        ConfigureDrive(leftSpring);
        ConfigureDrive(rightSpring);
        ConfigureDrive(leftFollower);
        ConfigureDrive(rightFollower);
    }

    private void ConfigureDrive(ArticulationBody body)
    {
        if (body == null)
        {
            return;
        }

        body.useGravity = false;
        ArticulationDrive drive = body.xDrive;
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        body.xDrive = drive;
    }

    private void ApplyCloseAmount(float closeAmount)
    {
        ApplyDriveTarget(leftDriver, closeAmount);
        ApplyDriveTarget(rightDriver, closeAmount);
        ApplyDriveTarget(leftSpring, closeAmount);
        ApplyDriveTarget(rightSpring, closeAmount);
        ApplyDriveTarget(leftFollower, closeAmount);
        ApplyDriveTarget(rightFollower, closeAmount);
    }

    private void ApplyDriveTarget(ArticulationBody body, float closeAmount)
    {
        if (body == null)
        {
            return;
        }

        ArticulationDrive drive = body.xDrive;
        drive.target = Mathf.Lerp(0.0f, closedAngleDegrees, closeAmount);
        body.xDrive = drive;
    }

    private void RefreshDeviceIfNeeded()
    {
        if (!controllerDevice.isValid)
        {
            controllerDevice = InputDevices.GetDeviceAtXRNode(controllerNode);
        }
    }

    private float ApplyTriggerDeadband(float triggerAmount)
    {
        float clamped = Mathf.Clamp01(triggerAmount);
        float deadband = Mathf.Clamp01(triggerDeadband);
        if (clamped <= deadband)
        {
            return 0.0f;
        }

        return Mathf.InverseLerp(deadband, 1.0f, clamped);
    }

    /// <summary>
    /// 使用滞回读取 Grip，避免握力处于阈值附近时夹爪命令反复启停。
    /// </summary>
    private bool ReadGripDeadman()
    {
        if (!controllerDevice.TryGetFeatureValue(CommonUsages.grip, out float gripAmount))
        {
            gripLatched = false;
            return false;
        }

        if (gripLatched)
        {
            gripLatched = gripAmount >= Mathf.Clamp01(gripReleaseThreshold);
        }
        else
        {
            gripLatched = gripAmount >= Mathf.Clamp01(gripPressThreshold);
        }

        return gripLatched;
    }

    private float SmoothScalar(float current, float target, float sharpness, float deltaTime)
    {
        float blend = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, sharpness) * Mathf.Max(0.0001f, deltaTime));
        return Mathf.Lerp(current, target, blend);
    }

    private void OnValidate()
    {
        triggerDeadband = Mathf.Clamp(triggerDeadband, 0.0f, 0.25f);
        gripPressThreshold = Mathf.Clamp01(gripPressThreshold);
        gripReleaseThreshold = Mathf.Clamp(gripReleaseThreshold, 0.0f, gripPressThreshold);
        triggerSmoothingSharpness = Mathf.Max(0.0f, triggerSmoothingSharpness);
        targetCloseAmount = Mathf.Clamp01(targetCloseAmount);
        closedAngleDegrees = Mathf.Max(0.0f, closedAngleDegrees);
        closeSpeedPerSecond = Mathf.Max(0.0f, closeSpeedPerSecond);
        openSpeedPerSecond = Mathf.Max(0.0f, openSpeedPerSecond);
        stiffness = Mathf.Max(0.0f, stiffness);
        damping = Mathf.Max(0.0f, damping);
        forceLimit = Mathf.Max(0.0f, forceLimit);
    }

    private ArticulationBody FindArticulationBody(string objectName)
    {
        foreach (ArticulationBody body in robotRoot.GetComponentsInChildren<ArticulationBody>(true))
        {
            if (body.name == objectName)
            {
                return body;
            }
        }

        Debug.LogWarning("Robotiq driver joint not found: " + objectName);
        return null;
    }
}
