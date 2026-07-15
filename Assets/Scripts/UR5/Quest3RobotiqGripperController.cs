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

    [Header("Gripper Motion")]
    [Range(0.0f, 1.0f)] public float targetCloseAmount;
    public float closedAngleDegrees = 51.5662f;
    public float closeSpeedPerSecond = 2.0f;
    public float stiffness = 8000.0f;
    public float damping = 200.0f;
    public float forceLimit = 100.0f;

    private ArticulationBody leftDriver;
    private ArticulationBody rightDriver;
    private ArticulationBody leftSpring;
    private ArticulationBody rightSpring;
    private ArticulationBody leftFollower;
    private ArticulationBody rightFollower;
    private InputDevice controllerDevice;
    private float currentCloseAmount;

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
                && controllerDevice.TryGetFeatureValue(CommonUsages.trigger, out float triggerAmount))
            {
                targetCloseAmount = Mathf.Clamp01(triggerAmount);
            }
        }

        if (Input.GetKey(KeyCode.O)) targetCloseAmount = 0.0f;
        if (Input.GetKey(KeyCode.P)) targetCloseAmount = 1.0f;

        currentCloseAmount = Mathf.MoveTowards(
            currentCloseAmount,
            targetCloseAmount,
            closeSpeedPerSecond * Time.deltaTime);
        ApplyCloseAmount(currentCloseAmount);
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
