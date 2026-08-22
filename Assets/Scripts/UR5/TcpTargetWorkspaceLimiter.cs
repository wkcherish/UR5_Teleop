using UnityEngine;

[DefaultExecutionOrder(100)]
public class TcpTargetWorkspaceLimiter : MonoBehaviour
{
    [Header("Reference Frame")]
    public Transform robotRoot;
    public bool constrainTarget = true;
    [Tooltip("Legacy fallback only. Quest teleop applies ClampWorldPosition before publishing its command so this writer stays disabled.")]
    public bool constrainInLateUpdate = true;
    public bool preserveInitialTargetPose = true;
    public TcpTargetWriteMonitor writeMonitor;

    [Header("Workspace Relative To UR5 Base")]
    public Vector3 minimumLocalPosition = new Vector3(-0.70f, 0.015f, -0.70f);
    public Vector3 maximumLocalPosition = new Vector3(0.70f, 0.85f, 0.70f);

    [Header("Base Keep-Out")]
    public bool keepAwayFromBase = true;
    public float baseKeepOutRadius = 0.18f;

    private Vector2 lastHorizontalDirection = Vector2.up;
    private bool hasPreservedInitialPose;
    private Vector3 preservedInitialPosition;

    private void Awake()
    {
        ResolveRobotRoot();
    }

    private void LateUpdate()
    {
        if (constrainTarget && constrainInLateUpdate)
        {
            if (preserveInitialTargetPose
                && hasPreservedInitialPose
                && (transform.position - preservedInitialPosition).sqrMagnitude < 0.000001f)
            {
                return;
            }

            hasPreservedInitialPose = false;
            Vector3 clamped = ClampWorldPosition(transform.position);
            if ((clamped - transform.position).sqrMagnitude > 0.0000000001f)
            {
                transform.position = clamped;
                if (writeMonitor != null)
                {
                    writeMonitor.RecordWrite("WorkspaceLimiterLegacy");
                }
            }
        }
    }

    public void PreserveCurrentTargetPose()
    {
        preservedInitialPosition = transform.position;
        hasPreservedInitialPose = true;
    }

    public Vector3 ClampWorldPosition(Vector3 worldPosition)
    {
        ResolveRobotRoot();
        if (robotRoot == null)
        {
            return worldPosition;
        }

        Vector3 localPosition = robotRoot.InverseTransformPoint(worldPosition);
        localPosition = new Vector3(
            Mathf.Clamp(localPosition.x, minimumLocalPosition.x, maximumLocalPosition.x),
            Mathf.Clamp(localPosition.y, minimumLocalPosition.y, maximumLocalPosition.y),
            Mathf.Clamp(localPosition.z, minimumLocalPosition.z, maximumLocalPosition.z));

        if (keepAwayFromBase && baseKeepOutRadius > 0.0f)
        {
            Vector2 horizontalPosition = new Vector2(localPosition.x, localPosition.z);
            if (horizontalPosition.sqrMagnitude > 0.000001f)
            {
                lastHorizontalDirection = horizontalPosition.normalized;
            }

            if (horizontalPosition.magnitude < baseKeepOutRadius)
            {
                horizontalPosition = lastHorizontalDirection * baseKeepOutRadius;
                localPosition.x = horizontalPosition.x;
                localPosition.z = horizontalPosition.y;
            }
        }

        return robotRoot.TransformPoint(localPosition);
    }

    private void ResolveRobotRoot()
    {
        if (writeMonitor == null)
        {
            writeMonitor = GetComponent<TcpTargetWriteMonitor>();
        }

        if (robotRoot != null)
        {
            return;
        }

        GameObject foundRobot = GameObject.Find("ur5_robot");
        if (foundRobot != null)
        {
            robotRoot = foundRobot.transform;
        }

    }
}
