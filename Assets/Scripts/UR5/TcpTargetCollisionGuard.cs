using UnityEngine;

[DefaultExecutionOrder(125)]
public class TcpTargetCollisionGuard : MonoBehaviour
{
    [Header("References")]
    public Transform robotRoot;

    [Header("Obstacle Clearance")]
    public bool preventObstaclePenetration = true;
    public float clearanceRadius = 0.055f;
    public LayerMask obstacleLayers = ~0;

    private bool hasSafePosition;
    private Vector3 lastSafePosition;

    private void Awake()
    {
        ResolveRobotRoot();
        lastSafePosition = transform.position;
        hasSafePosition = true;
    }

    private void LateUpdate()
    {
        if (!preventObstaclePenetration)
        {
            return;
        }

        ResolveRobotRoot();
        Collider[] colliders = Physics.OverlapSphere(
            transform.position,
            clearanceRadius,
            obstacleLayers,
            QueryTriggerInteraction.Ignore);

        foreach (Collider collider in colliders)
        {
            if (IsIgnoredCollider(collider))
            {
                continue;
            }

            if (hasSafePosition)
            {
                transform.position = lastSafePosition;
            }

            return;
        }

        lastSafePosition = transform.position;
        hasSafePosition = true;
    }

    private bool IsIgnoredCollider(Collider collider)
    {
        Transform colliderTransform = collider.transform;
        if (colliderTransform == transform || colliderTransform.IsChildOf(transform))
        {
            return true;
        }

        return robotRoot != null && colliderTransform.IsChildOf(robotRoot);
    }

    private void ResolveRobotRoot()
    {
        if (robotRoot != null)
        {
            return;
        }

        GameObject robot = GameObject.Find("ur5_robot");
        if (robot != null)
        {
            robotRoot = robot.transform;
        }
    }
}
