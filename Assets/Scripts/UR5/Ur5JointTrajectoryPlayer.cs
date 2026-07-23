using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(10)]
public class Ur5JointTrajectoryPlayer : MonoBehaviour
{
    public enum QueueMode
    {
        PreserveOrder,
        LatestOnly
    }

    [Header("References")]
    public Ur5ArticulationJointController jointController;

    [Header("Trajectory Playback")]
    public bool play = true;
    public QueueMode queueMode = QueueMode.LatestOnly;
    [Tooltip("0 表示每个物理帧提交最新 IK 解，避免 72 Hz Quest 运行时隔帧跳变。")]
    public float jointAssignmentIntervalSeconds = 0.0f;
    [Tooltip("必须关闭：直接写 Articulation Drive 会跳过关节闭环平滑，造成腕部弹簧式摆动。")]
    public bool applyDirectlyToDrive = false;
    public bool clampToDriveLimits = true;
    public int maxQueuedWaypoints = 1;

    [Header("Debug")]
    public bool logStatus;

    private readonly Queue<float[]> pendingWaypoints = new Queue<float[]>();
    private float nextAssignmentTime;
    private float[] lastAssignedWaypointDegrees = new float[0];
    private bool loggedMissingJointController;

    public int PendingWaypointCount => pendingWaypoints.Count;
    public bool HasPendingWaypoint => pendingWaypoints.Count > 0;
    public float LastAssignmentTime { get; private set; }
    public IReadOnlyList<float> LastAssignedWaypointDegrees => lastAssignedWaypointDegrees;

    private void Awake()
    {
        ResolveReferences();
    }

    private void FixedUpdate()
    {
        if (!play)
        {
            return;
        }

        ResolveReferences();
        if (jointController == null || jointController.JointCount == 0)
        {
            LogMissingJointController();
            return;
        }

        if (!ShouldAssignNow())
        {
            return;
        }

        float[] waypoint = DequeueNextWaypoint();
        if (waypoint == null)
        {
            return;
        }

        jointController.SetJointTargetsDegrees(
            waypoint,
            waypoint.Length,
            clampToDriveLimits,
            applyDirectlyToDrive);

        lastAssignedWaypointDegrees = CopyWaypoint(waypoint, waypoint.Length);
        LastAssignmentTime = Time.time;
        nextAssignmentTime = Time.time + Mathf.Max(0.0f, jointAssignmentIntervalSeconds);

        if (logStatus)
        {
            Debug.Log("UR5 trajectory waypoint assigned. Pending=" + pendingWaypoints.Count);
        }
    }

    private void OnDisable()
    {
        ClearQueue();
    }

    private void OnValidate()
    {
        jointAssignmentIntervalSeconds = Mathf.Max(0.0f, jointAssignmentIntervalSeconds);
        maxQueuedWaypoints = Mathf.Max(1, maxQueuedWaypoints);
    }

    public void EnqueueWaypointDegrees(IReadOnlyList<float> waypointDegrees)
    {
        EnqueueWaypointDegrees(waypointDegrees, waypointDegrees != null ? waypointDegrees.Count : 0);
    }

    public void EnqueueWaypointDegrees(IReadOnlyList<float> waypointDegrees, int waypointCount)
    {
        if (waypointDegrees == null || waypointCount <= 0)
        {
            return;
        }

        ResolveReferences();
        int jointCount = jointController != null && jointController.JointCount > 0
            ? Mathf.Min(waypointCount, jointController.JointCount)
            : waypointCount;
        float[] waypoint = CopyWaypoint(waypointDegrees, jointCount);

        if (queueMode == QueueMode.LatestOnly)
        {
            pendingWaypoints.Clear();
        }

        pendingWaypoints.Enqueue(waypoint);
        TrimQueueToCapacity();
    }

    public void ClearQueue()
    {
        pendingWaypoints.Clear();
    }

    public void HoldCurrentJointPose()
    {
        ClearQueue();
        ResolveReferences();
        if (jointController != null)
        {
            jointController.HoldCurrentJointPose();
            lastAssignedWaypointDegrees = jointController.GetJointTargetSnapshotDegrees();
            LastAssignmentTime = Time.time;
        }
    }

    private bool ShouldAssignNow()
    {
        return pendingWaypoints.Count > 0
            && Time.time + 0.0001f >= nextAssignmentTime;
    }

    private float[] DequeueNextWaypoint()
    {
        if (pendingWaypoints.Count == 0)
        {
            return null;
        }

        float[] waypoint = pendingWaypoints.Dequeue();
        if (queueMode != QueueMode.LatestOnly)
        {
            return waypoint;
        }

        while (pendingWaypoints.Count > 0)
        {
            waypoint = pendingWaypoints.Dequeue();
        }

        return waypoint;
    }

    private void TrimQueueToCapacity()
    {
        int capacity = Mathf.Max(1, maxQueuedWaypoints);
        while (pendingWaypoints.Count > capacity)
        {
            pendingWaypoints.Dequeue();
        }
    }

    private float[] CopyWaypoint(IReadOnlyList<float> source, int count)
    {
        int safeCount = Mathf.Min(count, source.Count);
        float[] copy = new float[safeCount];
        for (int i = 0; i < safeCount; i++)
        {
            copy[i] = source[i];
        }

        return copy;
    }

    private void ResolveReferences()
    {
        if (jointController != null)
        {
            return;
        }

        jointController = GetComponent<Ur5ArticulationJointController>();
        if (jointController == null)
        {
            jointController = GetComponentInParent<Ur5ArticulationJointController>();
        }

        if (jointController == null)
        {
            jointController = FindObjectOfType<Ur5ArticulationJointController>();
        }
    }

    private void LogMissingJointController()
    {
        if (!logStatus || loggedMissingJointController)
        {
            return;
        }

        loggedMissingJointController = true;
        Debug.LogWarning("UR5 joint trajectory player is waiting for Ur5ArticulationJointController.");
    }
}
