using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

[DefaultExecutionOrder(25)]
public class Ur5ArticulationJointController : MonoBehaviour
{
    [Header("UR5 Root")]
    public Transform robotRoot;

    [Header("Joint Names")]
    public string[] jointNameHints =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    [Header("Drive")]
    public float stepDegreesPerSecond = 35.0f;
    public float stiffness = 12000.0f;
    public float damping = 5200.0f;
    public float forceLimit = 30000.0f;

    [Header("Smooth Drive Targets")]
    public bool smoothDriveTargets = true;
    public float maxDriveSpeedDegreesPerSecond = 240.0f;
    public float maxDriveAccelerationDegreesPerSecondSquared = 7000.0f;
    public float driveTargetToleranceDegrees = 0.005f;

    [Header("Closed-Loop Target Lead")]
    [Tooltip("启用后，Articulation Drive 的目标不会超过实际关节角太多。这样 IK 的连续更新不会把物理关节当成弹簧甩动。")]
    public bool limitDriveTargetLeadFromMeasuredJoint = true;
    [Tooltip("肩、肘关节允许的最大目标超前角度。该值限制物理误差，不限制正常的最高运动速度。")]
    public float maximumDriveTargetLeadDegrees = 3.0f;
    [Tooltip("腕部惯量较小、最容易摆动，因此使用更小的最大目标超前角度。")]
    public float maximumWristDriveTargetLeadDegrees = 1.15f;
    [Tooltip("Ready Pose 是已知、安全的关节轨迹，可使用更大的受控领先量以缩短回到初始抓取位的时间。")]
    public float readyPoseMaximumDriveTargetLeadDegrees = 6.0f;
    [Tooltip("Ready Pose 下腕部允许的最大受控领先量，仍小于肩、肘关节以避免腕部摆动。")]
    public float readyPoseMaximumWristDriveTargetLeadDegrees = 2.25f;

    [Header("Digital Twin Physics")]
    public bool fixBaseOnStart = true;
    public bool disableGravityForDigitalTwin = true;

    private readonly List<ArticulationBody> joints = new List<ArticulationBody>();
    private readonly List<float> jointTargets = new List<float>();
    private readonly List<float> appliedJointTargets = new List<float>();
    private readonly List<float> appliedJointVelocities = new List<float>();
    private int selectedJointIndex;
    private bool useReadyPoseDriveLeadProfile;

    public IReadOnlyList<ArticulationBody> Joints => joints;
    public IReadOnlyList<float> JointTargets => jointTargets;
    public int SelectedJointIndex => selectedJointIndex;
    public int JointCount => joints.Count;

    private void Awake()
    {
        if (robotRoot == null)
        {
            robotRoot = transform;
        }

        ConfigureDigitalTwinPhysics();
        FindJoints();
        ConfigureJointDrives();
    }

    private void OnValidate()
    {
        stepDegreesPerSecond = Mathf.Max(0.0f, stepDegreesPerSecond);
        stiffness = Mathf.Max(0.0f, stiffness);
        damping = Mathf.Max(0.0f, damping);
        forceLimit = Mathf.Max(0.0f, forceLimit);
        maxDriveSpeedDegreesPerSecond = Mathf.Max(0.0f, maxDriveSpeedDegreesPerSecond);
        maxDriveAccelerationDegreesPerSecondSquared = Mathf.Max(0.0f, maxDriveAccelerationDegreesPerSecondSquared);
        driveTargetToleranceDegrees = Mathf.Max(0.0f, driveTargetToleranceDegrees);
        maximumDriveTargetLeadDegrees = Mathf.Max(0.0f, maximumDriveTargetLeadDegrees);
        maximumWristDriveTargetLeadDegrees = Mathf.Max(0.0f, maximumWristDriveTargetLeadDegrees);
        readyPoseMaximumDriveTargetLeadDegrees = Mathf.Max(0.0f, readyPoseMaximumDriveTargetLeadDegrees);
        readyPoseMaximumWristDriveTargetLeadDegrees = Mathf.Max(0.0f, readyPoseMaximumWristDriveTargetLeadDegrees);
    }

    private void Update()
    {
        if (joints.Count == 0)
        {
            return;
        }

        for (int i = 0; i < Mathf.Min(6, joints.Count); i++)
        {
            KeyCode key = (KeyCode)((int)KeyCode.Alpha1 + i);
            if (Input.GetKeyDown(key))
            {
                selectedJointIndex = i;
                Debug.Log("Selected UR5 joint " + (i + 1) + ": " + joints[i].name);
            }
        }

        float direction = 0.0f;
        if (Input.GetKey(KeyCode.J)) direction -= 1.0f;
        if (Input.GetKey(KeyCode.L)) direction += 1.0f;

        if (Mathf.Abs(direction) > 0.0f)
        {
            jointTargets[selectedJointIndex] += direction * stepDegreesPerSecond * Time.deltaTime;
            ApplyJointTarget(selectedJointIndex);
        }

        if (Input.GetKeyDown(KeyCode.Home))
        {
            ResetTargets();
        }
    }

    public float GetJointTargetDegrees(int index)
    {
        if (index < 0 || index >= jointTargets.Count)
        {
            return 0.0f;
        }

        return jointTargets[index];
    }

    public float GetAppliedJointTargetDegrees(int index)
    {
        if (index < 0 || index >= appliedJointTargets.Count)
        {
            return 0.0f;
        }

        return appliedJointTargets[index];
    }

    public float[] GetJointTargetSnapshotDegrees()
    {
        float[] snapshot = new float[jointTargets.Count];
        for (int i = 0; i < jointTargets.Count; i++)
        {
            snapshot[i] = jointTargets[i];
        }

        return snapshot;
    }

    /// <summary>
    /// 切换已知 Ready Pose 专用的关节驱动档位。它仍采用实际关节反馈限幅，
    /// 仅放宽安全轨迹的允许领先量，不影响手动遥操作的精密与稳定性。
    /// </summary>
    public void SetReadyPoseDriveLeadProfile(bool enabled)
    {
        useReadyPoseDriveLeadProfile = enabled;
    }

    private void FixedUpdate()
    {
        if (!smoothDriveTargets)
        {
            return;
        }

        float deltaTime = Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        for (int i = 0; i < joints.Count; i++)
        {
            float remaining = jointTargets[i] - appliedJointTargets[i];
            if (Mathf.Abs(remaining) <= driveTargetToleranceDegrees)
            {
                appliedJointTargets[i] = jointTargets[i];
                appliedJointVelocities[i] = 0.0f;
                ApplyDriveTarget(i, jointTargets[i]);
                continue;
            }

            float desiredVelocity = Mathf.Clamp(
                remaining / deltaTime,
                -maxDriveSpeedDegreesPerSecond,
                maxDriveSpeedDegreesPerSecond);
            appliedJointVelocities[i] = Mathf.MoveTowards(
                appliedJointVelocities[i],
                desiredVelocity,
                maxDriveAccelerationDegreesPerSecondSquared * deltaTime);

            float nextTarget = appliedJointTargets[i] + appliedJointVelocities[i] * deltaTime;
            if (Mathf.Sign(remaining) != Mathf.Sign(jointTargets[i] - nextTarget)
                || Mathf.Abs(jointTargets[i] - nextTarget) < 0.0001f)
            {
                nextTarget = jointTargets[i];
                appliedJointVelocities[i] = 0.0f;
            }

            appliedJointTargets[i] = nextTarget;
            ApplyDriveTarget(i, nextTarget);
        }
    }

    public void SetJointTargetDegrees(int index, float targetDegrees)
    {
        SetJointTargetDegrees(index, targetDegrees, true);
    }

    public void SetJointTargetDegrees(int index, float targetDegrees, bool clampToDriveLimits)
    {
        if (index < 0 || index >= jointTargets.Count)
        {
            return;
        }

        jointTargets[index] = clampToDriveLimits ? ClampToDriveLimits(index, targetDegrees) : targetDegrees;
        if (!smoothDriveTargets)
        {
            appliedJointTargets[index] = jointTargets[index];
            ApplyDriveTarget(index, jointTargets[index]);
        }
    }

    public void SetJointTargetsDegrees(IReadOnlyList<float> targetDegrees, bool clampToDriveLimits)
    {
        SetJointTargetsDegrees(targetDegrees, targetDegrees != null ? targetDegrees.Count : 0, clampToDriveLimits, false);
    }

    public void SetJointTargetsDegrees(
        IReadOnlyList<float> targetDegrees,
        int targetCount,
        bool clampToDriveLimits,
        bool applyDirectlyToDrive)
    {
        if (targetDegrees == null)
        {
            return;
        }

        int count = Mathf.Min(Mathf.Min(targetCount, targetDegrees.Count), jointTargets.Count);
        for (int i = 0; i < count; i++)
        {
            jointTargets[i] = clampToDriveLimits ? ClampToDriveLimits(i, targetDegrees[i]) : targetDegrees[i];
        }

        if (!applyDirectlyToDrive && smoothDriveTargets)
        {
            return;
        }

        for (int i = 0; i < count; i++)
        {
            appliedJointTargets[i] = jointTargets[i];
            appliedJointVelocities[i] = 0.0f;
            ApplyDriveTarget(i, jointTargets[i]);
        }
    }

    public void AddJointTargetDegrees(int index, float deltaDegrees)
    {
        AddJointTargetDegrees(index, deltaDegrees, true);
    }

    public void AddJointTargetDegrees(int index, float deltaDegrees, bool clampToDriveLimits)
    {
        if (index < 0 || index >= jointTargets.Count)
        {
            return;
        }

        SetJointTargetDegrees(index, jointTargets[index] + deltaDegrees, clampToDriveLimits);
    }

    public void HoldCurrentJointPose()
    {
        for (int i = 0; i < joints.Count; i++)
        {
            float holdTargetDegrees = GetMeasuredJointDegrees(i, appliedJointTargets[i]);
            jointTargets[i] = holdTargetDegrees;
            appliedJointTargets[i] = holdTargetDegrees;
            appliedJointVelocities[i] = 0.0f;
            ApplyDriveTarget(i, holdTargetDegrees);
        }
    }

    public float GetMeasuredJointDegrees(int index)
    {
        return GetMeasuredJointDegrees(index, 0.0f);
    }

    private float GetMeasuredJointDegrees(int index, float fallbackDegrees)
    {
        if (index < 0 || index >= joints.Count)
        {
            return fallbackDegrees;
        }

        ArticulationBody joint = joints[index];
        return joint.jointPosition.dofCount > 0
            ? joint.jointPosition[0] * Mathf.Rad2Deg
            : fallbackDegrees;
    }

    private void ConfigureDigitalTwinPhysics()
    {
        ArticulationBody[] bodies = robotRoot.GetComponentsInChildren<ArticulationBody>();
        if (bodies.Length == 0)
        {
            Debug.LogWarning("No ArticulationBody components found under UR5 root.");
            return;
        }

        if (disableGravityForDigitalTwin)
        {
            foreach (ArticulationBody body in bodies)
            {
                body.useGravity = false;
            }
        }

        if (fixBaseOnStart)
        {
            int fixedRootCount = 0;
            foreach (ArticulationBody body in bodies)
            {
                if (!HasArticulationAncestor(body.transform))
                {
                    body.immovable = true;
                    fixedRootCount++;
                    Debug.Log("UR5 articulation root fixed: " + body.name);
                }
            }

            if (fixedRootCount == 0)
            {
                bodies[0].immovable = true;
                Debug.Log("UR5 fallback articulation root fixed: " + bodies[0].name);
            }
        }
    }

    private bool HasArticulationAncestor(Transform transformToCheck)
    {
        Transform current = transformToCheck.parent;
        while (current != null && current != robotRoot.parent)
        {
            if (current.GetComponent<ArticulationBody>() != null)
            {
                return true;
            }

            current = current.parent;
        }

        return false;
    }

    private void FindJoints()
    {
        joints.Clear();
        jointTargets.Clear();
        appliedJointTargets.Clear();
        appliedJointVelocities.Clear();

        ArticulationBody[] bodies = robotRoot.GetComponentsInChildren<ArticulationBody>();
        foreach (string hint in jointNameHints)
        {
            ArticulationBody match = null;

            foreach (ArticulationBody body in bodies)
            {
                if (MatchesJointHint(body, hint))
                {
                    match = body;
                    break;
                }
            }

            if (match != null && !joints.Contains(match))
            {
                joints.Add(match);
                jointTargets.Add(match.xDrive.target);
                appliedJointTargets.Add(match.xDrive.target);
                appliedJointVelocities.Add(0.0f);
            }
        }

        if (joints.Count == 0)
        {
            foreach (ArticulationBody body in bodies)
            {
                if (body.jointType == ArticulationJointType.RevoluteJoint
                    && !IsLikelyGripperJoint(body))
                {
                    joints.Add(body);
                    jointTargets.Add(body.xDrive.target);
                    appliedJointTargets.Add(body.xDrive.target);
                    appliedJointVelocities.Add(0.0f);
                }
            }
        }

        Debug.Log("UR5 joint controller found " + joints.Count + " articulation joints.");
    }

    private bool MatchesJointHint(ArticulationBody body, string hint)
    {
        string lowerHint = hint.ToLowerInvariant();
        string lowerName = body.name.ToLowerInvariant();
        if (NameMatchesHint(lowerName, lowerHint))
        {
            return true;
        }

        string jointName = ReadUrdfJointName(body);
        if (!string.IsNullOrEmpty(jointName)
            && NameMatchesHint(jointName.ToLowerInvariant(), lowerHint))
        {
            return true;
        }

        string linkAlias = LinkAliasForJoint(lowerHint);
        return !string.IsNullOrEmpty(linkAlias) && lowerName == linkAlias;
    }

    private bool NameMatchesHint(string lowerName, string lowerHint)
    {
        return lowerName == lowerHint
            || lowerName.Contains(lowerHint)
            || lowerHint.Contains(lowerName);
    }

    private string ReadUrdfJointName(ArticulationBody body)
    {
        foreach (MonoBehaviour component in body.GetComponents<MonoBehaviour>())
        {
            if (component == null)
            {
                continue;
            }

            FieldInfo field = component.GetType().GetField(
                "jointName",
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            if (field == null)
            {
                continue;
            }

            object value = field.GetValue(component);
            if (value is string jointName && !string.IsNullOrEmpty(jointName))
            {
                return jointName;
            }
        }

        return string.Empty;
    }

    private string LinkAliasForJoint(string lowerHint)
    {
        switch (lowerHint)
        {
            case "shoulder_pan_joint":
                return "shoulder_link";
            case "shoulder_lift_joint":
                return "upper_arm_link";
            case "elbow_joint":
                return "forearm_link";
            case "wrist_1_joint":
                return "wrist_1_link";
            case "wrist_2_joint":
                return "wrist_2_link";
            case "wrist_3_joint":
                return "wrist_3_link";
            default:
                return string.Empty;
        }
    }

    private bool IsLikelyGripperJoint(ArticulationBody body)
    {
        string lowerName = body.name.ToLowerInvariant();
        string jointName = ReadUrdfJointName(body).ToLowerInvariant();
        return IsLikelyGripperName(lowerName) || IsLikelyGripperName(jointName);
    }

    private bool IsLikelyGripperName(string lowerName)
    {
        return lowerName.Contains("driver")
            || lowerName.Contains("spring")
            || lowerName.Contains("follower")
            || lowerName.Contains("pad")
            || lowerName.Contains("robotiq");
    }

    private void ConfigureJointDrives()
    {
        for (int i = 0; i < joints.Count; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            drive.target = smoothDriveTargets ? appliedJointTargets[i] : jointTargets[i];
            joints[i].xDrive = drive;
        }
    }

    public void ApplyConfiguredDriveSettings()
    {
        ConfigureJointDrives();
    }

    private void ApplyJointTarget(int index)
    {
        ApplyDriveTarget(index, jointTargets[index]);
    }

    private void ApplyDriveTarget(int index, float targetDegrees)
    {
        ArticulationDrive drive = joints[index].xDrive;
        // 所有写入 Articulation Drive 的路径（IK、轨迹、按键和保持）统一经过
        // 同一闭环保护，避免某条旁路重新引入瞬时大关节误差。
        drive.target = ConstrainDriveTargetLead(index, targetDegrees);
        joints[index].xDrive = drive;
    }

    /// <summary>
    /// 将物理 Drive 的命令限制在测得关节角附近。IK 仍可按正常速度持续更新
    /// jointTargets；但当物理链条暂时跟不上时，不再累积很大的弹簧误差，避免
    /// 尤其是腕部在运动和停止阶段出现摆动。
    /// </summary>
    private float ConstrainDriveTargetLead(int jointIndex, float requestedTargetDegrees)
    {
        if (!limitDriveTargetLeadFromMeasuredJoint || jointIndex < 0 || jointIndex >= joints.Count)
        {
            return requestedTargetDegrees;
        }

        float measuredDegrees = GetMeasuredJointDegrees(jointIndex, requestedTargetDegrees);
        float maximumLead;
        if (useReadyPoseDriveLeadProfile)
        {
            maximumLead = jointIndex >= 3
                ? readyPoseMaximumWristDriveTargetLeadDegrees
                : readyPoseMaximumDriveTargetLeadDegrees;
        }
        else
        {
            maximumLead = jointIndex >= 3
                ? maximumWristDriveTargetLeadDegrees
                : maximumDriveTargetLeadDegrees;
        }
        maximumLead = Mathf.Max(driveTargetToleranceDegrees, maximumLead);
        return Mathf.Clamp(
            requestedTargetDegrees,
            measuredDegrees - maximumLead,
            measuredDegrees + maximumLead);
    }

    private float ClampToDriveLimits(int index, float targetDegrees)
    {
        ArticulationDrive drive = joints[index].xDrive;
        if (drive.lowerLimit < drive.upperLimit)
        {
            return Mathf.Clamp(targetDegrees, drive.lowerLimit, drive.upperLimit);
        }

        return targetDegrees;
    }

    private void ResetTargets()
    {
        for (int i = 0; i < jointTargets.Count; i++)
        {
            jointTargets[i] = 0.0f;
            if (!smoothDriveTargets)
            {
                appliedJointTargets[i] = jointTargets[i];
                ApplyDriveTarget(i, jointTargets[i]);
            }
        }

        Debug.Log("UR5 joint targets reset to zero.");
    }
}
