using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// Optional, throttled runtime snapshot for Quest teleoperation diagnosis.
/// Disabled by default and allocates only while explicitly enabled.
/// </summary>
public class Ur5TeleopDiagnostics : MonoBehaviour
{
    public bool enableDiagnostics;
    [Range(0.25f, 5.0f)] public float reportIntervalSeconds = 1.0f;
    public Ur5CartesianVelocityTeleopController teleop;
    public Ur5TcpTargetFollower follower;
    public Ur5JointTrajectoryPlayer trajectoryPlayer;
    public Ur5ArticulationJointController jointController;
    public TcpTargetWriteMonitor targetWriteMonitor;

    private float nextReportTime;

    private void Update()
    {
        if (!enableDiagnostics || Time.unscaledTime < nextReportTime)
        {
            return;
        }

        ResolveReferences();
        nextReportTime = Time.unscaledTime + reportIntervalSeconds;
        InputDevice right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        InputDevice left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        ReadButton(right, CommonUsages.primaryButton, out bool a);
        ReadButton(left, CommonUsages.primaryButton, out bool x);
        ReadAxis(right, CommonUsages.trigger, out float trigger);
        ReadAxis(right, CommonUsages.grip, out float rightGrip);
        ReadAxis(left, CommonUsages.grip, out float leftGrip);
        ReadJoystick(left, out Vector2 stick);

        float maximumRequestedJointLead = 0.0f;
        float maximumDriveLead = 0.0f;
        if (jointController != null)
        {
            for (int i = 0; i < jointController.JointCount; i++)
            {
                maximumRequestedJointLead = Mathf.Max(maximumRequestedJointLead, Mathf.Abs(
                    jointController.GetJointTargetDegrees(i) - jointController.GetMeasuredJointDegrees(i)));
                maximumDriveLead = Mathf.Max(maximumDriveLead, jointController.GetDriveTargetLeadDegrees(i));
            }
        }

        float commandToFiltered = teleop != null
            ? Vector3.Distance(teleop.LogicalCommandPosition, teleop.FilteredCommandPosition)
            : 0.0f;
        float filteredToConstrained = teleop != null
            ? Vector3.Distance(teleop.FilteredCommandPosition, teleop.ConstrainedCommandPosition)
            : 0.0f;
        float rotationFilterGapDegrees = teleop != null
            ? Quaternion.Angle(teleop.LogicalCommandRotation, teleop.FilteredCommandRotation)
            : 0.0f;
        bool isContinuous6Dof = teleop != null && teleop.EnableContinuous6DofClutch;
        string controlLabel = isContinuous6Dof ? "continuous6dof" : "legacy";
        // 单模式下保留 mode 字段但标成 legacy-free，避免后续日志分析误判为仍在做 free/fine/insert 切换。
        string modeLabel = teleop != null
            ? (isContinuous6Dof ? "legacy-free" : teleop.ActiveTeleopMode.ToString())
            : "n/a";

        Debug.Log(
            "UR5 teleop diag "
            + "fixedDt=" + Time.fixedDeltaTime.ToString("F4")
            + " approxHz=" + (1.0f / Mathf.Max(0.0001f, Time.fixedDeltaTime)).ToString("F1")
            + " control=" + controlLabel
            + " fault=" + (teleop != null ? teleop.Continuous6DofFaultReason.ToString() : "n/a")
            + " handDistanceM=" + (teleop != null ? teleop.ContinuousControllerDistanceMeters.ToString("F4") : "n/a")
            + " handAngleDeg=" + (teleop != null ? teleop.ContinuousControllerAngleDegrees.ToString("F2") : "n/a")
            + " translationGain=" + (teleop != null ? teleop.ContinuousTranslationGain.ToString("F3") : "n/a")
            + " rotationGain=" + (teleop != null ? teleop.ContinuousRotationGain.ToString("F3") : "n/a")
            + " devices=" + (right.isValid ? "R1" : "R0") + (left.isValid ? "L1" : "L0")
            + " grip=" + rightGrip.ToString("F2") + "/" + leftGrip.ToString("F2")
            + " A=" + (a ? "1" : "0") + " X=" + (x ? "1" : "0")
            + " trigger=" + trigger.ToString("F2") + " stick=" + stick.ToString("F2")
            + " rawHand=" + (teleop != null ? teleop.RawControllerPositionWorld.ToString("F4") : "n/a")
            + " rawHandRotation=" + (teleop != null ? teleop.RawControllerRotationWorld.ToString("F4") : "n/a")
            + " stableHand=" + (teleop != null ? teleop.StabilizedControllerPositionWorld.ToString("F4") : "n/a")
            + " handDelta=" + (teleop != null ? teleop.ControllerPositionInputDifferenceMeters.ToString("F4") : "n/a")
            + " gate=" + (teleop != null && teleop.IsControllerPositionNoiseGateHolding ? "hold" : "pass")
            + " strategy=" + (teleop != null && teleop.IsAnchoredPoseStrategyActive ? "anchored" : "idle")
            + " mode=" + modeLabel
            + " state=" + (teleop != null ? teleop.TeleopControllerState.ToString() : "n/a")
            + " smoothStep=" + (teleop != null ? teleop.ActiveAnchoredPoseSmoothingStep.ToString("F2") : "n/a")
            + " logical=" + (teleop != null ? teleop.LogicalCommandPosition.ToString("F4") : "n/a")
            + " filtered=" + (teleop != null ? teleop.FilteredCommandPosition.ToString("F4") : "n/a")
            + " constrained=" + (teleop != null ? teleop.ConstrainedCommandPosition.ToString("F4") : "n/a")
            + " actual=" + (follower != null ? follower.ControlPointPosition.ToString("F4") : "n/a")
            + " err=" + (follower != null ? follower.PositionError.ToString("F4") + "m/" + follower.RotationErrorDegrees.ToString("F2") + "deg" : "n/a")
            + " filterGap=" + commandToFiltered.ToString("F4") + "m"
            + " rotationFilterGapDeg=" + rotationFilterGapDegrees.ToString("F2")
            + " constrainGap=" + filteredToConstrained.ToString("F4") + "m"
            + " limit=" + (teleop != null && teleop.IsPreviewLeadLimited ? "lead" : "none")
            + " queue=" + (trajectoryPlayer != null ? trajectoryPlayer.PendingWaypointCount.ToString() : "n/a")
            + " requestedLead=" + maximumRequestedJointLead.ToString("F2")
            + " driveLead=" + maximumDriveLead.ToString("F2")
            + " ikLeadGate=" + (follower != null && follower.WasIkCommandLeadLimited ? "on" : "off")
            + " pivot=" + (follower != null ? follower.LastDlsMinimumPivot.ToString("F5") : "n/a")
            + " stationarySeconds=" + (follower != null ? follower.TargetStationarySeconds.ToString("F3") : "n/a")
            + " settledHold=" + (follower != null && follower.IsSettledTargetHoldActive ? "on" : "off")
            + " nearSingularity=" + (follower != null && follower.IsNearSingularity ? "on" : "off")
            + " ikFail=" + (follower != null ? follower.IkFailureCount.ToString() : "n/a")
            + " owner=" + (targetWriteMonitor != null ? targetWriteMonitor.LastWriter : "n/a")
            + " conflicts=" + (targetWriteMonitor != null ? targetWriteMonitor.WriteConflictCount.ToString() : "n/a"));
    }

    private void ResolveReferences()
    {
        if (teleop == null) teleop = GetComponent<Ur5CartesianVelocityTeleopController>();
        if (follower == null) follower = FindObjectOfType<Ur5TcpTargetFollower>();
        if (trajectoryPlayer == null) trajectoryPlayer = FindObjectOfType<Ur5JointTrajectoryPlayer>();
        if (jointController == null) jointController = FindObjectOfType<Ur5ArticulationJointController>();
        if (targetWriteMonitor == null && teleop != null) targetWriteMonitor = teleop.targetWriteMonitor;
    }

    private static void ReadButton(InputDevice device, InputFeatureUsage<bool> usage, out bool value)
    {
        value = device.isValid && device.TryGetFeatureValue(usage, out bool read) && read;
    }

    private static void ReadAxis(InputDevice device, InputFeatureUsage<float> usage, out float value)
    {
        value = device.isValid && device.TryGetFeatureValue(usage, out float read) ? read : 0.0f;
    }

    private static void ReadJoystick(InputDevice device, out Vector2 value)
    {
        value = device.isValid && device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 read)
            ? read
            : Vector2.zero;
    }
}
