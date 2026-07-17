using System;
using UnityEngine;

/// <summary>
/// Maps a tracked VR controller pose to a robot TCP target using professional
/// teleoperation mechanics: clutching, delta motion, EMA smoothing, workspace
/// limiting, and previous-joint-angle IK seeding.
///
/// This script intentionally does not read Quest input directly. Feed the clutch
/// state from your preferred input layer by calling SetClutchInput(bool), e.g.
/// from XR Interaction Toolkit, Unity Input System, or a Quest XRNode reader.
/// </summary>
public class VRTeleoperationController : MonoBehaviour
{
    public delegate float[] IkSolverDelegate(
        Vector3 targetPos,
        Quaternion targetRot,
        float[] previousJointAngles);

    [Header("Scene References")]
    public Transform robotBaseTransform;
    public Transform tcpTargetTransform;
    public Transform vrController;

    [Header("EMA Smoothing")]
    [Range(0.0f, 1.0f)]
    public float movementAlpha = 0.15f;

    [Range(0.0f, 1.0f)]
    public float rotationAlpha = 0.15f;

    [Header("Workspace Limit")]
    [Min(0.01f)]
    public float maxReachRadius = 0.8f;

    /// <summary>
    /// Optional hook for an external IK solver. The solver should use
    /// previousJointAngles as its seed/hint and return the solved joint angles
    /// for use as the next frame's seed.
    /// </summary>
    public event IkSolverDelegate OnSolveIKRequested;

    private bool clutchInput;
    private bool isClutched;
    private bool hasSmoothedPose;
    private bool warnedMissingReferences;

    private Vector3 vrStartPosition;
    private Quaternion vrStartRotation;
    private Vector3 tcpStartPosition;
    private Quaternion tcpStartRotation;

    private Vector3 smoothedTcpPosition;
    private Quaternion smoothedTcpRotation;
    private float[] previousJointAngles = Array.Empty<float>();

    public bool IsClutched => isClutched;
    public Vector3 SmoothedTcpPosition => smoothedTcpPosition;
    public Quaternion SmoothedTcpRotation => smoothedTcpRotation;
    public float[] PreviousJointAngles => previousJointAngles;

    /// <summary>
    /// Call this from your Quest input code. For example:
    /// SetClutchInput(rightGripPressed) or SetClutchInput(rightTriggerPressed).
    /// </summary>
    public void SetClutchInput(bool pressed)
    {
        clutchInput = pressed;
    }

    /// <summary>
    /// Allows an external robot state reader to refresh the current joint-angle
    /// seed, e.g. from ArticulationBody positions, RTDE feedback, or a simulator.
    /// </summary>
    public void SetPreviousJointAngles(float[] jointAngles)
    {
        previousJointAngles = jointAngles ?? Array.Empty<float>();
    }

    private void Awake()
    {
        if (tcpTargetTransform != null)
        {
            smoothedTcpPosition = tcpTargetTransform.position;
            smoothedTcpRotation = tcpTargetTransform.rotation;
            hasSmoothedPose = true;
        }
    }

    private void Update()
    {
        if (!HasRequiredReferences())
        {
            WarnMissingReferencesOnce();
            return;
        }

        if (clutchInput && !isClutched)
        {
            BeginClutch();
        }
        else if (!clutchInput && isClutched)
        {
            EndClutch();
        }

        if (isClutched)
        {
            UpdateTcpTargetFromControllerDelta();
        }
    }

    private bool HasRequiredReferences()
    {
        return robotBaseTransform != null
            && tcpTargetTransform != null
            && vrController != null;
    }

    private void WarnMissingReferencesOnce()
    {
        if (warnedMissingReferences)
        {
            return;
        }

        warnedMissingReferences = true;
        Debug.LogWarning(
            "VRTeleoperationController needs robotBaseTransform, tcpTargetTransform, and vrController assigned.",
            this);
    }

    private void BeginClutch()
    {
        isClutched = true;

        // Capture both frames at the clutch edge. All subsequent motion is
        // incremental relative to this stable start pose, not absolute mapped.
        vrStartPosition = vrController.position;
        vrStartRotation = vrController.rotation;
        tcpStartPosition = tcpTargetTransform.position;
        tcpStartRotation = tcpTargetTransform.rotation;

        smoothedTcpPosition = tcpStartPosition;
        smoothedTcpRotation = tcpStartRotation;
        hasSmoothedPose = true;
    }

    private void EndClutch()
    {
        isClutched = false;
        hasSmoothedPose = false;
    }

    private void UpdateTcpTargetFromControllerDelta()
    {
        Vector3 rawTargetPosition = CalculateRawDeltaPosition();
        Quaternion rawTargetRotation = CalculateRawDeltaRotation();

        rawTargetPosition = ClampToReachRadius(rawTargetPosition);
        ApplyEma(rawTargetPosition, rawTargetRotation);

        tcpTargetTransform.SetPositionAndRotation(smoothedTcpPosition, smoothedTcpRotation);

        // External IK solvers should use previousJointAngles as the seed/hint.
        // This is the key to avoiding configuration jumps such as elbow-up to
        // elbow-down flips when multiple IK solutions exist.
        previousJointAngles = SolveIK(
            smoothedTcpPosition,
            smoothedTcpRotation,
            previousJointAngles) ?? previousJointAngles;
    }

    private Vector3 CalculateRawDeltaPosition()
    {
        Vector3 vrPositionDelta = vrController.position - vrStartPosition;
        return tcpStartPosition + vrPositionDelta;
    }

    private Quaternion CalculateRawDeltaRotation()
    {
        Quaternion vrRotationDelta = vrController.rotation * Quaternion.Inverse(vrStartRotation);
        return vrRotationDelta * tcpStartRotation;
    }

    private Vector3 ClampToReachRadius(Vector3 targetWorldPosition)
    {
        Vector3 baseToTarget = targetWorldPosition - robotBaseTransform.position;
        float sqrReach = maxReachRadius * maxReachRadius;

        if (baseToTarget.sqrMagnitude <= sqrReach)
        {
            return targetWorldPosition;
        }

        return robotBaseTransform.position + baseToTarget.normalized * maxReachRadius;
    }

    private void ApplyEma(Vector3 rawPosition, Quaternion rawRotation)
    {
        float positionAlpha = Mathf.Clamp01(movementAlpha);
        float orientationAlpha = Mathf.Clamp01(rotationAlpha);

        if (!hasSmoothedPose)
        {
            smoothedTcpPosition = rawPosition;
            smoothedTcpRotation = rawRotation;
            hasSmoothedPose = true;
            return;
        }

        smoothedTcpPosition =
            positionAlpha * rawPosition + (1.0f - positionAlpha) * smoothedTcpPosition;
        smoothedTcpRotation =
            Quaternion.Slerp(smoothedTcpRotation, rawRotation, orientationAlpha);
    }

    /// <summary>
    /// Dummy IK interface. Override this method in a subclass or subscribe to
    /// OnSolveIKRequested. The important contract is:
    ///
    /// targetPos/targetRot = desired TCP pose.
    /// previousJointAngles = seed/hint from the previous frame.
    /// return value = solved joint angles to reuse as the next frame's seed.
    /// </summary>
    public virtual float[] SolveIK(
        Vector3 targetPos,
        Quaternion targetRot,
        float[] previousJointAngles)
    {
        return OnSolveIKRequested != null
            ? OnSolveIKRequested.Invoke(targetPos, targetRot, previousJointAngles)
            : previousJointAngles;
    }

    private void OnValidate()
    {
        movementAlpha = Mathf.Clamp01(movementAlpha);
        rotationAlpha = Mathf.Clamp01(rotationAlpha);
        maxReachRadius = Mathf.Max(0.01f, maxReachRadius);
    }

    private void OnDrawGizmosSelected()
    {
        if (robotBaseTransform == null)
        {
            return;
        }

        Gizmos.color = new Color(0.2f, 0.8f, 1.0f, 0.35f);
        Gizmos.DrawWireSphere(robotBaseTransform.position, maxReachRadius);
    }
}
