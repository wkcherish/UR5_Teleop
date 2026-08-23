using UnityEngine;

/// <summary>
/// Implements the anchored relative-pose teleoperation loop used by
/// elpis-lab/UR10_Teleop: resume captures an input and tool pose together;
/// tracking always derives a global pose from that immutable pair; one
/// smoothing step is then applied to the commanded tool pose.
/// </summary>
public sealed class Ur5AnchoredPoseTeleopStrategy
{
    private Vector3 inputAnchorPosition;
    private Quaternion inputAnchorRotation = Quaternion.identity;
    private Vector3 toolAnchorPosition;
    private Quaternion toolAnchorRotation = Quaternion.identity;
    private Vector3 targetPosition;
    private Quaternion targetRotation = Quaternion.identity;

    public bool IsTracking { get; private set; }
    public Vector3 TargetPosition => targetPosition;
    public Quaternion TargetRotation => targetRotation;

    public bool Resume(
        Vector3 inputPosition,
        Quaternion inputRotation,
        Vector3 toolPosition,
        Quaternion toolRotation)
    {
        if (!IsFinite(inputPosition)
            || !IsValidRotation(inputRotation)
            || !IsFinite(toolPosition)
            || !IsValidRotation(toolRotation))
        {
            IsTracking = false;
            return false;
        }

        inputAnchorPosition = inputPosition;
        inputAnchorRotation = Normalize(inputRotation);
        toolAnchorPosition = toolPosition;
        toolAnchorRotation = Normalize(toolRotation);
        targetPosition = toolPosition;
        targetRotation = toolAnchorRotation;
        IsTracking = true;
        return true;
    }

    public void Pause()
    {
        IsTracking = false;
    }

    /// <summary>
    /// Re-establishes both anchors without changing the current command. This
    /// is used when a temporary position multiplier changes while Grip is held.
    /// </summary>
    public bool Rebase(
        Vector3 inputPosition,
        Quaternion inputRotation,
        Vector3 currentTargetPosition,
        Quaternion currentTargetRotation)
    {
        return Resume(inputPosition, inputRotation, currentTargetPosition, currentTargetRotation);
    }

    /// <summary>
    /// Updates the relative-pose anchors while preserving the filtered command.
    /// Workspace limits use this to discard hand overtravel at a boundary
    /// without making the commanded TCP pose jump to the boundary.
    /// </summary>
    public bool RebaseInputAnchorPreservingCommand(
        Vector3 inputPosition,
        Quaternion inputRotation,
        Vector3 constrainedToolPosition,
        Quaternion constrainedToolRotation)
    {
        if (!IsFinite(inputPosition)
            || !IsValidRotation(inputRotation)
            || !IsFinite(constrainedToolPosition)
            || !IsValidRotation(constrainedToolRotation))
        {
            return false;
        }

        inputAnchorPosition = inputPosition;
        inputAnchorRotation = Normalize(inputRotation);
        toolAnchorPosition = constrainedToolPosition;
        toolAnchorRotation = Normalize(constrainedToolRotation);
        IsTracking = true;
        return true;
    }

    public bool TryGetRequestedPose(
        Vector3 inputPosition,
        Quaternion inputRotation,
        Vector3 positionMapping,
        out Vector3 requestedPosition,
        out Quaternion requestedRotation)
    {
        requestedPosition = targetPosition;
        requestedRotation = targetRotation;
        if (!IsTracking
            || !IsFinite(inputPosition)
            || !IsValidRotation(inputRotation)
            || !IsFinite(positionMapping))
        {
            return false;
        }

        Quaternion relativeRotation = Normalize(inputRotation)
            * Quaternion.Inverse(inputAnchorRotation);
        return TryGetRequestedPose(
            inputPosition,
            inputRotation,
            positionMapping,
            relativeRotation * toolAnchorRotation,
            out requestedPosition,
            out requestedRotation);
    }

    public bool TryGetRequestedPose(
        Vector3 inputPosition,
        Quaternion inputRotation,
        Vector3 positionMapping,
        Quaternion requestedToolRotation,
        out Vector3 requestedPosition,
        out Quaternion requestedRotation)
    {
        requestedPosition = targetPosition;
        requestedRotation = targetRotation;
        if (!IsTracking
            || !IsFinite(inputPosition)
            || !IsValidRotation(inputRotation)
            || !IsFinite(positionMapping)
            || !IsValidRotation(requestedToolRotation))
        {
            return false;
        }

        Vector3 relativeTranslation = inputPosition - inputAnchorPosition;
        requestedPosition = toolAnchorPosition + Vector3.Scale(relativeTranslation, positionMapping);
        requestedRotation = Normalize(requestedToolRotation);
        return true;
    }

    /// <summary>
    /// The sole command filter. This is equivalent to the upstream tracker:
    /// target += step * (requested - target), with quaternion SLERP for pose.
    /// </summary>
    public bool FilterRequestedPose(
        Vector3 requestedPosition,
        Quaternion requestedRotation,
        float smoothingStep,
        out Vector3 filteredPosition,
        out Quaternion filteredRotation)
    {
        filteredPosition = targetPosition;
        filteredRotation = targetRotation;
        if (!IsTracking
            || !IsFinite(requestedPosition)
            || !IsValidRotation(requestedRotation)
            || !IsFinite(smoothingStep))
        {
            return false;
        }

        float step = Mathf.Clamp01(smoothingStep);
        targetPosition += step * (requestedPosition - targetPosition);
        targetRotation = Quaternion.Slerp(targetRotation, Normalize(requestedRotation), step);
        filteredPosition = targetPosition;
        filteredRotation = targetRotation;
        return true;
    }

    public void SetCommandPose(Vector3 position, Quaternion rotation)
    {
        if (!IsFinite(position) || !IsValidRotation(rotation))
        {
            return;
        }

        targetPosition = position;
        targetRotation = Normalize(rotation);
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static bool IsValidRotation(Quaternion value)
    {
        return IsFinite(value.x)
            && IsFinite(value.y)
            && IsFinite(value.z)
            && IsFinite(value.w)
            && SquaredMagnitude(value) > 0.000001f;
    }

    private static Quaternion Normalize(Quaternion value)
    {
        float inverseMagnitude = 1.0f / Mathf.Sqrt(SquaredMagnitude(value));
        return new Quaternion(
            value.x * inverseMagnitude,
            value.y * inverseMagnitude,
            value.z * inverseMagnitude,
            value.w * inverseMagnitude);
    }

    private static float SquaredMagnitude(Quaternion value)
    {
        return value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w;
    }
}
