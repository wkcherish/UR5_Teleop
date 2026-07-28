using UnityEngine;

public sealed class Ur5RelativePoseClutchMapper
{
    private Vector3 handPositionAtClutch;
    private Quaternion handRotationAtClutch = Quaternion.identity;
    private Vector3 targetPositionAtClutch;
    private Quaternion targetRotationAtClutch = Quaternion.identity;

    public bool IsActive { get; private set; }

    public bool Begin(Vector3 handPosition, Quaternion handRotation, Vector3 targetPosition, Quaternion targetRotation)
    {
        if (!IsFinite(handPosition) || !IsValidRotation(handRotation)
            || !IsFinite(targetPosition) || !IsValidRotation(targetRotation))
        {
            IsActive = false;
            return false;
        }

        handPositionAtClutch = handPosition;
        handRotationAtClutch = Normalize(handRotation);
        targetPositionAtClutch = targetPosition;
        targetRotationAtClutch = Normalize(targetRotation);
        IsActive = true;
        return true;
    }

    public void End() => IsActive = false;

    public bool Rebase(Vector3 handPosition, Quaternion handRotation, Vector3 currentTargetPosition, Quaternion currentTargetRotation)
    {
        return Begin(handPosition, handRotation, currentTargetPosition, currentTargetRotation);
    }

    public bool TryMap(Vector3 handPosition, Quaternion handRotation, float positionScale, float rotationScale,
        out Vector3 targetPosition, out Quaternion targetRotation)
    {
        targetPosition = targetPositionAtClutch;
        targetRotation = targetRotationAtClutch;
        if (!IsActive || !IsFinite(handPosition) || !IsValidRotation(handRotation)
            || !IsFinite(positionScale) || !IsFinite(rotationScale))
        {
            return false;
        }

        targetPosition = targetPositionAtClutch
            + (handPosition - handPositionAtClutch) * Mathf.Max(0.0f, positionScale);
        Quaternion relativeRotation = Normalize(handRotation) * Quaternion.Inverse(handRotationAtClutch);
        relativeRotation.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        targetRotation = axis.sqrMagnitude < 0.000001f
            ? targetRotationAtClutch
            : Quaternion.AngleAxis(angleDegrees * Mathf.Max(0.0f, rotationScale), axis.normalized)
                * targetRotationAtClutch;
        return IsFinite(targetPosition) && IsValidRotation(targetRotation);
    }

    public static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    public static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);

    public static bool IsValidRotation(Quaternion value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w)
            && SquaredMagnitude(value) > 0.000001f;
    }

    public static Quaternion Normalize(Quaternion value)
    {
        float magnitude = Mathf.Sqrt(SquaredMagnitude(value));
        return magnitude > 0.000001f
            ? new Quaternion(value.x / magnitude, value.y / magnitude, value.z / magnitude, value.w / magnitude)
            : Quaternion.identity;
    }

    private static float SquaredMagnitude(Quaternion value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w;
    }
}
