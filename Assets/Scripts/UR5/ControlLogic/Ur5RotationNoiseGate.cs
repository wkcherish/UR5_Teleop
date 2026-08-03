using UnityEngine;

/// <summary>
/// Rejects stationary controller rotation jitter without adding time-based lag.
/// The accepted orientation remains inside an angular deadband of the raw input
/// and moves immediately once deliberate rotation leaves that band.
/// </summary>
public sealed class Ur5RotationNoiseGate
{
    private bool isInitialized;
    private Quaternion acceptedRotation = Quaternion.identity;

    public bool IsInitialized => isInitialized;
    public Quaternion AcceptedRotation => acceptedRotation;

    public void Reset(Quaternion rotation)
    {
        acceptedRotation = Ur5RelativePoseClutchMapper.IsValidRotation(rotation)
            ? Ur5RelativePoseClutchMapper.Normalize(rotation)
            : Quaternion.identity;
        isInitialized = true;
    }

    public void Clear()
    {
        isInitialized = false;
    }

    public Quaternion Filter(Quaternion rawRotation, float deadbandDegrees)
    {
        if (!Ur5RelativePoseClutchMapper.IsValidRotation(rawRotation))
        {
            return acceptedRotation;
        }

        Quaternion normalizedRaw = Ur5RelativePoseClutchMapper.Normalize(rawRotation);
        if (!isInitialized)
        {
            Reset(normalizedRaw);
            return acceptedRotation;
        }

        Quaternion delta = normalizedRaw * Quaternion.Inverse(acceptedRotation);
        delta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        float absoluteAngle = Mathf.Abs(angleDegrees);
        float deadband = Mathf.Max(0.0f, deadbandDegrees);
        if (absoluteAngle <= deadband || axis.sqrMagnitude < 0.000001f)
        {
            return acceptedRotation;
        }

        float acceptedStepDegrees = Mathf.Sign(angleDegrees) * (absoluteAngle - deadband);
        acceptedRotation = Ur5RelativePoseClutchMapper.Normalize(
            Quaternion.AngleAxis(acceptedStepDegrees, axis.normalized) * acceptedRotation);
        return acceptedRotation;
    }
}
