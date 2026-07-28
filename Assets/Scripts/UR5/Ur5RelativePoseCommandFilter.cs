using UnityEngine;

/// <summary>Single, elapsed-time-invariant filter for final relative-pose commands.</summary>
public sealed class Ur5RelativePoseCommandFilter
{
    private bool isInitialized;
    private Vector3 filteredPosition;
    private Quaternion filteredRotation = Quaternion.identity;

    public void Reset(Vector3 position, Quaternion rotation)
    {
        filteredPosition = position;
        filteredRotation = rotation;
        isInitialized = true;
    }

    public void Clear() => isInitialized = false;

    public void Filter(Vector3 targetPosition, Quaternion targetRotation, float retention,
        float deltaTime, float referenceUpdateRateHz, out Vector3 position, out Quaternion rotation)
    {
        if (!isInitialized)
        {
            Reset(targetPosition, targetRotation);
        }

        float referenceStep = 1.0f / Mathf.Max(1.0f, referenceUpdateRateHz);
        float updateWeight = 1.0f - Mathf.Pow(Mathf.Clamp01(retention),
            Mathf.Max(0.000001f, deltaTime) / referenceStep);
        filteredPosition = Vector3.Lerp(filteredPosition, targetPosition, updateWeight);
        filteredRotation = Quaternion.Slerp(filteredRotation, targetRotation, updateWeight);
        position = filteredPosition;
        rotation = filteredRotation;
    }
}
