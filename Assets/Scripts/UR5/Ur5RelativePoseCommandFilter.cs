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

    public bool FilterByTimeConstants(
        Vector3 targetPosition,
        Quaternion targetRotation,
        float positionTimeConstantSeconds,
        float rotationTimeConstantSeconds,
        float deltaTimeSeconds,
        float maxLinearSpeedMetersPerSecond,
        float maxAngularSpeedDegreesPerSecond,
        out Vector3 position,
        out Quaternion rotation)
    {
        if (!IsFinite(targetPosition)
            || !IsValidRotation(targetRotation)
            || !IsFinite(positionTimeConstantSeconds)
            || !IsFinite(rotationTimeConstantSeconds)
            || !IsFinite(deltaTimeSeconds)
            || !IsFinite(maxLinearSpeedMetersPerSecond)
            || !IsFinite(maxAngularSpeedDegreesPerSecond))
        {
            position = filteredPosition;
            rotation = filteredRotation;
            return false;
        }

        targetRotation = Normalize(targetRotation);
        if (!isInitialized)
        {
            Reset(targetPosition, targetRotation);
            position = filteredPosition;
            rotation = filteredRotation;
            return true;
        }

        float dt = Mathf.Max(0.000001f, deltaTimeSeconds);
        float positionAlpha = 1.0f - Mathf.Exp(
            -dt / Mathf.Max(0.001f, positionTimeConstantSeconds));
        float rotationAlpha = 1.0f - Mathf.Exp(
            -dt / Mathf.Max(0.001f, rotationTimeConstantSeconds));

        // 由秒级时间常数计算本帧 alpha，保证 72/90/120 Hz 下滤波响应一致。
        Vector3 smoothedPosition =
            Vector3.Lerp(filteredPosition, targetPosition, positionAlpha);
        Quaternion smoothedRotation =
            Quaternion.Slerp(filteredRotation, targetRotation, rotationAlpha);

        // 硬速度限幅放在时间滤波之后，安全层只裁剪本帧最大位移/角度，
        // 不再引入第二级平滑或松手后的追赶轨迹。
        filteredPosition = maxLinearSpeedMetersPerSecond > 0.0f
            ? Vector3.MoveTowards(
                filteredPosition,
                smoothedPosition,
                maxLinearSpeedMetersPerSecond * dt)
            : smoothedPosition;
        filteredRotation = maxAngularSpeedDegreesPerSecond > 0.0f
            ? Quaternion.RotateTowards(
                filteredRotation,
                smoothedRotation,
                maxAngularSpeedDegreesPerSecond * dt)
            : smoothedRotation;

        position = filteredPosition;
        rotation = filteredRotation;
        return true;
    }

    private static Quaternion Normalize(Quaternion value)
    {
        float magnitude = Mathf.Sqrt(
            value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w);
        return new Quaternion(
            value.x / magnitude,
            value.y / magnitude,
            value.z / magnitude,
            value.w / magnitude);
    }

    private static bool IsValidRotation(Quaternion value)
    {
        float sqrMagnitude =
            value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w;
        return IsFinite(value.x)
            && IsFinite(value.y)
            && IsFinite(value.z)
            && IsFinite(value.w)
            && sqrMagnitude > 0.000001f;
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
