using System;
using UnityEngine;

/// <summary>
/// UR5 遥操作的离线位姿数学测试台。它不依赖场景对象，方便 EditMode
/// 测试单独定位坐标系转换和四元数乘法方向问题。
/// </summary>
public static class Ur5PoseMathTestBench
{
    private const float MinimumQuaternionSqrMagnitude = 0.000001f;

    public readonly struct FramePose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public FramePose(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = IsValidRotation(rotation) ? Normalize(rotation) : rotation;
        }

        public Vector3 TransformVector(Vector3 localVector)
        {
            return Rotation * localVector;
        }

        public Vector3 InverseTransformVector(Vector3 worldVector)
        {
            return Quaternion.Inverse(Rotation) * worldVector;
        }

        public Vector3 TransformPoint(Vector3 localPoint)
        {
            return Position + TransformVector(localPoint);
        }

        public Vector3 InverseTransformPoint(Vector3 worldPoint)
        {
            return InverseTransformVector(worldPoint - Position);
        }
    }

    public readonly struct ClutchSnapshot
    {
        public readonly FramePose XrOriginWorld;
        public readonly FramePose RobotBaseWorld;
        public readonly FramePose ControllerWorld;
        public readonly FramePose TcpWorld;

        public ClutchSnapshot(
            FramePose xrOriginWorld,
            FramePose robotBaseWorld,
            FramePose controllerWorld,
            FramePose tcpWorld)
        {
            XrOriginWorld = xrOriginWorld;
            RobotBaseWorld = robotBaseWorld;
            ControllerWorld = controllerWorld;
            TcpWorld = tcpWorld;
        }
    }

    public struct MappingSettings
    {
        public float PositionScale;
        public float RotationScale;

        public MappingSettings(float positionScale, float rotationScale)
        {
            PositionScale = positionScale;
            RotationScale = rotationScale;
        }

        public static MappingSettings Default => new MappingSettings(1.0f, 1.0f);
    }

    public readonly struct Sample
    {
        public readonly Vector3 ControllerWorldDelta;
        public readonly Vector3 ControllerXrOriginDelta;
        public readonly Vector3 ControllerRobotBaseDelta;
        public readonly Quaternion ControllerRotationDelta;
        public readonly float ControllerRotationDeltaDegrees;
        public readonly Vector3 MappedTcpWorldDelta;
        public readonly Vector3 MappedTcpRobotBaseDelta;
        public readonly Quaternion MappedTcpRotationDelta;
        public readonly FramePose FinalTargetWorld;

        public Sample(
            Vector3 controllerWorldDelta,
            Vector3 controllerXrOriginDelta,
            Vector3 controllerRobotBaseDelta,
            Quaternion controllerRotationDelta,
            float controllerRotationDeltaDegrees,
            Vector3 mappedTcpWorldDelta,
            Vector3 mappedTcpRobotBaseDelta,
            Quaternion mappedTcpRotationDelta,
            FramePose finalTargetWorld)
        {
            ControllerWorldDelta = controllerWorldDelta;
            ControllerXrOriginDelta = controllerXrOriginDelta;
            ControllerRobotBaseDelta = controllerRobotBaseDelta;
            ControllerRotationDelta = controllerRotationDelta;
            ControllerRotationDeltaDegrees = controllerRotationDeltaDegrees;
            MappedTcpWorldDelta = mappedTcpWorldDelta;
            MappedTcpRobotBaseDelta = mappedTcpRobotBaseDelta;
            MappedTcpRotationDelta = mappedTcpRotationDelta;
            FinalTargetWorld = finalTargetWorld;
        }
    }

    public static Sample Evaluate(
        ClutchSnapshot clutch,
        FramePose currentControllerWorld,
        MappingSettings settings)
    {
        if (!TryEvaluate(clutch, currentControllerWorld, settings, out Sample sample))
        {
            throw new ArgumentException("Invalid pose-math test bench input.");
        }

        return sample;
    }

    public static bool TryEvaluate(
        ClutchSnapshot clutch,
        FramePose currentControllerWorld,
        MappingSettings settings,
        out Sample sample)
    {
        sample = CreateStoppedSample(clutch);
        if (!IsFinite(clutch)
            || !IsFinite(currentControllerWorld)
            || !IsFinite(settings.PositionScale)
            || !IsFinite(settings.RotationScale))
        {
            return false;
        }

        Vector3 controllerWorldDelta = currentControllerWorld.Position - clutch.ControllerWorld.Position;
        Vector3 controllerXrOriginDelta = clutch.XrOriginWorld.InverseTransformVector(controllerWorldDelta);

        // robot_base delta 是所有 TCP 平移映射的唯一输入，避免 XR Origin 或父级层级误差泄漏进机器人坐标系。
        Vector3 controllerRobotBaseDelta = clutch.RobotBaseWorld.InverseTransformVector(controllerWorldDelta);
        Vector3 mappedTcpRobotBaseDelta = controllerRobotBaseDelta * Mathf.Max(0.0f, settings.PositionScale);
        Vector3 mappedTcpWorldDelta = clutch.RobotBaseWorld.TransformVector(mappedTcpRobotBaseDelta);

        Quaternion controllerRotationDelta = Normalize(currentControllerWorld.Rotation)
            * Quaternion.Inverse(clutch.ControllerWorld.Rotation);
        float rotationScale = Mathf.Max(0.0f, settings.RotationScale);

        // 左乘约定：worldDelta * tcpStart 表示在世界坐标中施加手柄相对旋转，再得到目标 TCP 姿态。
        // RotationScale 为 0 时只允许平移，TCP 姿态锁定在离合起点。
        Quaternion mappedTcpRotationDelta = ScaleWorldRotationDelta(controllerRotationDelta, rotationScale);
        FramePose finalTargetWorld = new FramePose(
            clutch.TcpWorld.Position + mappedTcpWorldDelta,
            mappedTcpRotationDelta * clutch.TcpWorld.Rotation);

        sample = new Sample(
            controllerWorldDelta,
            controllerXrOriginDelta,
            controllerRobotBaseDelta,
            controllerRotationDelta,
            GetSignedAngleDegrees(controllerRotationDelta),
            mappedTcpWorldDelta,
            mappedTcpRobotBaseDelta,
            mappedTcpRotationDelta,
            finalTargetWorld);

        return IsFinite(finalTargetWorld);
    }

    public static bool IsFinite(FramePose value)
    {
        return IsFinite(value.Position) && IsValidRotation(value.Rotation);
    }

    public static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    public static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    public static bool IsValidRotation(Quaternion value)
    {
        return IsFinite(value.x)
            && IsFinite(value.y)
            && IsFinite(value.z)
            && IsFinite(value.w)
            && SquaredMagnitude(value) > MinimumQuaternionSqrMagnitude;
    }

    public static Quaternion Normalize(Quaternion value)
    {
        if (!IsValidRotation(value))
        {
            return Quaternion.identity;
        }

        float inverseMagnitude = 1.0f / Mathf.Sqrt(SquaredMagnitude(value));
        return new Quaternion(
            value.x * inverseMagnitude,
            value.y * inverseMagnitude,
            value.z * inverseMagnitude,
            value.w * inverseMagnitude);
    }

    private static bool IsFinite(ClutchSnapshot value)
    {
        return IsFinite(value.XrOriginWorld)
            && IsFinite(value.RobotBaseWorld)
            && IsFinite(value.ControllerWorld)
            && IsFinite(value.TcpWorld);
    }

    private static Sample CreateStoppedSample(ClutchSnapshot clutch)
    {
        FramePose finalTargetWorld = IsFinite(clutch.TcpWorld)
            ? clutch.TcpWorld
            : new FramePose(Vector3.zero, Quaternion.identity);

        return new Sample(
            Vector3.zero,
            Vector3.zero,
            Vector3.zero,
            Quaternion.identity,
            0.0f,
            Vector3.zero,
            Vector3.zero,
            Quaternion.identity,
            finalTargetWorld);
    }

    private static Quaternion ScaleWorldRotationDelta(Quaternion rotationDelta, float scale)
    {
        rotationDelta = Normalize(rotationDelta);
        rotationDelta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180.0f)
        {
            angleDegrees -= 360.0f;
        }

        if (axis.sqrMagnitude < MinimumQuaternionSqrMagnitude
            || Mathf.Abs(angleDegrees) <= 0.0001f
            || scale <= 0.0f)
        {
            return Quaternion.identity;
        }

        return Quaternion.AngleAxis(angleDegrees * scale, axis.normalized);
    }

    private static float GetSignedAngleDegrees(Quaternion rotationDelta)
    {
        rotationDelta = Normalize(rotationDelta);
        rotationDelta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (axis.sqrMagnitude < MinimumQuaternionSqrMagnitude)
        {
            return 0.0f;
        }

        return angleDegrees > 180.0f ? angleDegrees - 360.0f : angleDegrees;
    }

    private static float SquaredMagnitude(Quaternion value)
    {
        return value.x * value.x
            + value.y * value.y
            + value.z * value.z
            + value.w * value.w;
    }
}
