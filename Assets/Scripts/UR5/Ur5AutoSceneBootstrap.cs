using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR;

public static class Ur5AutoSceneBootstrap
{
    private const float FallbackQuestRefreshRate = 72.0f;
    private static readonly List<XRDisplaySubsystem> DisplaySubsystems = new List<XRDisplaySubsystem>();
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void InstallAfterSceneLoad()
    {
        ApplyQuestRuntimeTiming();
        InstallStabilizer();
        InstallTargetWorkspaceLimiter();
        InstallTcpTargetFollower();
        InstallGripperController();
        InstallSpectatorCamera();
        InstallControllerVisualizer();
        SceneManager.sceneLoaded -= HandleSceneLoaded;
        SceneManager.sceneLoaded += HandleSceneLoaded;
    }

    private static void HandleSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        ApplyQuestRuntimeTiming();
        InstallStabilizer();
        InstallTargetWorkspaceLimiter();
        InstallTcpTargetFollower();
        InstallGripperController();
        InstallSpectatorCamera();
        InstallControllerVisualizer();
    }

    private static void InstallStabilizer()
    {
        Ur5PhysicsStabilizer existing = Object.FindObjectOfType<Ur5PhysicsStabilizer>();
        if (existing != null)
        {
            existing.robotRoot = FindRobotRoot();
            existing.Stabilize();
            return;
        }

        GameObject stabilizerObject = new GameObject("UR5AutoRuntimeStabilizer");
        Object.DontDestroyOnLoad(stabilizerObject);

        Ur5PhysicsStabilizer stabilizer = stabilizerObject.AddComponent<Ur5PhysicsStabilizer>();
        stabilizer.robotRoot = FindRobotRoot();
        stabilizer.scanSceneIfRobotRootMissing = true;
        stabilizer.disableGravity = true;
        stabilizer.fixArticulationRoots = true;
        stabilizer.makeRigidbodiesKinematic = true;
        stabilizer.lockRobotRootTransform = true;
        stabilizer.holdCurrentJointPose = true;
        stabilizer.Stabilize();
    }

    private static void InstallTcpTargetFollower()
    {
        Transform robotRoot = FindRobotRoot();
        GameObject target = GameObject.Find("TcpTarget");
        if (robotRoot == null || target == null)
        {
            return;
        }

        Ur5ArticulationJointController jointController = robotRoot.GetComponent<Ur5ArticulationJointController>();
        if (jointController == null)
        {
            jointController = robotRoot.gameObject.AddComponent<Ur5ArticulationJointController>();
        }

        jointController.robotRoot = robotRoot;

        Ur5TcpTargetFollower follower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
        if (follower == null)
        {
            follower = robotRoot.gameObject.AddComponent<Ur5TcpTargetFollower>();
        }

        follower.robotRoot = robotRoot;
        follower.tcpTarget = target.transform;
        follower.jointController = jointController;

        Ur5ActualTcpMarker actualMarker = target.GetComponent<Ur5ActualTcpMarker>();
        if (actualMarker == null)
        {
            actualMarker = target.AddComponent<Ur5ActualTcpMarker>();
        }

        actualMarker.follower = follower;
    }

    private static void InstallGripperController()
    {
        Transform robotRoot = FindRobotRoot();
        if (robotRoot == null)
        {
            return;
        }

        Quest3RobotiqGripperController gripperController = robotRoot.GetComponent<Quest3RobotiqGripperController>();
        if (gripperController == null)
        {
            gripperController = robotRoot.gameObject.AddComponent<Quest3RobotiqGripperController>();
        }

        gripperController.robotRoot = robotRoot;
    }

    private static void InstallTargetWorkspaceLimiter()
    {
        Transform robotRoot = FindRobotRoot();
        GameObject target = GameObject.Find("TcpTarget");
        if (robotRoot == null || target == null)
        {
            return;
        }

        TcpTargetWorkspaceLimiter workspaceLimiter = target.GetComponent<TcpTargetWorkspaceLimiter>();
        if (workspaceLimiter == null)
        {
            workspaceLimiter = target.AddComponent<TcpTargetWorkspaceLimiter>();
        }

        workspaceLimiter.robotRoot = robotRoot;

        TcpTargetCollisionGuard collisionGuard = target.GetComponent<TcpTargetCollisionGuard>();
        if (collisionGuard == null)
        {
            collisionGuard = target.AddComponent<TcpTargetCollisionGuard>();
        }

        collisionGuard.robotRoot = robotRoot;
    }

    private static void InstallSpectatorCamera()
    {
        Transform robotRoot = FindRobotRoot();
        if (robotRoot == null)
        {
            return;
        }

        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (bootstrap == null)
        {
            return;
        }

        Ur5EditorSpectatorCamera spectatorCamera = bootstrap.GetComponent<Ur5EditorSpectatorCamera>();
        if (spectatorCamera == null)
        {
            spectatorCamera = bootstrap.gameObject.AddComponent<Ur5EditorSpectatorCamera>();
        }

        spectatorCamera.robotRoot = robotRoot;
        spectatorCamera.tcpTarget = GameObject.Find("TcpTarget")?.transform;
    }

    private static void InstallControllerVisualizer()
    {
        Ur5ControlBootstrap bootstrap = Object.FindObjectOfType<Ur5ControlBootstrap>();
        if (bootstrap == null)
        {
            return;
        }

        Quest3ControllerVisualizer visualizer = bootstrap.GetComponent<Quest3ControllerVisualizer>();
        if (visualizer == null)
        {
            visualizer = bootstrap.gameObject.AddComponent<Quest3ControllerVisualizer>();
        }

        GameObject xrOrigin = GameObject.Find("XR Origin (VR)");
        visualizer.xrOrigin = xrOrigin != null ? xrOrigin.transform : null;
    }

    private static void ApplyQuestRuntimeTiming()
    {
        float refreshRate = ResolveDisplayRefreshRate();
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = Mathf.RoundToInt(refreshRate);
        Time.fixedDeltaTime = 1.0f / refreshRate;
        Time.maximumDeltaTime = Mathf.Max(Time.fixedDeltaTime * 4.0f, Time.fixedDeltaTime);
    }

    private static float ResolveDisplayRefreshRate()
    {
        float refreshRate = FallbackQuestRefreshRate;
        SubsystemManager.GetSubsystems(DisplaySubsystems);
        foreach (XRDisplaySubsystem displaySubsystem in DisplaySubsystems)
        {
            if (displaySubsystem == null || !displaySubsystem.running)
            {
                continue;
            }

            if (displaySubsystem.TryGetDisplayRefreshRate(out float displayRefreshRate)
                && displayRefreshRate > 1.0f)
            {
                refreshRate = displayRefreshRate;
                break;
            }
        }

        return Mathf.Clamp(refreshRate, 60.0f, 120.0f);
    }

    private static Transform FindRobotRoot()
    {
        string[] candidateNames =
        {
            "ur5_robot",
            "ur5",
            "UR5",
            "base_link"
        };

        foreach (string candidateName in candidateNames)
        {
            GameObject candidate = GameObject.Find(candidateName);
            if (candidate != null)
            {
                return candidate.transform;
            }
        }

        return null;
    }
}
