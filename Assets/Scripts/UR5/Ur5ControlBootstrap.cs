using UnityEngine;

public class Ur5ControlBootstrap : MonoBehaviour
{
    public Transform robotRoot;
    public Transform tcpTarget;
    public bool createTcpTargetIfMissing = true;
    public bool enableKeyboardControl = true;
    public bool enableQuest3Control = true;
    public bool enableTcpTargetFollower = true;

    private void Awake()
    {
        if (robotRoot == null)
        {
            robotRoot = FindRobotRoot();
        }

        DisableLegacyUrdfImporterController();

        if (tcpTarget == null)
        {
            GameObject foundTarget = GameObject.Find("TcpTarget");
            if (foundTarget != null)
            {
                tcpTarget = foundTarget.transform;
            }
        }

        if (tcpTarget == null && createTcpTargetIfMissing)
        {
            GameObject target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            target.name = "TcpTarget";
            target.transform.position = new Vector3(0.5f, 0.3f, 0.3f);
            target.transform.localScale = Vector3.one * 0.05f;

            Renderer renderer = target.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material.color = Color.red;
            }

            tcpTarget = target.transform;
        }

        if (tcpTarget != null)
        {
            TcpTargetWorkspaceLimiter workspaceLimiter = tcpTarget.GetComponent<TcpTargetWorkspaceLimiter>();
            if (workspaceLimiter == null)
            {
                workspaceLimiter = tcpTarget.gameObject.AddComponent<TcpTargetWorkspaceLimiter>();
            }

            workspaceLimiter.robotRoot = robotRoot;

            TcpTargetCollisionGuard collisionGuard = tcpTarget.GetComponent<TcpTargetCollisionGuard>();
            if (collisionGuard == null)
            {
                collisionGuard = tcpTarget.gameObject.AddComponent<TcpTargetCollisionGuard>();
            }

            collisionGuard.robotRoot = robotRoot;
        }

        if (enableKeyboardControl && tcpTarget != null && tcpTarget.GetComponent<TcpTargetKeyboardController>() == null)
        {
            tcpTarget.gameObject.AddComponent<TcpTargetKeyboardController>();
        }

        if (enableQuest3Control && tcpTarget != null && tcpTarget.GetComponent<Quest3TcpTargetController>() == null)
        {
            tcpTarget.gameObject.AddComponent<Quest3TcpTargetController>();
        }

        Ur5PhysicsStabilizer stabilizer = GetComponent<Ur5PhysicsStabilizer>();
        if (stabilizer == null)
        {
            stabilizer = gameObject.AddComponent<Ur5PhysicsStabilizer>();
        }

        stabilizer.robotRoot = robotRoot;
        stabilizer.Stabilize();

        Ur5ArticulationJointController jointController = null;
        if (robotRoot != null)
        {
            jointController = robotRoot.GetComponent<Ur5ArticulationJointController>();
            if (jointController == null)
            {
                jointController = robotRoot.gameObject.AddComponent<Ur5ArticulationJointController>();
            }

            jointController.robotRoot = robotRoot;
        }

        if (enableTcpTargetFollower && robotRoot != null && tcpTarget != null && jointController != null)
        {
            Ur5TcpTargetFollower follower = robotRoot.GetComponent<Ur5TcpTargetFollower>();
            if (follower == null)
            {
                follower = robotRoot.gameObject.AddComponent<Ur5TcpTargetFollower>();
            }

            follower.robotRoot = robotRoot;
            follower.tcpTarget = tcpTarget;
            follower.jointController = jointController;

            Ur5ActualTcpMarker actualMarker = tcpTarget.GetComponent<Ur5ActualTcpMarker>();
            if (actualMarker == null)
            {
                actualMarker = tcpTarget.gameObject.AddComponent<Ur5ActualTcpMarker>();
            }

            actualMarker.follower = follower;

            Quest3RobotiqGripperController gripperController = robotRoot.GetComponent<Quest3RobotiqGripperController>();
            if (gripperController == null)
            {
                gripperController = robotRoot.gameObject.AddComponent<Quest3RobotiqGripperController>();
            }

            gripperController.robotRoot = robotRoot;
        }

        Ur5EditorSpectatorCamera spectatorCamera = GetComponent<Ur5EditorSpectatorCamera>();
        if (spectatorCamera == null)
        {
            spectatorCamera = gameObject.AddComponent<Ur5EditorSpectatorCamera>();
        }

        spectatorCamera.robotRoot = robotRoot;
        spectatorCamera.tcpTarget = tcpTarget;

        Quest3ControllerVisualizer controllerVisualizer = GetComponent<Quest3ControllerVisualizer>();
        if (controllerVisualizer == null)
        {
            controllerVisualizer = gameObject.AddComponent<Quest3ControllerVisualizer>();
        }

        GameObject xrOrigin = GameObject.Find("XR Origin (VR)");
        controllerVisualizer.xrOrigin = xrOrigin != null ? xrOrigin.transform : null;

        Ur5PoseCsvRecorder recorder = GetComponent<Ur5PoseCsvRecorder>();
        if (recorder == null)
        {
            recorder = gameObject.AddComponent<Ur5PoseCsvRecorder>();
        }

        recorder.tcpTarget = tcpTarget;
        recorder.jointController = jointController;
        recorder.questController = tcpTarget != null ? tcpTarget.GetComponent<Quest3TcpTargetController>() : null;
    }

    private Transform FindRobotRoot()
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

    private void DisableLegacyUrdfImporterController()
    {
        if (robotRoot == null)
        {
            return;
        }

        MonoBehaviour[] components = robotRoot.GetComponents<MonoBehaviour>();
        foreach (MonoBehaviour component in components)
        {
            if (component == null)
            {
                continue;
            }

            string typeName = component.GetType().FullName;
            if (typeName == "Unity.Robotics.UrdfImporter.Control.Controller"
                || typeName == "Unity.Robotics.UrdfImporter.Control.FKRobot"
                || typeName == "Unity.Robotics.UrdfImporter.Control.IKRobot")
            {
                component.enabled = false;
                Debug.Log("Disabled legacy URDF Importer component: " + typeName);
            }
        }
    }
}
