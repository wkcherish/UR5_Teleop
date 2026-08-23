using UnityEngine;

[DefaultExecutionOrder(-10000)]
public class Ur5PhysicsStabilizer : MonoBehaviour
{
    [Header("Scope")]
    public Transform robotRoot;
    public bool scanSceneIfRobotRootMissing = true;

    [Header("Digital Twin Mode")]
    public bool disableGravity = true;
    public bool fixArticulationRoots = true;
    public bool makeRigidbodiesKinematic = true;
    public bool lockRobotRootTransform = true;

    [Header("Articulation Drive Hold")]
    public bool holdCurrentJointPose = true;
    public float stiffness = 10000.0f;
    public float damping = 1000.0f;
    public float forceLimit = 10000.0f;

    [Header("Startup Reinforcement")]
    public int reinforceFixedFrames = 120;
    public bool logStabilizeSummary = true;

    private Vector3 lockedRootPosition;
    private Quaternion lockedRootRotation;
    private int remainingReinforceFrames;
    private int lastArticulationCount = -1;
    private int lastRigidbodyCount = -1;
    private bool initialJointHoldApplied;

    private void Awake()
    {
        ConfigureRootAndStabilize(robotRoot);
    }

    private void FixedUpdate()
    {
        if (remainingReinforceFrames > 0)
        {
            Stabilize(false);
            remainingReinforceFrames--;
        }

        if (lockRobotRootTransform && robotRoot != null)
        {
            robotRoot.SetPositionAndRotation(lockedRootPosition, lockedRootRotation);
        }
    }

    public void Stabilize()
    {
        Stabilize(true);
    }

    public void Stabilize(bool allowLog)
    {
        ArticulationBody[] articulationBodies = GetArticulationBodies();
        Rigidbody[] rigidbodies = GetRigidbodies();

        StabilizeArticulationBodies(articulationBodies);
        StabilizeRigidbodies(rigidbodies);

        if (allowLog && logStabilizeSummary || CountsChanged(articulationBodies.Length, rigidbodies.Length))
        {
            lastArticulationCount = articulationBodies.Length;
            lastRigidbodyCount = rigidbodies.Length;
            Debug.Log("UR5 physics stabilized. ArticulationBodies=" + articulationBodies.Length + ", Rigidbodies=" + rigidbodies.Length);
        }
    }

    public void ConfigureRootAndStabilize(Transform root)
    {
        robotRoot = root != null ? root : robotRoot;
        if (robotRoot == null)
        {
            robotRoot = FindRobotRoot();
        }

        LockCurrentRootPose();
        remainingReinforceFrames = reinforceFixedFrames;
        Stabilize();
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

    private void LockCurrentRootPose()
    {
        if (robotRoot == null)
        {
            return;
        }

        lockedRootPosition = robotRoot.position;
        lockedRootRotation = robotRoot.rotation;
    }

    private bool CountsChanged(int articulationCount, int rigidbodyCount)
    {
        return articulationCount != lastArticulationCount || rigidbodyCount != lastRigidbodyCount;
    }

    private ArticulationBody[] GetArticulationBodies()
    {
        if (robotRoot != null)
        {
            return robotRoot.GetComponentsInChildren<ArticulationBody>(true);
        }

        return scanSceneIfRobotRootMissing ? FindObjectsOfType<ArticulationBody>() : new ArticulationBody[0];
    }

    private Rigidbody[] GetRigidbodies()
    {
        if (robotRoot != null)
        {
            return robotRoot.GetComponentsInChildren<Rigidbody>(true);
        }

        return scanSceneIfRobotRootMissing ? FindObjectsOfType<Rigidbody>() : new Rigidbody[0];
    }

    private void StabilizeArticulationBodies(ArticulationBody[] bodies)
    {
        bool applyInitialJointHold = holdCurrentJointPose && !initialJointHoldApplied;

        foreach (ArticulationBody body in bodies)
        {
            if (disableGravity)
            {
                body.useGravity = false;
            }

            if (fixArticulationRoots && IsArticulationRoot(body))
            {
                body.immovable = true;
            }

            if (applyInitialJointHold && body.jointType != ArticulationJointType.FixedJoint)
            {
                ArticulationDrive drive = body.xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                drive.target = body.jointPosition.dofCount > 0 ? body.jointPosition[0] * Mathf.Rad2Deg : drive.target;
                body.xDrive = drive;
            }
        }

        if (applyInitialJointHold)
        {
            initialJointHoldApplied = true;
        }
    }

    private void StabilizeRigidbodies(Rigidbody[] rigidbodies)
    {
        foreach (Rigidbody rigidbody in rigidbodies)
        {
            if (disableGravity)
            {
                rigidbody.useGravity = false;
            }

            if (makeRigidbodiesKinematic)
            {
                rigidbody.isKinematic = true;
                rigidbody.constraints = RigidbodyConstraints.FreezeAll;
            }

            rigidbody.velocity = Vector3.zero;
            rigidbody.angularVelocity = Vector3.zero;
        }
    }

    private bool IsArticulationRoot(ArticulationBody body)
    {
        Transform current = body.transform.parent;
        while (current != null)
        {
            if (current.GetComponent<ArticulationBody>() != null)
            {
                return false;
            }

            if (robotRoot != null && current == robotRoot.parent)
            {
                break;
            }

            current = current.parent;
        }

        return true;
    }
}
