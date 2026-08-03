# UR5 DLS/Drive Anti-Windup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent UR5 Cartesian DLS commands from accumulating against the Articulation Drive lead guard, so a stationary Quest TCP target cannot leave the robot pursuing saturated stale joint commands.

**Architecture:** `Ur5ArticulationJointController` will make its applied-state cache match the value actually assigned to `ArticulationBody.xDrive.target`. `Ur5TcpTargetFollower` will independently seed every DLS waypoint from that drive target, keeping Cartesian corrections relative to the physically commanded pose while preserving the existing UR10-style input mapping and all current safety limits.

**Tech Stack:** Tuanjie Editor 2022.3.62t11 / Tuanjie 1.9.3, Unity C#, Unity ArticulationBody, NUnit EditMode tests, Quest 3 Android, ADB.

## Global Constraints

- Project root: `/Users/imi-1/Desktop/project/ur5`; branch: `fix/teleop-pose-math-testbench`.
- Design source: `docs/superpowers/specs/2026-08-03-ur5-dls-drive-anti-windup-design.md`.
- Keep the existing proximal/wrist measured-joint lead limits at 6.0 and 2.5 degrees.
- Do not change DLS gain, damping, orientation weight, joint speed, drive stiffness/damping, the `0.05` target smoothing step, the `0.5` translation mapping, or workspace limits.
- Do not add a deadband, stationary freeze, coordinate conversion, or alternate orientation mode.
- Keep `Ur5UrScriptSpeedlClient.enableRealRobotOutput == false`; do not connect or command a real UR5.
- The worktree contains existing user changes. Never revert them, and stage only explicit task files. `Ur5TcpTargetFollower.cs` is already dirty, so do not commit that whole file without a separate diff review.
- Do not run batchmode against the GUI project. Create a safe temporary copy for command-line tests and compilation.
- Do not perform Quest Build & Run automatically. Stop after local verification and give the user the exact manual validation steps.

Create a fresh command-line test copy before the first RED run:

```bash
UR5_TEST_PARENT="$(mktemp -d /tmp/ur5-anti-windup.XXXXXX)"
UR5_TEST_COPY="$UR5_TEST_PARENT/ur5"
mkdir "$UR5_TEST_COPY"
rsync -a \
  --exclude '.git/' \
  --exclude 'Library/' \
  --exclude 'Temp/' \
  --exclude 'Logs/' \
  --exclude 'obj/' \
  --exclude 'Builds/' \
  ./ "$UR5_TEST_COPY/"
test -f "$UR5_TEST_COPY/ProjectSettings/ProjectVersion.txt"
```

For each later test run, repeat the `rsync` command into the same copy before launching Tuanjie. Do not delete or clean the source worktree.

---

## File Map

- Create: `Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs`
  - Builds an inactive, one-joint test fixture and verifies applied-state and follower waypoint semantics without running scene physics.
- Create (Tuanjie-generated): `Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs.meta`
- Modify: `Assets/Scripts/UR5/Ur5ArticulationJointController.cs`
  - Synchronizes `appliedJointTargets[index]` to the constrained value assigned to `xDrive.target`.
- Modify: `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`
  - Seeds each Cartesian waypoint from `GetDriveTargetDegrees(i)`.

No scene, bootstrap, controller-mapping, package, project-setting, or validation-document change belongs to this implementation.

---

### Task 1: Make Applied State Match the Assigned Drive Target

**Files:**
- Create: `Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs`
- Create: `Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs.meta`
- Modify: `Assets/Scripts/UR5/Ur5ArticulationJointController.cs:531`

**Interfaces:**
- Consumes: existing private `ApplyDriveTarget(int index, float targetDegrees)` and public `GetAppliedJointTargetDegrees(int index)` / `GetDriveTargetDegrees(int index)`.
- Produces: the invariant `GetAppliedJointTargetDegrees(i) == GetDriveTargetDegrees(i)` immediately after every drive assignment.

- [ ] **Step 1: Write the failing applied-state regression test**

Create the test fixture and first test:

```csharp
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public class Ur5JointCommandAntiWindupTests
{
    private GameObject controllerOwner;
    private GameObject jointOwner;
    private Ur5ArticulationJointController controller;
    private ArticulationBody joint;

    [SetUp]
    public void SetUp()
    {
        controllerOwner = new GameObject("anti-windup-controller");
        controllerOwner.SetActive(false);
        jointOwner = new GameObject("anti-windup-joint");
        joint = jointOwner.AddComponent<ArticulationBody>();
        controller = controllerOwner.AddComponent<Ur5ArticulationJointController>();
        controller.limitDriveTargetLeadFromMeasuredJoint = false;

        GetPrivateList<ArticulationBody>(controller, "joints").Add(joint);
        GetPrivateList<float>(controller, "jointTargets").Add(120.0f);
        GetPrivateList<float>(controller, "appliedJointTargets").Add(120.0f);
        GetPrivateList<float>(controller, "appliedJointVelocities").Add(0.0f);
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(controllerOwner);
        Object.DestroyImmediate(jointOwner);
    }

    [Test]
    public void ApplyDriveTarget_WhenAppliedStateDiverged_SynchronizesToAssignedDriveTarget()
    {
        InvokeNonPublic(controller, "ApplyDriveTarget", 0, 6.0f);

        Assert.AreEqual(6.0f, controller.GetDriveTargetDegrees(0), 0.0001f);
        Assert.AreEqual(
            controller.GetDriveTargetDegrees(0),
            controller.GetAppliedJointTargetDegrees(0),
            0.0001f,
            "The applied cache must not remain at an unapplied 120-degree request.");
    }

    private static List<T> GetPrivateList<T>(object target, string fieldName)
    {
        FieldInfo field = target.GetType().GetField(
            fieldName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, fieldName + " should exist.");
        return (List<T>)field.GetValue(target);
    }

    private static void InvokeNonPublic(object target, string methodName, params object[] arguments)
    {
        MethodInfo method = target.GetType().GetMethod(
            methodName,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(method, methodName + " should exist.");
        method.Invoke(target, arguments);
    }
}
```

The production change that will make this test pass is the assignment of the constrained drive value back to `appliedJointTargets[index]` inside `ApplyDriveTarget`.

- [ ] **Step 2: Run the focused test and verify RED**

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
"$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$UR5_TEST_COPY" \
  -runTests -testPlatform EditMode \
  -testFilter Ur5JointCommandAntiWindupTests \
  -testResults /tmp/ur5-anti-windup-task1-red.xml \
  -logFile /tmp/ur5-anti-windup-task1-red.log -quit
```

Expected: the test fails because the drive target is 6 degrees while `GetAppliedJointTargetDegrees(0)` remains 120 degrees. A compile error, reflection error, or unrelated test failure is not an acceptable RED result.

- [ ] **Step 3: Implement the minimal applied-state synchronization**

Replace the body of `ApplyDriveTarget` with:

```csharp
private void ApplyDriveTarget(int index, float targetDegrees)
{
    ArticulationDrive drive = joints[index].xDrive;
    float constrainedTargetDegrees = ConstrainDriveTargetLead(index, targetDegrees);
    drive.target = constrainedTargetDegrees;
    joints[index].xDrive = drive;
    appliedJointTargets[index] = constrainedTargetDegrees;
}
```

Do not assign `jointTargets[index]` here. That value remains the requested trajectory destination for ready-pose and direct joint commands.

- [ ] **Step 4: Re-sync the temporary copy and verify GREEN**

Repeat the plan's `rsync` command, then rerun the Task 1 command with output names ending in `task1-green`. Expected: 1 test passes, no `error CS` or unexpected Unity exception appears, and Tuanjie generates the `.meta` file for the new test.

If the GUI project has not already generated the source `.meta`, preserve the
GUID generated by the successfully imported temporary copy:

```bash
if test ! -f Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs.meta
then
  cp \
    "$UR5_TEST_COPY/Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs.meta" \
    Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs.meta
fi
test -f Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs.meta
```

- [ ] **Step 5: Review the isolated Task 1 diff**

```bash
git diff --check -- \
  Assets/Scripts/UR5/Ur5ArticulationJointController.cs \
  Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs
git diff -- \
  Assets/Scripts/UR5/Ur5ArticulationJointController.cs \
  Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs
```

Expected: one production assignment plus the focused test fixture. Do not stage generated project/package changes from the temporary copy.

---

### Task 2: Seed DLS Waypoints from the Physical Drive Command

**Files:**
- Modify: `Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs`
- Modify: `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs:1025`

**Interfaces:**
- Consumes: `Ur5ArticulationJointController.GetDriveTargetDegrees(int index)` from the existing controller API.
- Produces: `BeginJointWaypoint(int jointCount)` initializes `workingJointTargetsDegrees[i]` from the current `xDrive.target`, never from an unconstrained trajectory cache.

- [ ] **Step 1: Add the failing follower waypoint test**

Add this test and helper to `Ur5JointCommandAntiWindupTests`:

```csharp
[Test]
public void BeginJointWaypoint_WhenTrajectoryStateDiverged_SeedsFromDriveTarget()
{
    ArticulationDrive drive = joint.xDrive;
    drive.target = 6.0f;
    joint.xDrive = drive;
    GetPrivateList<float>(controller, "appliedJointTargets")[0] = 120.0f;

    var follower = controllerOwner.AddComponent<Ur5TcpTargetFollower>();
    follower.jointController = controller;
    InvokeNonPublic(follower, "BeginJointWaypoint", 1);

    float[] waypoint = GetPrivateField<float[]>(
        follower,
        "workingJointTargetsDegrees");
    Assert.AreEqual(6.0f, waypoint[0], 0.0001f);
}

private static T GetPrivateField<T>(object target, string fieldName)
{
    FieldInfo field = target.GetType().GetField(
        fieldName,
        BindingFlags.Instance | BindingFlags.NonPublic);
    Assert.IsNotNull(field, fieldName + " should exist.");
    return (T)field.GetValue(target);
}
```

The explicit divergence makes the test independent of Task 1 and proves the follower's own source-of-truth decision.

- [ ] **Step 2: Run the focused class and verify the new test is RED**

Re-sync the temporary copy and run the same focused Tuanjie command with output names ending in `task2-red`.

Expected: `ApplyDriveTarget_WhenAppliedStateDiverged_SynchronizesToAssignedDriveTarget` passes; `BeginJointWaypoint_WhenTrajectoryStateDiverged_SeedsFromDriveTarget` fails with expected 6 and actual 120.

- [ ] **Step 3: Implement the minimal waypoint seed change**

In `BeginJointWaypoint`, replace the applied-cache seed with the drive target and update the nearby comment:

```csharp
for (int i = 0; i < workingJointCount; i++)
{
    // DLS is evaluated from measured geometry, so each correction must start
    // from the target that was actually assigned to the Articulation Drive.
    workingJointTargetsDegrees[i] = jointController.GetDriveTargetDegrees(i);
}
```

Do not change DLS weights, gains, step limits, tolerances, or trajectory-player behavior.

- [ ] **Step 4: Re-sync the temporary copy and verify both tests are GREEN**

Run the focused class with output names ending in `task2-green`. Expected: 2 tests pass and the log contains no compile errors or unexpected exceptions.

- [ ] **Step 5: Review only the new follower hunk before any staging**

```bash
git diff --check -- \
  Assets/Scripts/UR5/Ur5TcpTargetFollower.cs \
  Assets/Tests/EditMode/Editor/Ur5JointCommandAntiWindupTests.cs
git diff -U8 -- Assets/Scripts/UR5/Ur5TcpTargetFollower.cs
```

Expected: the new Task 2 hunk changes only the waypoint seed and comment. The file also contains pre-existing UR10 release-path changes; keep them unstaged unless the user separately requests a combined commit.

---

### Task 3: Regression, Compilation, and Quest Handoff

**Files:**
- Verify only: all files listed in the File Map.

**Interfaces:**
- Consumes: the two anti-windup invariants established by Tasks 1 and 2.
- Produces: local test/compile evidence and a controlled manual Quest validation protocol.

- [ ] **Step 1: Run all related EditMode classes in the temporary copy**

After the final `rsync`, run each class separately so Tuanjie's semicolon-filter behavior cannot hide failures:

```bash
TUANJIE_EDITOR="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/Tuanjie.app/Contents/MacOS/Tuanjie"
for test_filter in \
  Ur5JointCommandAntiWindupTests \
  Ur5Continuous6DofBootstrapTests \
  Ur5RelativePoseClutchMapperTests \
  Ur5Continuous6DofClutchControllerTests \
  Ur5PoseMathTestBenchTests \
  Ur5ClutchModeControllerTests \
  Ur5LeftSafetyPoseControllerTests
do
  "$TUANJIE_EDITOR" -batchmode -nographics -projectPath "$UR5_TEST_COPY" \
    -runTests -testPlatform EditMode \
    -testFilter "$test_filter" \
    -testResults "/tmp/${test_filter}-anti-windup.xml" \
    -logFile "/tmp/${test_filter}-anti-windup.log" -quit || exit 1
done
```

Expected: every invoked class exits successfully; result XML, when generated, reports 0 failures; all logs are free of `error CS`, `Scripts have compiler errors`, and unexpected exceptions. If this Tuanjie version again omits XML, report that limitation and use exit status plus log inspection without claiming an unobserved test count.

- [ ] **Step 2: Run a clean Tuanjie compilation check in the temporary copy**

```bash
"$TUANJIE_EDITOR" -batchmode -nographics \
  -projectPath "$UR5_TEST_COPY" \
  -logFile /tmp/ur5-anti-windup-compile.log -quit
rg -n "error CS|Scripts have compiler errors|Unhandled Exception" \
  /tmp/ur5-anti-windup-compile.log
```

Expected: Tuanjie exits 0 and the final `rg` produces no matches.

- [ ] **Step 3: Audit the final source diff and safety defaults**

```bash
git diff --check
git status --short
rg -n "enableRealRobotOutput" Assets/Scripts/UR5
rg -n "anchoredPoseSmoothingStep = 0.05f|Ur10StyleSmoothingStep = 0.05f" \
  Assets/Scripts/UR5
```

Expected: no whitespace errors; real robot output remains disabled; the UR10-style smoothing value is unchanged; task changes are limited to the controller, follower, and new test pair alongside the pre-existing dirty files.

- [ ] **Step 4: Hand off the manual Quest A/B run**

Ask the user to use Tuanjie GUI Build And Run with real robot output disabled. The test sequence is:

1. Open the app and observe for 5 seconds without pressing Grip; the arm must not fall or move.
2. Hold right Grip without moving for 10 seconds; the arm must settle rather than enter a sustained sweep.
3. Move and rotate for 5 seconds, then hold the hand still while keeping Grip pressed for another 10 seconds.
4. Release and re-press Grip at least three times to verify each anchor begins from the measured TCP without a jump.
5. Close or pause the app so `ur5_pose_log.csv` is flushed.

Then pull the log:

```bash
ADB="/Applications/Tuanjie/Hub/Editor/2022.3.62t11/PlaybackEngines/AndroidPlayer/SDK/platform-tools/adb"
"$ADB" pull \
  /sdcard/Android/data/com.dgvlalab.ur5quest/files/ur5_pose_log.csv \
  /tmp/ur5_pose_log_anti_windup.csv
```

The anti-windup run passes when ordinary operation no longer sends logical joint targets to their joint limits, joint target-to-measured lead remains bounded, and stationary-hand windows no longer contain large continuing drive sweeps. Only after those conditions pass should smoothing or orientation ergonomics be tuned.

---

## Commit Policy

The specification is already committed as `34f1872`. This implementation runs in a dirty worktree:

- Task 1 may be committed independently because `Ur5ArticulationJointController.cs` was clean at plan creation and the new test is isolated.
- Task 2 must remain uncommitted unless a precise staged patch contains only the new `GetDriveTargetDegrees` hunk; never stage the entire pre-modified `Ur5TcpTargetFollower.cs` by accident.
- Do not commit scene, package, project-setting, deleted Windows `.meta`, UDP, recorder, or unrelated controller files as part of anti-windup.
