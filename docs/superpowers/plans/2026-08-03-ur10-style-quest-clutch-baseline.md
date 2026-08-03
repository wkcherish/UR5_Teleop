# UR10-Style Quest Clutch Baseline

> **Owner:** Unity UR5 project. Keep `Ur5UrScriptSpeedlClient.enableRealRobotOutput == false` until the virtual Quest behavior has been manually accepted.

## Objective

Replace the default Quest right-hand teleoperation path with the pose loop used by `elpis-lab/UR10_Teleop`: Grip is the only clutch/deadman; press anchors the raw controller pose and measured TCP pose; hold maps every fresh controller pose against those immutable anchors; release retains the final filtered target. The default mapping is `(0.5, 0.5, 0.5)` and the only target filter is a `0.05` step.

## Scope and constraints

- Do not alter UDP packet generation or make UDP a command source. It remains telemetry-only.
- Do not use the existing nonlinear continuous gain curve, controller position filter, target deadband, lead rebase, acceleration trajectory, or follower settled-target joint freeze while this profile is active.
- Do not re-enable the historical three-mode, joystick, left-hand, or keyboard paths for the default profile.
- Do not snap/re-anchor the target on right Grip release.

## Task 1: Lock down the pure anchored-pose contract

**Files:**
- Modify: `Assets/Tests/EditMode/Editor/Ur5RelativePoseClutchMapperTests.cs`
- Modify only if tests require a focused API: `Assets/Scripts/UR5/ControlLogic/Ur5AnchoredPoseTeleopStrategy.cs`

**Interface:** `Resume(inputPosition, inputRotation, toolPosition, toolRotation)`, `TryGetRequestedPose(inputPosition, inputRotation, Vector3.one * 0.5f, requestedToolRotation, out position, out rotation)`, `FilterRequestedPose(position, rotation, 0.05f, out position, out rotation)`, and `Pause()`.

1. Add a test that resumes at non-identity input and TCP poses, calls the strategy with the unchanged input, and proves that both requested pose values remain exactly at the TCP anchor. The implementation fault it catches is anchor mutation or an incorrect position offset.
2. Add a test that moves the controller exactly `0.020 m` on one axis after resume and asserts that the requested TCP delta is exactly `0.010 m` with a `Vector3.one * 0.5f` mapping. It catches variable/progressive mapping.
3. Add a test that calculates `currentControllerRotation * Quaternion.Inverse(anchorControllerRotation) * toolAnchorRotation`, passes that value to the strategy, and asserts the requested rotation matches it. It catches reversing the quaternion multiplication order or locking an axis.
4. Add a test that repeatedly filters a fixed requested position with `0.05f`, verifies that the target approaches it monotonically, and verifies a fresh unchanged-input request remains equal to the immutable anchor-derived requested pose. It catches target lead rebase and a second command filter.
5. Add a release test: call `Pause()`, then assert `TryGetRequestedPose` and `FilterRequestedPose` both return false and that `TargetPosition`/`TargetRotation` are unchanged. It catches release-time target writes or re-anchoring.
6. Run the targeted EditMode test and verify it is RED before production changes if any asserted behavior is absent. Implement only the smallest strategy change required for it, then rerun it green.

## Task 2: Make the default Unity profile use the contract end to end

**Files:**
- Modify: `Assets/Scripts/UR5/Ur5CartesianVelocityTeleopController.cs`
- Modify: `Assets/Scripts/UR5/Ur5ControlBootstrap.cs`
- Modify: `Assets/Scripts/UR5/Ur5TcpTargetFollower.cs`
- Modify: `Assets/Tests/EditMode/Editor/Ur5Continuous6DofBootstrapTests.cs`

**Controller flow:** Create a narrowly scoped `ApplyUr10StyleClutchPreview()` or replace the existing continuous preview branch. On Grip press call `anchoredPoseTeleop.Resume(rawPosition, rawRotation, GetActualToolPosition(), GetActualToolRotation())`. While Grip remains held calculate `relativeRotation = rawRotation * Quaternion.Inverse(anchorRotation)`, request the target using `(0.5, 0.5, 0.5)` and `relativeRotation * toolAnchorRotation`, filter it with `0.05f`, then write that one filtered target after workspace safety validation. On Grip release call `Pause()` only; do not write or snap the target.

1. Add a default-profile test that invokes `Ur5ControlBootstrap` and asserts the selected default is the UR10-style anchored path, with right-hand Grip for both pose components, `Vector3.one * 0.5f`, and `0.05f` smoothing. The test should fail against the present continuous-velocity default.
2. Add a default follower-profile test that asserts `holdJointPoseWhenTargetSettled == false` and `enablePrecisionAssemblyTracking == false`. It catches the settled-target joint freeze/restart state machine becoming active during Grip hold.
3. Add a controller-level release regression test by invoking the selected preview path with an inactive Grip and asserting the `tcpPreviewTarget` does not change. It catches `SnapTargetToActualPose`, rebase, or late target writers on release.
4. Change the controller's standard branch and defaults to use only the anchored strategy. Keep `continuous6DofController` available for compatibility but make it inactive in the Quest default profile. Remove the default profile's adaptive position filter, progressive scaling, target deadband, relative command filter, lead limiter/rebase, and acceleration trajectory from this path.
5. Change the bootstrap defaults to disable follower precision tracking and settled target freeze. Preserve the follower's one-time safety hold only for its explicit safe-release paths, not for a normal Grip release.
6. Run the targeted tests, then the complete `Ur5*` EditMode suite in a copied test project using the Tuanjie batch executable. Inspect both result XML and exit status.

## Manual Quest validation

1. Build only after the EditMode suite is green, with real robot output still disabled.
2. At rest with right Grip held for 10 seconds, the target and joints must remain stationary aside from physics settling; no repeated re-anchors or target writes should appear in diagnostics.
3. Move the hand 20 mm along a world axis and inspect the target/log for a 10 mm command delta before workspace clamping.
4. Rotate the hand around each axis and confirm the TCP follows the full relative quaternion with no rotation locking.
5. Release Grip while moving, then continue moving the hand. The final target must remain held without a snap back or renewed command writes.
6. Pull the Android CSV and Unity `logcat` after the run. Do not claim motion quality is resolved until this physical virtual-scene validation is complete.
