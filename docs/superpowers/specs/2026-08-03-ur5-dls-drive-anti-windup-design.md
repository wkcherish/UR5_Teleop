# UR5 DLS/Drive Anti-Windup Design

## Problem

The Quest 3 validation log `ur5_pose_log.csv` shows that controller input is
not the first unstable layer. During several 0.5-second windows in which the
raw right-controller pose changed by no more than 4 mm and 2 degrees:

- the filtered TCP target changed by only 0.26-2.0 mm and 0.15-0.58 degrees;
- the measured TCP moved by 61-80 mm and 9-12 degrees;
- the joint drive targets continued to move by up to 15 degrees.

The logical joint commands also reached their configured limits, including
`+/-360` degrees on several joints and `+/-180` degrees on the elbow, while
the measured joints remained tens or hundreds of degrees away from those
commands. This is command-state windup, not ordinary Quest tracking noise.

## Root Cause

`Ur5TcpTargetFollower` evaluates DLS from the measured robot geometry, but
builds the next joint waypoint from
`Ur5ArticulationJointController.GetAppliedJointTargetDegrees()`.

Despite its name, `appliedJointTargets` currently stores the unconstrained
output of the drive-target trajectory. `ApplyDriveTarget()` separately clamps
the value to a measured-joint lead window before assigning `xDrive.target`,
but it does not write the constrained value back to `appliedJointTargets`.

The resulting loop is:

1. DLS adds a correction to an unconstrained command state.
2. The command state advances faster than the physical joint.
3. The Articulation Drive receives only the measured-joint-limited value.
4. The unconstrained state continues accumulating toward the joint limits.
5. DLS eventually reverses against a large stale command, producing long
   sweeps, oscillation, poor orientation control, and apparent input lag.

The UR10-style anchored controller-to-TCP mapping is not the source of this
failure and remains unchanged in this repair.

## Design

### Authoritative Applied Command

The actual `ArticulationBody.xDrive.target` is the authoritative applied joint
command. Every path that writes a drive target will keep the controller's
`appliedJointTargets` state synchronized with the constrained value that was
actually assigned to the drive.

The requested `jointTargets` state remains available as the short-lived input
to the drive trajectory, but it must not be used as accumulated IK state after
the drive lead guard changes the command.

### Follower Waypoint Seed

`Ur5TcpTargetFollower.BeginJointWaypoint()` will seed each DLS waypoint from
`GetDriveTargetDegrees()`, not from a pre-clamp trajectory state. A DLS step is
therefore always relative to the latest physically commanded joint pose.

This is intentional defense in depth: even if another controller path submits
a distant requested target, the Cartesian follower cannot integrate from that
unapplied value.

### Existing Safety Limits

The existing measured-joint lead limits remain active:

- proximal joints: 6 degrees;
- wrist joints: 2.5 degrees;
- separate wider limits for the known ready-pose trajectory.

This change corrects their state semantics; it does not widen or remove them.
The existing drive speed, acceleration, stiffness, damping, DLS gain, target
smoothing step, controller mapping, and workspace limits remain unchanged for
the first Quest A/B run.

### Release and Failure Behavior

Grip release continues to hold the measured joints once without rewriting the
UR10-style logical TCP target. Non-finite IK protection, joint-limit clamping,
and ready-pose handling remain unchanged.

No additional deadband, stationary freeze, coordinate conversion, or
orientation mode is introduced in this repair. Those changes would obscure
whether anti-windup fixed the observed failure.

## Testing

Implementation follows red-green-refactor.

1. Add a failing controller test proving that a requested target outside the
   measured-joint lead window cannot leave the applied state beyond the value
   actually assigned to the drive.
2. Add a failing follower test proving that a new waypoint is based on the
   constrained drive target rather than an unconstrained trajectory state.
3. Run the focused EditMode tests, then all related UR5 control tests.
4. Run a Tuanjie batch compilation/build check.
5. Build and run on Quest 3 and compare a new CSV against this baseline.

The Quest run passes the anti-windup check when:

- logical joint commands no longer run to joint limits during ordinary
  hand-held operation;
- `abs(joint_target - measured_joint)` remains bounded by the configured
  command pipeline instead of growing without limit;
- with Grip held and the hand stationary, drive targets settle instead of
  continuing a large directional sweep;
- no free-fall, non-finite IK result, or control-writer conflict occurs.

Fine responsiveness and orientation ergonomics will be evaluated after this
root fix. If input-to-filtered-target lag is still objectionable while joint
tracking is stable, smoothing and orientation mapping will be tuned in a
separate, evidence-isolated change.

## Scope

In scope:

- synchronize the applied joint state with the constrained drive command;
- seed DLS waypoints from the actual drive target;
- focused regression tests and Quest telemetry validation.

Out of scope:

- replacing the UR10-style anchored pose mapping;
- changing coordinate frames or controller gains;
- adding deadbands or extra filters;
- enabling real UR5 output;
- implementing the PC-side RTDE `servoL` architecture.
