# FR3 NMPC Real-Robot Test Plan

This plan uses acceptance gates. Do not advance when a gate fails.

## Gate 0: Hardware and communication

- Robot is firmly mounted and the workspace is clear.
- User stop is held by an observer or immediately reachable.
- Wired FCI connection is stable.
- The control computer passes the Franka communication test.
- Correct FR3 model, end-effector, gripper, and payload are configured.
- Joint states are finite, correctly ordered, and fresh.
- Only one arm controller can claim the command interfaces.

**Pass condition:** no communication failures or unexpected controller transitions during a stationary logging trial.

## Gate 1: Reference pipeline

- Start `reference_monitor.launch.py`.
- Feed minimum-jerk references without commanding the robot.
- Feed a time-parameterized MoveIt trajectory.
- Confirm the first reference matches the measured initial state.
- Confirm position and velocity are continuous.
- Confirm every point respects configured position, velocity, acceleration, and jerk limits.

**Pass condition:** every malformed trajectory is rejected and every valid trajectory is sampled continuously.

## Gate 2: Offline and simulated NMPC

- Use the complete state `x = [q, dq]`.
- Add position, velocity, input, and input-rate constraints.
- Warm-start from the previous solution.
- Inject model mismatch, sensor noise, and disturbances.
- Force solver timeout and infeasibility events.

**Pass condition:** bounded tracking error, no constraint violation, and safe fallback in every forced solver-failure test.

## Gate 3: Real-time controller shell

Before real torque output, implement and review:

- non-blocking communication between solver and 1 kHz update loop;
- stale-solution watchdog;
- finite-value checks;
- torque saturation;
- torque-rate saturation;
- joint position and velocity safety margins;
- collision thresholds appropriate to the laboratory;
- gravity-aware joint-impedance fallback;
- safe activation and deactivation behavior;
- logging that does not block the real-time loop.

**Pass condition:** deadline and failure-injection tests pass without hardware connected.

## Gate 4: Posture hold

Use the measured activation posture as the reference:

```math
q_ref = q_measured, \qquad \dot q_ref = 0.
```

Start with conservative torque and torque-rate bounds. Do not use MoveIt or vision.

**Stop immediately for:** oscillation, drift, saturation, stale state, repeated solver failure, reflex stop, or communication error.

**Pass condition:** repeatable stationary hold with bounded error and no safety event.

## Gate 5: Small joint-space motion

Progress in this order:

1. one joint, `0.02 rad`, at least `8 s`;
2. one joint, forward and return;
3. two joints;
4. slow seven-joint minimum-jerk trajectory;
5. slow MoveIt reference.

Increase only one experimental variable at a time.

**Pass condition:** repeatable tracking, no saturation, no missed deadline, no solver failure, and acceptable final error.

## Gate 6: Controller comparison

Track exactly the same reference using:

1. standard joint trajectory control;
2. joint impedance or computed-torque control;
3. closed-loop NMPC.

Record:

- `q_ref`, `q`, `dq_ref`, and `dq`;
- Cartesian reference and measured pose;
- commanded and measured torque;
- torque rate;
- solver status and solve time;
- missed deadlines and stale solutions;
- active constraints;
- cycle time and success/failure.

Report RMS error, maximum error, 95th/99th-percentile solve time, peak torque, peak torque rate, and constraint violations.

## Gate 7: Aggressive trajectories

Keep the geometric path fixed and increase velocity/acceleration scaling gradually. A faster trial is accepted only if all prior safety and performance criteria still pass.

For object transport, also measure grasp success and object slip/drop. Stop the campaign after any unexplained safety event and diagnose it before continuing.
