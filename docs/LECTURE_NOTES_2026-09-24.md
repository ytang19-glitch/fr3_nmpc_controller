# Lecture notebook — fast, smooth FR3 manipulation (24 September 2026)

## Research question

**How fast can the FR3 carry and place an object while maintaining tracking accuracy, respecting joint and actuator limits, and avoiding slip or drops?**

The research target is a *measured task outcome*, rather than simply making the robot accelerate faster. Plan a feasible reference, track it with feedback, and measure how speed affects jerk, torque, object retention, and placement accuracy.

## From today's notes to a control pipeline

```text
Pick/place goal → MoveIt collision-free path → time parameterization / smooth reference
              → future reference samples → constrained receding-horizon controller
              → robot command → measured state → next optimization
```

A reference is a desired motion `q_ref(t), dq_ref(t), ddq_ref(t)`, not a command that guarantees motion. MoveIt supplies a geometric path and, after time parameterization, timed waypoints. A tracking controller uses current measurements to follow it. For the initial experiment, compare several durations of the **same path** so speed is the primary variable.

### Minimum jerk, acceleration, and object slip

Jerk is the *rate of change of acceleration*: `j = d³q/dt³ = d(ddq)/dt`. Reducing jerk can soften abrupt changes, but low jerk alone does not ensure that an object stays in the gripper. High acceleration, contact force, friction, grip force, orientation, and payload also matter. Record object slip/drop directly; do not treat jerk as a substitute for that measurement.

For a stationary-to-stationary point-to-point joint move, a simple reference is

```math
q_{ref}(t)=q_0+(q_f-q_0)(10s^3-15s^4+6s^5),\qquad s=\mathrm{clip}(t/T,0,1).
```

The repository implements this polynomial in [`scripts/publish_minimum_jerk_reference.py`](../scripts/publish_minimum_jerk_reference.py) for a dry-run joint reference. Scaling the duration `T` changes peak velocity approximately as `1/T`, acceleration as `1/T²`, and jerk as `1/T³`. Use this to design a speed sweep. For obstacle avoidance, retain the MoveIt path and apply appropriate time parameterization; a single joint-space polynomial between endpoints does not check collisions.

## What the current repository actually solves

[`scripts/offline_nmpc_demo.py`](../scripts/offline_nmpc_demo.py) uses seven joints, state `x=[q; dq]`, acceleration input `a=ddq`, a double-integrator prediction model, CasADi, and IPOPT. It repeatedly solves a finite-horizon problem and applies only the first predicted acceleration **in simulation**. Its discrete model is

```math
q_{k+1}=q_k+\Delta t\dot q_k+\tfrac12\Delta t^2a_k,\qquad
\dot q_{k+1}=\dot q_k+\Delta t a_k.
```

With this linear model, quadratic objective, and box bounds, the offline example is **constrained linear-model MPC**, even though the file is named `offline_nmpc_demo.py`. CasADi and IPOPT are optimization tools; their use does not make the robot model nonlinear. There are no FR3 torque commands.

The current stage cost penalizes position error, velocity error, acceleration magnitude, and the change in acceleration `a_k-a_{k-1}`; it also has a terminal tracking cost. If the interval is fixed, `(a_k-a_{k-1})/Δt` approximates jerk. The current code penalizes the unscaled *difference*, so changing `Δt` also changes the physical interpretation of that weight.

A candidate research objective is

```math
J=\sum_{k=0}^{N-1}
\bigl(
\|q_k-q_k^{ref}\|_Q^2+
\|\dot q_k-\dot q_k^{ref}\|_V^2+
\|u_k\|_R^2+
w_j\|(a_k-a_{k-1})/\Delta t\|^2
\bigr)
+\|x_N-x_N^{ref}\|_{Q_f}^2.
```

Here the precise input and jerk terms depend on the chosen model. In the present offline model `u=a`. For torque NMPC, `u=τ`, acceleration is computed from dynamics, and a penalty on `τ_k-τ_{k-1}` means **torque rate**, not jerk. Set hard bounds separately: weights trade objectives, while bounds define limits.

## Jacobian and manipulability

The manipulator Jacobian maps joint velocity to end-effector twist:

```math
v_{EE}=J(q)\dot q.
```

A manipulability measure such as `\sqrt{\det(JJ^T)}` indicates local ability to move in different Cartesian directions. Close to singular configurations, some motions become difficult. It is configuration dependent and requires a defined Jacobian and scaling convention for mixed translation/rotation units. A manipulability reward may compete with tracking, collision clearance, and joint limits, so first **log** it along the path; add it to the optimizer only after a baseline works. For a seven-joint arm, redundant configurations can realize similar end-effector poses with different manipulability.

## Path from today's lecture to a publishable experiment

1. **Reference baseline:** select one repeatable pick/place path, record the MoveIt joint trajectory, inspect collision clearance and timestamps, and compare its time parameterization with a minimum-jerk point-to-point reference where appropriate.
2. **Offline speed sweep:** run the same path at increasing speeds using the current double-integrator MPC. Report RMS/max joint tracking error, velocity and acceleration bounds, peak estimated jerk, solve-time distribution, and infeasible solves. Add disturbances and model mismatch; compare open loop and receding-horizon feedback.
3. **Nonlinear dynamics:** verify `M(q)ddq+C(q,dq)dq+g(q)=τ` and payload estimates in simulation. Then formulate torque-input NMPC with joint, velocity, torque, and torque-rate constraints. Validate how acceleration and jerk are computed from predicted states.
4. **Physical task:** compare a standard trajectory controller, a well-tuned feedback baseline, and the proposed controller on the **same** path, speeds, payload, and gripper setting. Measure tracking, cycle time, slip/drop rate, placement error, constraints, and solver deadlines across repeated trials.
5. **Research contribution:** identify where a constrained controller improves the speed–reliability tradeoff; test ablations for reference timing, jerk penalty, and manipulability only when the earlier baseline reveals a specific limitation.

A PID loop by itself does not optimize predicted constraints; a complete robot system may still enforce limits outside PID. NMPC is not automatically faster or safer than PID. Compare under matched conditions and count missed deadlines. The existing README records multi-millisecond offline solves, so this Python/IPOPT implementation has not demonstrated 1 kHz hardware control. Follow [`REAL_ROBOT_TEST_PLAN.md`](REAL_ROBOT_TEST_PLAN.md) before any hardware command.

## Immediate next notebook entries

- Sketch one FR3 pick/place path and choose three feasible duration values.
- Plot `q_ref, dq_ref, ddq_ref` and estimate peak jerk for each duration.
- Log minimum singular value (or a carefully defined manipulability measure) along that path.
- State the main hypothesis: **for the same path, at shorter durations, constrained feedback improves task success or tracking while respecting limits**. Accept that the result may show no improvement.
- Read one paper on high-frequency manipulator NMPC, one on robotic time parameterization/jerk constraints, and one on manipulation with object slip or acceleration limits. Record each paper's robot model, input, solver, frequency, reference, baseline, and task metric.

**Working title:** *Fast and reliable FR3 pick-and-place through smooth reference timing and constrained feedback control.*
