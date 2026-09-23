# FR3 NMPC Controller

Research project for tracking smooth MoveIt reference trajectories with nonlinear model predictive control on a Franka FR3.

> [!IMPORTANT]
> The project is divided into two parts:
>
> 1. **Offline controller development:** prove the optimization, reference tracking, constraints, and timing without robot hardware.
> 2. **Online robot control:** connect the validated controller to live FR3 measurements and eventually command safe joint torque.

> [!WARNING]
> The repository does **not yet contain a hardware-ready FR3 torque NMPC controller**. The ROS 2 tools currently operate in read-only/dry-run mode and never publish effort commands.

## Current status

| Capability | Status |
|---|---|
| Generate a smooth minimum-jerk reference | Implemented |
| Solve a seven-joint constrained MPC problem offline | Implemented |
| Measure offline tracking error and solution time | Implemented |
| Receive live `/joint_states` | Implemented |
| Receive and interpolate a MoveIt `JointTrajectory` | Implemented |
| Publish sampled reference and tracking error | Implemented |
| Full nonlinear FR3 rigid-body dynamics | Not implemented |
| Real-time C++ NMPC solver | Not implemented |
| Torque/rate safety filter and fallback controller | Not implemented |
| Command FR3 effort interfaces | Not implemented |

# Part I — Offline controller development

## Why test the controller offline first?

An MPC/NMPC controller contains several independent elements:

- a robot model;
- a reference trajectory;
- a prediction horizon;
- a cost function;
- constraints;
- a numerical optimizer;
- receding-horizon feedback.

Testing these first without hardware separates optimization problems from ROS, network, timing, and robot-safety problems.

Offline testing answers the following questions safely:

1. Does the optimizer converge?
2. Does the predicted state follow the reference?
3. Are position, velocity, acceleration, and input constraints respected?
4. How do horizon length and cost weights change the result?
5. How long does every optimization take?
6. What happens when the problem becomes infeasible?

If these questions are unresolved offline, connecting the optimizer to a real FR3 makes diagnosis harder and may produce unsafe commands.

## What the current offline example does

The script is:

```text
scripts/offline_nmpc_demo.py
```

It uses the state

```math
x = \begin{bmatrix}q & \dot q\end{bmatrix}^{T}
```

and joint acceleration as its input:

```math
u = \ddot q.
```

The prediction model is:

```math
q_{k+1}=q_k+\Delta t\dot q_k+\frac{1}{2}\Delta t^2u_k
```

```math
\dot q_{k+1}=\dot q_k+\Delta t u_k.
```

At every simulation step, the optimizer predicts a sequence of future inputs but applies only the first input. It then receives the new simulated state and solves again.

### Scientific limitation

The current plant model is a **double integrator**, so this example validates the MPC structure and software pipeline. It is not yet the final nonlinear FR3 controller.

The final torque-controlled model must include:

```math
M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau,
```

with torque input:

```math
u=\tau.
```

## Run the offline demonstration

No ROS 2 or Franka hardware is required.

```bash
cd ~/fr3_nmpc_ws/src/fr3_nmpc_controller

python3 -m venv .venv
source .venv/bin/activate
pip install casadi numpy

python3 scripts/offline_nmpc_demo.py
```

## Measured result

A test on September 23, 2026 produced:

```text
Offline NMPC demonstration complete
Final position-error norm: 0.000031 rad
Maximum position-error norm: 0.002039 rad
Mean solve time: 7.634 ms
Maximum solve time: 13.266 ms
No robot commands were produced.
```

Interpretation:

- the simplified simulated system tracked its reference accurately;
- the optimizer completed successfully throughout this test;
- the mean time corresponds to roughly 131 solutions per second;
- the worst observed time corresponds to roughly 75 solutions per second;
- this Python/IPOPT implementation is therefore **not a 1 kHz controller**;
- accurate offline tracking does not yet prove stability or safety on the real FR3.

## Next offline experiments

Keep the same reference and compare:

| Experiment | Change | Question |
|---|---|---|
| Horizon study | 10, 20, 30 steps | How do preview and solution time change? |
| Speed study | Reduce trajectory duration | When does tracking error increase? |
| Weight study | Change tracking and input weights | How does aggressiveness change? |
| Constraint study | Tighten velocity/acceleration bounds | When does the problem become infeasible? |
| Disturbance study | Add state disturbance/model error | Does feedback recover? |

The next major implementation step is replacing the double-integrator model with verified FR3 rigid-body dynamics and testing it in simulation.

# Part II — Online FR3 integration

## What “online” means

Online control uses the latest robot measurement in every feedback cycle:

```text
MoveIt reference trajectory
          |
          v
Future reference sampler
          |
          v
NMPC solver <---------- measured q and dq
          |
          v
Torque and torque-rate safety filter
          |
          v
FR3 effort interface
          |
          +------------> new measured q and dq
```

At each update:

1. read measured joint position and velocity;
2. sample the future reference across the prediction horizon;
3. solve the NMPC problem;
4. apply only the first safe torque command;
5. repeat using the new measured state.

## Current online capability: read-only dry run

The current C++ node can:

- subscribe to `/joint_states`;
- receive `trajectory_msgs/msg/JointTrajectory`;
- validate joint names, dimensions, timestamps, and finite values;
- interpolate position and velocity references;
- publish the sampled reference;
- calculate joint-position tracking error.

It cannot move the FR3.

### Build the ROS 2 package

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/fr3_nmpc_ws/src
cd ~/fr3_nmpc_ws/src
git clone https://github.com/ytang19-glitch/fr3_nmpc_controller.git

cd ~/fr3_nmpc_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fr3_nmpc_controller --symlink-install
source ~/fr3_nmpc_ws/install/setup.bash
```

If the repository is already cloned:

```bash
cd ~/fr3_nmpc_ws/src/fr3_nmpc_controller
git pull origin main

cd ~/fr3_nmpc_ws
colcon build --packages-select fr3_nmpc_controller --symlink-install
source ~/fr3_nmpc_ws/install/setup.bash
```

Do not source `/opt/franka_ros2_ws/install/setup.bash` unless that exact file exists on the computer. When Franka-specific packages are added later, first locate and source the actual Franka workspace installed on the system.

### Start the reference monitor

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fr3_nmpc_ws/install/setup.bash
ros2 launch fr3_nmpc_controller reference_monitor.launch.py
```

The node uses:

| Topic | Purpose |
|---|---|
| `/joint_states` | Measured FR3 joint state |
| `/nmpc/reference_trajectory` | Incoming reference trajectory |
| `/nmpc/reference_state` | Interpolated reference |
| `/nmpc/tracking_error` | Reference position minus measured position |

### Publish a minimum-jerk reference for monitoring

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fr3_nmpc_ws/install/setup.bash

ros2 run fr3_nmpc_controller publish_minimum_jerk_reference.py --ros-args \
  -p joint_index:=0 \
  -p displacement:=0.02 \
  -p duration:=8.0
```

The publisher waits for a complete `/joint_states` message and builds the reference from the current measured posture. It does not send a robot command.

## MoveIt reference integration

For the research experiment, MoveIt should generate a collision-free, time-parameterized joint trajectory:

```math
q_{ref}(t),\quad \dot q_{ref}(t),\quad \ddot q_{ref}(t).
```

During dry-run validation, plan the trajectory but do not execute it through the standard trajectory controller. Publish it to the reference topic:

```cpp
const auto & trajectory = plan.trajectory.joint_trajectory;
reference_publisher->publish(trajectory);
```

Apply time parameterization and jerk smoothing before using the trajectory as an NMPC reference.

## What must be implemented before online torque control

The following components are mandatory before commanding the real robot:

1. verified FR3 mass, Coriolis, gravity, and payload model;
2. C++ solver with bounded worst-case execution time;
3. non-blocking separation between optimization and the 1 kHz hardware loop;
4. joint-position and velocity safety margins;
5. torque saturation and torque-rate saturation;
6. finite-value and stale-state checks;
7. stale-solution watchdog;
8. gravity-aware impedance fallback controller;
9. safe controller activation, switching, and deactivation;
10. controlled tests for solver timeout and infeasibility.

The first hardware experiment must be posture holding at the measured activation pose. Progress afterward to one small, slow joint movement and only then to multi-joint MoveIt references.

See [docs/REAL_ROBOT_TEST_PLAN.md](docs/REAL_ROBOT_TEST_PLAN.md) for the staged acceptance gates.

# Research comparison

Use the same time-parameterized MoveIt trajectory for:

1. the standard joint-trajectory controller;
2. joint impedance or computed-torque control;
3. closed-loop NMPC.

Measure:

- RMS and maximum joint error;
- Cartesian tracking error;
- peak torque and torque rate;
- mean, maximum, and 99th-percentile solution time;
- missed deadlines and solver failures;
- constraint violations;
- cycle time;
- grasp success and object slip/drop.

This comparison answers the research question:

> As the reference trajectory becomes faster and more dynamically demanding, can closed-loop NMPC maintain tracking accuracy and constraint satisfaction better than conventional control?

# License

MIT
