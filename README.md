# FR3 NMPC Controller

Research scaffold for testing reference-trajectory tracking before connecting a nonlinear MPC controller to a real Franka FR3.

The repository currently provides:

- a ROS 2 C++ reference monitor that consumes `trajectory_msgs/JointTrajectory`;
- cubic-Hermite sampling of position and velocity references;
- joint-name, timing, dimension, and finite-value validation;
- a minimum-jerk reference publisher for conservative bench tests;
- an offline CasADi NMPC example using a constrained seven-joint double-integrator model;
- a staged real-robot test plan.

> [!WARNING]
> This version is intentionally **dry-run only**. It does not claim effort interfaces and does not command robot torque. Do not connect experimental NMPC torque output to the FR3 until the model, limits, watchdog, torque-rate limiter, fallback controller, and real-time behavior have been reviewed with the laboratory supervisor.

## Intended architecture

```text
MoveIt trajectory
      |
      v
Reference monitor / sampler ---- measured joint states
      |
      v
NMPC solver (next stage)
      |
      v
Safety filter + fallback (next stage)
      |
      v
FR3 effort controller (only after staged validation)
```

## Platform

Designed as a starting point for:

- Ubuntu 24.04
- ROS 2 Jazzy
- Franka FR3
- `franka_ros2` 3.x
- `libfranka` 0.20.x

The dry-run package itself uses standard ROS 2 messages and does not require Franka hardware.

## Clone and build

```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/ytang19-glitch/fr3_nmpc_controller.git

cd ~/franka_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fr3_nmpc_controller --symlink-install
source install/setup.bash
```

## 1. Start the dry-run reference monitor

```bash
ros2 launch fr3_nmpc_controller reference_monitor.launch.py
```

By default it reads:

- measured state: `/joint_states`
- reference trajectory: `/nmpc/reference_trajectory`
- sampled reference: `/nmpc/reference_state`
- tracking error: `/nmpc/tracking_error`

It never publishes effort commands.

## 2. Publish a conservative minimum-jerk reference

Keep the robot controller in its normal safe mode. The script reads the current joint state, adds a small displacement to one selected joint, and publishes a smooth reference for monitoring only.

```bash
ros2 run fr3_nmpc_controller publish_minimum_jerk_reference.py --ros-args \
  -p joint_index:=0 \
  -p displacement:=0.02 \
  -p duration:=8.0
```

The default displacement is `0.02 rad`. Publishing this message does not move the robot.

Inspect the sampled reference and tracking error:

```bash
ros2 topic echo /nmpc/reference_state
ros2 topic echo /nmpc/tracking_error
```

## 3. Feed a MoveIt trajectory

Plan with MoveIt, but do not call `execute()` when testing this monitor. Publish the planned joint trajectory instead:

```cpp
const auto & trajectory = plan.trajectory.joint_trajectory;
reference_publisher->publish(trajectory);
```

The trajectory should contain all seven FR3 joints, strictly increasing `time_from_start`, positions, and velocities at every point. Apply MoveIt time parameterization and jerk smoothing before publishing it.

## 4. Run the offline NMPC demonstration

Install Python dependencies in an isolated environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install casadi numpy
python3 scripts/offline_nmpc_demo.py
```

The demonstration solves a receding-horizon problem with:

- state: joint position and velocity;
- input: joint acceleration;
- position, velocity, and acceleration bounds;
- position/velocity tracking cost;
- input and input-rate regularization.

This simplified model is for controller development only. A real torque NMPC must use FR3 rigid-body dynamics:

```math
M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau.
```

## Development stages

1. Validate reference generation and interpolation with this dry-run monitor.
2. Validate the NMPC formulation offline and in simulation.
3. Add an FR3 dynamics backend and compare predicted motion against recorded robot data.
4. Add a C++ real-time solver, watchdog, torque and torque-rate limits, and impedance fallback.
5. Test posture hold, then one small joint motion, then slow multi-joint references.
6. Only then test time-parameterized MoveIt trajectories and gradually increase speed.

See [docs/REAL_ROBOT_TEST_PLAN.md](docs/REAL_ROBOT_TEST_PLAN.md) for acceptance gates and measurements.

## Research comparison

Use the same time-parameterized MoveIt reference for every controller:

- standard joint trajectory controller;
- joint impedance or computed-torque controller;
- closed-loop NMPC.

Record tracking error, Cartesian error, torque, torque rate, solver time, deadline misses, constraint violations, cycle time, and task success rate.

## License

MIT
