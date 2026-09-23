#!/usr/bin/env python3

"""Offline seven-joint NMPC demo. It never connects to robot hardware."""

import math

import casadi as ca
import numpy as np


DOF = 7
DT = 0.02
HORIZON_STEPS = 20
SIMULATION_STEPS = 250


def minimum_jerk_reference(time_seconds: float, duration: float = 4.0) -> np.ndarray:
    q_start = np.zeros(DOF)
    q_goal = np.array([0.12, -0.10, 0.08, -0.12, 0.06, 0.08, -0.05])
    s = np.clip(time_seconds / duration, 0.0, 1.0)
    blend = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    blend_rate = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / duration
    q = q_start + (q_goal - q_start) * blend
    dq = (q_goal - q_start) * blend_rate
    return np.concatenate((q, dq))


def build_solver() -> tuple[ca.Function, int]:
    state_size = 2 * DOF
    x = ca.SX.sym("x", state_size, HORIZON_STEPS + 1)
    acceleration = ca.SX.sym("a", DOF, HORIZON_STEPS)
    initial_state = ca.SX.sym("x0", state_size)
    reference = ca.SX.sym("reference", state_size, HORIZON_STEPS + 1)
    previous_acceleration = ca.SX.sym("a_previous", DOF)

    q_weight = ca.diag(ca.DM([80.0] * DOF))
    dq_weight = ca.diag(ca.DM([8.0] * DOF))
    acceleration_weight = ca.diag(ca.DM([0.08] * DOF))
    rate_weight = ca.diag(ca.DM([0.25] * DOF))

    objective = 0.0
    constraints = [x[:, 0] - initial_state]

    for step in range(HORIZON_STEPS):
        q_error = x[:DOF, step] - reference[:DOF, step]
        dq_error = x[DOF:, step] - reference[DOF:, step]
        delta_acceleration = (
            acceleration[:, step] - previous_acceleration
            if step == 0
            else acceleration[:, step] - acceleration[:, step - 1]
        )
        objective += ca.mtimes([q_error.T, q_weight, q_error])
        objective += ca.mtimes([dq_error.T, dq_weight, dq_error])
        objective += ca.mtimes(
            [acceleration[:, step].T, acceleration_weight, acceleration[:, step]]
        )
        objective += ca.mtimes(
            [delta_acceleration.T, rate_weight, delta_acceleration]
        )

        q_next = (
            x[:DOF, step]
            + DT * x[DOF:, step]
            + 0.5 * DT**2 * acceleration[:, step]
        )
        dq_next = x[DOF:, step] + DT * acceleration[:, step]
        constraints.append(x[:, step + 1] - ca.vertcat(q_next, dq_next))

    terminal_error = x[:, HORIZON_STEPS] - reference[:, HORIZON_STEPS]
    terminal_weight = ca.diag(ca.DM([160.0] * DOF + [16.0] * DOF))
    objective += ca.mtimes([terminal_error.T, terminal_weight, terminal_error])

    decision = ca.vertcat(
        ca.reshape(x, -1, 1), ca.reshape(acceleration, -1, 1)
    )
    parameters = ca.vertcat(
        initial_state, ca.reshape(reference, -1, 1), previous_acceleration
    )
    problem = {
        "x": decision,
        "f": objective,
        "g": ca.vertcat(*constraints),
        "p": parameters,
    }
    options = {
        "ipopt.print_level": 0,
        "print_time": False,
        "ipopt.max_iter": 100,
        "ipopt.tol": 1.0e-6,
    }
    solver = ca.nlpsol("nmpc", "ipopt", problem, options)
    state_variables = state_size * (HORIZON_STEPS + 1)
    return solver, state_variables


def run_demo() -> None:
    solver, state_variables = build_solver()
    state = np.zeros(2 * DOF)
    previous_acceleration = np.zeros(DOF)
    guess = np.zeros(2 * DOF * (HORIZON_STEPS + 1) + DOF * HORIZON_STEPS)

    lower_decision = np.full_like(guess, -np.inf)
    upper_decision = np.full_like(guess, np.inf)
    for step in range(HORIZON_STEPS + 1):
        offset = step * 2 * DOF
        lower_decision[offset : offset + DOF] = -2.7
        upper_decision[offset : offset + DOF] = 2.7
        lower_decision[offset + DOF : offset + 2 * DOF] = -1.0
        upper_decision[offset + DOF : offset + 2 * DOF] = 1.0
    lower_decision[state_variables:] = -2.0
    upper_decision[state_variables:] = 2.0

    equality_size = 2 * DOF * (HORIZON_STEPS + 1)
    lower_constraints = np.zeros(equality_size)
    upper_constraints = np.zeros(equality_size)

    errors = []
    solve_times = []
    for simulation_step in range(SIMULATION_STEPS):
        time_seconds = simulation_step * DT
        horizon_reference = np.column_stack(
            [
                minimum_jerk_reference(time_seconds + step * DT)
                for step in range(HORIZON_STEPS + 1)
            ]
        )
        parameters = np.concatenate(
            (state, horizon_reference.reshape(-1, order="F"), previous_acceleration)
        )

        start = __import__("time").perf_counter()
        solution = solver(
            x0=guess,
            lbx=lower_decision,
            ubx=upper_decision,
            lbg=lower_constraints,
            ubg=upper_constraints,
            p=parameters,
        )
        solve_times.append(1000.0 * (__import__("time").perf_counter() - start))
        decision = np.asarray(solution["x"]).reshape(-1)
        acceleration_sequence = decision[state_variables:].reshape(
            (DOF, HORIZON_STEPS), order="F"
        )
        command = acceleration_sequence[:, 0]

        state[:DOF] += DT * state[DOF:] + 0.5 * DT**2 * command
        state[DOF:] += DT * command
        previous_acceleration = command
        errors.append(np.linalg.norm(state[:DOF] - horizon_reference[:DOF, 0]))
        guess = decision

    print("Offline NMPC demonstration complete")
    print(f"Final position-error norm: {errors[-1]:.6f} rad")
    print(f"Maximum position-error norm: {max(errors):.6f} rad")
    print(f"Mean solve time: {np.mean(solve_times):.3f} ms")
    print(f"Maximum solve time: {max(solve_times):.3f} ms")
    print("No robot commands were produced.")


if __name__ == "__main__":
    run_demo()
