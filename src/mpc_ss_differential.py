import casadi
import matplotlib.pyplot as plt
import numpy as np
import timeit
from matplotlib import animation


class ModelParameters:
    def __init__(self):
        self.sampling_time = 0.1  # Sampling time [s]
        self.n_steps = 50  # Prediction horizon steps [-]
        self.time_initial = 0  # Simulation start time [s]
        self.time_span = 7.5  # Simulation end time [s]
        self.map_limits = (-3.0, 3.0)  # Map extents in the X direction
        self.state_initial = [0.0, 0.0, 0.0]  # AGV initial state
        self.state_reference = [2.0, -1.0, np.pi / 4]  # AGV final state
        self.v_max = 0.5  # Minimum longitudinal speed [m/s]
        self.v_min = -0.5  # Maximum longitudinal speed [m/s]
        self.omega_min = -np.pi / 3  # Minimum angular speed [m/s]
        self.omega_max = np.pi / 3  # Maximum angular speed [m/s]
        self.solver_linear = 'ma27'  # IPOPT linear solver (mumps, ma27)


def main():
    model_params = ModelParameters()

    bounds, f, solver = control_problem(model_params)

    ###################
    # Simulation loop #
    ###################

    time_solver_start = timeit.default_timer()

    x_0 = casadi.vertcat(model_params.state_initial)  # Initial state
    x_goal = casadi.vertcat(model_params.state_reference)  # Reference state

    # Initialise history of the states.
    time_steps = int(model_params.time_span / model_params.sampling_time)
    states_simulated = np.zeros((x_0.shape[0], time_steps + 1))
    states_simulated[:, :0] = x_0[:, 0]

    t = np.zeros(time_steps)
    t[0] = model_params.time_initial
    u_0 = np.zeros((model_params.n_steps, 2))

    i = 0
    u_control_actions = np.zeros((1, 2))

    time = model_params.time_initial

    while casadi.norm_2(x_0 - x_goal) > 1e-2 and i < time_steps:
        # print(f"""casadi norm value: {casadi.norm_2(x_0 - x_goal)}""")
        # Set the values of the parameters vector.
        bounds['p'] = casadi.vertcat(x_0, x_goal)

        # Initial value of the optimisation.
        bounds['x0'] = casadi.reshape(u_0.T, 2 * model_params.n_steps, 1)

        the_solver = solver(
            x0=bounds['x0'],
            lbx=bounds['lbx'],
            ubx=bounds['ubx'],
            lbg=bounds['lbg'],
            ubg=bounds['ubg'],
            p=bounds['p']
        )

        u = casadi.reshape((the_solver['x']).T, 2, model_params.n_steps).T

        # Save first control action
        if i == 0:
            u_control_actions[0] = u[0, :]
        else:
            u_control_actions = casadi.vertcat(u_control_actions, u[0, :])

        # Set the initial state and control.
        x_state = x_0
        u_control = u[0, :].T

        # Compute the function for the given state and control.
        f_value = f(x_state, u_control)

        # Advance the state using an Euler integration step.
        x_state = x_state + (model_params.sampling_time * f_value)

        # Advance time.
        x_0 = x_state.full()
        u_0 = casadi.vertcat(u[1::u.shape[1]], u[u.shape[0]:])

        # Store the current time and state.
        t[i] = time
        states_simulated[:, i:i + 1] = x_0

        # Advance time and i counter.
        time += 1
        i += 1

    time_solver_end = timeit.default_timer()
    elapsed_time = time_solver_end - time_solver_start

    print(f'''
    MPC with linear solver {model_params.solver_linear.upper()} took {elapsed_time :0.2f} s
    for {model_params.time_span} s of simulation
    ''')

    plot_results(states_simulated, t, u_control_actions)
    print("plots done")

    # ignore for now
    trajectory_animation(model_params, states_simulated)


def control_problem(model_params):
    f, n_controls, n_states = problem_dynamics()

    ############################################
    # Optimal control problem (OCP) definition #
    ############################################
    P_parameter, U_controls, X_states = define_ocp_variables(f, model_params, n_controls, n_states)
    phi = create_objective_function(P_parameter, U_controls, X_states, model_params)
    g = state_constraints(X_states, model_params)
    bounds = create_bounds(model_params)

    # Define a nonlinear programming problem (NLP).
    variables = casadi.reshape(U_controls, 2 * model_params.n_steps, 1)
    nlp = {'f': phi, 'x': variables, 'g': g, 'p': P_parameter}
    opts = {
        'ipopt.print_level': 0,
        'ipopt.sb': 'yes',
        'print_time': 0,
        'ipopt.tol': 1e-3,
        'ipopt.max_iter': 20,
        'ipopt.linear_solver': model_params.solver_linear
    }
    solver = casadi.nlpsol('solver', 'ipopt', nlp, opts)
    return bounds, f, solver


def create_bounds(model_params):
    lbx = np.zeros(2 * model_params.n_steps)
    ubx = np.zeros(2 * model_params.n_steps)
    lbx[0::2] = model_params.v_min
    ubx[0::2] = model_params.v_max
    lbx[1::2] = model_params.omega_min
    ubx[1::2] = model_params.omega_max
    bounds = {
        'lbg': model_params.map_limits[0],
        'ubg': model_params.map_limits[1],
        'lbx': lbx,
        'ubx': ubx
    }
    return bounds


def state_constraints(X, model_params):
    # Add state constraints.
    g = []
    for k in range(0, model_params.n_steps + 1):
        g = casadi.vertcat(g, X[0, k])  # Constraint for the X position
        g = casadi.vertcat(g, X[1, k])  # Constraint for the Y position
    return g


def create_objective_function(P, U, X, model_params):
    Q = np.zeros((3, 3))  # Reference state weights
    Q[0, 0] = 5.0
    Q[1, 1] = 5.0
    Q[2, 2] = 0.5
    R = np.zeros((2, 2))  # Control weights
    R[0, 0] = 0.5
    R[1, 1] = 0.05
    # Sum up all the contributions to the objective function.
    phi = 0
    for k in range(0, model_params.n_steps):
        current_state = X[:, k]
        current_control = U[:, k]
        phi += (
                (current_state - P[3:6]).T @ Q @ (current_state - P[3:6]) +
                current_control.T @ R @ current_control
        )
    return phi


def define_ocp_variables(f, model_params, n_controls, n_states):
    # Define the decision variables (controls).
    # There are as many control actions as prediction steps.
    U = casadi.SX.sym('U', n_controls, model_params.n_steps)
    # Define the problem parameters.
    # These include the initial state and the reference state of the robot.
    P = casadi.SX.sym('P', 2 * n_states)
    # Define the states over the optimisation problem.
    # The size is n_states x (1 + n_steps) as it stores all states for the initial
    # state of the robot and all prediction steps.
    X = casadi.SX.sym('X', n_states, model_params.n_steps + 1)
    # Initialise the prediction states with the initial state of the robot.
    X[:, 0] = P[0:3]
    # Perform integration using Euler method.
    for k in range(0, model_params.n_steps):
        current_state = X[:, k]
        current_control = U[:, k]
        f_linearised = f(current_state, current_control)
        X[:, k + 1] = current_state + (model_params.sampling_time * f_linearised)
    return P, U, X


def problem_dynamics():
    # Define the states
    x = casadi.SX.sym('x')
    y = casadi.SX.sym('y')
    theta = casadi.SX.sym('theta')
    states = casadi.vertcat(x, y, theta)
    n_states = states.shape[0]
    # Define the controls
    v = casadi.SX.sym('v')
    omega = casadi.SX.sym('omega')
    controls = casadi.vertcat(v, omega)
    n_controls = controls.shape[0]
    # Define the RHS of the motion model equation.
    rhs = casadi.vertcat(v * casadi.cos(theta), v * casadi.sin(theta), omega)
    # Define a function for the robot kinematics.
    # The function receives the state of the robot and the controls and
    # returns the RHS of the model equation.
    f = casadi.Function('f', [states, controls], [rhs])
    return f, n_controls, n_states


def plot_results(states_simulated, t, u_cl):
    ################
    # Plot results #
    ################
    plt.style.use('bmh')
    # Robot controls plot.
    figure, axes = plt.subplots(2, 1,
                                figsize=(8.0, 6.0)
                                )
    axes[0].plot(t, u_cl[:, 0])
    axes[0].set_xlim((0, None))
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Speed (m/s)')
    axes[1].plot(t, u_cl[:, 1])
    axes[1].set_xlim((0, None))
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Yaw rate (rad/s)')
    figure.tight_layout()
    figure.align_ylabels()
    figure.savefig('assets/img/dd-controls.png')
    # Robot states plot.
    figure, axes = plt.subplots(3, 1,
                                figsize=(8.0, 6.0)
                                )
    axes[0].plot(t, states_simulated[0, :-1])
    axes[0].set_xlim((t[0], t[-1]))
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('X coordinate (m)')
    axes[1].plot(t, states_simulated[1, :-1])
    axes[1].set_xlim((t[0], t[-1]))
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Y coordinate (m)')
    axes[2].plot(t, states_simulated[2, :-1])
    axes[2].set_xlim((t[0], t[-1]))
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Yaw (rad)')
    figure.tight_layout()
    figure.align_ylabels()
    figure.savefig('assets/img/dd-states.png')


def trajectory_animation(model_params, states_simulated):
    ########################
    # Trajectory animation #
    ########################
    figure, axes = plt.subplots(1, 1,
                                figsize=(8.0, 8.0)
                                )
    figure.tight_layout()
    axes.set_xlim(model_params.map_limits[0], model_params.map_limits[1])
    axes.set_ylim(model_params.map_limits[0], model_params.map_limits[1])
    axes.set_aspect('equal', 'box')
    axes.set_xlabel('X coordinate (m)')
    axes.set_ylabel('Y coordinate (m)')
    line, = axes.plot([], [])
    patch_robot_envelope = plt.Circle((0.0, 0.0), 0.45)
    patch_robot_orientation = plt.Circle((0.1, 0.0), 0.1,
                                         color=(1.0, 1.0, 0.0)
                                         )
    # Add an arrow for the reference state.
    axes.add_patch(
        plt.Arrow(
            model_params.state_reference[0],
            model_params.state_reference[1],
            0.5 * casadi.cos(model_params.state_reference[2]),
            0.5 * casadi.sin(model_params.state_reference[2]),
            width=0.25,
            color=[1.0, 0.0, 0.0],
            zorder=100
        )
    )

    axes.add_patch(patch_robot_envelope)
    axes.add_patch(patch_robot_orientation)
    line.set_data([], [])

    animation_function = animation.FuncAnimation(figure, animate_robot,
                                                 fargs=(
                                                 states_simulated, line, patch_robot_envelope, patch_robot_orientation),
                                                 # init_func=initialise_animation,
                                                 frames=len(states_simulated[0, :]),
                                                 interval=100,
                                                 blit=True
                                                 )
    animation_function.save('assets/img/dd-trajectory.gif',
                            writer='imagemagick',
                            fps=5
                            )


def initialise_animation():
    axes.add_patch(patch_robot_envelope)
    axes.add_patch(patch_robot_orientation)
    line.set_data([], [])

    return []


def animate_robot(i, states_simulated, line, patch_robot_envelope, patch_robot_orientation):
    animate_envelope(i, states_simulated, line, patch_robot_envelope)
    animate_orientation(i, states_simulated, patch_robot_orientation)
    return []


def animate_envelope(i, states_simulated, line, patch_robot_envelope):
    x = states_simulated[0, :i]
    y = states_simulated[1, :i]
    line.set_data(x, y)
    patch_robot_envelope.center = (states_simulated[0, i], states_simulated[1, i])
    return patch_robot_envelope,


def animate_orientation(i, states_simulated, patch_robot_orientation):
    patch_robot_orientation.center = (
        states_simulated[0, i] + 0.25 * casadi.cos(states_simulated[2, i]),
        states_simulated[1, i] + 0.25 * casadi.sin(states_simulated[2, i])
    )
    return patch_robot_orientation,


if __name__ == '__main__':
    main()
