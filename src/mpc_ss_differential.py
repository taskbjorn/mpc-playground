import casadi as csd
import matplotlib.pyplot as plt
import numpy as np
import timeit
from matplotlib import animation

# Model parameters
sampling_time   = 0.1                  # Sampling time [s]
n_steps         = 50                   # Prediction horizon steps [-]
time_initial    = 0                    # Simulation start time [s]
time_span       = 7.5                  # Simulation end time [s]
map_limits      = (-3.0, 3.0)          # Map extents in the X direction
state_initial   = [0.0, 0.0, 0.0]      # AGV initial state
state_reference = [2.0, -1.0, np.pi/4] # AGV final state
v_max           = 0.5                  # Minimum longitudinal speed [m/s]
v_min           = -0.5                 # Maximum longitudinal speed [m/s]
omega_min       = -np.pi/3             # Minimum angular speed [m/s]
omega_max       = np.pi/3              # Maximum angular speed [m/s]
solver_linear   = 'ma27'               # IPOPT linear solver (mumps, ma27)

# Define the states
x = csd.SX.sym('x')
y = csd.SX.sym('y')
theta = csd.SX.sym('theta')
states = csd.vertcat(x, y, theta)
n_states = states.shape[0]

# Define the controls
v = csd.SX.sym('v')
omega = csd.SX.sym('omega')
controls = csd.vertcat(v, omega)
n_controls = controls.shape[0]

# Define the RHS of the motion model equation.
rhs = csd.vertcat(v * csd.cos(theta), v * csd.sin(theta), omega)

# Define a function for the robot kinematics.
# The function receives the state of the robot and the controls and
# returns the RHS of the model equation.
f = csd.Function('f', [states, controls], [rhs])

############################################
# Optimal control problem (OCP) definition #
############################################

# Define the decision variables (controls).
# There are as many control actions as prediction steps.
U = csd.SX.sym('U', n_controls, n_steps)

# Define the problem parameters.
# These include the initial state and the reference state of the robot.
P = csd.SX.sym('P', 2 * n_states)

# Define the states over the optimisation problem.
# The size is n_states x (1 + n_steps) as it stores all states for the initial
# state of the robot and all prediction steps.
X = csd.SX.sym('X', n_states, (1 + n_steps))

# Initialise the prediction states with the initial state of the robot.
X[:, 0] = P[0:3]

# Perform integration using Euler method.
for k in range(0, n_steps):
    current_state = X[:, k]
    current_control = U[:, k]
    f_linearised = f(current_state, current_control)
    X[:,k + 1] = current_state + (sampling_time * f_linearised)

Q = np.zeros((3, 3)) # Reference state weights
Q[0, 0] = 5.0
Q[1, 1] = 5.0
Q[2, 2] = 0.5

R = np.zeros((2, 2)) # Control weights
R[0, 0] = 0.5
R[1, 1] = 0.05

# Sum up all the contributions to the objective function.
phi = 0
for k in range(0, n_steps):
    current_state = X[:,k]
    current_control = U[:,k]
    phi += (
        (current_state - P[3:6]).T @ Q @ (current_state - P[3:6]) +
        current_control.T @ R @ current_control
    )

# Add state constraints.
g = [] 
for k in range(0, n_steps + 1):
    g = csd.vertcat(g, X[0,k]) # Constraint for the X position
    g = csd.vertcat(g, X[1,k]) # Constraint for the Y position

# Define a nonlinear programming problem (NLP).
variables = csd.reshape(U, 2 * n_steps, 1)
nlp = {'f': phi, 'x': variables, 'g': g, 'p': P}
opts = {
    'ipopt.print_level': 0,
    'ipopt.sb': 'yes',
    'print_time': 0,
    'ipopt.tol': 1e-3,
    'ipopt.max_iter': 20,
    'ipopt.linear_solver': solver_linear
}

solver = csd.nlpsol('solver', 'ipopt', nlp, opts)

lbx = np.zeros(2*n_steps)
ubx = np.zeros(2*n_steps)
lbx[0::2] = v_min
ubx[0::2] = v_max
lbx[1::2] = omega_min
ubx[1::2] = omega_max

args = {
    'lbg': map_limits[0],
    'ubg': map_limits[1],
    'lbx': lbx,
    'ubx': ubx
}

###################
# Simulation loop #
###################

time_solver_start = timeit.default_timer()

x0 = csd.vertcat(state_initial)   # Initial state
xs = csd.vertcat(state_reference) # Reference state

# Initialise history of the states.
states_simulated = np.zeros((x0.shape[0], int(time_span / sampling_time) + 1))
states_simulated[:, :0] = x0[:, 0]

t = np.zeros(int(time_span / sampling_time))
t[0] = time_initial
u0 = np.zeros((n_steps, 2))

i = 0
u_cl = np.zeros((1, 2))

while (csd.norm_2(x0 - xs) > 1e-2 and i < time_span / sampling_time):
    # Set the values of the parameters vector.
    args['p'] = csd.vertcat(x0, xs)

    # Initial value of the optimisation.
    args['x0'] = csd.reshape(u0.T, 2 * n_steps, 1)

    sol = solver(
        x0=args['x0'],
        lbx=args['lbx'],
        ubx=args['ubx'],
        lbg=args['lbg'],
        ubg=args['ubg'],
        p=args['p']
    )

    u = csd.reshape((sol['x']).T, 2, n_steps).T

    # Save first control action
    if i == 0:
        u_cl[0] = u[0, :]
    else:
        u_cl = csd.vertcat(u_cl, u[0,:])

    # Set the initial state and control.
    state = x0
    control = u[0,:].T

    # Compute the function for the given state and control.
    f_value = f(state, control)

    # Advance the state using an Euler integration step.
    state = state + (sampling_time * f_value)

    # Advance time.
    x0 = state.full()
    u0 = csd.vertcat(u[1::u.shape[1]], u[u.shape[0]:])

    # Store the current time and state.
    t[i] = time_initial
    states_simulated[:, i:i+1] = x0

    # Advance time and i counter.
    time_initial += 1
    i += 1

time_solver_end = timeit.default_timer()
elapsed_time = time_solver_end - time_solver_start

print(f'''
MPC with linear solver {solver_linear.upper()} took {(elapsed_time):0.2f} s
for {time_span} s of simulation
''')

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

########################
# Trajectory animation #
########################

figure, axes = plt.subplots(1, 1,
    figsize=(8.0, 8.0)
)
figure.tight_layout()
axes.set_xlim(map_limits[0], map_limits[1])
axes.set_ylim(map_limits[0], map_limits[1])
axes.set_aspect('equal', 'box')
axes.set_xlabel('X coordinate (m)')
axes.set_ylabel('Y coordinate (m)')
line, = axes.plot([], [])
patch_robot_envelope = plt.Circle((0.0,0.0), 0.45)
patch_robot_orientation = plt.Circle((0.1,0.0), 0.1,
    color=(1.0, 1.0, 0.0)
)

# Add an arrow for the reference state.
axes.add_patch(
    plt.Arrow(
        state_reference[0],
        state_reference[1],
        0.5 * csd.cos(state_reference[2]),
        0.5 * csd.sin(state_reference[2]),
        width=0.25,
        color=[1.0, 0.0, 0.0],
        zorder=100
    )
)

def initialise_animation():

    axes.add_patch(patch_robot_envelope)
    axes.add_patch(patch_robot_orientation)
    line.set_data([], [])
    
    return []

def animate_envelope(i):

    x = states_simulated[0,:i]
    y = states_simulated[1,:i]
    line.set_data(x, y)
    patch_robot_envelope.center = (states_simulated[0,i], states_simulated[1,i])
    
    return patch_robot_envelope, 

def animate_orientation(i):

    patch_robot_orientation.center = (
        states_simulated[0,i] + 0.25 * csd.cos(states_simulated[2,i]),
        states_simulated[1,i] + 0.25 * csd.sin(states_simulated[2,i])
    )

    return patch_robot_orientation,

def animate_robot(i):

    animate_envelope(i)
    animate_orientation(i)

    return []

animation_function = animation.FuncAnimation(figure, animate_robot,
    init_func=initialise_animation,
    frames=len(states_simulated[0,:]),
    interval=100,
    blit=True
)

animation_function.save('assets/img/dd-trajectory.gif',
    writer='imagemagick',
    fps=5
)
