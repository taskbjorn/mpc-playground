import matplotlib.pyplot as plt


def print_stuff(x, y):
    print(x)
    print(y)
    print("printed from module")


def create_plots(x_opt, y_opt, t_opt, j_opt, delta_dot_opt):
    fig, ax = plt.subplots()
    ax.plot(x_opt, y_opt, "-o")
    ax.set_title("AGV trajectory")
    ax.set_xlabel("X Coordinate [m]")
    ax.set_ylabel("Y Coordinate [m]")
    ax.set_aspect('equal', adjustable='box')

    dir = 'assets/img'
    plt.savefig(f"""{dir}/ack-trajectory.png""")

    fig, ax = plt.subplots()
    ax.set_title('AGV longitudinal jerk')
    ax.plot(t_opt, j_opt, "-o")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Longitudinal jerk [m/s^3]")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig(f"""{dir}/ack-controls-1.png""")

    fig, ax = plt.subplots()
    ax.set_title('AGV steering rate')
    ax.plot(t_opt, delta_dot_opt, "-o")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Steering rate [rad/s]")
    ax.set_aspect('equal', adjustable='box')
    plt.savefig(f"""{dir}/ack-controls-2.png""")
    print("saved figs")


def main():
    print_stuff(0, 1.1)


if __name__ == '__main__':
    main()
