#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <mpc_playground/mpc_interface.hpp>

using namespace MPC;
namespace py = pybind11;
using namespace py::literals;

int main(int argc, char* argv[]) {
    auto mpc = ModelPredictiveController();

    mpc.SetMass(1000.0);
    mpc.SetGoalXPosition(2.0);
    mpc.SetGoalYPosition(-4.0);
    mpc.SetSteeringAngleLowerBound(-0.57596);
    mpc.SetSteeringAngleUpperBound(0.57596);
    mpc.SetLongitudinalSpeedInitialValue(0.2);
    mpc.SetLongitudinalSpeedLowerBound(0.1);
    mpc.SetLongitudinalSpeedUpperBound(5.0);
    mpc.SetLongitudinalAccelerationLowerBound(-5.0);
    mpc.SetLongitudinalAccelerationUpperBound(5.0);
    mpc.SetLongitudinalJerkLowerBound(-5.5);
    mpc.SetLongitudinalJerkUpperBound(5.5);
    mpc.SetSteeringRateLowerBound(-1.2 * 0.57596);
    mpc.SetSteeringRateUpperBound(1.2 * 0.57596);

    mpc.Initialise();
    mpc.Plan();

    auto j_opt = mpc.GetOptimalLongitudinalJerk();
    auto delta_dot_opt = mpc.GetOptimalSteeringRate();
    auto x_opt = mpc.GetOptimalXPosition();
    auto y_opt = mpc.GetOptimalYPosition();
    auto t_opt = mpc.GetOptimalTimeSpan();

//    std::vector<float> j_opt{0, 1, 2};
//    std::vector<float> x_opt{0, 1, 2};
//    std::vector<float> y_opt{0, 1, 2};
//    std::vector<float> t_opt{0, 1, 2};
//    std::vector<float> delta_dot_opt{0, 1, 2};

    py::scoped_interpreter guard{};
    py::dict locals =
        py::dict{"x_opt"_a = x_opt, "y_opt"_a = y_opt, "j_opt"_a = j_opt, "delta_dot_opt"_a = delta_dot_opt, "t_opt"_a =
                                                                                                                 t_opt};
    py::exec(R"(
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    ax.plot(x_opt, y_opt, "-o")
    ax.set_title("AGV trajectory")
    ax.set_xlabel("X Coordinate [m]")
    ax.set_ylabel("Y Coordinate [m]")
    ax.set_aspect('equal', adjustable='box')

    dir = 'assets/img'
    plt.savefig(f"""{dir}/ack-trajectory.png""")
    )", py::globals(), locals);

    py::exec(R"(
    fig, ax = plt.subplots()
    ax.set_title('AGV longitudinal jerk')
    ax.plot(t_opt, j_opt, "-o")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Longitudinal jerk [m/s^3]")
    ax.set_aspect('equal', adjustable = 'box')
    plt.savefig(f"""{dir}/ack-controls-1.png""")

    fig, ax = plt.subplots()
    ax.set_title('AGV steering rate')
    ax.plot(t_opt, delta_dot_opt, "-o")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Steering rate [rad/s]")
    ax.set_aspect('equal', adjustable = 'box')
    plt.savefig(f"""{dir}/ack-controls-2.png""")
    )", py::globals(), locals);

    return EXIT_SUCCESS;
}