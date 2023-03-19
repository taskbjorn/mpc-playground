#include <matplot/matplot.h>

#include <mpc_playground/mpc_interface.hpp>

using namespace MPC;

int main(int argc, char* argv[]) {
    auto mpc = ModelPredictiveController();

    mpc.SetMass(1000.0);
    mpc.SetGoalXPosition(4.0);
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

    matplot::plot(x_opt, y_opt, "-o");
    matplot::title("AGV trajectory");
    matplot::xlabel("X Coordinate (m)");
    matplot::ylabel("Y Coordinate (m)");
    matplot::save("ack-trajectory.jpg");

    matplot::plot(t_opt, j_opt, "-o");
    matplot::title("AGV longitudinal jerk");
    matplot::xlabel("Time (s)");
    matplot::ylabel("Longitudinal jerk (m/s3)");
    matplot::save("ack-controls-1.jpg");

    matplot::plot(t_opt, delta_dot_opt, "-o");
    matplot::title("AGV steering rate");
    matplot::xlabel("Time (s)");
    matplot::ylabel("Steering rate (rad/s)");
    matplot::save("ack-controls-2.jpg");

    return EXIT_SUCCESS;
}