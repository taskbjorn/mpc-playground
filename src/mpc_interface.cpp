#include <mpc_playground/mpc_interface.hpp>

namespace MPC {

ModelPredictiveController::ModelPredictiveController() {}

ModelPredictiveController::~ModelPredictiveController() {}

void
ModelPredictiveController::Initialise() {
    //
    x_opt_ = std::vector<double>(nodes_count_);
    y_opt_ = std::vector<double>(nodes_count_);
    psi_opt_ = std::vector<double>(nodes_count_);
    j_opt_ = std::vector<double>(nodes_count_);
    delta_dot_opt_ = std::vector<double>(nodes_count_);

    // Define the vehicle weight
    double f_z = m_ * 9.81;

    // Define the normal load at the front and rear axles.
    double f_z_f = f_z * d_b_ / (d_a_ + d_b_);
    double f_z_r = f_z * d_a_ / (d_a_ + d_b_);

    SX alpha_f = if_else(abs(u_) > epsilon_, atan((v_ + d_a_ * r_) / (u_)) - delta_, u_/abs(u_) * epsilon_);
    SX alpha_r = if_else(abs(u_) > epsilon_, atan((v_ - d_b_ * r_) / (u_)), u_/abs(u_) * epsilon_);

    // Define the front and rear axle lateral force equations.
    SX f_y_f = c_alpha_f_ * f_z_f * alpha_f;
    SX f_y_r = c_alpha_r_ * f_z_r * alpha_r;

    SX x_dot = u_ * cos(psi_) - (v_ + d_a_ * r_) * sin(psi_);
    SX y_dot = u_ * sin(psi_) + (v_ + d_a_ * r_) * cos(psi_);
    SX v_dot = (f_y_f + f_y_r) / m_ - r_ * u_;
    SX r_dot = (d_a_ * f_y_f - d_b_ * f_y_r) / i_zz_;

    states_ = vertcat(vertcat(x_, y_, v_, r_), vertcat(psi_, delta_, u_, a_));
    states_count_ = states_.size1();

    controls_ = vertcat(j_, delta_dot_);
    controls_count_ = controls_.size1();

    ode_ = vertcat(vertcat(x_dot, y_dot, v_dot, r_dot), vertcat(r_, delta_dot_, a_, j_));

    integrator_opts_ = Dict({
        {"t0", 0.0},
        {"tf", time_span_ / nodes_count_}
    });

    UpdateObjectiveFunction();
    std::cout << quadrature_ << std::endl;

    nlp_opts_ = Dict({
        {"ipopt.tol", tol_},
        {"ipopt.max_iter", max_iters_},
        {"ipopt.linear_solver", linear_solver_}
    });
}

void
ModelPredictiveController::UpdateObjectiveFunction() {
    quadrature_ =
        1.0 * j_ * j_ +
        1.0 * delta_dot_ * delta_dot_ +
        1.0 * sqrt(pow((x_ - goal_x_), 2) + pow((y_ - goal_y_), 2));

    dae_ = SXDict({
        {"x", states_},
        {"p", controls_},
        {"ode", ode_},
        {"quad", quadrature_}
    });

    integrator_ = integrator("integrator", "cvodes", dae_, integrator_opts_);
}

void
ModelPredictiveController::Plan() {
    std::vector<double> lbw;
    std::vector<double> ubw;
    std::vector<double> w0;
    MXVector w;
    MXVector g;
    std::vector<double> lbg;
    std::vector<double> ubg;

    // Lift the initial conditions
    MX xi_0 = MX::sym("Xi_0", states_count_);
    w.insert(w.end(), xi_0);
    lbw.insert(lbw.end(), {x_0_, y_0_, v_0_, r_0_, psi_0_, delta_0_, u_0_, a_0_});
    ubw.insert(ubw.end(), {x_0_, y_0_, v_0_, r_0_, psi_0_, delta_0_, u_0_, a_0_});
    w0.insert(w0.end(), {x_0_, y_0_, v_0_, r_0_, psi_0_, delta_0_, u_0_, a_0_});

    phi_ = 0.0;
    MX xi_k = xi_0;
    for (int i = 0; i < nodes_count_; ++i) {
        MX ubar_k = MX::sym("U_" + std::to_string(i), controls_count_);
        w.insert(w.end(), ubar_k);

        lbw.insert(lbw.end(), {j_low_, delta_dot_low_});
        ubw.insert(ubw.end(), {j_up_, delta_dot_up_});
        w0.insert(w0.end(), {j_0_, delta_dot_0_});
        
        MXDict initial_conditions = MXDict{
            {"x0", xi_k},
            {"p", ubar_k}
        };

        MXDict F_k = integrator_(initial_conditions);

        MX X_k_end = F_k["xf"];

        phi_ += F_k["qf"];

        xi_k = MX::sym("Xi_" + std::to_string(i + 1), states_count_);
        w.insert(w.end(), xi_k);
        lbw.insert(lbw.end(), {x_low_, y_low_, v_low_, r_low_, psi_low_, delta_low_, u_low_, a_low_});
        ubw.insert(ubw.end(), {x_up_, y_up_, v_up_, r_up_, psi_up_, delta_up_, u_up_, a_up_});
        w0.insert(w0.end(), {x_0_, y_0_, v_0_, r_0_, psi_0_, delta_0_, u_0_, a_0_});

        g.insert(g.end(), X_k_end - xi_k);
        lbg.insert(lbg.end(), {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        ubg.insert(ubg.end(), {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    }

    // Define the NLP.
    MXDict nlp = {
        {"x", vertcat(w)},
        {"f", phi_},
        {"g", vertcat(g)}
    };

    // Allocate the NLP solver.
    Function solver = nlpsol("nlpsol", "ipopt", nlp, nlp_opts_);

    // Set the constraints for the mock NLP.
    std::map<std::string, DM> solver_args;
    solver_args["lbx"] = lbw;
    solver_args["ubx"] = ubw;
    solver_args["lbg"] = lbg;
    solver_args["ubg"] = ubg;
    solver_args["x0"] = w0;

    // Solve the mock NLP.
    std::map<std::string, DM> solver_result;
    solver_result = solver(solver_args);

    std::vector<double> sol_opt(solver_result.at("x"));

    // Get the optimal trajectory and control
    for(int i = 0; i < nodes_count_; ++i){
        t_opt_.push_back(i * time_span_ / nodes_count_);
        x_opt_[i] = sol_opt.at(i * (states_count_ + controls_count_));
        y_opt_[i] = sol_opt.at(i * (states_count_ + controls_count_) + 1);
        psi_opt_[i] = sol_opt.at(i * (states_count_ + controls_count_) + 4);
        j_opt_[i] = sol_opt.at(i * (states_count_ + controls_count_) + 8);
        delta_dot_opt_[i] = sol_opt.at(i * (states_count_ + controls_count_) + 9);
    }
}

double
ModelPredictiveController::GetGoalXPosition() {
    return goal_x_;
}

double
ModelPredictiveController::GetGoalYPosition() {
    return goal_y_;
}

double
ModelPredictiveController::GetLateralSpeedInitialValue() {
    return v_0_;
}

double
ModelPredictiveController::SetLateralSpeedLowerBound() {
    return v_low_;
}

double
ModelPredictiveController::GetLateralSpeedUpperBound() {
    return v_up_;
}

double
ModelPredictiveController::GetLongitudinalAccelerationInitialValue() {
    return a_0_;
}

double
ModelPredictiveController::GetLongitudinalAccelerationLowerBound() {
    return a_low_;
}

double
ModelPredictiveController::GetLongitudinalAccelerationUpperBound() {
    return a_up_;
}

double
ModelPredictiveController::GetLongitudinalJerkInitialValue() {
    return j_0_;
}

double
ModelPredictiveController::GetLongitudinalJerkLowerBound() {
    return j_low_;
}

double
ModelPredictiveController::GetLongitudinalJerkUpperBound() {
    return j_up_;
}

double
ModelPredictiveController::GetLongitudinalSpeedInitialValue() {
    return u_0_;
}

double
ModelPredictiveController::GetLongitudinalSpeedLowerBound() {
    return u_low_;
}

double
ModelPredictiveController::GetLongitudinalSpeedUpperBound() {
    return u_up_;
}

std::vector<double>
ModelPredictiveController::GetOptimalLongitudinalJerk() {
    return j_opt_;
}

std::vector<double>
ModelPredictiveController::GetOptimalSteeringRate() {
    return delta_dot_opt_;
}

std::vector<double>
ModelPredictiveController::GetOptimalTimeSpan() {
    return t_opt_;
}

std::vector<double>
ModelPredictiveController::GetOptimalXPosition() {
    return x_opt_;
}

std::vector<double>
ModelPredictiveController::GetOptimalYPosition() {
    return y_opt_;
}

double
ModelPredictiveController::GetSteeringAngleInitialValue() {
    return delta_0_;
}

double
ModelPredictiveController::GetSteeringAngleLowerBound() {
    return delta_low_;
}

double
ModelPredictiveController::GetSteeringAngleUpperBound() {
    return delta_up_;
}

double
ModelPredictiveController::GetSteeringRateInitialValue() {
    return delta_dot_0_;
}

double
ModelPredictiveController::GetSteeringRateLowerBound() {
    return delta_dot_low_;
}

double
ModelPredictiveController::GetSteeringRateUpperBound() {
    return delta_dot_up_;
}

double
ModelPredictiveController::GetXPositionInitialValue() {
    return x_0_;
}

double
ModelPredictiveController::GetXPositionLowerBound() {
    return x_low_;
}

double
ModelPredictiveController::GetXPositionUpperBound() {
    return x_up_;
}

double
ModelPredictiveController::GetYPositionInitialValue() {
    return y_0_;
}

double
ModelPredictiveController::GetYPositionLowerBound() {
    return y_low_;
}

double
ModelPredictiveController::GetYPositionUpperBound() {
    return y_up_;
}

double
ModelPredictiveController::GetYawInitialValue() {
    return psi_0_;
}

double
ModelPredictiveController::GetYawLowerBound() {
    return psi_low_;
}

double
ModelPredictiveController::GetYawUpperBound() {
    return psi_up_;
}

double
ModelPredictiveController::GetYawRateInitialValue() {
    return r_0_;
}

double
ModelPredictiveController::GetYawRateLowerBound() {
    return r_low_;
}

double
ModelPredictiveController::GetYawRateUpperBound() {
    return r_up_;
}

void
ModelPredictiveController::SetGoalXPosition(double goal_x) {
    goal_x_ = goal_x;
}

void
ModelPredictiveController::SetGoalYaw(double goal_psi) {
    goal_psi_ = goal_psi;
}

void
ModelPredictiveController::SetGoalYPosition(double goal_y) {
    goal_y_ = goal_y;
}

void
ModelPredictiveController::SetLateralSpeedInitialValue(double v_0) {
    v_0_ = v_0;
}

void
ModelPredictiveController::SetLateralSpeedLowerBound(double v_low) {
    v_low_ = v_low;
}

void
ModelPredictiveController::SetLateralSpeedUpperBound(double v_up) {
    v_up_ = v_up;
}

void
ModelPredictiveController::SetLongitudinalAccelerationInitialValue(double a_0) {
    a_0_ = a_0;
}

void
ModelPredictiveController::SetLongitudinalAccelerationLowerBound(double a_low) {
    a_low_ = a_low;
}

void
ModelPredictiveController::SetLongitudinalAccelerationUpperBound(double a_up) {
    a_up_ = a_up;
}

void
ModelPredictiveController::SetLongitudinalJerkInitialValue(double j_0) {
    j_0_ = j_0;
}

void
ModelPredictiveController::SetLongitudinalJerkLowerBound(double j_low) {
    j_low_ = j_low;
}

void
ModelPredictiveController::SetLongitudinalJerkUpperBound(double j_up) {
    j_up_ = j_up;
}

void
ModelPredictiveController::SetLongitudinalSpeedInitialValue(double u_0) {
    u_0_ = u_0;
}

void
ModelPredictiveController::SetLongitudinalSpeedLowerBound(double u_low) {
    u_low_ = u_low;
}

void
ModelPredictiveController::SetLongitudinalSpeedUpperBound(double u_up) {
    u_up_ = u_up;
}

void
ModelPredictiveController::SetSteeringAngleInitialValue(double delta_0) {
    delta_0_ = delta_0;
}

void
ModelPredictiveController::SetSteeringAngleLowerBound(double delta_low) {
    delta_low_ = delta_low;
}

void
ModelPredictiveController::SetSteeringAngleUpperBound(double delta_up) {
    delta_up_ = delta_up;
}

void
ModelPredictiveController::SetSteeringRateInitialValue(double delta_dot_0) {
    delta_dot_0_ = delta_dot_0;
}

void
ModelPredictiveController::SetSteeringRateLowerBound(double delta_dot_low) {
    delta_dot_low_ = delta_dot_low;
}

void
ModelPredictiveController::SetSteeringRateUpperBound(double delta_dot_up) {
    delta_dot_up_ = delta_dot_up;
}

void
ModelPredictiveController::SetXPositionInitialValue(double x_0) {
    x_0_ = x_0;
}

void
ModelPredictiveController::SetXPositionLowerBound(double x_low) {
    x_low_ = x_low;
}

void
ModelPredictiveController::SetXPositionUpperBound(double x_up) {
    x_up_ = x_up;
}

void
ModelPredictiveController::SetYPositionInitialValue(double y_0) {
    y_0_ = y_0;
}

void
ModelPredictiveController::SetYPositionLowerBound(double y_low) {
    y_low_ = y_low;
}

void
ModelPredictiveController::SetYPositionUpperBound(double y_up) {
    y_up_ = y_up;
}

void
ModelPredictiveController::SetYawInitialValue(double psi_0) {
    psi_0_ = psi_0;
}

void
ModelPredictiveController::SetYawLowerBound(double psi_low) {
    psi_low_ = psi_low;
}

void
ModelPredictiveController::SetYawUpperBound(double psi_up) {
    psi_up_ = psi_up;
}

void
ModelPredictiveController::SetYawRateInitialValue(double r_0) {
    r_0_ = r_0;
}

void
ModelPredictiveController::SetYawRateLowerBound(double r_low) {
    r_low_ = r_low;
}

void
ModelPredictiveController::SetYawRateUpperBound(double r_up) {
    r_up_ = r_up;
}

void
ModelPredictiveController::SetFrontAxleCentreOfGravityDistance(double d_a) {
    d_a_= d_a;
}

void
ModelPredictiveController::SetFrontAxleCorneringStiffness(double c_alpha_f) {
    c_alpha_f_ = c_alpha_f;
}

void
ModelPredictiveController::SetLinearSolver(std::string linear_solver) {
    linear_solver_ = linear_solver;
}

void
ModelPredictiveController::SetLinearSolverTolerance(double tol) {
    tol_ = tol;
}

void
ModelPredictiveController::SetLinearSolverMaximumIterations(int max_iters) {
    max_iters_ = max_iters;
}

void
ModelPredictiveController::SetMass(double m) {
    m_ = m;
}

void
ModelPredictiveController::SetRearAxleCorneringStiffness(double c_alpha_r) {
    c_alpha_r_ = c_alpha_r;
}

void
ModelPredictiveController::SetRearAxleCentreOfGravityDistance(double d_b) {
    d_b_ = d_b;
}

void
ModelPredictiveController::SetYawMomentOfInertia(double i_zz) {
    i_zz_ = i_zz;
}

} // end namespace MPC