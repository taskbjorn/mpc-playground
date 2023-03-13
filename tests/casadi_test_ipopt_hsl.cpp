#include <iostream>

#include <casadi/casadi.hpp>

using namespace casadi;

int main(int argc, char* argv[]) {
    // Initialise a single variable for the mock NLP.
    auto x = SX::sym("x");
    
    // Define the mock NLP.
    SXDict nlp = {
        {"x", x},
        {"f", x}
    };

    // Set the mock NLP linear solver to HSL MA27.
    Dict nlp_opts{
        {"ipopt.linear_solver", "ma27"}
    };  

    // Allocate the mock NLP solver.
    Function solver = nlpsol("nlpsol", "ipopt", nlp, nlp_opts);

    // Set the constraints for the mock NLP.
    std::map<std::string, DM> solver_args;
    solver_args["lbx"] = 0.0;
    solver_args["ubx"] = 0.0;
    solver_args["lbg"] = 0.0;
    solver_args["ubg"] = 0.0;
    solver_args["x0"] = 0.0;

    // Solve the mock NLP.
    std::map<std::string, DM> solver_result;
    solver_result = solver(solver_args);

    return EXIT_SUCCESS;
}