#include <iostream>

#include <casadi/casadi.hpp>

int main(int argc, char* argv[]) {
    // Print a CasADi symbolic variable to terminal.
    std::cout << 
        "Testing CasADi..." << "\n" <<
        "Here is a CasADi symbol: " << casadi::SX::sym("x") << std::endl;

    return EXIT_SUCCESS;
}
