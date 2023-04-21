cmake_minimum_required(VERSION 3.8)

project(mpc_playground)

find_package(casadi REQUIRED)
find_package(Matplot++ REQUIRED)
#add_subdirectory(3rd-party/matplotplusplus)
find_package(pybind11 REQUIRED)

###############
# MPC example #
###############

add_executable(mpc_example
  "src/mpc_interface.cpp"
  "src/mpc_ms_ackermann.cpp"
)

add_executable(mpc_example_pybind
    src/mpc_ms_ackermann_pybind.cpp
    src/mpc_interface.cpp
    include/mpc_playground/mpc_interface.hpp
    )

target_include_directories(mpc_example
  PUBLIC
    "include"
    )

target_include_directories(mpc_example_pybind
    PUBLIC "include")

target_link_libraries(mpc_example
  casadi
  Matplot++::matplot
)

target_link_libraries(mpc_example_pybind
    casadi
    pybind11::embed
    )
###############
# CasADi test #
###############

add_executable(casadi_test
  "tests/casadi_test.cpp"
)

target_link_libraries(casadi_test
  casadi
)

##################################
# CasADi IPOPT MUMPS solver test #
##################################

add_executable(casadi_test_ipopt_mumps
  "tests/casadi_test_ipopt_mumps.cpp"
)

target_link_libraries(casadi_test_ipopt_mumps
  casadi
)

#####################################
# CasADi IPOPT HSL MA27 solver test #
#####################################

add_executable(casadi_test_ipopt_hsl
  "tests/casadi_test_ipopt_hsl.cpp"
)

target_link_libraries(casadi_test_ipopt_hsl
  casadi
)