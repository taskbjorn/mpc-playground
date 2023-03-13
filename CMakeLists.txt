cmake_minimum_required(VERSION 3.8)

project(mpc_playground)

find_package(casadi REQUIRED)

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