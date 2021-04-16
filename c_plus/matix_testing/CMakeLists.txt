cmake_minimum_required(VERSION 3.1)

set (CMAKE_CXX_STANDARD 11)

project(OsqpEigen-Example)
find_package(OsqpEigen)
find_package(Eigen3)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_executable(osqp src/main.cc)
target_link_libraries(osqp OsqpEigen::OsqpEigen)
