cmake_minimum_required(VERSION 3.1)
project(osqp)
set(CMAKE_CXX_STANDARD 11)

# Find OSQP library and headers
find_package(osqp REQUIRED)

add_executable(main main.cc)

# Link the OSQP shared library
target_link_libraries(main PRIVATE osqp::osqp)