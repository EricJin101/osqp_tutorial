cmake_minimum_required(VERSION 3.1)
project(osqp)
set(CMAKE_CXX_STANDARD 11)

# Find OSQP library and headers
find_package(osqp REQUIRED)

add_executable(main main.cc)

# Link the OSQP shared library
target_link_libraries(main PRIVATE osqp::osqp)
#在cmake语法中，link_libraries和target_link_libraries是很重要的两个链接库的方式，虽然写法上很相似，但是功能上有很大区别：
#
#1，link_libraries用在add_executable之前，target_link_libraries用在add_executable之后
#
#2，link_libraries用来链接静态库，target_link_libraries用来链接导入库，即按照header file + .lib + .dll方式隐式调用动态库的.lib库