cmake_minimum_required(VERSION 3.4...3.18)
project(unitree_udp)

add_compile_options(-std=c++11)

add_subdirectory(pybind11)

include_directories(include)
#link_directories(lib)
#string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
#set(EXTRA_LIBS ${LEGGED_SDK_NAME})
set(EXTRA_LIBS /home/zhenz/workspaces/UnitreeRepos/unitree_udp_pybind11/lib/libunitree_legged_sdk_amd64.so)

#set(PYTHON_EXECUTABLE /home/zhenz/anaconda3/envs/py36/bin/python)
#set(PYBIND11_PYTHON_VERSION 3.6)

#pybind11_add_module(cmake_example src/main.cpp)
pybind11_add_module(unitree_udp src/udp_recv_pub.cpp)
target_link_libraries(unitree_udp PRIVATE ${EXTRA_LIBS} lcm)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(unitree_udp
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
