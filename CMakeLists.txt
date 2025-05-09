cmake_minimum_required(VERSION 3.10)
project(ME553_2025)

set(CMAKE_CXX_STANDARD 17)
include_directories(include) # Add this line
include(cmake/helper.cmake)

# exercise
create_executable(exercise1 src/exercise1.cpp)
create_executable(exercise2 src/exercise2.cpp)
create_executable(exercise3 src/exercise3.cpp)
create_executable(exercise4 src/exercise4.cpp)
create_executable(exercise5 src/exercise5.cpp)

# examples
create_executable(rotation_example_angle_axis src/rotation_example_angle_axis.cpp)
create_executable(rotation_example_euler_angles src/rotation_example_euler_angles.cpp)
create_executable(panda_example src/panda.cpp)

# custom examples
create_executable(forward_kinematics_panda examples/forward_kinematics_panda.cpp)
create_executable(forward_kinematics_minicheetah examples/forward_kinematics_minicheetah.cpp)
create_executable(forward_velocity_panda examples/forward_velocity_panda.cpp)
create_executable(forward_velocity_minicheetah examples/forward_velocity_minicheetah.cpp)
create_executable(inertia_world_minicheetah examples/inertia_world_minicheetah.cpp)

# exam
