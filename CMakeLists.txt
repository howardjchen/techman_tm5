cmake_minimum_required(VERSION 2.8.2)
project(techman_tm5)

find_package(Eigen3)

include_directories(tm_reflexxes/include 
					ReflexxesTypeII/include 
					tm_driver/include
					tm_kinematics/include
)

file(GLOB TM_MODERN_DRIVER_SRC
	tm_reflexxes/src/*.cpp
	ReflexxesTypeII/src/*.cpp
	tm_driver/src/tm_print.cpp
	tm_driver/src/tm_robot_state_rt.cpp
	tm_driver/src/tm_communication.cpp
	tm_driver/src/tm_driver.cpp
	tm_kinematics/src/tm_kin.cpp
	tm_kinematics/src/tm_jacob.cpp
)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

add_executable(tm_bringup tm_bringup.cpp ${TM_MODERN_DRIVER_SRC})
add_executable(tm_otg tm_otg.cpp ${TM_MODERN_DRIVER_SRC})

install(TARGETS tm_bringup RUNTIME DESTINATION bin)
