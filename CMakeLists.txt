cmake_minimum_required(VERSION 2.8.2)
project(techman_tm5)

find_package(Eigen3)

include_directories(tm_reflexxes/include 
					ReflexxesTypeII/include 
					tm_driver/include
					tm_kinematics/include
)


######################################################
# Set the directories of source code

file(GLOB RMLtypeII_DIR    ReflexxesTypeII/src/*.cpp)
file(GLOB tm_reflexxex_DIR tm_reflexxes/src/*.cpp)

set(tm_driver_DIR 
	tm_driver/src/tm_print.cpp
	tm_driver/src/tm_robot_state_rt.cpp
	tm_driver/src/tm_communication.cpp
	tm_driver/src/tm_driver.cpp)

set(tm_kin_DIR 
	tm_kinematics/src/tm_kin.cpp
	tm_kinematics/src/tm_jacob.cpp)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")


######################################################
# build source to library and Linking CXX static library

add_library(RMLTYPEII_LIB     ${RMLtypeII_DIR}   )
add_library(TM_REFLEXXES_LIB  ${tm_reflexxex_DIR})
add_library(TM_DRIVER_LIB     ${tm_driver_DIR}   )
add_library(TM_KINEMATICS_LIB ${tm_kin_DIR}      )

target_link_libraries(TM_REFLEXXES_LIB 
	RMLTYPEII_LIB
	TM_DRIVER_LIB
	TM_KINEMATICS_LIB)

######################################################
# build source to exe and Linking CXX static library

add_executable(tm_bringup tm_bringup.cpp)
target_link_libraries(tm_bringup 
	RMLTYPEII_LIB 
	TM_REFLEXXES_LIB 
	TM_DRIVER_LIB 
	TM_KINEMATICS_LIB)

add_executable(tm_otg tm_otg.cpp)
target_link_libraries(tm_otg 
	RMLTYPEII_LIB 
	TM_REFLEXXES_LIB 
	TM_DRIVER_LIB 
	TM_KINEMATICS_LIB)

install(TARGETS tm_bringup RUNTIME DESTINATION bin)
