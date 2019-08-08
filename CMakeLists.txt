cmake_minimum_required(VERSION 2.8.3)
project(chomp_predict)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
 CATKIN_DEPENDS 
 INCLUDE_DIRS include
 LIBRARIES chomp_predict
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
			src/chomp_predict.cpp 
				src/chomp_utils.cpp
				src/chomp_subroutine.cpp
				src/chomp_ros_wrapper.cpp)

# module1 : standalone node 
add_executable(chomp_predict_basic_test_node 
src/chomp_standalone_test.cpp 
src/chomp_utils.cpp
src/chomp_subroutine.cpp
src/chomp_ros_wrapper.cpp
)

target_link_libraries(chomp_predict_basic_test_node
  ${catkin_LIBRARIES}
)

# module2 : prediction node 
add_executable(chomp_predict_node
src/chomp_predict_main.cpp
src/chomp_predict.cpp 
src/chomp_utils.cpp
src/chomp_subroutine.cpp
src/chomp_ros_wrapper.cpp
)

target_link_libraries(chomp_predict_node
  ${catkin_LIBRARIES}
)




