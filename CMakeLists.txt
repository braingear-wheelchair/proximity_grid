set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(proximity_grid)
find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs 
			 message_generation 
			 tf 
			 dynamic_reconfigure)


SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(include ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
	cfg/LaserScanGrid.cfg
	cfg/PointGrid.cfg
)

add_message_files(
	FILES
	ProximityGridMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
)

add_library(${PROJECT_NAME} 
			src/ProximityGrid.cpp
			src/ProximityGridConverter.cpp
			src/LaserScanToProximityGrid.cpp
			src/PointToProximityGrid.cpp
			src/ProximityGridVisualizer.cpp
			)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(laserscan_to_proximitygrid src/laserscan_to_proximitygrid.cpp)
target_link_libraries(laserscan_to_proximitygrid ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(laserscan_to_proximitygrid ${PROJECT_NAME}_gencfg)

add_executable(point_to_proximitygrid src/point_to_proximitygrid.cpp)
target_link_libraries(point_to_proximitygrid ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(point_to_proximitygrid ${PROJECT_NAME}_gencfg)

add_executable(proximitygrid_visualizer src/proximitygrid_visualizer.cpp)
target_link_libraries(proximitygrid_visualizer ${PROJECT_NAME} ${catkin_LIBRARIES})
