#include <ros/ros.h>
#include "proximity_grid/ProximityGridVisualizer.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "proximitysector_visualizer");
	
	proximitygrid::ProximityGridVisualizer t;
	ros::spin();
	
	return 0;
}
