#include <ros/ros.h>
#include "proximity_grid/LaserScanToProximityGrid.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "laserscan_to_proximitygrid");
	
	proximitygrid::LaserScanToProximityGrid t;
	
	t.Run();
	
	return 0;
}
