#include <ros/ros.h>
#include "proximity_grid/PointToProximityGrid.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "point_to_proximitygrid");
	
	proximitygrid::PointToProximityGrid t;

	t.Run();

	return 0;
}
