#ifndef PROXIMITY_GRID_PROXIMITYGRIDCONVERTER_HPP
#define PROXIMITY_GRID_PROXIMITYGRIDCONVERTER_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>

// Package include
#include "proximity_grid/ProximityGrid.hpp"
#include "proximity_grid/ProximityGridMsg.h"

namespace proximitygrid {

class ProximityGridConverter {

	public:

		static bool FromMessage(const proximity_grid::ProximityGridMsg& msg, ProximityGrid& grid);

		static bool FromPoint(const geometry_msgs::PointStamped& msg, ProximityGrid& grid,
							  tf::TransformListener* listener);

		static bool FromLaserScan(const sensor_msgs::LaserScan& msg, ProximityGrid& grid,
								  tf::TransformListener* listener);
								
		static bool ToMessage(const ProximityGrid& grid, proximity_grid::ProximityGridMsg& msg);

		static bool ToLaserScan(const ProximityGrid& grid, sensor_msgs::LaserScan& scan);
};


}
#endif
