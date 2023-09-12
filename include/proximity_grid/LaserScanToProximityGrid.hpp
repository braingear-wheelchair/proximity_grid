#ifndef PROXIMITY_GRID_LASERSCANTOPROXIMITYGRID_HPP
#define PROXIMITY_GRID_LASERSCANTOPROXIMITYGRID_HPP

// Systema
#include <unordered_map>

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>

// Package includes
#include "proximity_grid/ProximityGrid.hpp"
#include "proximity_grid/ProximityGridMsg.h"
#include "proximity_grid/ProximityGridConverter.hpp"
#include "proximity_grid/LaserScanGridConfig.h"

namespace proximitygrid {


class LaserScanToProximityGrid {

    public:
		LaserScanToProximityGrid(void);
		virtual ~LaserScanToProximityGrid(void);
		
		virtual bool configure(void);

		void Run(void);

	private:
		void on_received_laserscan(const sensor_msgs::LaserScanConstPtr& msgin, unsigned int id);
		void on_dynamic_reconfiguration(proximity_grid::LaserScanGridConfig &config, 
										uint32_t level);

		void init_update_rate(float rate);
		bool update_if_different(const float& first, float& second, float epsilon = 0.000001f);
		float rad2deg(float angle);
		float deg2rad(float angle);

    protected:
		ros::NodeHandle	nh_;
  		ros::NodeHandle	private_nh_;
		ros::Rate*		rate_;
		float			publish_frequency_;
		
		ros::Publisher					pub_;
		std::vector<ros::Subscriber>	sub_src_;
		
		std::string				 pub_topic_;
		std::vector<std::string> src_topic_;

		dynamic_reconfigure::Server<proximity_grid::LaserScanGridConfig> cfgserver_;
		dynamic_reconfigure::Server<proximity_grid::LaserScanGridConfig>::CallbackType f_;
		
		ProximityGrid grid_;
		std::unordered_map<unsigned int, ProximityGrid>	grid_src_;

		tf::TransformListener listener_;
};

}



#endif
