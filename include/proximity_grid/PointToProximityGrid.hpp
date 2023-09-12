#ifndef PROXIMITY_GRID_POINTTOPROXIMITYGRID_HPP
#define PROXIMITY_GRID_POINTTOPROXIMITYGRID_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>

// Package includes
#include "proximity_grid/ProximityGrid.hpp"
#include "proximity_grid/ProximityGridMsg.h"
#include "proximity_grid/ProximityGridConverter.hpp"
#include "proximity_grid/PointGridConfig.h"

namespace proximitygrid {

class PointToProximityGrid {

    public:
		PointToProximityGrid(void);
		virtual ~PointToProximityGrid(void);
		
		virtual bool configure(void);

		void Run(void);
    
	private:
		void on_received_point(const geometry_msgs::PointStamped& msg);
		void on_dynamic_reconfiguration(proximity_grid::PointGridConfig &config, 
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
		
		ros::Publisher	pub_;
		ros::Subscriber	sub_;

		std::string 	ptopic_;
		std::string		stopic_;

		dynamic_reconfigure::Server<proximity_grid::PointGridConfig> cfgserver_;
		dynamic_reconfigure::Server<proximity_grid::PointGridConfig>::CallbackType f_;

		ProximityGrid				grid_;

		tf::TransformListener listener_;
};



}

#endif
