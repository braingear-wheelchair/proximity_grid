#ifndef PROXIMITY_GRID_PROXIMITYGRIDVISUALIZER_HPP
#define PROXIMITY_GRID_PROXIMITYGRIDVISUALIZER_HPP

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// Package includes
#include "proximity_grid/ProximityGrid.hpp"
#include "proximity_grid/ProximityGridMsg.h"
#include "proximity_grid/ProximityGridConverter.hpp"

namespace proximitygrid {

class ProximityGridVisualizer {

    public:
		ProximityGridVisualizer(void);
		virtual ~ProximityGridVisualizer(void);
		
		virtual bool configure(void);
    
	private:
		void on_received_grid(const proximity_grid::ProximityGridMsg& msg);

    protected:
		ros::NodeHandle	nh_;
  		ros::NodeHandle	private_nh_;
		
		ros::Subscriber	sub_;
		ros::Publisher	pub_;
		std::string		stopic_;
		std::string 	ptopic_;

		float radius_;
};



}

#endif
