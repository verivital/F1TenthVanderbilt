// -*- mode:c++; fill-column: 100; -*-

#ifndef RACECAR_POTENTIAL_FIELD_CONTROLLER_CONTROLLER_H_
#define RACECAR_POTENTIAL_FIELD_CONTROLLER_CONTROLLER_H_

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <dynamic_reconfigure/server.h>

#include "racecar_potential_field_controller/RacecarPotentialFieldControllerConfig.h"

namespace racecar_potential_field_controller
{

class Controller
{
public:

  Controller(ros::NodeHandle nh,
	     ros::NodeHandle private_nh);

private:
  // parameters
  double force_scale_x_;
  double force_scale_y_;
  double force_offset_x_;
  double force_offset_y_;
  double speed_p_gain_;
  double steering_p_gain_;
  double steering_d_gain_;
  double viz_forces_scale_;
  double viz_net_force_scale_;

  // state
  ackermann_msgs::AckermannDriveStamped last_cmd_;
  double force_angle_last_;

  // ROS services
  ros::Publisher vel_pub_;
  ros::Publisher viz_pub_;
  ros::Subscriber scan_sub_;
  ros::Timer timer_;
  dynamic_reconfigure::Server<RacecarPotentialFieldControllerConfig> dynamic_param_server_;

  // ROS callbacks
  void timerCallback(const ros::TimerEvent& event);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void paramCallback(RacecarPotentialFieldControllerConfig& config, uint32_t level);
};

} // namespace racecar_potential_field_controller

#endif // RACECAR_POTENTIAL_FIELD_CONTROLLER_CONTROLLER_H_
