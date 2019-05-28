#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "racecar_potential_field_controller/controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "potential_field_controller");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  boost::shared_ptr<racecar_potential_field_controller::Controller> controller;
  controller.reset(new racecar_potential_field_controller::Controller(nh, private_nh));

  ros::spin();

  return 0;
}
