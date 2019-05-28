#include <ros/ros.h>

#include "sparkfun_9dof_razor_imu_m0/driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_driver");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  sparkfun_9dof_razor_imu_m0::Driver driver(nh, private_nh);

  ros::spin();

  std::cout << "exiting" << std::endl;
  return 0;
}
