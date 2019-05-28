#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "sparkfun_9dof_razor_imu_m0/driver.h"

namespace sparkfun_9dof_razor_imu_m0
{

class DriverNodelet: public nodelet::Nodelet
{
public:

  DriverNodelet() {}

private:

  virtual void onInit(void);

  boost::shared_ptr<Driver> driver_;

}; // class DriverNodelet

void DriverNodelet::onInit()
{
  NODELET_DEBUG("Initializing SparkFun 9DoF Razor IMU M0 driver nodelet");
  driver_.reset(new Driver(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sparkfun_9dof_razor_imu_m0

PLUGINLIB_EXPORT_CLASS(sparkfun_9dof_razor_imu_m0::DriverNodelet, nodelet::Nodelet);
