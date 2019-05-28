#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "racecar_potential_field_controller/controller.h"

namespace racecar_potential_field_controller
{

class Nodelet: public nodelet::Nodelet
{
public:

  Nodelet() {}

private:

  virtual void onInit(void);

  boost::shared_ptr<Controller> controller_;

}; // class Nodelet

void Nodelet::onInit()
{
  NODELET_DEBUG("Initializing racecar_potential_field_controller nodelet");
  controller_.reset(new Controller(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace racecar_potential_field_controller

PLUGINLIB_EXPORT_CLASS(racecar_potential_field_controller::Nodelet, nodelet::Nodelet);
