// -*- mode:c++; fill-column: 100; -*-

#include "racecar_potential_field_controller/controller.h"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <sstream>

#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/bind.hpp>

namespace racecar_potential_field_controller
{

/* helper functions for visualization */
visualization_msgs::Marker marker(const std::string& frame_id, const std::string& ns, int id);
visualization_msgs::Marker arrow(visualization_msgs::Marker viz, double x, double y);
visualization_msgs::Marker lines(visualization_msgs::Marker viz, int skip,
                                 const Eigen::ArrayXf& points_x, const Eigen::ArrayXf& points_y,
                                 const Eigen::ArrayXf& forces_x, const Eigen::ArrayXf& forces_y);
visualization_msgs::Marker text(visualization_msgs::Marker viz, double x, double y, std::string text);

Controller::Controller(ros::NodeHandle nh,
		       ros::NodeHandle private_nh) :
  force_scale_x_(0.07), force_scale_y_(0.07), force_offset_x_(100), force_offset_y_(0),
  speed_p_gain_(0.05), steering_p_gain_(1.0), steering_d_gain_(0.1), viz_forces_scale_(0.07),
  viz_net_force_scale_(0.014), force_angle_last_(0.0)
{
  // dynamic parameters
  dynamic_param_server_.setCallback(boost::bind(&Controller::paramCallback, this, _1, _2));

  // create velocity publisher
  vel_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("navigation", 10);
  viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);

  // subscribe to laser scanner
  scan_sub_ = nh.subscribe("/scan", 10, &Controller::scanCallback, this);

  // create a 40Hz timer for setting commands
  timer_ = nh.createTimer(ros::Duration(1.0/40.0), &Controller::timerCallback, this);
}

void Controller::paramCallback(RacecarPotentialFieldControllerConfig& cfg, uint32_t level)
{
  ROS_INFO("racecar_potential_field_controller reconfigure request");
  force_scale_x_ = cfg.force_scale_x;
  force_scale_y_ = cfg.force_scale_y;
  force_offset_x_ = cfg.force_offset_x;
  force_offset_y_ = cfg.force_offset_y;
  speed_p_gain_ = cfg.speed_p_gain;
  steering_p_gain_ = cfg.steering_p_gain;
  steering_d_gain_ = cfg.steering_d_gain;
  viz_forces_scale_ = cfg.viz_forces_scale;
  viz_net_force_scale_ = cfg.viz_net_force_scale;
}

void Controller::timerCallback(const ros::TimerEvent& event)
{
  static bool updating(false);
  ackermann_msgs::AckermannDriveStamped::Ptr cmd(new ackermann_msgs::AckermannDriveStamped);

  if (ros::Time::now() - last_cmd_.header.stamp > ros::Duration(0.1)) {
    if (updating) {
      ROS_WARN("racecar_potential_field_controller not updating navigation command");
      updating = false;
    }
    return;
  }
  else if (!updating) {
    ROS_INFO("racecar_potential_field_controller updating navigation command");
    updating = true;
  }

  *cmd = last_cmd_;
  vel_pub_.publish(cmd);
}

void Controller::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  assert(scan && scan->ranges.size() > 0);

  // get index of scans between -/+ 90 degrees
  int idx_begin = ceil((-M_PI_2 - scan->angle_min) / scan->angle_increment);
  int idx_end = ceil((M_PI_2 - scan->angle_min) / scan->angle_increment); // past-the-end
  assert(idx_begin >= 0 && idx_begin < scan->ranges.size() &&
         idx_end >= 0 && idx_end <= scan->ranges.size() && idx_begin < idx_end);

  // copy ranges
  Eigen::ArrayXf ranges(idx_end - idx_begin);
  std::copy(scan->ranges.begin() + idx_begin, scan->ranges.begin() + idx_end, ranges.data());

  // convert scan index to angles
  Eigen::ArrayXf angles(Eigen::ArrayXf::LinSpaced(idx_end - idx_begin,
      scan->angle_min + idx_begin * scan->angle_increment,
      scan->angle_min + (idx_end - 1) * scan->angle_increment));
  assert(ranges.size() == angles.size());

  // compute "force" from points
  Eigen::ArrayXf neg_ranges_2(-1*ranges.square());
  Eigen::ArrayXf angles_cos(angles.cos());
  Eigen::ArrayXf angles_sin(angles.sin());
  Eigen::ArrayXf forces_x(angles_cos/neg_ranges_2);
  Eigen::ArrayXf forces_y(angles_sin/neg_ranges_2);
  Eigen::ArrayXf points_x(ranges * angles_cos);
  Eigen::ArrayXf points_y(ranges * angles_sin);
  float net_force_x = force_scale_x_ * forces_x.sum() + force_offset_x_;
  float net_force_y = force_scale_y_ * forces_y.sum() + force_offset_y_;

  double force_angle = atan2(net_force_y, net_force_x);
  if (fabs(force_angle) > 0.5) {
    force_angle = (force_angle < 0 ? -1.0 : 1.0) * 0.5;
  }
  last_cmd_.header.stamp = scan->header.stamp;
  last_cmd_.drive.speed = speed_p_gain_ * sqrt(net_force_x*net_force_x + net_force_y*net_force_y);
  if (net_force_x < 0)
    last_cmd_.drive.speed *= -1;
  last_cmd_.drive.steering_angle = steering_p_gain_ * force_angle +
    steering_d_gain_ * (force_angle - force_angle_last_);

  // set force_angle_last_ for use in derivative term on next iteration
  // (note: there are better approximations for derivative)
  force_angle_last_ = force_angle;

  // visualization of "forces"
  visualization_msgs::MarkerArray::Ptr viz(new visualization_msgs::MarkerArray);
  if (fabs(viz_net_force_scale_) > 0) {
    viz->markers.push_back(arrow(marker("laser", "net_force", 0),
                                 net_force_x * viz_net_force_scale_,
                                 net_force_y * viz_net_force_scale_));
  }
  if (fabs(viz_forces_scale_) > 0) {
    viz->markers.push_back(lines(marker("laser", "forces", 0), 12,
                                 points_x, points_y,
                                 forces_x * viz_forces_scale_,
                                 forces_y * viz_forces_scale_));
  }
  std::ostringstream status;
  status << std::setprecision(2) << "Speed: " << last_cmd_.drive.speed << std::setprecision(3) << "\nSteering: " << last_cmd_.drive.steering_angle;
  viz->markers.push_back(text(marker("base_link", "status", 0), -0.5, 0, status.str()));
  viz_pub_.publish(viz);
}

visualization_msgs::Marker marker(const std::string& frame_id, const std::string& ns, int id)
{
  // Marker constructor zero-initializes
  visualization_msgs::Marker viz;

  viz.header.frame_id = frame_id;
  viz.ns = ns;
  viz.id = id;
  viz.action = 0;
  viz.pose.orientation.w = 1;
  viz.color.a = 1;
  viz.lifetime = ros::Duration(0.1);
  viz.frame_locked = true;
  viz.points.clear();
  viz.colors.clear();

  return viz;
}

visualization_msgs::Marker arrow(visualization_msgs::Marker viz, double x, double y)
{
  viz.type = visualization_msgs::Marker::ARROW;
  viz.scale.x = 0.03;
  viz.scale.y = 0.06;
  viz.scale.z = 0.06;
  viz.color.r = 1;
  viz.color.g = 0;
  viz.color.b = 1;
  viz.points.resize(2);
  viz.points[0].x = 0;
  viz.points[0].y = 0;
  viz.points[0].z = viz.scale.z;
  viz.points[1].x = x;
  viz.points[1].y = y;
  viz.points[1].z = viz.scale.z;

  return viz;
}

visualization_msgs::Marker lines(visualization_msgs::Marker viz, int skip,
                                 const Eigen::ArrayXf& points_x, const Eigen::ArrayXf& points_y,
                                 const Eigen::ArrayXf& forces_x, const Eigen::ArrayXf& forces_y)
{
  assert(points_x.size() == points_y.size() &&
         points_y.size() == forces_x.size() &&
         forces_x.size() == forces_y.size());

  viz.type = visualization_msgs::Marker::LINE_LIST;
  viz.scale.x = 0.03;
  viz.scale.y = 0;
  viz.scale.z = 0;
  viz.color.r = 0;
  viz.color.g = 0;
  viz.color.b = 1;
  viz.points.reserve(points_x.size() / skip * 2);

  for (int i = 0; i < points_x.size(); i += skip) {
    visualization_msgs::Marker::_points_type::value_type origin, end;
    origin.x = points_x(i);
    origin.y = points_y(i);
    end.x = points_x(i) + forces_x(i);
    end.y = points_y(i) + forces_y(i);
    viz.points.push_back(origin);
    viz.points.push_back(end);
  }

  return viz;
}

visualization_msgs::Marker text(visualization_msgs::Marker viz, double x, double y, std::string text)
{
  viz.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  viz.pose.position.x = x;
  viz.pose.position.y = y;
  viz.scale.z = 0.3;
  viz.color.r = 1;
  viz.color.g = 1;
  viz.color.b = 1;
  viz.text = text;

  return viz;
}

} // namespace racecar_potential_field_controller
