// -*- mode:c++; fill-column: 100; -*-

#include "sparkfun_9dof_razor_imu_m0/driver.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include <cassert>
#include <cstdlib>
#include <sstream>
#include <algorithm>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/console.h>

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, \
							    __LINE__, (message) )

namespace sparkfun_9dof_razor_imu_m0
{

Driver::Driver(ros::NodeHandle nh,
	       ros::NodeHandle private_nh) :
  serial_(std::string(), 115200, serial::Timeout::simpleTimeout(100),
	  serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none),
  serial_thread_run_(true), frame_id_("imu"), rate_(50), gyro_fsr_(500), accel_fsr_(2)
{
  // set the logger level to debug from a parameter, e.g. settable from launch
  bool set_log_debug(false);
  if (private_nh.getParam("set_logger_level_debug", set_log_debug) && set_log_debug) {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  ROS_DEBUG("Initializing sparkfun_9dof_razor_imu_m0::Driver");

  // imu serial port must be provided
  if (!private_nh.getParam("port", port_)) {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 communication port parameter required.");
    ros::shutdown();
    return;
  }

  // get IMU's coordinate frame id
  private_nh.getParam("frame_id", frame_id_);
  if (frame_id_.empty()) {
    ROS_FATAL("SparkFun 9DoF Razor IMU M0 frame_id_ parameter must not be empty.");
    ros::shutdown();
    return;
  }
  
  // create publishers for the IMU data
  imu_data_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 10);
  imu_mag_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/mag", 10);
  
  // must start up a serial port worker thread
  if (0 != pthread_create(&serial_thread_, NULL, &Driver::serialThreadHelper, this)) {
    ROS_FATAL("Failed to create serial port worker thread.");
    ros::shutdown();
    return;
  }
}

Driver::~Driver()
{
  ROS_DEBUG("Destructing sparkfun_9dof_razor_imu_m0::Driver");

  // trigger serial port worker thread to terminate
  serial_thread_run_ = false;
  int result = pthread_join(serial_thread_, NULL);
  assert(0 == result);
}

std::vector<double> parseSensorMeasurement(std::string const& input)
{
  std::istringstream istream(input);
    
  std::vector<double> output;
  double current_val;
  while (istream >> current_val) {
    output.push_back(current_val);
    char comma_eater;
    if (istream >> comma_eater && comma_eater != ',') {
      // we expected either end of string or a comma
      output.clear();
      break;
    }
  }

  return output;
}

bool isValidSensorMeasurement(std::vector<double> const& input)
{
  // @todo: additional checks?
  return input.size() > 0;
}
  
void* Driver::serialThread(void)
{
  ROS_DEBUG("sparkfun_9dof_razor_imu_m0::Driver::serialThread starting.");

  // attempt to connect to the serial port
  try {
    serial_.setPort(port_);
    serial_.open();
  }
  catch (std::exception const& e) {
    ROS_FATAL("Failed to connect to the SparkFun 9DoF Razor IMU M0, %s.", e.what());
    ros::shutdown();
    return NULL;
  }

  // pause briefly to give serial port time to stabilize
  // not sure that this is necessary
  ros::Duration(0.05).sleep();

  if (configureImu()) {
  
    std::vector<double> measurement;
    while (serial_thread_run_) {
      measurement = parseSensorMeasurement(boost::trim_copy(serial_.readline()));
      if (isValidSensorMeasurement(measurement)) {
	if (measurement.size() == 14) {
	  sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
	  geometry_msgs::Vector3Stamped::Ptr mag_msg(new geometry_msgs::Vector3Stamped);

	  // set header
	  imu_msg->header.stamp = ros::Time::now();
	  imu_msg->header.frame_id = frame_id_;
	  mag_msg->header = imu_msg->header;
	  
	  // time
	  // time = measurement[0]
	  // acceleration in m/s^2
	  imu_msg->linear_acceleration.x = 9.80665 * measurement[1];
	  imu_msg->linear_acceleration.y = 9.80665 * measurement[2];
	  imu_msg->linear_acceleration.z = 9.80665 * measurement[3];
	  // gyroscope in rad/sec
	  imu_msg->angular_velocity.x = M_PI * measurement[4] / 180.0;
	  imu_msg->angular_velocity.y = M_PI * measurement[5] / 180.0;
	  imu_msg->angular_velocity.z = M_PI * measurement[6] / 180.0;
	  // magnetometer in Tesla
	  // convert coordinate frame to match IMU
	  mag_msg->vector.x = 1e-6 * measurement[8];
	  mag_msg->vector.y = 1e-6 * measurement[7];
	  mag_msg->vector.z = 1e-6 * -1 * measurement[9];
	  // quaternion
	  imu_msg->orientation.w = measurement[10];
	  imu_msg->orientation.x = measurement[11];
	  imu_msg->orientation.y = measurement[12];
	  imu_msg->orientation.z = measurement[13];

	  imu_data_pub_.publish(imu_msg);
	  imu_mag_pub_.publish(mag_msg);
	}
      }
    }

  }

    
  serial_.close();

  std::cout << "Thread terminating" << std::endl;
  ROS_DEBUG("sparkfun_9dof_razor_imu_m0::Driver::serialThread terminating.");
}

bool Driver::serialWriteVerify(std::string const& data)
{
  if (!serial_thread_run_)
    return false;

  ROS_DEBUG_STREAM("Write: '" << data << "'");
  
  size_t num_written = serial_.write(data);
  if (num_written != data.length()) {
    ROS_ERROR("Failed to write to SparkFun 9DoF Razor IMU M0 serial port. "
	      "Attempted to write %zd bytes, actually wrote %zd bytes.",
	      data.length(), num_written);
    return false;
  }

  return true;
}

bool Driver::sequentialCommand(std::string const& command, std::string const& response_format,
			       std::string const& response_desired_value, double response_timeout,
			       int max_sequential_commands)
{
  // @todo a full regex seems a little over the top, replace with a simple function
  boost::regex regex(response_format);
  assert(regex.mark_count() == 1);

  int command_count(0);
  while (command_count < max_sequential_commands) {

    // send a command to step to the next setting
    if (!serialWriteVerify(command))
      return false;
    command_count++;

    // check the response to see if we've reached the desired setting
    ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(response_timeout));
    while(ros::WallTime::now() < expire) {

      if (!serial_thread_run_)
	return false;
      std::string received = boost::trim_copy(serial_.readline());

      boost::cmatch matches;
      if (boost::regex_match(received.c_str(), matches, regex)) {
	if (response_desired_value.compare(std::string(matches[1].first, matches[1].second)) == 0) {
	  // successfully set command to desired value
	  ROS_DEBUG_STREAM(received);
	  return true;
	}
      }
      // else: received line could have been a sensor reading, continue
      
    }
  }

  ROS_ERROR("Sent %d '%s' commands to the IMU but did not receive an expected response matching "
	    "format '%s' with value '%s'.", command_count, command.c_str(),
	    response_format.c_str(), response_desired_value.c_str());
  return false;
}

bool Driver::togglePauseCommand(bool do_pause)
{
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  bool is_paused(true);
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1)));
  while(ros::WallTime::now() < expire) {
    
      if (!serial_thread_run_)
	return false;
      if (isValidSensorMeasurement(parseSensorMeasurement(boost::trim_copy(serial_.readline())))) {
	is_paused = false;
	break;
      }

  }

  // if current state is desired state return success
  if (do_pause == is_paused)
    return true;

  // current state is not the desired state, toggle pause
  if (!serialWriteVerify(" "))
    return false;
  
  return true;
}

bool Driver::toggleSensorCommand(std::string const& command, bool turn_on, unsigned num_values)
{
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  std::vector<double> initial_measurement;
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1)));
  while(ros::WallTime::now() < expire) {

    if (!serial_thread_run_)
      return false;
    initial_measurement = parseSensorMeasurement(boost::trim_copy(serial_.readline()));
    
    if (isValidSensorMeasurement(initial_measurement))
      break;
  }  

  if (!isValidSensorMeasurement(initial_measurement))
    return false; // timed out

  // send toggle command to see if increased or decreased number of measurements
  if (!serialWriteVerify(command))
    return false;

  // wait another 3 * sensor sample period for measurment length to change
  std::vector<double> new_measurement;
  expire = ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1));
  while(ros::WallTime::now() < expire) {

    if (!serial_thread_run_)
      return false;
    new_measurement = parseSensorMeasurement(boost::trim_copy(serial_.readline()));


    if (isValidSensorMeasurement(new_measurement) &&
	new_measurement.size() != initial_measurement.size())
      break;
  }

  if (!(isValidSensorMeasurement(new_measurement) &&
	new_measurement.size() != initial_measurement.size()))
    return false; // timed out with a change in measurement size

  // is magnitude of change in the measurement length correct
  int delta = static_cast<int>(new_measurement.size()) -
    static_cast<int>(initial_measurement.size());
  if (abs(delta) != num_values)
    return false; // unexpected number of values changed
  
  // did the measurement size change in the direction we wanted?
  if (turn_on && delta > 0)
    return true; // success!
  else if (!turn_on && delta < 0)
    return true; // success!

  // command had the opposite effect of desired, send it again to toggle to correct state
  initial_measurement = new_measurement;
  new_measurement.clear();
  if (!serialWriteVerify(command))
    return false;

  // wait another 3 * sensor sample period for measurment length to change
  expire = ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1));
  while(ros::WallTime::now() < expire) {

    if (!serial_thread_run_)
      return false;
    new_measurement = parseSensorMeasurement(boost::trim_copy(serial_.readline()));


    if (isValidSensorMeasurement(new_measurement) &&
	new_measurement.size() != initial_measurement.size())
      break;
  }

  if (!(isValidSensorMeasurement(new_measurement) &&
	new_measurement.size() != initial_measurement.size()))
    return false; // timed out with a change in measurement size

  // is magnitude of change in the measurement length correct
  delta = static_cast<int>(new_measurement.size()) -
    static_cast<int>(initial_measurement.size());
  if (abs(delta) != num_values)
    return false; // unexpected number of values changed
  
  // did the measurement size change in the direction we wanted?
  if (turn_on && delta > 0)
    return true; // success!
  else if (!turn_on && delta < 0)
    return true; // success!

  // not sure what is going on if we get here
  return false;
}

bool isSensorMeasurementEngineeringUnits(std::vector<double> const& input)
{
  // @todo heading seems to always be in floating point
  
  bool is_engineering_units(false);
  for (std::vector<double>::const_iterator i(input.begin()); i != input.end(); i++) {
    if (floor(*i) != *i) {
      is_engineering_units = true;
    }
  }
  return is_engineering_units;
}
  
bool Driver::toggleEngineeringUnitsCommand(bool turn_on)
{
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  std::vector<double> measurement;
  ros::WallTime expire(ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1)));
  while(ros::WallTime::now() < expire) {

    if (!serial_thread_run_)
      return false;
    measurement = parseSensorMeasurement(boost::trim_copy(serial_.readline()));
    
    if (isValidSensorMeasurement(measurement))
      break;
  }  

  if (!isValidSensorMeasurement(measurement))
    return false; // timed out

  bool is_engineering_units(isSensorMeasurementEngineeringUnits(measurement));

  if (turn_on && is_engineering_units)
    return true;
  else if (!turn_on && !is_engineering_units)
    return true;

  // send toggle command to see if increased or decreased number of measurements
  if (!serialWriteVerify("c"))
    return false;
  
  // wait for 3 * sensor sample period (1/rate_) for a measurement
  expire = ros::WallTime::now() + ros::WallDuration(std::max(3.0 / rate_, 0.1));
  while(ros::WallTime::now() < expire) {

    if (!serial_thread_run_)
      return false;
    measurement = parseSensorMeasurement(boost::trim_copy(serial_.readline()));
    
    if (isValidSensorMeasurement(measurement))
      break;
  }  

  if (!isValidSensorMeasurement(measurement))
    return false; // timed out

  is_engineering_units = isSensorMeasurementEngineeringUnits(measurement);

  if (turn_on && is_engineering_units)
    return true;
  else if (!turn_on && !is_engineering_units)
    return true;

  // command did not work
  return false;
}
  
std::string itos(int i) {
  return static_cast< std::ostringstream & >(std::ostringstream() << std::dec << i).str();
}
  
bool Driver::configureImu()
{
  // Configure the SparkFun 9DoF Razor IMU M0 default firmware 

  // set output rate
  if (!sequentialCommand("r", "IMU rate set to (\\d+) Hz",
			 itos(static_cast<int>(rate_)), 0.2, 12))
    return false;

  // set acceleration full-scale range
  if (!sequentialCommand("A", "Accel FSR set to \\+/-(\\d+) g",
			 itos(static_cast<int>(accel_fsr_)), 0.2, 5))
    return false;

  // set gyroscope full-scale range
  if (!sequentialCommand("G", "Gyro FSR set to \\+/-(\\d+) dps",
			 itos(static_cast<int>(gyro_fsr_)), 0.2, 5))
    return false;

  // unpause
  if (!togglePauseCommand(false))
    return false;

  // toggle time on
  if (!toggleSensorCommand("t", true, 1))
    return false;

  // toggle accelerometer on
  if (!toggleSensorCommand("a", true, 3))
    return false;

  // toggle gyro on
  if (!toggleSensorCommand("g", true, 3))
    return false;

  // toggle magnetometer on
  if (!toggleSensorCommand("m", true, 3))
    return false;

  // toggle quaternion on
  if (!toggleSensorCommand("q", true, 4))
    return false;

  // toggle Euler angle off
  if (!toggleSensorCommand("e", false, 3))
    return false;

  // toggle heading off
  if (!toggleSensorCommand("h", false, 1))
    return false;

  // toggle engineering units on
  if (!toggleEngineeringUnitsCommand(true))
    return false;


  ROS_DEBUG("SparkFun 9DoF IMU M0 configuration successful.");
  return true;
}

class ConfigureException : public std::exception
{
  // Disable copy constructors
  ConfigureException& operator=(const ConfigureException&);
  std::string e_what_;
  
public:
  ConfigureException(const char *description)
  {
    std::ostringstream ss;
    ss << "ConfigureException: " << description << " failed.";
    e_what_ = ss.str();
  }

  ConfigureException(std::ostringstream& description)
  {
    e_what_ = description.str();
  }
  
  ConfigureException(const ConfigureException& other) : e_what_(other.e_what_) {}

  virtual ~ConfigureException() throw() {}

  virtual const char* what() const throw()
  {
    return e_what_.c_str();
  }
};

} // namespace sparkfun_9dof_razor_imu_m0
