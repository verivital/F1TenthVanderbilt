// -*- mode:c++; fill-column: 100; -*-

#ifndef SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_
#define SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_

#include <pthread.h>

#include <string>

#include <serial/serial.h>

#include <ros/ros.h>

namespace sparkfun_9dof_razor_imu_m0
{

class Driver
{
public:

  Driver(ros::NodeHandle nh,
	 ros::NodeHandle private_nh);
  ~Driver();
  
private:
  serial::Serial serial_;               ///< serial interface to the IMU
  pthread_t serial_thread_;             ///< serial port worker thread
  bool serial_thread_run_;              ///< serial port worker thread sentinel

  std::string port_;                    ///< serial port path (set by parameter)
  std::string frame_id_;                ///< IMU coordinate frame ID (set by parameter)
  double rate_;                         ///< sensor output rate (set by parameter)
  double gyro_fsr_;                     ///< gyro full scale range (set by parameter)
  double accel_fsr_;                    ///< accelerometer full scale range (set by parameter)

  // ROS services
  ros::Publisher imu_data_pub_;         ///< publisher for IMU message
  ros::Publisher imu_mag_pub_;          ///< publisher for magnetometer message
  
  /// serial port interface worker thread function
  void* serialThread(void);

  /// helper function to get class instance for serial port interface worker thread
  static void* serialThreadHelper(void *context)
  {
    return (static_cast<Driver*>(context)->serialThread());
  }

  bool serialWriteVerify(std::string const& data);
  bool sequentialCommand(std::string const& command, std::string const& response_format,
			 std::string const& response_desired_value, double response_timeout,
			 int max_sequential_commands);
  bool toggleSensorCommand(std::string const& command, bool turn_on, unsigned num_values);
  bool toggleEngineeringUnitsCommand(bool turn_on);
  bool togglePauseCommand(bool do_pause);
  bool configureImu();
};

} // namespace sparkfun_9dof_razor_imu_m0

#endif // SPARKFUN_9DOF_RAZOR_IMU_M0_DRIVER_H_
