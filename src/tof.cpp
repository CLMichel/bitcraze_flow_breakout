#include "ros/ros.h"
#include "bitcraze_flow_breakout/tof.h"

#include <sstream>
#include <csignal>
#include <limits>

// Local includes
#include <VL53L0X/VL53L0X.hpp>

// SIGINT (CTRL-C) exit flag and signal handler
volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) {
	exitFlag = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tof");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Paramètres configurables
  double rate;
  std::string topic_name;
  std::string frame_id;
  int timeout_ms;
  bool long_range;
  bool high_speed;
  bool high_accuracy;

  nh_private.param<double>("rate", rate, 10.0);
  nh_private.param<std::string>("topic", topic_name, "tof");
  nh_private.param<std::string>("frame_id", frame_id, "tof");
  nh_private.param<int>("timeout_ms", timeout_ms, 200);
  nh_private.param<bool>("long_range", long_range, false);
  nh_private.param<bool>("high_speed", high_speed, false);
  nh_private.param<bool>("high_accuracy", high_accuracy, false);

  // Publisher tof to send distance between sensor and ground
  ros::Publisher tof_pub = nh.advertise<bitcraze_flow_breakout::tof>(topic_name, 10);

  ros::Rate loop_rate(rate);

  // Validation des paramètres
  if (high_speed && high_accuracy) {
    ROS_FATAL("HIGH_SPEED and HIGH_ACCURACY cannot be both enabled at once!");
    return 1;
  }

  // Register SIGINT handler
  signal(SIGINT, sigintHandler);

  // Create the sensor with default values
  VL53L0X sensor;
  try {
    // Initialize the sensor
    sensor.initialize();
    // Set measurement timeout value
    sensor.setTimeout(timeout_ms);
    ROS_INFO("VL53L0X sensor initialized successfully");
  } catch (const std::exception & error) {
    ROS_FATAL("Error initializing sensor: %s", error.what());
    return 1;
  }

  // Configure long range mode if requested
  if (long_range) {
    try {
      // Lower the return signal rate limit (default is 0.25 MCPS)
      sensor.setSignalRateLimit(0.1);
      // Increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor.setVcselPulsePeriod(VcselPeriodPreRange, 18);
      sensor.setVcselPulsePeriod(VcselPeriodFinalRange, 14);
      ROS_INFO("Long range mode enabled");
    } catch (const std::exception & error) {
      ROS_ERROR("Error enabling long range mode: %s", error.what());
      return 2;
    }
  }

  // Configure speed/accuracy mode
  if (high_speed) {
    try {
      // Reduce timing budget to 20 ms (default is about 33 ms)
      sensor.setMeasurementTimingBudget(20000);
      ROS_INFO("High speed mode enabled (20ms timing budget)");
    } catch (const std::exception & error) {
      ROS_ERROR("Error enabling high speed mode: %s", error.what());
      return 3;
    }
  } else if (high_accuracy) {
    try {
      // Increase timing budget to 200 ms
      sensor.setMeasurementTimingBudget(200000);
      ROS_INFO("High accuracy mode enabled (200ms timing budget)");
    } catch (const std::exception & error) {
      ROS_ERROR("Error enabling high accuracy mode: %s", error.what());
      return 3;
    }
  }

  ROS_INFO("Publishing ToF data on topic '%s' at %.1f Hz", topic_name.c_str(), rate);

  uint16_t distance;
  bool valid;

  while (ros::ok() && !exitFlag)
  {
    bitcraze_flow_breakout::tof msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;

    try {
      // Read the range. Note that it's a blocking call
      distance = sensor.readRangeSingleMillimeters();
      valid = true;
    } catch (const std::exception & error) {
      ROS_WARN_THROTTLE(5, "Error getting measurement: %s", error.what());
      // Error means issues on I2C bus read/write
      // Mark as invalid instead of using arbitrary value
      valid = false;
      distance = 0;
    }

    msg.distance = distance / 1000.0;  // Convert mm to meters
    msg.valid = valid;

    // Publishing message
    tof_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Shutting down tof node");
  return 0;
}
