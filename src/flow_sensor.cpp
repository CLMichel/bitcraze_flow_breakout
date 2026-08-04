#include "ros/ros.h"
#include "bitcraze_flow_breakout/flow_sensor.h"

#include <sstream>

// Local includes
#include <PMW3901/PMW3901.hpp>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "flow_sensor");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Paramètres configurables
  double rate;
  std::string topic_name;
  std::string frame_id;
  int spi_cs_pin;

  nh_private.param<double>("rate", rate, 10.0);
  nh_private.param<std::string>("topic", topic_name, "flow_sensor");
  nh_private.param<std::string>("frame_id", frame_id, "flow_sensor");
  nh_private.param<int>("spi_cs_pin", spi_cs_pin, 1);

  // Publisher flow_sensor to publish value of delta X and delta Y
  ros::Publisher flow_pub = nh.advertise<bitcraze_flow_breakout::flow_sensor>(topic_name, 10);

  ros::Rate loop_rate(rate);

  // Sensor initialization with error handling
  PMW3901 flow(spi_cs_pin);
  if (!flow.begin()) {
    ROS_FATAL("Failed to initialize PMW3901 sensor. Check wiring and permissions (must run as root).");
    return 1;
  }
  ROS_INFO("PMW3901 flow sensor initialized successfully on CS pin %d", spi_cs_pin);
  ROS_INFO("Publishing flow data on topic '%s' at %.1f Hz", topic_name.c_str(), rate);

  int16_t deltaX, deltaY;

  while (ros::ok())
  {
    flow.readMotionCount(&deltaX, &deltaY);

    bitcraze_flow_breakout::flow_sensor msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.deltaX = deltaX;
    msg.deltaY = deltaY;

    // Publishing message
    flow_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // Cleanup resources
  ROS_INFO("Shutting down flow_sensor node");
  bcm2835_close();

  return 0;
}
