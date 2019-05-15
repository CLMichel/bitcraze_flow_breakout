#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// Local includes
#include <tof/tof.hpp>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "tof");


  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("tof", 1000);

  ros::Rate loop_rate(10);

  int i;
  int Distance;
  int model, revision;

	// For Raspberry Pi's, the I2C channel is usually 1
	// For other boards (e.g. OrangePi) it's 0
	while(tofInit(0, 0x29, 1)) // set long range mode (up to 2m)
	{
		printf("Sensor initialisation...\n");
	}
	printf("VL53L0X device successfully opened.\n");
	i = tofGetModel(&model, &revision);
	printf("Model ID - %d\n", model);
	printf("Revision ID - %d\n", revision);



  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    Distance = tofReadDistance();
    std::stringstream ss;
    ss << "h = " << Distance;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
