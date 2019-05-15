#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// Local includes
#include <PMW3901/PMW3901.hpp>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "flow_sensor");


  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int16_t deltaX,deltaY;
	PMW3901 flow(1);
	flow.begin();
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    flow.readMotionCount(&deltaX,&deltaY);
    std::stringstream ss;
    ss << "deltaX = " << deltaX << " deltaY = " << deltaY;
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
