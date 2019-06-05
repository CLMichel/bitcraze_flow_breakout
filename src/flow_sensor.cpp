#include "ros/ros.h"
#include "bitcraze_flow_breakout/flow_sensor.h"

#include <sstream>

// Local includes
#include <PMW3901/PMW3901.hpp>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "flow_sensor");


  ros::NodeHandle n;

    // publisher flow_sensor to publish value of delta X and delta Y
  ros::Publisher flow_pub = n.advertise<bitcraze_flow_breakout::flow_sensor>("flow_sensor", 1000);

  ros::Rate loop_rate(10);

  int16_t deltaX,deltaY;
  // message
  bitcraze_flow_breakout::flow_sensor msg;
  //sensor initialization
	PMW3901 flow(1);
	flow.begin();
  while (ros::ok())
  {




    flow.readMotionCount(&deltaX,&deltaY);
    msg.deltaX =  deltaX;
    msg.deltaY = deltaY;



     //publishing message
    flow_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
