#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <csignal>

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


  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("tof", 1000);

  ros::Rate loop_rate(10);

  #ifdef HIGH_SPEED
  	#ifdef HIGH_ACCURACY
  		#error HIGH_SPEED and HIGH_ACCURACY cannot be both enabled at once!
  	#endif
  #endif

  // Register SIGINT handler
	signal(SIGINT, sigintHandler);

	// Create the sensor with default values
	VL53L0X sensor;
	try {
		// Initialize the sensor
		sensor.initialize();
		// Set measurement timeout value
		sensor.setTimeout(200);
	} catch (const std::exception & error) {
		std::cerr << "Error initializing sensor with reason:" << std::endl << error.what() << std::endl;
		return 1;
	}

	#ifdef LONG_RANGE
		try {
			// Lower the return signal rate limit (default is 0.25 MCPS)
			sensor.setSignalRateLimit(0.1);
			// Increase laser pulse periods (defaults are 14 and 10 PCLKs)
			sensor.setVcselPulsePeriod(VcselPeriodPreRange, 18);
			sensor.setVcselPulsePeriod(VcselPeriodFinalRange, 14);
		} catch (const std::exception & error) {
			std::cerr << "Error enabling long range mode with reason:" << std::endl << error.what() << std::endl;
			return 2;
		}
	#endif

	#if defined HIGH_SPEED
		try {
			// Reduce timing budget to 20 ms (default is about 33 ms)
			sensor.setMeasurementTimingBudget(20000);
		} catch (const std::exception & error) {
			std::cerr << "Error enabling high speed mode with reason:" << std::endl << error.what() << std::endl;
			return 3;
		}
	#elif defined HIGH_ACCURACY
		try {
			// Increase timing budget to 200 ms
			sensor.setMeasurementTimingBudget(200000);
		} catch (const std::exception & error) {
			std::cerr << "Error enabling high accuracy mode with reason:" << std::endl << error.what() << std::endl;
			return 3;
		}
	#endif

  uint16_t distance;



  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    try {
			// Read the range. Note that it's a blocking call
			distance = sensor.readRangeSingleMillimeters();
		} catch (const std::exception & error) {
			std::cerr << "Error geating measurement with reason:" << std::endl << error.what() << std::endl;
			// You may want to bail out here, depending on your application - error means issues on I2C bus read/write.
			// return 3;
			distance = 8096;
		}
    std::stringstream ss;
    ss << "h = " << distance;
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
