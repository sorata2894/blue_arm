#include <ros/ros.h>
#include <signal.h>
#include "manipulator/Manipulator.h"
#include "maxon_epos2/EposController.hpp"

Manipulator* blue_arm;

void sigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  blue_arm->closeDevice();
  delete blue_arm;
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "blue_arm_control", ros::init_options::NoSigintHandler);
	ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::NodeHandle nodeHandle;

	signal(SIGINT, sigintHandler);

	blue_arm = new Manipulator(nodeHandle);

	ros::Rate loop_rate(blue_arm->sample_rate);
	while (ros::ok()){
		ros::Time start = ros::Time::now();
		blue_arm->process(loop_rate);
		ros::spinOnce();
		loop_rate.sleep();
		std::cout<<"loop_time = "<<(ros::Time::now() - start).toSec() * 1000<<" ms"<<std::endl;
	}
	spinner.stop();
	return 0;
}