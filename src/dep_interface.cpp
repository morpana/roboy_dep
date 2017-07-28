// need to create publisher node that will publish messages to DEP node
// the aim is to allow for a user to input commands on the publisher node
// to modify the behavior or parameters of the DEP node

#include "ros/ros.h"
#include "roboy_DEP/command.h"

using namespace std;
int main(int argc, char **argv){
	ros::init(argc, argv, "dep_interface");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<roboy_DEP::command>("/roboy_DEP/command", 1);
	ros::Rate loop_rate(10);
	while (ros::ok()){
		roboy_DEP::command msg;
		cout << "Waiting for user input: " << endl;
		cin >> msg.command;
		ROS_INFO("Command: %s", msg.command.c_str());
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}