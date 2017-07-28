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

	std::string force = "force";
	std::string init = "init";
	std::string update = "update";

	int mode = 0;
	while (ros::ok()){
		roboy_DEP::command msg;
		if (mode == 0){
			ROS_INFO("Enter \"force\" to initialize force mode");
			cout << "Waiting for user input: " << endl;
			cin >> msg.command;
			if (msg.command.c_str() == force){
				pub.publish(msg);		
				mode += 1;
			} else {
				ROS_INFO("Invalid command!");
			}
		} else if (mode == 1){
			ROS_INFO("Enter \"init\" to initialize arm");
			cout << "Waiting for user input: " << endl;
			cin >> msg.command;
			if (msg.command.c_str() == init){
				pub.publish(msg);
				mode += 1;
			} else {
				ROS_INFO("Invalid command!");
			}		
		} else if (mode == 2){
			ROS_INFO("Enter \"update\" to start controller");
			cout << "Waiting for user input: " << endl;
			cin >> msg.command;
			if (msg.command.c_str() == update){
				pub.publish(msg);
				mode += 1;
			} else {
				ROS_INFO("Invalid command!");
			}
		} else if (mode == 3){
			ROS_INFO("Enter command: ");
			cout << "Waiting for user input: " << endl;
			cin >> msg.command;
			pub.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}