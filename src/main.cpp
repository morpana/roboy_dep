#include <roboy_DEP/DEP.hpp>

#define UPDATE_RATE 20000	//20 ms

int main(int argc, char **argv) {

	//initialize myoMaster    
    MyoMaster myoMaster;
    myoMaster.initialize(argc,argv);
    myoMaster.start();

	// initialize DEP
	DEP dep;
	dep.myoMaster = &myoMaster;
	
	while(ros::ok()){
		if (dep.mode == 1){
			dep.force();
			ROS_INFO("Force mode initialized");
			dep.mode = 0;
		} else if (dep.mode == 2){
			dep.initialize();
			ROS_INFO("Arm initialized");
			dep.mode = 0;
		} else if (dep.mode == 3){
			ROS_INFO("Controller running");
			dep.mode = 4;
		} else if (dep.mode == 4){
			dep.update();
		}
		usleep(UPDATE_RATE);
	}
	return 0;
}