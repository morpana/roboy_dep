#include <roboy_dep/DEP.hpp>

int main(int argc, char **argv) {

	//initialize myoMaster    
    MyoMaster myoMaster;
    myoMaster.initialize(argc,argv);
    myoMaster.start();

	// initialize DEP
	DEP* dep = new DEP();
	dep->myoMaster = &myoMaster;
	
	while(ros::ok()){
		if (dep->mode == 0){
			usleep(20000);
		} else if (dep->mode == 1){
			dep->force();
			ROS_INFO("Force mode initialized");
			dep->mode = 0;
		} else if (dep->mode == 2){
			dep->initialize();
			ROS_INFO("Arm initialized");
			dep->mode = 0;
		} else if (dep->mode == 3){
			ROS_INFO("Controller running");
			dep->mode = 4;
		} else if (dep->mode == 4){
			//dep->update();
			//update occurs in motorStatus subcsriber as this is updated at regular 20 ms interval
			sleep(1);
		}
	}
	return 0;
}