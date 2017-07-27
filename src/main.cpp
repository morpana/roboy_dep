#include <roboy_DEP/DEP.hpp>
#include <signal.h>

#define UPDATE_RATE 20000	//20 ms

int main(int argc, char **argv) {

	//initialize myoMaster    
    MyoMaster myoMaster;
    myoMaster.initialize(argc,argv);
    myoMaster.start();

	// initialize DEP
	DEP dep;
	dep.myoMaster = &myoMaster;
	
	// apply constant force to all muscles and move arm manually to symmetric initial position
	dep.force();
	
	// initialize arm position
	dep.initialize();

	while(1){
		dep.update();
		//printf("test");
		usleep(UPDATE_RATE);
	}
	return 1;
}