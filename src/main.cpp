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
	
	while(1){sleep(1);}
	return 1;
}