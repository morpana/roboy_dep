#include <roboy_DEP/DEP.hpp>

DEP::DEP(){
	mode = 0;
	// initialize ROS node
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "roboy_DEP");
	nh = ros::NodeHandlePtr(new ros::NodeHandle);
	motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &DEP::MotorStatus, this);
	depCommand = nh->subscribe("/roboy_DEP/command", 1, &DEP::DepCommand, this);
	motorConfig = nh->advertise<roboy_communication_middleware::MotorConfig>("/roboy/middleware/MotorConfig", 1);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(5));
	spinner->start();

	// initialize matrices
	positions = matrix::Matrix(NUMBER_OF_MOTORS,1);
	displacements = matrix::Matrix(NUMBER_OF_MOTORS,1);
	motorRefs = matrix::Matrix(NUMBER_OF_MOTORS,1);

	// initialize controller
	soctrl = new DerMartiusController(NUMBER_OF_MOTORS,NUMBER_OF_MOTORS,true);

	// initialize parameters
	control_mode=0;
	outputPosMax=1000;
	outputNegMax=-1000;
	spPosMax=100000000;
	spNegMax=-100000000;
	IntegralPosMax=100;
	IntegralNegMax=-100;
	Kp=80;
	Ki=0;
	Kd=0;
	forwardGain=0;
	deadBand=0;
	target_force=20;
	range=5;

	encoder_to_rad = 2.0*3.14159/(2000.0*53.0);

	//where to get polyPar?
}

DEP::~DEP(){}

void DEP::force(){
	control_mode = 2;
	setMotorConfig();
	std::lock_guard<std::mutex> lock(myoMaster->mux);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i, target_force);
	}
}

void DEP::initialize(){
	//matrix::Matrix m = matrix::Matrix(NUMBER_OF_MOTORS,1);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		params[i][0] = positions.val(i,0)*encoder_to_rad;
		//m.val(i,0) = encoder_to_rad*positions.val(i,0);
		params[i][1] = params[i][0]-range;
		params[i][2] = params[i][0]+range;
	}
	/*for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		m.val(i,0) = scale_position(i, encoder_to_rad*positions.val(i,0));
	}*/
	//printArray(m);
	std::lock_guard<std::mutex> lock(myoMaster->mux);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i,positions.val(i,0));
	}
	control_mode = 0;
	Kp = 1;
	setMotorConfig();
}

void DEP::update(){
	ROS_INFO_THROTTLE(1,"update");
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		// convert encoder ticks to radians, and then scale them
		positions.val(i,0) = scale_position(i, positions.val(i,0)*encoder_to_rad);
		displacements.val(i,0) = scale_displacement(i, displacements.val(i,0));
	}
	//ROS_INF0("pos and disp: ");
	//printArray(positions);
	//printArray(displacements);
	motorRefs = soctrl->update(positions,displacements);
	std::lock_guard<std::mutex> lock(myoMaster->mux);
	matrix::Matrix m = matrix::Matrix(NUMBER_OF_MOTORS,1);
	//matrix::Matrix m1 = matrix::Matrix(NUMBER_OF_MOTORS,1);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i, getMuscleLengthScaledInv(i, motorRefs.val(i,0))*(1/encoder_to_rad));
		//myoMaster->changeSetPoint(i, getMuscleLengthScaledInv(i, motorRefs.val(i,0)));
		//myoMaster->changeSetPoint(i, 3000);
		//if (i == 2){
		//	myoMaster->changeSetPoint(1,(1/encoder_to_rad)*1);
		//}
		//m.val(i,0) = motorRefs.val(i,0);
		m.val(i,0) = getMuscleLengthScaledInv(i, motorRefs.val(i,0))*(1/encoder_to_rad);
	}
	//ROS_INF0_THROTTLE(1, "motorRefs: ");
	//printArray(m1);
	printArray(m);
}

void DEP::printArray(const matrix::Matrix& array){
	std::string s;
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		s.append(boost::lexical_cast<std::string>(array.val(i,0))+",");
	}
	ROS_INFO("%s", s.c_str());
}

void DEP::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg){
	ROS_INFO_THROTTLE(1,"DEP received motor status");
	// get positions and displacements
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		positions.val(i,0) = msg->position[i];
		//printArray(positions);
		displacements.val(i,0) = msg->displacement[i];
	}
}

void DEP::DepCommand(const roboy_DEP::command::ConstPtr &msg){
	ROS_INFO("Command: %s", msg->command.c_str());
	std::string force = "force";
	std::string init = "init";
	std::string update = "update";
	if (msg->command.c_str() == force){
		mode = 1;
	} else if (msg->command.c_str() == init){
		mode = 2;
	} else if (msg->command.c_str() == update){
		mode = 3;
	}
}

void DEP::setMotorConfig(){
	roboy_communication_middleware::MotorConfig msg;
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		msg.motors.push_back(i);
		msg.control_mode.push_back(control_mode);
		msg.outputPosMax.push_back(outputPosMax);
		msg.outputNegMax.push_back(outputNegMax);
		msg.spPosMax.push_back(spPosMax);
		msg.spNegMax.push_back(spNegMax);
		msg.IntegralPosMax.push_back(IntegralPosMax);
		msg.IntegralNegMax.push_back(IntegralNegMax);
		msg.Kp.push_back(Kp);
		msg.Ki.push_back(Ki);
		msg.Kd.push_back(Kd);
		msg.forwardGain.push_back(forwardGain);
		msg.deadBand.push_back(deadBand);
	}
	motorConfig.publish(msg);
}

double DEP::scale_position(int motor_index, double value){
	double min = params[motor_index][1];
	double max = params[motor_index][2];
	return (value-min)/(max-min)*2.0-1.0; // -1/1 -> min/max
}

double DEP::scale_displacement(int motor_index, double value){
	//return value;
	return value*0.237536 + value*value*0.000032;
	//return polyPar[motor_index][0] + value*polyPar[motor_index][1] + value*value*polyPar[motor_index][2] + value*value*value*polyPar[motor_index][3];
	//soft spring
	//remember relative encoders -> initialization procedure
}

double DEP::getMuscleLengthScaledInv(int motor_index,double value){
	double min = params[motor_index][1];
	double max = params[motor_index][2];
	return (value+1.0)/2.0*(max-min)+min; // -1/1 -> min/max
}