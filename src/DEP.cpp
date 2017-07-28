#include <roboy_DEP/DEP.hpp>

DEP::DEP(){
	
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
	outputPosMax=1200;
	outputNegMax=-1200;
	spPosMax=1000;
	spNegMax=-1000;
	IntegralPosMax=4000;
	IntegralNegMax=-4000;
	Kp=3000;
	Ki=0;
	Kd=0;
	forwardGain=0;
	deadBand=0;
	target_force=100;
	range=3;

	//where to get polyPar?
}

DEP::~DEP(){}

void DEP::force(){
	control_mode = 2;
	setMotorConfig();
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i, target_force);
	}
}

void DEP::initialize(){
	control_mode = 0;
	setMotorConfig();
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		params[i][0] = positions.val(i,0);
		params[i][1] = params[i][0]-range;
		params[i][2] = params[i][0]+range;
	}
}

void DEP::update(){
	//ROS_INFO("update");
	motorRefs = soctrl->update(positions,displacements);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i, getMuscleLengthScaledInv(i, motorRefs.val(i,0)));
	}
}

void DEP::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg){
	ROS_INFO_THROTTLE(1,"DEP received motor status");
	// get positions and displacements
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		positions.val(i,0) = scale_position(i,msg->position[i]);
		displacements.val(i,0) = scale_displacement(i,msg->displacement[i]);
	}
}

void DEP::DepCommand(const roboy_DEP::command::ConstPtr &msg){
	ROS_INFO("Command: %s", msg->command.c_str());
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
	return value;
	//return polyPar[motor_index][0] + value*polyPar[motor_index][1] + value*value*polyPar[motor_index][2] + value*value*value*polyPar[motor_index][3];
}

double DEP::getMuscleLengthScaledInv(int motor_index,double value){
	double min = params[motor_index][1];
	double max = params[motor_index][2];
	return (value+1.0)/2.0*(max-min)+min; // -1/1 -> min/max
}