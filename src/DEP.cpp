#include <roboy_dep/DEP.hpp>

DEP::DEP(){
	mode = 0;
	// initialize ROS node
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "roboy_dep");
	nh = ros::NodeHandlePtr(new ros::NodeHandle);
	motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &DEP::MotorStatus, this);
	depCommand = nh->subscribe("/roboy_dep/command", 1, &DEP::DepCommand, this);
	depParameters = nh->subscribe("/roboy_dep/depParameters", 1, &DEP::DepParameters, this);
	depLoadMatrix = nh->subscribe("/roboy_dep/depLoadMatrix", 1, &DEP::DepLoadMatrix, this);
	motorConfig = nh->advertise<roboy_communication_middleware::MotorConfig>("/roboy/middleware/MotorConfig", 1);
	DepMatrix = nh->advertise<roboy_dep::depMatrix>("/roboy_dep/depMatrix", 1);
	transition_start_pub = nh->advertise<roboy_dep::transition_start>("/roboy_dep/transition_start", 1);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(5));
	spinner->start();

	// initialize matrices
	positions = matrix::Matrix(NUMBER_OF_MOTORS,1);
	displacements = matrix::Matrix(NUMBER_OF_MOTORS,1);
	motorRefs = matrix::Matrix(NUMBER_OF_MOTORS,1);

	// initialize controller
	soctrl = new DerMartiusController(NUMBER_OF_MOTORS,NUMBER_OF_MOTORS,true);
	init_params();

	sensorDelay = nh->subscribe("/roboy_dep/sensor_delay", 1, &DEP::loadSensorDelay, this);
	sensor_delay = matrix::Matrix(100,NUMBER_OF_MOTORS);
	sensor_delay_count = 0;

	stop_sub = nh->subscribe("/roboy_dep/stop", 1, &DEP::stopCallback, this);
	stop = false;
}

DEP::~DEP(){}

void DEP::stopCallback(const roboy_dep::stop::ConstPtr &msg){
	stop = msg->stop;
}


void DEP::loadSensorDelay(const roboy_dep::depMatrix::ConstPtr &msg){
	//ROS_INFO("test");
	//ROS_INFO("%i", msg->size);
	sensor_delay = matrix::Matrix(msg->size, msg->depMatrix[0].size);
	for (int i = 0; i < msg->size; i++) {
		for (int j = 0; j < msg->depMatrix[i].size; j++) {
			sensor_delay.val(msg->size-i,j) = msg->depMatrix[i].cArray[j];
		}
	}
	sensor_delay_count = msg->size-1;
	//printMatrix(sensor_delay);
}

void DEP::DepLoadMatrix(const roboy_dep::depMatrix::ConstPtr &msg){
	//there needs to be at least 1 second in between two consecutive depLoadMatrix messages to be considered a transition start
	if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() > 1000){
		roboy_dep::transition_start msg;
		msg.start = true;
		transition_start_pub.publish(msg);
	}
	ROS_INFO("Received matrix");
	matrix::Matrix C = matrix::Matrix(msg->size, msg->depMatrix[0].size);
	//DEBUG
	//ROS_INFO("msg sizes -- motors: %i, sensors: %i", msg->size, msg->depMatrix[0].size);
	// assign the data from the depMatrix
	for (int i = 0; i < msg->size; i++) {
		for (int j = 0; j < msg->depMatrix[i].size; j++) {
			C.val(i,j) = msg->depMatrix[i].cArray[j];
		}
	}
	soctrl->setC(C);
	t_start = std::chrono::high_resolution_clock::now();
}

void DEP::init_params(){
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
	Kd=-1;
	forwardGain=0;
	deadBand=0;
	target_force=20;
	range=5;

	encoder_to_rad = 2.0*3.14159/(2000.0*53.0);

	motor_status_lock = 1;
	t_start = std::chrono::high_resolution_clock::now();
}

void DEP::force(){
	init_params();
	std::lock_guard<std::mutex> lock(myoMaster->mux);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i, target_force);
	}
	control_mode = 2;
	setMotorConfig();
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
	//printMatrix(m);
	std::lock_guard<std::mutex> lock(myoMaster->mux);
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i,positions.val(i,0));
	}
	control_mode = 0;
	Kp = 1;
	setMotorConfig();
}

void DEP::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg){
	if (mode == 4){
		if (not stop){
			//DEBUG
			/*t_end = std::chrono::high_resolution_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
			ROS_INFO("%i", diff);
			t_start = std::chrono::high_resolution_clock::now();*/
			//ROS_INFO("DEP received motor status");
			// get positions and displacements
			for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
				positions.val(i,0) = msg->position[i];
				//printMatrix(positions);
				displacements.val(i,0) = msg->displacement[i];
			}
			//update();
			//ROS_INFO("update");
			for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
				// convert encoder ticks to radians, and then scale them
				positions.val(i,0) = scale_position(i, positions.val(i,0)*encoder_to_rad);
				displacements.val(i,0) = scale_displacement(i, displacements.val(i,0));
			}
			
			if (sensor_delay_count > 0){
				matrix::Matrix delays = matrix::Matrix(NUMBER_OF_MOTORS,1);
				for (int i=0; i<NUMBER_OF_MOTORS; i++){	
					delays.val(i,0) = scale_position(i,sensor_delay.val(sensor_delay_count,i)*encoder_to_rad);
				}
				sensor_delay_count -= 1;
				printMatrix(delays);
				soctrl->setDelayedSensor(delays);
			}
			motorRefs = soctrl->update(positions,displacements);
			std::lock_guard<std::mutex> lock(myoMaster->mux);
			for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
				myoMaster->changeSetPoint(i, getMuscleLengthScaledInv(i, motorRefs.val(i,0))/encoder_to_rad);
			}
			pubDepMatrix();
		}
	} else {

	}
}

/*
void DEP::update(){
	t_start = std::chrono::high_resolution_clock::now();
	ROS_INFO("start");
	motor_status_lock = 1;
	while (motor_status_lock == 1){usleep(500);}
	t_end = std::chrono::high_resolution_clock::now();
	auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	ROS_INFO("%i", diff);
	ROS_INFO("update");
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		// convert encoder ticks to radians, and then scale them
		positions.val(i,0) = scale_position(i, positions.val(i,0)*encoder_to_rad);
		displacements.val(i,0) = scale_displacement(i, displacements.val(i,0));
	}
	//ROS_INF0("pos and disp: ");
	//printMatrix(positions);
	//printMatrix(displacements);
	motorRefs = soctrl->update(positions,displacements);
	std::lock_guard<std::mutex> lock(myoMaster->mux);
	//matrix::Matrix m = matrix::Matrix(NUMBER_OF_MOTORS,1);
	//matrix::Matrix m1 = matrix::Matrix(NUMBER_OF_MOTORS,1);
	t_end = std::chrono::high_resolution_clock::now();
	diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	ROS_INFO("%i", diff);
	ROS_INFO("set muscles");
	for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
		myoMaster->changeSetPoint(i, getMuscleLengthScaledInv(i, motorRefs.val(i,0))/encoder_to_rad);
		//myoMaster->changeSetPoint(i, getMuscleLengthScaledInv(i, motorRefs.val(i,0)));
		//myoMaster->changeSetPoint(i, 3000);
		//if (i == 2){
		//	myoMaster->changeSetPoint(1,(1/encoder_to_rad)*1);
		//}
		//m.val(i,0) = motorRefs.val(i,0);
		//m.val(i,0) = getMuscleLengthScaledInv(i, motorRefs.val(i,0))*(1/encoder_to_rad);
	}
	//ROS_INF0_THROTTLE(1, "motorRefs: ");
	//printMatrix(m1);
	//printMatrix(motorRefs);
	pubDepMatrix();
	t_end = std::chrono::high_resolution_clock::now();
	diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
	ROS_INFO("%i", diff);
	while (diff < 20){ 
		usleep(500);
		t_end = std::chrono::high_resolution_clock::now();
		diff = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
		ROS_INFO("%i", diff);
	}
	motor_status_lock = 1;
}*/

void DEP::pubDepMatrix(){
	matrix::Matrix C = soctrl->getC();
	int motors = C.getM();
	int sensors = C.getN();

	roboy_dep::depMatrix msg;
	msg.size = motors;
	for (int i = 0; i < motors; i++) {
		roboy_dep::cArray msg_temp;
		msg_temp.size = sensors;
		for (int j = 0; j < sensors; j++) {
			msg_temp.cArray.push_back(C.val(i,j));
		}
		msg.depMatrix.push_back(msg_temp);
	}
	DepMatrix.publish(msg);
	//DEBUG
	//ROS_INFO("Published matrix");
	//DEBUG
	//ROS_INFO("msg sizes -- motors: %i, sensors: %i", msg.size, msg.depMatrix[0].size);
	//printMatrix(C);
}

void DEP::printMatrix(const matrix::Matrix& m){
	for (int i = 0; i < m.getN(); i++) {
		std::string s;
		for (int j = 0; j < m.getM(); j++) {
			s.append(boost::lexical_cast<std::string>(m.val(i,j))+",");
		}
		ROS_INFO("%s", s.c_str());
	}
}

void DEP::DepCommand(const roboy_dep::command::ConstPtr &msg){
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


void DEP::DepParameters(const roboy_dep::depParameters::ConstPtr &msg){
	ROS_INFO("Set DEP parameters");
	soctrl->timedist = msg->timedist;
	soctrl->urate = msg->urate;
	soctrl->initFeedbackStrength = msg->initFeedbackStrength;
	soctrl->regularization = msg->regularization;
	soctrl->synboost = msg->synboost;
	soctrl->maxSpeed = msg->maxSpeed;
	soctrl->epsh = msg->epsh;
	soctrl->pretension = msg->pretension;
	soctrl->maxForce = msg->maxForce;
	soctrl->springMult1 = msg->springMult1;
	soctrl->springMult2 = msg->springMult2;
	soctrl->diff = msg->diff;
	soctrl->amplitude = msg->amplitude;
	soctrl->delay = msg->delay;
	soctrl->guideType = msg->guideType;
	soctrl->guideIdx = msg->guideIdx;
	soctrl->guideAmpl = msg->guideAmpl;
	soctrl->guideFreq = msg->guideFreq;
	soctrl->learning = msg->learning;
	target_force = msg->targetForce;
	range = msg->range;
}

double DEP::scale_position(int motor_index, double value){
	double min = params[motor_index][1];
	double max = params[motor_index][2];
	return (value-min)/(max-min)*2.0-1.0; // -1/1 -> min/max
}

double DEP::scale_displacement(int motor_index, double value){
	return 0.2375*value;
	//return value*0.237536 + value*value*0.000032;
	//return polyPar[motor_index][0] + value*polyPar[motor_index][1] + value*value*polyPar[motor_index][2] + value*value*value*polyPar[motor_index][3];
	//soft spring
	//remember relative encoders -> initialization procedure
}

double DEP::getMuscleLengthScaledInv(int motor_index,double value){
	double min = params[motor_index][1];
	double max = params[motor_index][2];
	return (value+1.0)/2.0*(max-min)+min; // -1/1 -> min/max
}