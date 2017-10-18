#ifndef _DEP
#define _DEP

#include <roboy_managing_node/myoMaster.hpp>
#include <ros/ros.h>
#include <roboy_dep/matrix.h>
#include <roboy_dep/DerMartiusController.h>
#include <roboy_dep/command.h>
#include <roboy_dep/depParameters.h>
#include <roboy_dep/depMatrix.h>
#include "roboy_dep/transition_start.h"

#include <chrono>

#define NUMBER_OF_MOTORS  14
//#define ENCODER_TO_RAD 2.0*3.14159/(2000.0*53.0)

using namespace std;

class DEP {
	public:
		DEP();
		~DEP();
		MyoMaster* myoMaster;
		void force();
		void initialize();
		void update();
		int mode;
	private:
		void init_params();
		void setMotorConfig();
		void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
		void DepCommand(const roboy_dep::command::ConstPtr &msg);
		void DepParameters(const roboy_dep::depParameters::ConstPtr &msg);
		void DepLoadMatrix(const roboy_dep::depMatrix::ConstPtr &msg);
		void pubDepMatrix();

		void printMatrix(const matrix::Matrix& array);

		double scale_position(int motor_index, double value);
		double scale_displacement(int motor_index, double value);
		double getMuscleLengthScaledInv(int motor_index,double value);

		ros::NodeHandlePtr nh;
		ros::Subscriber motorStatus, depCommand, depParameters, depLoadMatrix;
		ros::Publisher motorConfig, DepMatrix, transition_start_pub;
		boost::shared_ptr<ros::AsyncSpinner> spinner;

		DerMartiusController* soctrl;
		matrix::Matrix positions, displacements, motorRefs, C;

		double encoder_to_rad;
		double params[NUMBER_OF_MOTORS][3];
		double polyPar[NUMBER_OF_MOTORS][4];
		int control_mode, outputPosMax, outputNegMax, spPosMax, spNegMax, IntegralPosMax, IntegralNegMax, Kp, Ki, Kd, forwardGain, deadBand;
		int target_force, range;

		bool motor_status_lock;
		std::chrono::high_resolution_clock::time_point t_start, t_end;
};

#endif // _DEP

//sudo kill -9 $(ps -al|grep '\(roboy_dep\|dep_interface\|dep_gui\)'|awk '{print $4;}'|tr '\n' ' ')
//sudo killall -9 rosmaster (if roscore not closing)
//fpga spi lights
// sounds dont qworry