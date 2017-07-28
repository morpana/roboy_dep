#ifndef _DEP
#define _DEP

#include <roboy_managing_node/myoMaster.hpp>
#include <ros/ros.h>
#include <roboy_DEP/matrix.h>
#include <roboy_DEP/DerMartiusController.h>
#include <roboy_DEP/command.h>

#define NUMBER_OF_MOTORS  9

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
		void init();
		void setMotorConfig();
		void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
		void DepCommand(const roboy_DEP::command::ConstPtr &msg);

		double scale_position(int motor_index, double value);
		double scale_displacement(int motor_index, double value);
		double getMuscleLengthScaledInv(int motor_index,double value);

		ros::NodeHandlePtr nh;
		ros::Subscriber motorStatus, depCommand;
		ros::Publisher motorConfig;
		boost::shared_ptr<ros::AsyncSpinner> spinner;

		DerMartiusController* soctrl;
		matrix::Matrix positions, displacements, motorRefs;

		double params[NUMBER_OF_MOTORS][3];
		double polyPar[NUMBER_OF_MOTORS][4];
		int control_mode, outputPosMax, outputNegMax, spPosMax, spNegMax, IntegralPosMax, IntegralNegMax, Kp, Ki, Kd, forwardGain, deadBand;
		int target_force, range;
};

#endif // _DEP