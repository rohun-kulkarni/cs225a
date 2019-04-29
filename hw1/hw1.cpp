#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4
#define QUESTION_5   5
#define QUESTION_6   6
#define DEG2RAD M_PI/180.0
// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;



int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	ofstream c1file("controller1.txt");
	ofstream c2file("controller2.txt");
	ofstream c3file("controller3.txt");
	ofstream c4file("controller4.txt");
	ofstream c5file("controller5.txt");
	ofstream c6file("controller6.txt");



	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();


		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_5;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 50;      // chose your d gain
			// TODO : values goes to nan at kv > 40

			// Desired joint angle ((90, −45, 0, −125, 0, 80, 0);)
			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question
			q_desired << 90, -45, 0, -125, 0, 80, 0;
			q_desired = q_desired * DEG2RAD;

	
			//std::cout << "robot_q size\n";
			//std::cout << robot->_q << endl;
			//std::cout << robot->_q.size();
			//std::cout << "q_des size\n";
			//std::cout << q_desired.size() << endl;
			c1file << kv;
			c1file << ", ";
			c1file << robot->_q(0);
			c1file << ", ";
			c1file << robot->_q(2);
			c1file << ", ";
			c1file << robot->_q(3) << endl;
			
			command_torques = -(kp) * (robot->_q - q_desired) - (kv)*robot->_dq; // change to the control torques you compute
			//std::cout << "Commanded Torques" << endl;
			//std::cout << command_torques << endl;

		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 52.0;      // chose your d gain
	
			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question
			q_desired << 90, -45, 0, -125, 0, 80, 0;
			q_desired = q_desired * DEG2RAD;
			VectorXd gravity(dof); // Empty Gravity Vector

			robot->gravityVector(gravity);

			command_torques = -kp * (robot->_q - q_desired) - kv * robot->_dq + gravity;

			c2file << kv;
			c2file << ", ";
			c2file << robot->_q(0);
			c2file << ", ";
			c2file << robot->_q(2);
			c2file << ", ";
			c2file << robot->_q(3) << endl;



		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 52.0;      // chose your d gain
	
			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question
			q_desired << 90, -45, 0, -125, 0, 80, 0;
			q_desired = q_desired * DEG2RAD;
			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);
	

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) +  gravity;
			
			c3file << kv;
			c3file << ", ";
			c3file << robot->_q(0);
			c3file << ", ";
			c3file << robot->_q(2);
			c3file << ", ";
			c3file << robot->_q(3) << endl;


		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 52.0;      // chose your d gain

			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question
			q_desired << 90, -45, 0, -125, 0, 80, 0;
			q_desired = q_desired * DEG2RAD;

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);
			VectorXd coriolis(dof);
			robot->coriolisForce(coriolis);

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + coriolis + gravity;
			
			c4file << kv;
			c4file << ", ";
			c4file << robot->_q(0);
			c4file << ", ";
			c4file << robot->_q(2);
			c4file << ", ";
			c4file << robot->_q(3) << endl;

		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 52.0;      // chose your d gain

			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question
			q_desired << 90, -45, 0, -125, 0, 80, 0;
			q_desired = q_desired * DEG2RAD;

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);
			VectorXd coriolis(dof);
			robot->coriolisForce(coriolis);

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + coriolis + gravity;
			
			c5file << kv;
			c5file << ", ";
			c5file << robot->_q(0);
			c5file << ", ";
			c5file << robot->_q(2);
			c5file << ", ";
			c5file << robot->_q(3) << endl;
		}
		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_6)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 52.0;      // chose your d gain

			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question
			q_desired << 90, -45, 0, -125, 0, 80, 0;
			q_desired = q_desired * DEG2RAD;

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);
			VectorXd coriolis(dof);
			robot->coriolisForce(coriolis);

			command_torques = robot->_M * (-kp * (robot->_q - q_desired) - kv * robot->_dq) + coriolis + gravity;
			
			c6file << kv;
			c6file << ", ";
			c6file << robot->_q(0);
			c6file << ", ";
			c6file << robot->_q(2);
			c6file << ", ";
			c6file << robot->_q(3) << endl;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	c1file.close();
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
