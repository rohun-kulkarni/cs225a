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
#define DEG2RAD  (M_PI / 180.0)

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

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
		int controller_number = QUESTION_1;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			VectorXd q_desired(dof);  // change to the desired robot joint angles for the question

			q_desired << 90, -45, 0, -125, 0, 80, 0.0;
			q_desired = q_desired * DEG2RAD;

			double Kp16 = 400;			 // N m rad^-1
			double Kv16 = 50;			 // N m s rad^-1

			double Kp7 = 50;		     // N m rad^-1
			double Kv7 = -0.275;		     // N m s rad^-1 set for no damping
			double q7d = 0.1;			 // rad
			double t = timer.elapsedTime();
			q_desired(7) = q7d;

			// Kp and Kv Matrix 
			MatrixXd Kp = MatrixXd::Zero(dof, dof);
			MatrixXd Kv = MatrixXd::Zero(dof, dof);
			// Fill in the diagonals of the matrix. 
			
			for (int i = 0; i <= 5; i++)
			{
				Kp(i,i) = Kp16;
				Kv(i,i) = Kv16;
			}

			Kp(6,6) = Kp7;
			Kv(6,6) = Kv7;
			//cout << Kv << endl;

			
			Eigen::Vector3d ee_position = Eigen::Vector3d::Zero(); // 3d vector of zeros to fill with the end effector position
			robot->position(ee_position, link_name, pos_in_link);

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);
			VectorXd coriolis(dof);
			robot->coriolisForce(coriolis);





			//Change the Mass Matrix to remove dynamic decoupling. 
			//robot->_M(7,7) = 0.25; 		// N m rad^-1


			command_torques = -Kv*(robot->_dq) - Kp*(robot->_q - q_desired) + coriolis + gravity;

			c1file << t;
			c1file << ", ";
			c1file << ee_position(0);
			c1file << ", ";
			c1file << ee_position(1);
			c1file << ", ";
			c1file << ee_position(2);
			c1file << ", ";
			c1file << robot->_q(6) << endl;

		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 200; 
			double kv = 32.0;
			VectorXd xdes(3);
			xdes << 0.3, 0.1, 0.5;
			
			std::string ee_link_name = "link7";
			
			Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.1); 
			Eigen::Vector3d ee_position = Eigen::Vector3d::Zero(); // 3d vector of zeros to fill with the end effector position
			robot->position(ee_position, ee_link_name, ee_pos_in_link);

			Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
			robot->linearVelocity(ee_velocity, ee_link_name, ee_pos_in_link);
			
			Eigen::MatrixXd ee_jacobian(3,dof); // Empty Jacobian Matrix sized to right size
			robot->Jv(ee_jacobian,ee_link_name,ee_pos_in_link);

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);

			VectorXd coriolis(dof); // Empty coriolis vector
			robot->coriolisForce(coriolis);
			/*
			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			*/


			VectorXd command_force(3);
			command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity);

			double Kvj = 40;

			command_torques = ee_jacobian.transpose()*command_force + gravity - (N.transpose()*robot->_M*kv*robot->_dq);

			c2file << kv;
			c2file << ",";
			c2file << ee_position(0);
			c2file << ", ";
			c2file << ee_position(1);
			c2file << ", ";
			c2file << ee_position(2);
			c2file << ", ";
			for (int i = 0; i <=6; i++)
			{
				c2file << robot->_q(i);
				c2file << ", ";
			}
			c2file << endl;




		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 200; 
			double kv = 32.0;
			VectorXd xdes(3);
			xdes << 0.3, 0.1, 0.5;
			

			Eigen::Vector3d ee_position = Eigen::Vector3d::Zero(); // 3d vector of zeros to fill with the end effector position
			robot->position(ee_position, link_name, pos_in_link);

			Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
			robot->linearVelocity(ee_velocity, link_name, pos_in_link);
			
			robot->Jv(Jv, link_name, pos_in_link);

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);

			VectorXd coriolis(dof); // Empty coriolis vector
			robot->coriolisForce(coriolis);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			
			VectorXd p(dof,1); 
			// Dimensions 3x1 = (7x3)' x 7x1
			//command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity);

			p  = J_bar.transpose() * gravity;

			
			VectorXd command_force(3);
			// Dimensions 3x1 = 3x3 * (3x1) + 3x1 

			command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity) + p;
			

			// 7x1 = (3x7)' *3x1 - 7x7 * 7x7 * 7x1
			command_torques = Jv.transpose() * command_force - N.transpose() * robot->_M * kv * robot->_dq;

			c3file << kv;
			c3file << ",";
			c3file << ee_position(0);
			c3file << ", ";
			c3file << ee_position(1);
			c3file << ", ";
			c3file << ee_position(2);
			c3file << ", ";
			for (int i = 0; i <=6; i++)
			{
				c3file << robot->_q(i);
				c3file << ", ";
			}
			c3file << endl;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			double kp = 400; 
			double kv = 30.0;
			double kpq = 50;

			double t = timer.elapsedTime();

			VectorXd xdes(3);
			Vector3d trajConst;
			Vector3d trajSinusoid;
			trajConst << 0.3, 0.1, 0.5;
			trajSinusoid << 0.1*sin(M_PI*t),  0.1*cos(M_PI*t), 0;
			
			xdes  = trajConst + trajSinusoid;

			Eigen::Vector3d ee_position = Eigen::Vector3d::Zero(); // 3d vector of zeros to fill with the end effector position
			robot->position(ee_position, link_name, pos_in_link);

			Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
			
			robot->linearVelocity(ee_velocity, link_name, pos_in_link);
			robot->Jv(Jv, link_name, pos_in_link);

			VectorXd gravity(dof); // Empty Gravity Vector
			robot->gravityVector(gravity);

			VectorXd coriolis(dof); // Empty coriolis vector
			robot->coriolisForce(coriolis);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);
			
			VectorXd p(dof,1); 
			// part i
			// Dimensions 3x1 = (7x3)' x 7x1
			p  = J_bar.transpose() * gravity;
			
			VectorXd command_force(3);
			// Dimensions 3x1 = 3x3 * (3x1) + 3x1 
			//command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity) + p;
			
			// 7x1 = (3x7)' *3x1 - 7x7 * 7x7 * 7x1
			//command_torques = Jv.transpose() * command_force - N.transpose() * robot->_M * kv * robot->_dq;
			
			// part ii
			//Lambda = MatrixXd::Identity(3, 3);
			//command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity) + p;
			//command_torques = Jv.transpose() * command_force - N.transpose() * robot->_M * kv * robot->_dq;

			// part iii
			//command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity) + p;
			//command_torques = Jv.transpose() * command_force -  robot->_M*kpq*(robot->_q);

			// part iv
			command_force = Lambda * (kp*(xdes - ee_position) - kv*ee_velocity) + p;
			command_torques = Jv.transpose() * command_force -  robot->_M*kpq*(robot->_q) + gravity;


			// Log to file
			c4file << t;
			c4file << ",";
			c4file << ee_position(0);
			c4file << ", ";
			c4file << ee_position(1);
			c4file << ", ";
			c4file << ee_position(2);
			c4file << ", ";
			for (int i = 0; i <=6; i++)
			{
				c4file << robot->_q(i);
				c4file << ", ";
			}
			c4file << endl;
		
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

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
