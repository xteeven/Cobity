/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <BaseCyclicClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

///* TechTest
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>  
//*/
namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000

bool printer = true;
// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Main Variables

k_api::TransportClientTcp* transport;
k_api::RouterClient* router;
k_api::SessionManager* session_manager;

k_api::Base::BaseClient* base;
k_api::BaseCyclic::BaseCyclicClient* base_cyclic;
k_api::BaseCyclic::Feedback feedback;

//Gripper 

//k_api::Base::GripperCommand gripper_command;
//k_api::Base::Finger* finger;

//k_api::Base::Gripper gripper_feedback;
//k_api::Base::GripperRequest gripper_request;



float t_x_pos = 0.457665;
float t_y_pos = 0.0f;
float t_z_pos = 0.421093;
float t_pitch = 90;
float t_yaw = 90;
float t_roll = 0.0f;


float l_e_x = 0.0f;
float l_e_y = 0.0f;
float l_e_z = 0.0f;
float l_e_pitch = 0.0f;
float l_e_yaw = 0.0f;
float l_e_roll = 0.0f;

//float t_finger = 0.0f;
float finger_speed = 1.0f;
float proportional = 0.7;
float derivative = 1.0f;
float proportionalRotation = 0.7;
float derivativeRotation = 1.0f;


//std::thread* print;
std::thread* speedCicleThread;

// Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
	return [&finish_promise](k_api::Base::ActionNotification notification)
	{
		const auto action_event = notification.action_event();
		switch (action_event)
		{
		case k_api::Base::ActionEvent::ACTION_END:
		case k_api::Base::ActionEvent::ACTION_ABORT:
			finish_promise.set_value(action_event);
			break;
		default:
			break;
		}
	};
}

extern "C" __declspec(dllexport) bool example_move_to_home_position()
{
	// Make sure the arm is in Single Level Servoing before executing an Action
	auto servoingMode = k_api::Base::ServoingModeInformation();
	servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
	base->SetServoingMode(servoingMode);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Move arm to ready position
	std::cout << "Moving the arm to a safe position" << std::endl;
	auto action_type = k_api::Base::RequestedActionType();
	action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
	auto action_list = base->ReadAllActions(action_type);
	auto action_handle = k_api::Base::ActionHandle();
	action_handle.set_identifier(0);
	for (auto action : action_list.action_list())
	{
		if (action.name() == "Home")
		{
			action_handle = action.handle();
		}
	}

	if (action_handle.identifier() == 0)
	{
		std::cout << "Can't reach safe position, exiting" << std::endl;
		return false;
	}
	else
	{
		// Connect to notification action topic
		std::promise<k_api::Base::ActionEvent> promise;
		auto future = promise.get_future();
		auto notification_handle = base->OnNotificationActionTopic(
			create_action_event_listener_by_promise(promise),
			k_api::Common::NotificationOptions{}
		);

		base->ExecuteActionFromReference(action_handle);

		// Wait for action to finish
		const auto status = future.wait_for(TIMEOUT_DURATION);
		base->Unsubscribe(notification_handle);

		if (status != std::future_status::ready)
		{
			std::cout << "Timeout on action notification wait" << std::endl;
			return false;
		}

		return true;
	}
}


float limit(float value, float min, float max) {
	if (value < min)
		return min;

	if (value > max)
		return max;

	return value;
}


extern "C" __declspec(dllexport) void setPose(float x = t_x_pos, float y = t_y_pos, float z = t_z_pos) {
	t_x_pos = limit(x, -1, 1);
	t_y_pos = limit(y, -1, 1);
	t_z_pos = limit(z, -1, 1);
}


extern "C" __declspec(dllexport) void setAngle(float pitch = t_pitch, float yaw = t_yaw, float roll = t_roll) {
	t_pitch = pitch;
	t_yaw = yaw;
	t_roll = roll;
}

extern "C" __declspec(dllexport) void setProportional(float prop = proportional) {
	proportional = prop;
}

extern "C" __declspec(dllexport) void setProportionalRotation(float prop = proportional) {
	proportionalRotation = prop;
}

extern "C" __declspec(dllexport) void setDerivative(float der = derivative) {
	derivative = der;
}

extern "C" __declspec(dllexport) void setDerivativeRotation(float der = derivative) {
	derivativeRotation = der;
}

/*
extern "C" __declspec(dllexport) void gripperStart() {

		gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
		gripper_command.set_duration(0.15);
		gripper_request.set_mode(k_api::Base::GRIPPER_POSITION);
		finger = gripper_command.mutable_gripper()->add_finger();
		
		finger->set_finger_identifier(1);
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	
	

}
*/
/*
extern "C" __declspec(dllexport) void setGriperValue(float val, float speed = 1.0f) {
	
		finger_speed = limit(speed, 0.0f, 1.0f);
		t_finger = limit(val, 0.0f, 1.0f);


}

*/

float getError(float target, float measurement) {
	return (target - measurement);
}

void SpeedCicle() {
	auto command = k_api::Base::TwistCommand();
	command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_MIXED);
	command.set_duration(30);  
	

	while (printer)
	{
		try {
			feedback = base_cyclic->RefreshFeedback();
		}
		catch (const std::exception& e) {
			const std::chrono::time_point<std::chrono::system_clock> now =
				std::chrono::system_clock::now();

			const std::time_t t_c = std::chrono::system_clock::to_time_t(now);
			std::cout << std::ctime(&t_c);

			std::cout << "Feedback Error" << std::endl;
		}



		float e_x = getError(t_x_pos, feedback.base().tool_pose_x());
		float e_y = getError(t_y_pos, feedback.base().tool_pose_y());
		float e_z = getError(t_z_pos, feedback.base().tool_pose_z());
		float e_pitch = getError(t_pitch, feedback.base().tool_pose_theta_x());
		float e_yaw = getError(t_yaw, feedback.base().tool_pose_theta_z());
		float e_roll = getError(t_roll, -feedback.base().tool_pose_theta_y());




		float x_pos = (e_x) * proportional + (e_x - l_e_x) * derivative;
		float y_pos = (e_y) * proportional + (e_y - l_e_y) * derivative;
		float z_pos = (e_z) * proportional + (e_z - l_e_z) * derivative;
		float pitch = (e_pitch) * proportionalRotation + (e_pitch - l_e_pitch) * derivative;
		float yaw = (e_yaw) * proportionalRotation + (e_yaw - l_e_yaw) * derivative;
		float roll = (e_roll) + (e_roll - l_e_roll) * derivative;
		


		l_e_x = e_x;
		l_e_y = e_y;
		l_e_z = e_z;
		l_e_pitch = e_pitch;
		l_e_yaw = e_yaw;
		l_e_roll = e_roll;


		/*
		if (gripper_command.has_gripper()) {
			
			gripper_feedback = base->GetMeasuredGripperMovement(gripper_request);
			float finger_pos = (gripper_feedback.finger(0).value() - t_finger) * finger_speed;
	
		
			if (abs(finger_pos) > finger_speed * 0.20) {
				finger->set_value(finger_speed - 2 * signbit(finger_pos) * finger_speed);
			}
			else
			{
				finger->set_value(finger_pos * 0.1);
			}

			base->SendGripperCommand(gripper_command);


			
				//finger->set_value(finger_speed - 2*signbit(finger_pos)* finger_speed);
			finger->set_value(finger_pos);
			base->SendGripperCommand(gripper_command);
		//
		}
	*/	
		k_api::Base::Twist* twist = command.mutable_twist();
		twist->set_linear_x(x_pos);
		twist->set_linear_y(y_pos);
		twist->set_linear_z(z_pos);
		twist->set_angular_x(pitch); //Pitch
		twist->set_angular_y(yaw); // Yaw
		twist->set_angular_z(roll); // Roll

		try {
			base->SendTwistCommand(command);
			
		}
		catch(const std::exception& e){
			

			const std::chrono::time_point<std::chrono::system_clock> now =
				std::chrono::system_clock::now();

			const std::time_t t_c = std::chrono::system_clock::to_time_t(now);

			std::cout << std::ctime(&t_c);

			std::cout << " Communication Error " << std::endl;
		}
		


		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}

	std::cout << "Stopping robot ..." << std::endl;
}

extern "C" __declspec(dllexport) float getActuatorFeedback(int actuator) {
		return feedback.mutable_actuators(actuator)->position();
}
	
extern "C" __declspec(dllexport) float getxpos() {
	return feedback.base().tool_pose_x();
}

extern "C" __declspec(dllexport) float getypos() {
	return feedback.base().tool_pose_y();
}

extern "C" __declspec(dllexport) float getzpos() {
	return feedback.base().tool_pose_z();
}

void stopThreads() {
	printer = false;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//print->join();
	speedCicleThread->join();
}

extern "C" __declspec(dllexport) bool wakeUpRobot()
{
	// Let time for twist to be executed
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	setPose(0.4, 0, 0.4);
	setAngle(80, 120);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	setPose(0.4, 0.2, 0.4);
	setAngle(80, 75);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	setPose(0.4, -0.2, 0.4);
	setAngle(80, 120);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	setPose(0.4, 0, 0.4);
	setAngle(80, 75);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	setPose(0.4, 0, 0.4);
	setAngle(80, 120);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	setPose(0.4, 0, 0.4);
	setAngle(80, 75);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));

	return true;
}

extern "C" __declspec(dllexport) void sessionStart(const char *ipaddress = IP_ADDRESS, std::uint32_t port = PORT) {
//extern "C" __declspec(dllexport) void sessionStart() {
	auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };
	transport = new k_api::TransportClientTcp();
	router = new k_api::RouterClient(transport, error_callback);
	transport->connect(ipaddress, port);
	// Set session data connection information
	auto create_session_info = k_api::Session::CreateSessionInfo();
	create_session_info.set_username("admin");
	create_session_info.set_password("admin");
	create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
	create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

	// Session manager service wrapper
	std::cout << "Creating session for communication" << std::endl;
	session_manager = new k_api::SessionManager(router);
	session_manager->CreateSession(create_session_info);
	std::cout << "Session created" << std::endl;
}

extern "C" __declspec(dllexport) void createServices() {
	base = new k_api::Base::BaseClient(router);
	base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

	bool success = true;
	t_x_pos = 0.457665;
	t_y_pos = 0;
	t_z_pos = 0.421093;
	t_pitch = 90;
	t_yaw = 90;
	t_roll = 0;
	//t_finger = 0.0f;
	//finger_speed = 1.0f;
	proportional = 1.0;
	proportionalRotation = 1;
	
	/*
	if(gripper_command.mutable_gripper()->finger_size()>0){
		gripper_command.mutable_gripper()->clear_finger();
		finger->Clear();
	}*/
	success &= example_move_to_home_position();
	feedback = base_cyclic->RefreshFeedback();
	printer = true;
	//print = new std::thread(printFeedback);
	speedCicleThread = new std::thread(SpeedCicle);

}

extern "C" __declspec(dllexport) void closeSession() {
	stopThreads();

	base->Stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// Close API session
	session_manager->CloseSession();

	// Deactivate the router and cleanly disconnect from the transport object
	router->SetActivationStatus(false);
	transport->disconnect();

	// Destroy the API
	delete base;
	delete session_manager;
	delete router;

	//Session
	delete transport;
}

void transtest(){

	//X [0.4 - 0.8] 0 0.3

	//Y 0.5 [0 - 0.4]  0.3

	//Z 0.5 0 [0.2 - 0.6]

	float proportionaltest = 1.0f;


	//X TEST
	for (int i=0; i<1; i++)
	{
	
	setProportional(1);
	setDerivative(0);
	setPose(0.5, 0.0, 0.2);

	std::ofstream myfile;
	std::string filename = "C:/Users/hcump/Dropbox/Steeven_PhD/UIST/TechicTestz" + std::to_string(proportionaltest * 10) + ".csv";
	myfile.open(filename);
	myfile << "Time, Axis, Target, P, Measured\n";
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	typedef std::chrono::milliseconds ms;
	auto begintest = std::chrono::steady_clock::now();

	setPose(0.5, 0.0, 0.6);
	for (int t = 0; t < 500; t++)
	{
		auto elapsed = std::chrono::steady_clock::now() - begintest;
		ms d = std::chrono::duration_cast<ms>(elapsed);
		myfile << d.count() << ", z," << 0.6 << "," << proportionaltest << "," << getxpos() << "\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	setPose(0.5, 0.0, 0.2);
	for (int t = 0; t < 500; t++)
	{
		auto elapsed = std::chrono::steady_clock::now() - begintest;
		ms d = std::chrono::duration_cast<ms>(elapsed);
		myfile << d.count() << ", z," << 0.2 << "," << proportionaltest << "," << getxpos() << "\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}


	setPose(0.5, 0.0, 0.6);
	for (int t = 0; t < 500; t++)
	{
		auto elapsed = std::chrono::steady_clock::now() - begintest;
		ms d = std::chrono::duration_cast<ms>(elapsed);
		myfile << d.count() << ", z," << 0.6 << "," << proportionaltest << "," << getxpos() << "\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	setPose(0.5, 0.0, 0.2);
	for (int t = 0; t < 500; t++)
	{
		auto elapsed = std::chrono::steady_clock::now() - begintest;
		ms d = std::chrono::duration_cast<ms>(elapsed);
		myfile << d.count() << ", z," << 0.2 << "," << proportionaltest << "," << getxpos() << "\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}


	myfile.close();
	proportionaltest += 1.0f;
	}
}


void rottest() {

	//X [0.4 - 0.8] 0 0.3
	//Y 0.5 [0 - 0.4]  0.3

	//Z 0.5 0 [0.2 - 0.6]

	int proportionaltest = 1;
	setPose(0.6, 0, 0.4);
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	//X TEST
	for (int i = 0; i < 1; i++)
	{

		setProportionalRotation(1);
		setDerivativeRotation(0.0);

		setAngle(90, 90, 0);

		std::ofstream myfile;
		std::string filename = "C:/Users/hcump/Dropbox/Steeven_PhD/UIST/TechicTestroll" + std::to_string(proportionaltest * 10) + ".csv";
		myfile.open(filename);
		myfile << "Time, Axis, Target, P, Measured\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		typedef std::chrono::milliseconds ms;
		auto begintest = std::chrono::steady_clock::now();

		setAngle(90, 90, 0);
		for (int t = 0; t < 500; t++)
		{
			auto elapsed = std::chrono::steady_clock::now() - begintest;
			ms d = std::chrono::duration_cast<ms>(elapsed);
			myfile << d.count() << ", pitch," << 135 << "," << proportionaltest << "," << feedback.base().tool_pose_theta_z() << "\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}

		setAngle(90, 90, 0);
		for (int t = 0; t < 500; t++)
		{
			auto elapsed = std::chrono::steady_clock::now() - begintest;
			ms d = std::chrono::duration_cast<ms>(elapsed);
			myfile << d.count() << ", pitch," << 45 << "," << proportionaltest << "," << feedback.base().tool_pose_theta_z() << "\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}


		setAngle(90, 90, 0);
		for (int t = 0; t < 500; t++)
		{
			auto elapsed = std::chrono::steady_clock::now() - begintest;
			ms d = std::chrono::duration_cast<ms>(elapsed);
			myfile << d.count() << ", pitch," << 135 << "," << proportionaltest << "," << feedback.base().tool_pose_theta_z() << "\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}

		setAngle(90, 90, 0);
		for (int t = 0; t < 500; t++)
		{
			auto elapsed = std::chrono::steady_clock::now() - begintest;
			ms d = std::chrono::duration_cast<ms>(elapsed);
			myfile << d.count() << ", pitch," << 45 << "," << proportionaltest << "," << feedback.base().tool_pose_theta_z() << "\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}


		myfile.close();
		proportionaltest += 1;
	}
}


/*
int main(int argc, char** argv)
{
	// Create API objects
	sessionStart();

	// Create services
	createServices();

	//gripperStart();

	
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	//while (true) {
	
	//	transtest();
		rottest();
	
	
	
	//}

	// Example core
	//rottest();
	//example_move_to_home_position();
	/*
	for(float i=0;i<1;i+=0.1){
		std::cout << "Setting position to " << i << std::endl;
	setAngle(90, 135+i, 0);
	//setGriperValue(i,1.0f);
	std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}

	for (float i = 0; i < 1; i += 0.1) {
		std::cout << "Setting position to " << 1- i << std::endl;
		setAngle(90, 135 + i, 0);
		//setGriperValue(1- i, 1.0f);
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	*/
/*



	closeSession();

	return 0;
}

//*/