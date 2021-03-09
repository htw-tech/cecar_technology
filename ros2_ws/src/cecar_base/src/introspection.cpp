/*Introspection node for CeCar: Display relevant information
Author(s)		: Lukas Mirow
Date of creation	: 2/24/2021
*/

#include "shared.hpp"
#include <chrono>
#include <memory>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <nav_msgs/msg/odometry.hpp>

using namespace std;

class Cecar_introspection : public rclcpp::Node
{

	public:

		Cecar_introspection();

	private:

		void status_callback(const Cecar_status::SharedPtr status);
		rclcpp::Subscription<Cecar_status>::SharedPtr status_subscription;

};

Cecar_introspection::Cecar_introspection() : Node("cecar_introspection")
{
	status_subscription = create_subscription<Wheel_encoder_cecar>("cecar_status", MSG_BUFFER_SIZE, bind(&Cecar_introspection::wheelenc_callback, this, placeholders::_1));
}

void Cecar_introspection::status_callback(const Cecar_status::SharedPtr status)
{
	cout << "Remote control forward: " << status.rc_adrive.speed << endl;
	cout << "Remote control steering: " << status.rc_adrive.angle << endl;
	cout << "Filtered drive forward: " << status.adrive.speed << endl;
	cout << "Filtered drive steering: " << status.adrive.angle << endl;
	cout << "Wheel encoder left" << status.wheel_encoder.left_rear << endl;
	cout << "Wheel encoder right" << status.wheel_encoder.right_rear << endl;
	cout << "----------------------------------------------------------" << endl;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_introspection>());
	rclcpp::shutdown();
	return 0;
}
