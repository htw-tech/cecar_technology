/*Placeholder of a self-control node for the CeCar
Author(s)		: Lukas Mirow
Date of creation	: 2/24/2020
*/

#include "shared.hpp"
#include <chrono>
#include <memory>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdio.h>

using namespace std;

class Cecar_self_control : public rclcpp::Node
{

	public:

		Cecar_self_control();

	private:

		rclcpp::Publisher<Ackermann_drive>::SharedPtr publisher;
		rclcpp::Subscription<Ackermann_drive>::SharedPtr subscription;
		void publish() const;
		void callback(const Ackermann_drive::SharedPtr adrive) const; //FIXME: Adjust message type
		rclcpp::TimerBase::SharedPtr timer;

};

Cecar_self_control::Cecar_self_control() : Node("cecar_self_control")
{
	publisher = create_publisher<Ackermann_drive>("cecar_self_control_output", MSG_BUFFER_SIZE);
	function<void(const Ackermann_drive::SharedPtr)> cb = bind(&Cecar_self_control::callback, this, placeholders::_1);
	subscription = create_subscription<Ackermann_drive>("cecar_self_control_input", MSG_BUFFER_SIZE, cb);
	timer = create_wall_timer(DRIVE_INPUT_MSG_DELAY, bind(&Cecar_self_control::publish, this));
}

void Cecar_self_control::callback(const Ackermann_drive::SharedPtr adrive) const
{
	//TODO: Add your self-control input code here!
	adrive->speed = 0; //This is only here so that the compiler doesn't emit a warning saying that this variable is not being used. You may remove it.
}

void Cecar_self_control::publish() const
{
	//TODO: Add your self-control output code here!
	Ackermann_drive adrive;
	adrive.speed = 0;
	adrive.acceleration = 0;
	adrive.jerk = 0;
	adrive.steering_angle = 0;
	adrive.steering_angle_velocity = 0;
	publisher->publish(adrive);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_self_control>());
	rclcpp::shutdown();
	return 0;
}
