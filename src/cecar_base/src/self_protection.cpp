/*Placeholder of a self-protection node for the CeCar
Author(s)		: Lukas Mirow
Date of creation	: 2/19/2020
*/

#include "shared.hpp"
#include <chrono>
#include <memory>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdio.h>

using namespace std;

class Cecar_self_protection : public rclcpp::Node
{

	public:

		Cecar_self_protection();

	private:

		rclcpp::Publisher<Ackermann_drive>::SharedPtr publisher;
		rclcpp::Subscription<Ackermann_drive>::SharedPtr subscription;
		void filter(const Ackermann_drive::SharedPtr adrive) const;

};

Cecar_self_protection::Cecar_self_protection() : Node("cecar_self_protection")
{
	publisher = create_publisher<Ackermann_drive>("cecar_drive", MSG_BUFFER_SIZE);
	function<void(const Ackermann_drive::SharedPtr)> cb = bind(&Cecar_self_protection::filter, this, placeholders::_1);
	subscription = create_subscription<Ackermann_drive>("cecar_self_protection_input", MSG_BUFFER_SIZE, cb);
	declare_parameter("speed_amplitude_forwards",rclcpp::ParameterValue(SPEED_AMPLITUDE_FORWARDS));
	declare_parameter("speed_amplitude_backwards",rclcpp::ParameterValue(SPEED_AMPLITUDE_BACKWARDS));
	declare_parameter("steering_amplitude_right",rclcpp::ParameterValue(STEERING_AMPLITUDE_RIGHT));
	declare_parameter("steering_amplitude_left",rclcpp::ParameterValue(STEERING_AMPLITUDE_LEFT));
}

void Cecar_self_protection::filter(const Ackermann_drive::SharedPtr adrive) const //Add your intervention message code here!
{
	float max_speed_forwards, max_speed_backwards, max_right_steering, max_left_steering;
	get_parameter("speed_amplitude_forwards",max_speed_forwards);
	get_parameter("speed_amplitude_backwards",max_speed_backwards);
	get_parameter("steering_amplitude_right",max_right_steering);
	get_parameter("steering_amplitude_left",max_left_steering);

	if(adrive->speed > max_speed_forwards)
		adrive->speed = max_speed_forwards;
	else if (adrive->speed < max_speed_backwards)
		adrive->speed = max_speed_backwards;

	if(adrive->steering_angle > max_right_steering)
		adrive->steering_angle = max_right_steering;
	else if (adrive->steering_angle < max_left_steering)
		adrive->steering_angle = max_left_steering;
	
	publisher->publish(*adrive); //Dummy - just forward
	//stringstream ss;
	//ss << "{\"speed\": " << adrive->speed << ", \"steering_angle\": " << adrive->steering_angle << "}\r\n";
	//RCLCPP_INFO(get_logger(), ss.str());
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_self_protection>());
	rclcpp::shutdown();
	return 0;
}
