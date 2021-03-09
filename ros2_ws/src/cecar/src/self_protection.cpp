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
}

void Cecar_self_protection::filter(const Ackermann_drive::SharedPtr adrive) const //Add your intervention message code here!
{
	publisher->publish(*adrive); //Dummy - just forward
	stringstream ss;
	ss << "{\"speed\": " << adrive->speed << ", \"steering_angle\": " << adrive->steering_angle << "}\r\n";
	//RCLCPP_INFO(get_logger(), ss.str());
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_self_protection>());
	rclcpp::shutdown();
	return 0;
}
