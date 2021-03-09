/*Intelligence collection node for CeCar
Author(s)		: Lukas Mirow
Date of creation	: 11/18/2020
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

class Cecar_intelligence_collection : public rclcpp::Node
{

	public:

		Cecar_intelligence_collection();

	private:

		void wheelenc_callback(const Wheel_encoder_cecar::SharedPtr wheel_encoder);
		void adrive_callback(const Ackermann_drive::SharedPtr adrive);
		void rc_adrive_callback(const Ackermann_drive::SharedPtr adrive);
		void publish_status() const;
		Wheel_encoder_cecar last_wheelenc_msg;
		Ackermann_drive last_drive_msg;
		Ackermann_drive last_rc_drive_msg;
		rclcpp::Publisher<Cecar_status>::SharedPtr status_publisher;
		rclcpp::Publisher<Wheel_encoder_cecar>::SharedPtr wheelenc_publisher;
		rclcpp::Subscription<Wheel_encoder_cecar>::SharedPtr wheelenc_subscription;
		rclcpp::Subscription<Ackermann_drive>::SharedPtr adrive_subscription;
		rclcpp::Subscription<Ackermann_drive>::SharedPtr rc_adrive_subscription;
		rclcpp::TimerBase::SharedPtr timer_status_msgs;

};

void Cecar_intelligence_collection::publish_status() const
{
	Cecar_status status;
	status.wheel_encoder.left_rear = last_wheelenc_msg.left_rear;
	status.wheel_encoder.right_rear = last_wheelenc_msg.right_rear;
	status.rc_drive = last_rc_drive_msg;
	status.drive = last_drive_msg;
	status_publisher->publish(status);
}

Cecar_intelligence_collection::Cecar_intelligence_collection() : Node("cecar_intelligence_collection")
{
	status_publisher = create_publisher<Cecar_status>("cecar_status", MSG_BUFFER_SIZE);
	wheelenc_publisher = create_publisher<Wheel_encoder_cecar>("cecar_wheelenc", MSG_BUFFER_SIZE);
	//self_control_publisher = create_publisher</*TODO: Choose topic type*/>("cecar_self_control_input", MSG_BUFFER_SIZE); TODO: Self-control
	//self_protection_publisher = create_publisher</*TODO: Choose topic type*/>("cecar_self_protection_input", MSG_BUFFER_SIZE); TODO: Self-protection
	function<void(const Wheel_encoder_cecar::SharedPtr)> cb = bind(&Cecar_intelligence_collection::wheelenc_callback, this, placeholders::_1); //TODO: Replace with inline `bind()` call?
	wheelenc_subscription = create_subscription<Wheel_encoder_cecar>("cecar_rcu_wheelenc", MSG_BUFFER_SIZE, cb);
	adrive_subscription = create_subscription<Ackermann_drive>("cecar_drive", MSG_BUFFER_SIZE, bind(&Cecar_intelligence_collection::adrive_callback, this, placeholders::_1));
	rc_adrive_subscription = create_subscription<Ackermann_drive>("cecar_remote_control", MSG_BUFFER_SIZE, bind(&Cecar_intelligence_collection::rc_adrive_callback, this, placeholders::_1));
	timer_status_msgs = create_wall_timer(DRIVE_INPUT_MSG_DELAY, bind(&Cecar_intelligence_collection::publish_status, this));
}

void Cecar_intelligence_collection::wheelenc_callback(const Wheel_encoder_cecar::SharedPtr wheel_encoder)
{
	last_wheelenc_msg = *wheel_encoder; //FIXME: Might this be thread-unsafe?
	wheelenc_publisher->publish(*wheel_encoder);
}

void Cecar_intelligence_collection::rc_adrive_callback(const Ackermann_drive::SharedPtr adrive)
{
	last_drive_msg = *adrive; //FIXME: Might this be thread-unsafe?
}

void Cecar_intelligence_collection::adrive_callback(const Ackermann_drive::SharedPtr adrive)
{
	last_rc_drive_msg = *adrive; //FIXME: Might this be thread-unsafe?
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_intelligence_collection>());
	rclcpp::shutdown();
	return 0;
}
