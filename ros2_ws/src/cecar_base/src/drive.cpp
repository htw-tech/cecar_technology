/*Drive message controller for the CeCar
Author(s)		: Lukas Mirow
Date of creation	: 2/19/2020
*/

//Higher priority values beat lower priority values:
#define SELF_CONTROL_PRIORITY 1
#define REMOTE_CONTROL_PRIORITY 2

#define SEMAPHORE_SHARED_BETWEEN_PROCESSES true
#define SEMAPHORE_INIT_VALUE 1

#include "shared.hpp"
#include <chrono>
#include <memory>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include "linux_semaphore.hpp"

using namespace std;

class Cecar_drive_control : public rclcpp::Node
{

	public:

		Cecar_drive_control();

	private:

		struct Drive_container
		{
			Ackermann_drive adrive;
			int priority;

			Drive_container() {reset();}

			void consider(const Ackermann_drive adrive, int priority)
			{
				if (priority >= this->priority)
				{
					this->priority = priority;
					this->adrive = adrive;
				}
			}

			void reset()
			{
				adrive.speed = 0;
				adrive.acceleration = 0;
				adrive.jerk = 0;
				adrive.steering_angle = 0;
				adrive.steering_angle_velocity = 0;
				priority = 0;
			}
		};

		Drive_container highest_priority_drive;
		rclcpp::Publisher<Ackermann_drive>::SharedPtr publisher;
		rclcpp::Subscription<Ackermann_drive>::SharedPtr self_control_subscription;
		rclcpp::Subscription<Ackermann_drive>::SharedPtr remote_control_subscription;
		rclcpp::TimerBase::SharedPtr timer;
		void remote_control_callback(const Ackermann_drive::SharedPtr adrive);
		void self_control_callback(const Ackermann_drive::SharedPtr adrive);
		void publish();
		Linux_semaphore semaphore;

};

Cecar_drive_control::Cecar_drive_control() : Node("cecar_drive_control"), semaphore(SEMAPHORE_SHARED_BETWEEN_PROCESSES, SEMAPHORE_INIT_VALUE)
{
	function<void(const Ackermann_drive::SharedPtr)> cb;
	publisher = create_publisher<Ackermann_drive>("cecar_self_protection_input", MSG_BUFFER_SIZE);
	cb = bind(&Cecar_drive_control::self_control_callback, this, placeholders::_1);
	self_control_subscription = create_subscription<Ackermann_drive>("cecar_self_control_output", MSG_BUFFER_SIZE, cb);
	cb = bind(&Cecar_drive_control::remote_control_callback, this, placeholders::_1);
	remote_control_subscription = create_subscription<Ackermann_drive>("cecar_remote_control", MSG_BUFFER_SIZE, cb);
	timer = create_wall_timer(DRIVE_OUTPUT_MSG_DELAY, bind(&Cecar_drive_control::publish, this));
}

void Cecar_drive_control::publish()
{
	try
	{
		semaphore.lock();
		publisher->publish(highest_priority_drive.adrive);
		highest_priority_drive.reset();
		semaphore.unlock();
	}
	catch (runtime_error e)
	{
		RCLCPP_ERROR(get_logger(), e.what());
	}
}

void Cecar_drive_control::remote_control_callback(const Ackermann_drive::SharedPtr adrive)
{
	try
	{
		semaphore.lock();
		highest_priority_drive.consider(*adrive, REMOTE_CONTROL_PRIORITY);
		semaphore.unlock();
	}
	catch (runtime_error e)
	{
		RCLCPP_ERROR(get_logger(), e.what());
	}
}

void Cecar_drive_control::self_control_callback(const Ackermann_drive::SharedPtr adrive)
{
	try
	{
		semaphore.lock();
		highest_priority_drive.consider(*adrive, SELF_CONTROL_PRIORITY);
		semaphore.unlock();
	}
	catch (runtime_error e)
	{
		RCLCPP_ERROR(get_logger(), e.what());
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_drive_control>());
	rclcpp::shutdown();
	return 0;
}
