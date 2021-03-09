/*Using the Xbox One controller to send Ackermann Drive messages
Author(s)		: Lukas Mirow
Date of creation	: 2/13/2020
*/

#define JOYSTICK_PATH "/dev/input/js0"
#define STEERING_AXIS 0
#define THRUST_AXIS 5
#define BREAKING_AXIS 2

#include <stdexcept>
#include "gamepad.h"
#include <chrono>
#include <memory>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "shared.hpp"

using namespace std;

class Cecar_gamepad : public rclcpp::Node
{

	public:

		Cecar_gamepad();
		~Cecar_gamepad();

	private:

		struct Gamepad_direction
		{
			float steering_left = 0;
			float forward = 0;
			bool is_zero() {return steering_left == 0 and forward == 0;}
		};

		Ackermann_drive gamepad_steering_to_ackermann_drive();
		void read_gamepad_steering();
		void publish();
		rclcpp::Publisher<Ackermann_drive>::SharedPtr publisher;
		rclcpp::TimerBase::SharedPtr timer;
		GAMEPAD_DEVICE gpd;
		Gamepad_direction gdir;

};

Cecar_gamepad::~Cecar_gamepad()
{
	GamepadShutdown();
}

Cecar_gamepad::Cecar_gamepad() : Node("cecar_gamepad")
{
	GamepadInit();
	publisher = create_publisher<Ackermann_drive>("cecar_remote_control", MSG_BUFFER_SIZE);
	timer = create_wall_timer(DRIVE_INPUT_MSG_DELAY, bind(&Cecar_gamepad::publish, this));
}

void Cecar_gamepad::publish()
{
	read_gamepad_steering();
	if (not gdir.is_zero())
		publisher->publish(gamepad_steering_to_ackermann_drive());
	RCLCPP_INFO(get_logger(), "^: %f <>: %f", gdir.forward, gdir.steering_left);
}

void Cecar_gamepad::read_gamepad_steering()
{
	float stick_angle, stick_magnitude;
	GamepadUpdate();
	if (GamepadIsConnected(gpd))
	{
		//HACK: Because GamepadStickNormXY doesn't work properly for me, we are calculating x from angle and magnitude
		stick_magnitude = GamepadStickLength(gpd, STICK_LEFT);
		stick_angle = GamepadStickAngle(gpd, STICK_LEFT);
		gdir.steering_left = stick_magnitude * cos(stick_angle); //FIXME: Steering was inverted - why??
		gdir.forward = GamepadTriggerLength(gpd, TRIGGER_RIGHT) - GamepadTriggerLength(gpd, TRIGGER_LEFT);
	}
	else
	{
		gdir.steering_left = 0;
		gdir.forward = 0;
	}
}

Ackermann_drive Cecar_gamepad::gamepad_steering_to_ackermann_drive()
{
	Ackermann_drive adrive;
	adrive.steering_angle = gdir.steering_left;
	adrive.speed = gdir.forward;
	adrive.steering_angle_velocity = 0;
	adrive.jerk = 0;
	adrive.acceleration = 0;
	return adrive;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Cecar_gamepad>());
	rclcpp::shutdown();
	return 0;
}
