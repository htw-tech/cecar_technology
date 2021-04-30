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
		void read_gamepad_control();
		float read_throttle();
		float adjust_steering(float steer);
		void publish();
		rclcpp::Publisher<Ackermann_drive>::SharedPtr publisher;
		rclcpp::TimerBase::SharedPtr timer;
		GAMEPAD_DEVICE gpd;
		Gamepad_direction gdir;
		float steering_offset = 0;

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
	declare_parameter("gamepad_safe_mode_amplitude");
	declare_parameter("gamepad_speed_amplitude_forwards");
	declare_parameter("gamepad_speed_amplitude_backwards");
	declare_parameter("gamepad_steering_amplitude");
}

void Cecar_gamepad::publish()
{
	read_gamepad_control();

	if (not gdir.is_zero())
	{
		publisher->publish(gamepad_steering_to_ackermann_drive());
	}
	RCLCPP_INFO(get_logger(), "^: %f <>: %f", gdir.forward, gdir.steering_left);
}

float Cecar_gamepad::adjust_steering(float steering)
{
	float new_steering_offset, steering_amplitude;
	get_parameter("gamepad_steering_amplitude",steering_amplitude);

	if (GamepadButtonTriggered(gpd, BUTTON_DPAD_LEFT))
	{
		new_steering_offset = steering_offset - STEERING_OFFSET_STEP;
		if (new_steering_offset < (-steering_amplitude))
			new_steering_offset = (-steering_amplitude);
		steering_offset = new_steering_offset;
	}
	else if (GamepadButtonTriggered(gpd, BUTTON_DPAD_RIGHT))
	{
		new_steering_offset = steering_offset + STEERING_OFFSET_STEP;
		if (new_steering_offset > steering_amplitude)
			new_steering_offset = steering_amplitude;
		steering_offset = new_steering_offset;
	}
	
	float steer_angle = steering * steering_amplitude + steering_offset;

	if(steer_angle > steering_amplitude)
		steer_angle = steering_amplitude;
	else if (steer_angle < (-steering_amplitude))
		steer_angle = (-steering_amplitude);

	return steer_angle;
}

float Cecar_gamepad::read_throttle()
{
	float safe_mode_amplitude, throttle, forward_speed_amplitude, backward_speed_amplitude;
	get_parameter("gamepad_safe_mode_amplitude",safe_mode_amplitude);
	get_parameter("gamepad_speed_amplitude_forwards",forward_speed_amplitude);
	get_parameter("gamepad_speed_amplitude_backwards",backward_speed_amplitude);

	if (GamepadButtonDown(gpd, BUTTON_RIGHT_SHOULDER))
		return safe_mode_amplitude;
	else {
		throttle = GamepadTriggerLength(gpd, TRIGGER_RIGHT) - GamepadTriggerLength(gpd, TRIGGER_LEFT);

		if (throttle >= 0.0)
			throttle = throttle * forward_speed_amplitude;
		else
			throttle = throttle * backward_speed_amplitude; 
	}

	if (throttle > forward_speed_amplitude)
		throttle = forward_speed_amplitude;
	else if (throttle < (-backward_speed_amplitude))
		throttle = (-backward_speed_amplitude);

	return throttle;
}

void Cecar_gamepad::read_gamepad_control()
{
	float stick_angle, stick_magnitude;
	GamepadUpdate();
	if (GamepadIsConnected(gpd))
	{
		//HACK: Because GamepadStickNormXY doesn't work properly for me, we are calculating x from angle and magnitude
		stick_magnitude = GamepadStickLength(gpd, STICK_LEFT);
		stick_angle = GamepadStickAngle(gpd, STICK_LEFT);
		gdir.steering_left = adjust_steering(stick_magnitude * cos(stick_angle)); //FIXME: Steering was inverted - why??
		gdir.forward = read_throttle();
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
