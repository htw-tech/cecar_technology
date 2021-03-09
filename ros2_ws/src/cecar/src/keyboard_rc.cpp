/*Using the keyboard to send Ackermann Drive messages
Author(s)		: Lukas Mirow
Date of creation	: 2/12/2020
*/

#define DIRECTION_UP 0
#define DIRECTION_RIGHT 1
#define DIRECTION_DOWN 2
#define DIRECTION_LEFT 3

#include <chrono>
#include <memory>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <curses.h>
#include "shared.hpp"

using namespace std;
using namespace std::chrono_literals;

class Cecar_rc_keyboard : public rclcpp::Node
{

	public:

		Cecar_rc_keyboard();
		void publish();
		~Cecar_rc_keyboard();

	private:

		struct Keyboard_direction
		{
			int steering_left = 0;
			int speed = 0;
			bool is_zero();
		};

		static void read_arrow_keys(bool arrows[4]);
		static Keyboard_direction read_direction();
		static Ackermann_drive direction_to_ackermann_drive(const Keyboard_direction& kdir);
		rclcpp::Publisher<Ackermann_drive>::SharedPtr publisher;
		rclcpp::TimerBase::SharedPtr timer;
		static void terminate();

};

bool Cecar_rc_keyboard::Keyboard_direction::is_zero()
{
	return steering_left == 0 and speed == 0;
}

Cecar_rc_keyboard::~Cecar_rc_keyboard()
{
	endwin();
}

void Cecar_rc_keyboard::terminate()
{
	rclcpp::shutdown();
}

Cecar_rc_keyboard::Cecar_rc_keyboard() : Node("cecar_rc_keyboard")
{
	publisher = create_publisher<Ackermann_drive>("cecar_remote_control", MSG_BUFFER_SIZE);
	timer = create_wall_timer(DRIVE_INPUT_MSG_DELAY, bind(&Cecar_rc_keyboard::publish, this));
}

void Cecar_rc_keyboard::publish()
{
	char s[1024];
	Keyboard_direction kdir = read_direction();
	if (kdir.is_zero())
		return;
	sprintf(s, "^: %i, <: %i  \r", kdir.speed, kdir.steering_left);
	printw(s);
	publisher->publish(direction_to_ackermann_drive(kdir));
}

void Cecar_rc_keyboard::read_arrow_keys(bool arrows[4])
{
	int c;
	for (unsigned i = 0; i<4; i++)
		arrows[i] = false;
	do
	{
		c = getch();
		switch (c)
		{
			case 'q':
				terminate();
				break;
			case KEY_UP:
				arrows[DIRECTION_UP] = true;
				break;
			case KEY_RIGHT:
				arrows[DIRECTION_RIGHT] = true;
				break;
			case KEY_DOWN:
				arrows[DIRECTION_DOWN] = true;
				break;
			case KEY_LEFT:
				arrows[DIRECTION_LEFT] = true;
				break;
		}
	}
	while (c != ERR);
}

Cecar_rc_keyboard::Keyboard_direction Cecar_rc_keyboard::read_direction()
{
	Keyboard_direction kdir;
	bool arrows[4];
	read_arrow_keys(arrows);
	if (arrows[DIRECTION_UP])
		kdir.speed++;
	if (arrows[DIRECTION_RIGHT])
		kdir.steering_left--;
	if (arrows[DIRECTION_DOWN])
		kdir.speed--;
	if (arrows[DIRECTION_LEFT])
		kdir.steering_left++;
	return kdir;
}

Ackermann_drive Cecar_rc_keyboard::direction_to_ackermann_drive(const Cecar_rc_keyboard::Keyboard_direction& kdir)
{
	Ackermann_drive adrive;
	adrive.steering_angle = kdir.steering_left;
	adrive.speed = kdir.speed;
	adrive.steering_angle_velocity = 0;
	adrive.acceleration = 0;
	adrive.jerk = 0;
	return adrive;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	initscr();
	printw("Press `q` for quit\n");
	cbreak();
	noecho();
	nodelay(stdscr, TRUE);
	keypad(stdscr, TRUE);
	refresh();
	rclcpp::spin(make_shared<Cecar_rc_keyboard>());
	rclcpp::shutdown();
	return 0;
}
