/*MCU communication node to RCU via UART and JSON
Author(s)		: Lukas Mirow
Date of creation	: 3/2/2020
*/

#define DRIVE_MSG_LEN 45
#define DRIVE_BUF_SIZE DRIVE_MSG_LEN + 1
#define TOKENS_EXPECTED 16
#define LEFT_REAR_LABEL "\"rl\""
#define RIGHT_REAR_LABEL "\"rr\""
#define JSMN_STRICT

#include <functional>
#include <memory>
#include <iostream>
#include "shared.hpp"
#include "uart.hpp"
#include "jsmn/jsmn.h"
#include <cmath>
#include <stdexcept>
#include <csignal>

using namespace std;


class Cecar_rcu_communication : public rclcpp::Node
{
	public:
		Cecar_rcu_communication(const string& path_uart);

	private:
		rclcpp::Subscription<Ackermann_drive>::SharedPtr subscription;
		void send_uart(const Ackermann_drive::SharedPtr adrive);
		void publish_wheel_encoder(const float& left_rear_wheel_encoder, const float& right_rear_wheel_encoder);
		rclcpp::Publisher<Wheel_encoder_cecar>::SharedPtr publisher;
		rclcpp::TimerBase::SharedPtr timer;
		char drvmsg[DRIVE_BUF_SIZE];
		jsmn_parser jparser;
		jsmntok_t tokens[TOKENS_EXPECTED];
		int extract_wheel_encoder(const string& uart_msg, float& left_rear_wheel_encoder, float& right_rear_wheel_encoder);
		void wheel_encoder_callback();
		int receive_uart(string& uart_msg);
		Uart uart;

};

int Cecar_rcu_communication::extract_wheel_encoder(const string& uart_msg, float& left_rear_wheel_encoder, float& right_rear_wheel_encoder)
{
	const char *substr, *next_substr;
	int parse_count = jsmn_parse(&jparser, uart_msg.c_str(), uart_msg.length(), tokens, TOKENS_EXPECTED);
	if (parse_count < 0)
		return -1;
	for (int i = 1; i < parse_count; i++)
	{
		substr = &uart_msg[tokens[i].start];
		next_substr = &uart_msg[tokens[i + 1].start - 1];
		if (strncmp(substr, LEFT_REAR_LABEL, strlen(LEFT_REAR_LABEL)) == 0)
			left_rear_wheel_encoder = strtof(next_substr + 1, NULL); //HACK: Sometimes, the token starts with  "': " instead of " ", which makes `strtof` fail, we are cutting the first char out here
		else if (strncmp(substr, RIGHT_REAR_LABEL, strlen(RIGHT_REAR_LABEL)) == 0)
			right_rear_wheel_encoder = strtof(next_substr + 1, NULL); //HACK: See above
	}
	return 0;
}


void Cecar_rcu_communication::wheel_encoder_callback()
{
	float left_rear_wheel_encoder, right_rear_wheel_encoder;
	string uart_msg;
	if (receive_uart(uart_msg) < 0)
		return;
	if (extract_wheel_encoder(uart_msg, left_rear_wheel_encoder, right_rear_wheel_encoder) < 0)
		return;
	publish_wheel_encoder(left_rear_wheel_encoder, right_rear_wheel_encoder);
}

void Cecar_rcu_communication::publish_wheel_encoder(const float& left_rear_wheel_encoder, const float& right_rear_wheel_encoder)
{
	Wheel_encoder_cecar msg;
	if (left_rear_wheel_encoder == NAN or right_rear_wheel_encoder == NAN)
		return;
	msg.left_rear = left_rear_wheel_encoder;
	msg.right_rear = right_rear_wheel_encoder;
	publisher->publish(msg);
#ifdef DEBUG
	stringstream ss;
	ss << "lr: " << msg.left_rear << ", rr: " << msg.right_rear;
	RCLCPP_INFO(get_logger(), ss.str());
#endif //DEBUG
}

Cecar_rcu_communication::Cecar_rcu_communication(const string& path_uart) : Node("cecar_rcu_communication"), uart(path_uart)
{
	function<void(const Ackermann_drive::SharedPtr)> callback = bind(&Cecar_rcu_communication::send_uart, this, placeholders::_1);
	subscription = create_subscription<Ackermann_drive>("cecar_drive", MSG_BUFFER_SIZE, callback);
	publisher = create_publisher<Wheel_encoder_cecar>("cecar_rcu_wheelenc", MSG_BUFFER_SIZE);
	jsmn_init(&jparser);
	timer = create_wall_timer(WHEEL_ENCODER_MSG_DELAY, bind(&Cecar_rcu_communication::wheel_encoder_callback, this));
}

void Cecar_rcu_communication::send_uart(const Ackermann_drive::SharedPtr adrive) //TODO: Reduce complexity!
{
	int printed_chars = sprintf(drvmsg, "{\"speed\": % 5.3f, \"steering_angle\": % 5.3f}\r\n", adrive->speed, adrive->steering_angle);
	if (printed_chars != DRIVE_MSG_LEN)
	{
		stringstream ss;
		ss << "Unexpected drive msg length: " << printed_chars << " instead of " << DRIVE_MSG_LEN << " -> discarded, drive message was: `" << drvmsg << "`";
		RCLCPP_WARN(get_logger(), ss.str());
	}
	else
	{
		//RCLCPP_INFO(get_logger(), drvmsg);
		uart.send(drvmsg);
	}
}

int Cecar_rcu_communication::receive_uart(string& uart_msg)
{
	try
	{
		uart_msg = uart.receive_line();
		return 0;
	}
	catch (Nothing_to_read err)
	{
		// cerr << err.what() << endl;
	}
	catch (Read_error err)
	{
		RCLCPP_WARN(get_logger(), err.what());
	}
	return -1;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Cecar_rcu_communication>(UART_PATH));
	rclcpp::shutdown();
	return 0;
}
