/*Shared header file for CeCar
Author(s)		: Lukas Mirow
Date of creation	: 3/13/2020
*/

#ifndef CECAR_SHARED
#define CECAR_SHARED

#define MSG_BUFFER_SIZE 5
#define DRIVE_INPUT_MSG_DELAY 5ms
#define DRIVE_OUTPUT_MSG_DELAY 3 * DRIVE_INPUT_MSG_DELAY
#define WHEEL_ENCODER_MSG_DELAY 100ms
#define SPEED_AMPLITUDE_FORWARDS 3.5
#define SPEED_AMPLITUDE_BACKWARDS -3.5
#define STEERING_AMPLITUDE_RIGHT 45.0
#define STEERING_AMPLITUDE_LEFT -45.0
#define STEERING_OFFSET_STEP 0.5
#define UART_SPEED B57600
#define UART_PATH "/dev/ttyTHS2"

#include "rclcpp/rclcpp.hpp"
#include "cecar_msgs/msg/ackermann_drive.hpp"
#include "cecar_msgs/msg/wheel_enc.hpp"
#include "cecar_msgs/msg/cecar_status.hpp"

typedef cecar_msgs::msg::AckermannDrive Ackermann_drive;
typedef cecar_msgs::msg::WheelEnc Wheel_encoder_cecar;
typedef cecar_msgs::msg::CecarStatus Cecar_status;

#endif //CECAR_SHARED
