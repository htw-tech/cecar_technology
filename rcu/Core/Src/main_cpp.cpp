/*
 * mainUART_HandleTypeDef huart2;.cpp
 *
 *		Adapted for CeCar by Lukas Mirow at UoAS Berlin (HTW Berlin) in 2019 & 2020
 *
 *      Copyright (c) 2019, Frank Bauernoeppel
 *
 *      TwistStampedMsg related computations and message handling derived from
 *      https://gitlab.com/NeuroRace/neuroracer-robot-engine/
 *
 *      Parts Copyright (c) 2016, JetsonHacks
 *      Parts Copyright (c) 2017, Timm Fröhlich
 *      Parts Copyright (c) 2018, HTW Berlin, Patrick Baumann
 *
 *      rosserial_stm32 implementation derived from
 *      https://github.com/yoneken/rosserial_stm32/
 *      Copyright (c) 2018, Kenta Yonekura
 *
 */

#define TOKENS_EXPECTED 16
#define DRIVE_MSG_LEN 44
#define MSG_LEN_EXPECTED 64
#define TOKEN_LEN_EXPECTED 64
#define JSMN_STRICT
#define UART_DELAY_MS 30
#define ODO_MSG_LEN 30


#include "shared.hpp"
#include <stdio.h>
#include <main.h>
#include <main_cpp.h>
#include "jsmn/jsmn.h"
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal_dma.h"

constexpr double pi = 3.14159265358979323846;

extern UART_HandleTypeDef huart2;

jsmn_parser jparser;
jsmntok_t tokens[TOKENS_EXPECTED];
uint8_t uart_dma_buffer[MSG_LEN_EXPECTED]; //Buffer for DMA recv
char rx_msg[MSG_LEN_EXPECTED]; //Linearized buffer for message parsing
unsigned int read_ptr = 0; //Byte offset that was last copied to linearized buffer
unsigned int write_ptr = 0; //Byte offset that was last written to from DMA
int testsem = 1; //Test semaphore

//////////////////////////////////////////////////////

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{}

// 500 (=0.5 µs) is 100%
const int diffSteer_cecar1		= 450;
const int diffThrottle_cecar1	= 200;


// range of incoming /nr/engine/input/actions messages
const double boundary_low	= -1.0;
const double boundary_high	= +1.0;


// General bounds for the steering servo and the ESC
// timer htim3 runs @ 1 MHz, pulse width in µs in range [1ms .. 2ms]:
const int servoNeutral		= 1500;

// PWM parameters:
int usSteerRange			= diffSteer_cecar1;
int usThrottleRange			= diffThrottle_cecar1;

const int minSteering		= servoNeutral - usSteerRange;
const int maxSteering		= servoNeutral + usSteerRange;

const int minThrottle		= servoNeutral - usThrottleRange;
const int maxThrottle		= servoNeutral + usThrottleRange;

/**
 * Arduino 'map' funtion for floating point
 */
static double fmap (double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Combined MIN and MAX.
 */
inline int between(int x, int minimum, int maximum) {
  return x < minimum ? minimum : (x > maximum ? maximum : x);
}

/**
 * Map steering input to a valid servo angle.
 *
 * @param double steer Value from boundary_low to boundary_high
 * (its the same value but could also be different from mapThrottle)
 * @return int Steering angle
 */
int mapSteering(double steer) {
  int servoAngle = (int)fmap(steer, boundary_low, boundary_high, minSteering, maxSteering);
  return between(servoAngle, minSteering, maxSteering);
}

/**
 * Map throttle input to a valid servo angle for the ESC.
 *
 * @param double throttle Value from boundary_low to boundary_high
 * (its the same value but could also be different from mapSteering)
 * @return int Throttle angle / value
 */
int mapThrottle(double throttle) {
  int servoAngle = (int)fmap(throttle, boundary_low, boundary_high, minThrottle, maxThrottle);
  return between(servoAngle, minThrottle, maxThrottle);
}

extern TIM_HandleTypeDef htim3;

void driveCallback(const double throttleIn, const double steerIn) {
   const int steer = mapSteering(steerIn);
   const int throttle = mapThrottle(throttleIn);

   //PRINTF("PWM: steer: %4d; throttle: %4d\n", steer, throttle );
   __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_1, steer );
   __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_2, throttle );
}

//////////////////////////////////////////////////////////////////

// wheel encoders via EXIT

uint32_t wenc_tacho[4];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( GPIO_Pin==WENC_TACHO_FL_Pin) {
		++wenc_tacho[0];
	}
	if( GPIO_Pin==WENC_TACHO_FR_Pin) {
		++wenc_tacho[1];
	}
	if( GPIO_Pin==WENC_TACHO_RL_Pin) {
		++wenc_tacho[2];
	}
	if( GPIO_Pin==WENC_TACHO_RR_Pin) {
		++wenc_tacho[3];
	}
}

void harvest_wheel_encoders()
{
	static uint32_t last_wenc_tacho[4];
	float slots[4];
	for( int i=0; i<4; ++i ) {
		slots[i] = wenc_tacho[i] - last_wenc_tacho[i];
		last_wenc_tacho[i] = wenc_tacho[i];
	}

	// Two HOA902 wheel encoders are installed on rear wheels.
	// Each encoder disk has 60 slots.
	const float slots_per_revolution = 60;

	//	Wheel diameter is 83.5mm --> circumference 262.3mm.
	const float wheel_circumference = 3.14 * 0.0835; // m

	// measurement period is 100 ms
	const float measurement_period = 0.1; // s

	float speed[4];
	for( int i=0; i<4; ++i ) {
		speed[i] = ((slots[i] / measurement_period) / slots_per_revolution) * wheel_circumference;
	}
	// Target speed 0.1 m/s --> 0.381 wheel rotations/s --> 22.86 slots/s --> 45.72 tacho pulses/s
	// Target speed 1.0 m/s --> 3.81 wheel rotations/s --> 228.6 slots/s --> 457.2 tacho pulses/s
	// Target speed 10.0 m/s --> 38.1 wheel rotations/s --> 2286 slots/s --> 4572 tacho pulses/s

	// change signs depending on WHHEL_ENC_DIR GPIOs
	if( HAL_GPIO_ReadPin( WENC_DIR_RL_GPIO_Port, WENC_DIR_RL_Pin) == GPIO_PIN_RESET) {
		speed[0] = -speed[0];
	}
	if( HAL_GPIO_ReadPin( WENC_DIR_RR_GPIO_Port, WENC_DIR_RR_Pin) == GPIO_PIN_SET) {
		speed[1] = -speed[1];
	}
	if( HAL_GPIO_ReadPin( WENC_DIR_RL_GPIO_Port, WENC_DIR_RL_Pin) == GPIO_PIN_RESET) {
		speed[2] = -speed[2];
	}
	if( HAL_GPIO_ReadPin( WENC_DIR_RR_GPIO_Port, WENC_DIR_RR_Pin) == GPIO_PIN_SET) {
		speed[3] = -speed[3];
	}

	//TODO: Send odometry data through UART
	char odo_msg[ODO_MSG_LEN];
	int r = sprintf(odo_msg, "{\"rl\": % 5.3f, \"rr\": % 5.3f}\n", speed[2], speed[3]);
	printf("%i: %s\n", r, odo_msg);
	HAL_UART_Transmit(&huart2, (uint8_t*)odo_msg, ODO_MSG_LEN, 500);
}

// wheel encoder timers
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// wheel encoder periodic interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2) {
		harvest_wheel_encoders(); // 100ms
	}
}

void setup(UART_HandleTypeDef *huart)
{
	// servo PWM
	__HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_1, servoNeutral );
    __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_2, servoNeutral );

    // PWM timer for the servos: steer, throttle
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	// use hard coded defaults
	usSteerRange = diffSteer_cecar1;
	usThrottleRange = diffThrottle_cecar1;

	// clock source for wheel encoder
	HAL_TIM_Base_Start_IT(&htim2);

	//JSON parser
	jsmn_init(&jparser);

	//Start receiving using DMA on UART2:
	HAL_UART_Receive_DMA(huart, (uint8_t*)uart_dma_buffer, MSG_LEN_EXPECTED);
}

unsigned get_dma_uart_buffer_pos(DMA_HandleTypeDef *hdma)
{
	return MSG_LEN_EXPECTED - __HAL_DMA_GET_COUNTER(hdma);
}

int find_json_object(char* const msg, const char **start, const char **end)
{
	int fail;
	*start = strchr(msg, '{');
	*end = strchr(*start, '}');
	if (*start == NULL or *end == NULL)
		fail = 1;
	else
		fail = 0;
	(*end)++;
	return fail;
}

char *extract_json_object(char *msg)
{
	const char *start, *end;
	unsigned slen;
	if (find_json_object(msg, &start, &end))
		return NULL;
	slen = end - start;
	memmove(msg, start, slen);
	msg[slen] = '\0';
	return msg;
}

void extract_drive_parameters(const char *json_msg, float * const speed, float * const steering_angle_left)
{
	const char *substr, *next_substr;
	int parse_count;
	*speed = 0;
	*steering_angle_left = 0;
	parse_count = jsmn_parse(&jparser, json_msg, strlen(json_msg), tokens, TOKENS_EXPECTED);
	if (parse_count <= 0)
		return;
	for (int i = 1; i < parse_count; i++)
	{
		substr = &json_msg[tokens[i].start];
		next_substr = &json_msg[tokens[i + 1].start - 1];
		if (strncmp(substr, "speed", strlen("speed")) == 0)
			*speed = strtof(next_substr, NULL);
		else if (strncmp(substr, "steering_angle", strlen("steering_angle")) == 0)
			*steering_angle_left = strtof(next_substr, NULL);
	}
}

void apply_drive_parameters(const float speed, const float steering_angle_left)
{
	driveCallback(speed, steering_angle_left);
}

void precautious_stop()
{
	driveCallback(0, 0);
}

void handle_drive_msg(char *msg, DMA_HandleTypeDef *hdma)
{
	float speed, steering_angle_left;
	if (extract_json_object(msg) == NULL)
		return;
	extract_drive_parameters(msg, &speed, &steering_angle_left);
	apply_drive_parameters(speed, steering_angle_left); //noah
	//apply_drive_parameters(steering_angle_left, speed); //emma
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

static inline uint32_t getRdmaInd(UART_HandleTypeDef *huart)
{
        return (MSG_LEN_EXPECTED - __HAL_DMA_GET_COUNTER(huart->hdmarx)) & (MSG_LEN_EXPECTED - 1);
}

uint32_t rind;
// read a single char form DMA ring buffer. return -1 if ring buffer was empty
int read(UART_HandleTypeDef *huart)
{
  int c = EOF;
  if(rind != getRdmaInd(huart))
  {
    c = uart_dma_buffer[rind++];
    rind &= MSG_LEN_EXPECTED - 1;
  }
  return c;
}

void loop(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
	int c;
	static char msg[MSG_LEN_EXPECTED];
	constexpr char msg_start_char = '{';
	constexpr char msg_end_char = '\n';
	constexpr int not_in_msg = -1;
	int i = not_in_msg;
	while (1)
	{
		c = read(huart);
		if (c == EOF)
		{
			continue;
		}
		if (i > MSG_LEN_EXPECTED)
		{
			i = not_in_msg;
			continue;
		}
		if (c ==  msg_start_char)
			i = 0;
		if (i == not_in_msg)
			continue;
		msg[i++] = c;
		if (c == msg_end_char)
		{
			msg[i] = '\0';
			handle_drive_msg(msg, hdma);
			i = not_in_msg;
		}
	}
}
