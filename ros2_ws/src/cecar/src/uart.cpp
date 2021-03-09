/*UART wrapper implementation
Author(s)		: Lukas Mirow
Date of creation	: 3/2/2020
*/

#define SECS_READ_TIMEOUT 0
#define USECS_READ_TIMEOUT 5000

#include "uart.hpp"
#include <sstream>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
//
#include <iostream>

void Uart::open_uart(const std::string& uart_path)
{
	uart = open(uart_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (uart < 0)
	{
		std::stringstream ss;
		ss << "Failed to open UART at `" << uart_path << "` (errno `" << errno << "`)";
		throw std::runtime_error(ss.str());
	}
}

void Uart::init() //https://www.cmrr.umn.edu/~strupp/serial.html
{
	struct termios tty;
	if (tcgetattr(uart, &tty) != 0)
	{
		std::stringstream ss;
		ss << "Failed to create UART initialization struct (errno `" << errno << "`)";
		throw std::runtime_error(ss.str());
	}
	memset(&tty, 0, sizeof(tty));
	cfsetospeed(&tty, UART_SPEED);
	cfsetispeed(&tty, UART_SPEED);
	tty.c_cflag |= CLOCAL; //Don't take ownership of TTY
	tty.c_cflag |= CREAD; //Enable receiver
	tty.c_cflag &= ~CSIZE; //Reset character size bitmask
	tty.c_cflag |= CS8; //Set character size to 8
	tty.c_cflag &= ~PARENB; //No parity
	tty.c_cflag &= ~CSTOPB; //No second stop bit
	tty.c_cflag &= ~CRTSCTS; //No hardware flow control
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Set raw
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); //No software flow control
	tty.c_oflag &= ~OPOST; //Set raw too
	if (tcsetattr(uart, TCSANOW, &tty) != 0)
	{
		std::stringstream ss;
		ss << "Failed to apply UART initialization (errno `" << errno << "`)";
		throw std::runtime_error(ss.str());
	}
}

Uart::Uart(const std::string uart_path)
{
	open_uart(uart_path);
	init();
}

Uart::~Uart()
{
	close(uart);
}

int Uart::send(const std::string& msg) const
{
	return dprintf(uart, "%s", msg.c_str());
}

void create_timeout(struct timeval* timeout)
{
	timeout->tv_sec = SECS_READ_TIMEOUT;
	timeout->tv_usec = USECS_READ_TIMEOUT;
}

void prepare_read(fd_set *read_fds, fd_set *write_fds, fd_set *except_fds, int uart, struct timeval *timeout)
{
	FD_ZERO(read_fds);
	FD_ZERO(write_fds);
	FD_ZERO(except_fds);
	FD_SET(uart, read_fds);
	create_timeout(timeout);
}

char perform_read(fd_set *read_fds, fd_set *write_fds, fd_set *except_fds, int uart, struct timeval *timeout)
{
	char ret;
	int ret_val;
	errno = 0;
	ret_val = select(uart + 1, read_fds, write_fds, except_fds, timeout);
	if (ret_val == 0)
		throw Nothing_to_read("Nothing to read");
	if (ret_val < 0 or read(uart, &ret, sizeof(char)) < 0)
	{
		std::stringstream ss;
		ss << "Error reading from UART (errno `" << errno << "`)";
		throw Read_error(ss.str());
	}
	return ret;
}

char Uart::receive() const
{
	fd_set read_fds, write_fds, except_fds;
	struct timeval timeout;
	prepare_read(&read_fds, &write_fds, &except_fds, uart, &timeout);
	return perform_read(&read_fds, &write_fds, &except_fds, uart, &timeout);
}

std::string Uart::receive_line() const
{
	std::stringstream ss;
	char this_char;
	while (this_char != '\n')
	{
		this_char = receive();
		ss << this_char;
	}
	return ss.str();
}
