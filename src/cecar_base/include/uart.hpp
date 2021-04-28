/*UART Wrapper Header
Author(s)               : Lukas Mirow
Date of creation        : 3/2/2020
*/

#ifndef UART_HPP
#define UART_HPP

#include <string>
#include "shared.hpp"
#include <stdexcept>
#include <string>

typedef struct spi_ioc_transfer SPI_Transfer;

class Nothing_to_read : public std::runtime_error
{
	public:
		Nothing_to_read(const std::string& what) : std::runtime_error(what) {};
};

class Read_error : public std::runtime_error
{
	public:
		Read_error(const std::string& what) : std::runtime_error(what) {};
};

class Uart
{
        public:
                Uart(const std::string uart_path);
                ~Uart();
		char receive() const;
		std::string receive_line() const;
		int send(const std::string&) const;

        private:
                int uart;
		void open_uart(const std::string& uart_path);
		void init();
};

#endif //UART_HPP
