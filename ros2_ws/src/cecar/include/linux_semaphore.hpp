/*Linux semaphore header
Author(s)		: Lukas Mirow
Date of creation	: 2/19/2020
*/

#ifndef LINUX_SEMAPHORE_HPP
#define LINUX_SEMAPHORE_HPP

//Don't forget to link with `-pthread`!

#include <semaphore.h>
#include <stdexcept>
#include <stdlib.h>

// struct Semaphore_exception : private std::exception
// {
// 	explicit Semaphore_exception(const std::string msg) : msg(msg) {}
// 	const std::string what() const {return msg;}

// 	private:
// 		const std::string msg;
// };

class Linux_semaphore
{

	public:
		Linux_semaphore(const bool& shared_between_processes, const unsigned& initial_value);
		~Linux_semaphore();
		void unlock();
		void lock();

	private:
		sem_t semaphore;
};

Linux_semaphore::Linux_semaphore(const bool& shared_between_processes, const unsigned& initial_value)
{
	int sbp;
	if (shared_between_processes)
		sbp = 1;
	else
		sbp = 0;
	if (sem_init(&semaphore, sbp, initial_value))
		throw std::runtime_error("Semaphore initialization failed");
}

Linux_semaphore::~Linux_semaphore()
{
	if (sem_destroy(&semaphore))
		exit(1);
}

void Linux_semaphore::lock()
{
	if (sem_wait(&semaphore))
		throw std::runtime_error("Locking the semaphore has been interrupted - nothing was done");
}

void Linux_semaphore::unlock()
{
	if (sem_post(&semaphore))
		throw std::runtime_error("Unlocking the semaphore has failed");
}


#endif //LINUX_SEMAPHORE_HPP
