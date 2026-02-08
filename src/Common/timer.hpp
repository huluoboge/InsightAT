#ifndef timer_h__
#define timer_h__

#include "common_global.h"

#include <chrono>
#include <iostream>

#ifndef HAVE_CXX11_CHRONO
#define HAVE_CXX11_CHRONO
#endif
namespace insight{
//! \brief Timer class with microsecond accuracy.
class Timer
{
public:
	//! Default constructor
	Timer();
	//! Reset the timer to zero.
	void reset();
	void start();
	void pause();
	void resume();
	void restart();
	//! Returns the elapsed time in seconds.
	double elapsed() const;
	//! Returns the elapsed time in milliseconds.
	double elapsedMs() const;

	double elapsedMinutes() const;

	double elapsedHours() const;
private:

#ifdef HAVE_CXX11_CHRONO
	std::chrono::high_resolution_clock::time_point start_;
	std::chrono::high_resolution_clock::time_point pause_time_;
#else
	double start_;
#ifdef _WIN32
	double frequency_;
#endif
#endif // HAVE_CXX11_CHRONO

	bool started_;
	bool paused_;
};

// print the elapsed time
std::ostream& operator << (std::ostream&, const Timer&);

}//name space insight

#endif 

