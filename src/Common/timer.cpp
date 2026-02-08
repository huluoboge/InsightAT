
#include "timer.hpp"

#ifdef _WIN32
# include <windows.h>
#else
# include <sys/time.h>
#endif

#include <chrono>

using namespace std::chrono;
namespace insight{

#define HAVE_CXX11_CHRONO

Timer::Timer()
{
#ifdef HAVE_CXX11_CHRONO
#else
#ifdef _WIN32
	LARGE_INTEGER freq;
	if (!QueryPerformanceFrequency(&freq))
	{
		const char *msg = "Failed to initialize high resolution timer!";
		std::cerr << msg << std::endl;
		throw std::runtime_error(msg);
	}
	frequency_ = static_cast<double>(freq.QuadPart);
#endif
#endif
	reset();
}

void Timer::reset()
{
	started_ = false;
	paused_ = false;

// #ifdef HAVE_CXX11_CHRONO
// 	start_ = std::chrono::high_resolution_clock::now();
// 	started_ = true;
// 	paused_ = false;
// #else
// 
// #ifdef _WIN32
// 	LARGE_INTEGER li_start_;
// 	QueryPerformanceCounter(&li_start_);
// 	start_ = static_cast<double>(li_start_.QuadPart);
// #else
// 	timeval start;
// 	gettimeofday(&start, NULL);
// 	start_ = start.tv_sec + start.tv_usec * 1e-6;
// #endif
// 
// #endif // HAVE_CXX11_CHRONO
}


void Timer::pause()
{
	paused_ = true;
	pause_time_ = std::chrono::high_resolution_clock::now();
}

void Timer::start()
{
	started_ = true;
	paused_ = false;
	start_ = std::chrono::high_resolution_clock::now();
}


void Timer::resume()
{
	paused_ = false;
	start_ += std::chrono::high_resolution_clock::now() -pause_time_;
}


void Timer::restart()
{
	started_ = false;
	start();
}

double Timer::elapsed() const
{
	return elapsedMs() / 1e6;
}

double Timer::elapsedMs() const
{
	if (!started_) {
		return 0.0;
	}
	if (paused_) {
		return duration_cast<microseconds>(pause_time_ - start_).count();
	}
	else {
		return duration_cast<microseconds>(high_resolution_clock::now() -
			start_)
			.count();
	}
}


double Timer::elapsedHours() const
{
	return elapsedMinutes() / 60;
}

double Timer::elapsedMinutes() const
{
	return elapsed() / 60;
}

std::ostream& operator << (std::ostream& str, const Timer& t)
{
	return str << t.elapsed() << " s elapsed";
}
}//name space insight
