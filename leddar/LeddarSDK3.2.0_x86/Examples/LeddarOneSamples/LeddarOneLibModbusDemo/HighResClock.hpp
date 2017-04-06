#pragma once

#include <chrono>

#ifdef _MSC_VER

#include <Windows.h>

// The implementation of the standard library provided with up to Visual Studio 2013 simply
// define std::chrono::high_resolution_clock as std::chrono::system_clock.
// The following class fixes the issue (taken from: http://stackoverflow.com/questions/16299029/resolution-of-stdchronohigh-resolution-clock-doesnt-correspond-to-measureme).
struct HighResClock
{
	typedef long long                               rep;
	typedef std::nano                               period;
	typedef std::chrono::duration<rep, period>      duration;
	typedef std::chrono::time_point<HighResClock>   time_point;
	static const bool is_steady = true;

	static time_point now();
};

const long long g_uFrequency = []() -> long long
{
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);
	return frequency.QuadPart;
}();

HighResClock::time_point HighResClock::now()
{
	LARGE_INTEGER count;
	QueryPerformanceCounter(&count);
	return time_point(duration(count.QuadPart * static_cast<rep>(period::den) / g_uFrequency));
}

#else
// Other platforms 
typedef std::chrono::high_resolution_clock HighResClock;

#endif
