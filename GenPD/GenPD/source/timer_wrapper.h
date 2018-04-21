// ---------------------------------------------------------------------------------//
// Copyright (c) 2015, Regents of the University of Pennsylvania                    //
// All rights reserved.                                                             //
//                                                                                  //
// Redistribution and use in source and binary forms, with or without               //
// modification, are permitted provided that the following conditions are met:      //
//     * Redistributions of source code must retain the above copyright             //
//       notice, this list of conditions and the following disclaimer.              //
//     * Redistributions in binary form must reproduce the above copyright          //
//       notice, this list of conditions and the following disclaimer in the        //
//       documentation and/or other materials provided with the distribution.       //
//     * Neither the name of the <organization> nor the                             //
//       names of its contributors may be used to endorse or promote products       //
//       derived from this software without specific prior written permission.      //
//                                                                                  //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND  //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED    //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           //
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY               //
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES       //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;     //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND      //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS    //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                     //
//                                                                                  //
// Contact Tiantian Liu (ltt1598@gmail.com) if you have any questions.              //
//----------------------------------------------------------------------------------//


#pragma once

#define USE_WINDOWS_HIGH_ACCURACY_TIMER
#define USE_GLOBAL_HEADER

#ifdef USE_GLOBAL_HEADER
#include "global_headers.h"
#else
typedef double ScalarType;
#endif

#ifdef USE_WINDOWS_HIGH_ACCURACY_TIMER
// high resolution timer, unit: microseconds
#include <stdio.h>
#include <windows.h>

typedef enum
{
	TIMER_OUTPUT_MICROSECONDS,
	TIMER_OUTPUT_MILLISECONDS,
	TIMER_OUTPUT_SECONDS,
	TIMER_OUTPUT_AUTO,
	TIMER_OUTPUT_TOTAL
}TimerOutputUnit;

class TimerWrapper
{
public:
	inline TimerWrapper() { QueryPerformanceFrequency(&m_frequency); }
	inline virtual ~TimerWrapper() {}

	// start timer
	inline void Tic() { QueryPerformanceCounter(&m_start); m_pause_time.QuadPart = 0; }
	// end timer
	inline void Toc() { QueryPerformanceCounter(&m_end); }
	// pause timer
	inline void Pause() { QueryPerformanceCounter(&m_pause_start); }
	// resume timer
	inline void Resume() { QueryPerformanceCounter(&m_pause_end); m_pause_time.QuadPart += (m_pause_end.QuadPart - m_pause_start.QuadPart); }
	inline long long Duration()
	{
		m_elapse.QuadPart = m_end.QuadPart - m_start.QuadPart - m_pause_time.QuadPart;
		//m_elapse.QuadPart *= 1000; // convert to milliseconds
		m_elapse.QuadPart *= 1000000; // convert to microseconds
		return m_elapse.QuadPart / m_frequency.QuadPart; // only divide the frequency AFTER converted to milli/micro seconds to avoid loss of accuracy
	}
	inline ScalarType DurationInSeconds()
	{
		m_elapse.QuadPart = m_end.QuadPart - m_start.QuadPart - m_pause_time.QuadPart;
		return (ScalarType) m_elapse.QuadPart / (ScalarType) m_frequency.QuadPart; // only divide the frequency AFTER converted to milli/micro seconds to avoid loss of accuracy
	}
	inline void Report(const char* msg, bool verbose = true, TimerOutputUnit output_format = TIMER_OUTPUT_AUTO)
	{
		if (verbose)
		{
			ScalarType d = (ScalarType)(Duration());
			switch (output_format)
			{
			case TIMER_OUTPUT_MICROSECONDS:
				printf("%s, time elapse: %.0lf microseconds.\n", msg, d);
				break;
			case TIMER_OUTPUT_MILLISECONDS:
				d = d * 0.001;
				printf("%s, time elapse: %.3lf milliseconds.\n", msg, d);
				break;
			case TIMER_OUTPUT_SECONDS:
				d = d * 0.000001;
				printf("%s, time elapse: %.6lf seconds.\n", msg, d);
				break;
			case TIMER_OUTPUT_AUTO:
				// print in different scales
				if (d < 1000)
				{
					printf("%s, time elapse: %.0lf microseconds.\n", msg, d);
				}
				else if (d < 1000000)
				{
					d = d * 0.001;
					printf("%s, time elapse: %.1lf milliseconds.\n", msg, d);
				}
				else
				{
					d = d * 0.000001;
					printf("%s, time elapse: %.1lf seconds.\n", msg, d);
				}
				break;
			default:
				break;
			}
		}
	}
	inline void TocAndReport(const char* msg, bool verbose = true, TimerOutputUnit output_format = TIMER_OUTPUT_MICROSECONDS)
	{
		Toc();
		Report(msg, verbose, output_format);
	}
protected:
	LARGE_INTEGER m_frequency;

	LARGE_INTEGER m_start;
	LARGE_INTEGER m_end;
	LARGE_INTEGER m_elapse;

	LARGE_INTEGER m_pause_start;
	LARGE_INTEGER m_pause_end;
	LARGE_INTEGER m_pause_time;
};

#else
// low resolution timer, unit: milliseconds
#include <time.h>
#include <iostream>

class TimerWrapper
{
public:
	inline TimerWrapper() { m_sec_per_clock = 1.0 / (ScalarType)CLOCKS_PER_SEC; }
	inline virtual ~TimerWrapper() {}

	// start timer
	inline void Tic() { m_start = clock(); m_pause_time = 0; }
	// end timer
	inline void Toc() { m_end = clock(); }
	// pause timer
	inline void Pause() { m_pause_start = clock(); }
	// resume timer
	inline void Resume() { m_pause_time += (clock() - m_pause_start); }
	inline ScalarType Duration() { return (m_end - m_start - m_pause_time)*m_sec_per_clock; }
	inline void Report(const char* msg, bool verbose = true)
	{
		if (verbose)
		{
			ScalarType d = Duration();
			std::cout << msg << ", time elapse: " << d << " seconds" << std::endl;
		}
	}
	inline void TocAndReport(const char* msg, bool verbose = true)
	{
		Toc();
		Report(msg, verbose);
	}

protected:
	clock_t m_start;
	clock_t m_end;
	clock_t m_pause_start;
	clock_t m_pause_time;
	ScalarType m_sec_per_clock;
};
#endif // USE_WINDOWS_HIGH_ACCURACY_TIMER
