/*! \file timer.h

  \brief Provides a class for timing application events.

  \author Eric Chan
  \date July 2003
*/

#ifndef __MMC_TIMER_H__
#define __MMC_TIMER_H__

#ifdef WIN32

/*! win32 implementation -- use OS-provided high-precision timer */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif // WIN32_LEAN_AND_MEAN
#include <windows.h>

typedef _int64 MMC_TIME_TYPE;

#else
#ifdef LINUX

#include <time.h>
#include <sys/time.h>
typedef double MMC_TIME_TYPE;

#else

/*! not win32 -- use C-provided clock timing methods; less precise, but
  but provide same basic functionality;

  \todo Other OS's may provide similar OS-specific high-precision
  timer routines, which are preferable to the clock() method. */
#include <time.h>
typedef clock_t MMC_TIME_TYPE;

#endif // LINUX
#endif // win32

namespace mmc
{

    /*! \brief Class for timing application events.

     Only one of these is needed per application. */
    class Timer
    {
    public:

        /*! Get a new timer, initially stopped. */
        inline Timer ();

        /*! Start the timer. */
        inline void start ();

        /*! Call this once per frame to advance internal timer state. */
        inline void inc ();

        /*! Returns number of ms elapsed since start() was called.  Use
          this to determine how long the timer has been running. */
        inline MMC_TIME_TYPE queryElapsed () const;

        /*! Returns number of ms elapsed between the two previous calls to
          \p inc().  Use this to determine the time between frames. */
        inline MMC_TIME_TYPE queryInc () const;

        /*! Returns 1 / frequency of the internal timing clock actually
          being used. */
        inline double getInvFreq () const;

    private:
        MMC_TIME_TYPE freq_;
        MMC_TIME_TYPE start_;
        MMC_TIME_TYPE now_;
        MMC_TIME_TYPE last_;
        MMC_TIME_TYPE elapsed_;
        double invFreq_;
    };

    //////////////////////////////////////////////////////////////////////
    //  Implementation.
    //

    inline
    Timer::Timer ()
        : start_(0), last_(0), now_(0), elapsed_(0)
    {
#ifdef WIN32
        QueryPerformanceFrequency((LARGE_INTEGER *) &freq_);
#else
#ifdef LINUX
        freq_ = 1.0;
#else
        freq_ = CLOCKS_PER_SEC;
#endif        
#endif
        invFreq_ = 1.0 / (double) freq_;
    }

    inline void
    Timer::start ()
    {
#ifdef WIN32
        QueryPerformanceCounter((LARGE_INTEGER *) &start_);
#else
#ifdef LINUX
        struct timeval tv;
        struct timezone tz;
        gettimeofday(&tv, &tz);
        start_ = (double) (tv.tv_sec + 1e-6 * tv.tv_usec);
#else
        start_ = clock();
#endif
#endif
        last_ = now_ = start_;
    }

    inline MMC_TIME_TYPE
    Timer::queryElapsed () const
    {
        return now_ - start_;
    }

    inline void
    Timer::inc ()
    {
        last_ = now_;
#ifdef WIN32
        QueryPerformanceCounter((LARGE_INTEGER *) &now_);
#else
#ifdef LINUX
        struct timeval tv;
        struct timezone tz;
        gettimeofday(&tv, &tz);
        now_ = (double) (tv.tv_sec + 1e-6 * tv.tv_usec);        
#else
        now_ = clock();
#endif
#endif
    }

    inline MMC_TIME_TYPE
    Timer::queryInc () const
    {
        return now_ - last_;
    }

    inline double
    Timer::getInvFreq () const
    {
        return invFreq_;
    }

} // namespace mmc

#endif // __MMC_TIMER_H__

