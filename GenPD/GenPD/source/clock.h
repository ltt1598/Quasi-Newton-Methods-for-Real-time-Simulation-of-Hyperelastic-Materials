/*! \file clock.h

  \brief Provides a class for timing events with high precision.

  \author Eric Chan
  \date July 2003
*/

#ifndef __MMC_CLOCK_H__
#define __MMC_CLOCK_H__

#include "timer.h"

namespace mmc
{

    /*! \brief High precision clock for timing events. 
      
      This class is implemented on top of the Timer class.  Usually it is
      not necessary to use the Timer class directly; instead, use this
      class. */
    class Clock
    {
    public:
        /*! Construct a clock. */
        inline Clock ();

        /*! Reset the clock (0 time elapsed). */
        inline void reset ();

        /*! Call once per frame to update the internal clock state. */
        inline void inc ();

        /*! Returns the amount of time (in ms) elapsed between last
          two calls to \p inc(). */
        inline long queryInc () const;

        /*! Returns the amount of time (in ms) elapsed since clock creation or
          reset() was called. */
        inline long queryTime () const;

        /*! Pause the clock. */
        inline void pauseToggle ();

    private:
        Timer *timer_;
        MMC_TIME_TYPE inc_;
        MMC_TIME_TYPE curTime_;
        MMC_TIME_TYPE start_;
        bool paused_;
        double invFreq_;
    };

    //////////////////////////////////////////////////////////////////////
    //  Implementation.
    //

    inline
    Clock::Clock ()
        : paused_(false)
    {
        timer_ = new Timer;
        invFreq_ = timer_->getInvFreq();
        timer_->start();
        reset();
    }

    void
    Clock::reset ()
    {
        start_ = timer_->queryElapsed();
        inc_ = curTime_ = 0;
    }

    void
    Clock::inc ()
    {
        timer_->inc();
        if (!paused_)
        {
            inc_ = timer_->queryInc();
            curTime_ += inc_;
        }
        else
        {
            inc_ = 0;
        }
    }

    // Converts internal clock increment to long in milliseconds.
    long
    Clock::queryInc () const
    {
        return (long) (1000.0 * inc_ * invFreq_);
    }

    // Converts internal clock time to long in milliseconds.
    long
    Clock::queryTime () const
    {
        return (long) (1000.0 * curTime_ * invFreq_);
    }

    void
    Clock::pauseToggle ()
    {
        if (paused_)
        {
            paused_ = false;
        }
        else
        {
            paused_ = true;
            inc_ = 0;
        }
    }

} // namespace mmc

#endif // __MMC_CLOCK_H__

