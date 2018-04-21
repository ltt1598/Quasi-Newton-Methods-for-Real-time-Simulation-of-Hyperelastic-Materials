/*! \file fps.h

  \brief Defines a class for keeping track of frames/sec.

  \author Eric Chan
  \date July 2003
*/

#ifndef __MMC_FPS_H__
#define __MMC_FPS_H__

#include <vector>

namespace mmc
{
    class Clock;

    /*! \brief Utility class for measuring framerate. */
    class FpsTracker
    {
    public:
        /*! Create a FpsTracker.  \p smoothSteps is the window size
          over which to average the time measurements. */
        FpsTracker (int smoothSteps = 4);

        /*! Deallocate a FpsTracker. */
        ~FpsTracker ();

        /*! Specify the window size \p smoothSteps over which the time
          measurements will be averaged. */
        void setNumSteps (int smoothSteps);

        /*! Makes a timestamp (i.e. takes a snapshot); measures
          time intervals between successful calls.  Usually, you want
          to call this function once per rendering loop. */
        void timestamp ();

        /*! Get the average FPS (averaged over \p smoothSteps
          interval). */
        float fpsAverage () const;

        /*! Get the instantaneous FPS (estimated from the last frame
          only). */
        float fpsInstant () const;

    private:
        Clock *clock_;
        int steps_;
        int nSnaps_;
        long *snaps_;
    };

} // namespace mmc

#endif // __MMC_FPS_H__
