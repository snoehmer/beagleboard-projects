/*
 * SystemTimeMeasure.h
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#ifndef SYSTEMTIMEMEASURE_H_
#define SYSTEMTIMEMEASURE_H_

#include "TimeMeasureBase.h"
#include <sys/time.h>

class SystemTimeMeasure : public TimeMeasureBase
{
  /**
   * will be called the first time a time-measuring starts...
   */
  virtual void init()
  {

  }

  /**
   * returns the current time in seconds and mseconds
   */
  virtual timeval getCurrentTime()
  {
    timeval t;

    gettimeofday(&t, NULL);

    return t;
  }
public:
  SystemTimeMeasure()
  {

  }

  virtual ~SystemTimeMeasure()
  {

  }
};

#endif /* SYSTEMTIMEMEASURE_H_ */
