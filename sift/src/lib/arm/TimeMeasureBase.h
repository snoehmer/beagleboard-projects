/*
 * TimeMeasureBase.h
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#ifndef TIMEMEASUREBASE_H_
#define TIMEMEASUREBASE_H_

#include <sys/time.h>                // for gettimeofday()
#include <vector>

using namespace std;

struct TimeMeasureObject
{
  TimeMeasureObject(const char* id) : identifier(id)
  {

  }

  const char* identifier;
  /**
   *  The total time, spent so far...
   *  if running = true: currentTime - lastStart hasn't been added yet.
   */
  timeval totalTime;

  /**
   * a counter which holds, how often a time-measuring has been triggered.
   */
  long callCount;

  /**
   * if running = true: the last time the Timer was started ...
   */
  timeval lastStart;

  /**
   * if a timer is running at the moment.
   */
  bool running;
};

class TimeMeasureBase
{
  std::vector<TimeMeasureObject> timers;
  bool first;

  TimeMeasureObject* getTimeMeasureObjectByIdentifier(const char* identifier);

  static TimeMeasureBase* instance;

protected:
  /**
   * will be called the first time a time-measuring starts...
   */
  virtual void init() = 0;

  /**
   * returns the current time in seconds and mseconds
   */
  virtual timeval getCurrentTime() = 0;

public:
  TimeMeasureBase();
  virtual ~TimeMeasureBase();

  /**
   * starts measuring
   */
  void startTimer(const char* identifier);

  /**
   * stops measuring
   */
  void stopTimer(const char* identifier);

  /**
   * returns the total time spent.
   */
  timeval getTotalTime(const char* identifier);

  int getCallCount(const char* identifier);

  void printStatistic();

  static TimeMeasureBase* getInstance();
};

#endif /* TIMEMEASUREBASE_H_ */
