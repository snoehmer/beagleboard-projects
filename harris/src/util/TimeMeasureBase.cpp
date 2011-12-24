/*
 * TimeMeasureBase.cpp
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#include "TimeMeasureBase.h"
#include "SystemTimeMeasure.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Logger.h"


TimeMeasureBase* TimeMeasureBase::instance = 0;

TimeMeasureBase* TimeMeasureBase::getInstance()
{
  if(instance == 0)
    instance = new SystemTimeMeasure();

  return instance;
}


TimeMeasureBase::TimeMeasureBase()
{
  first = true;
}

TimeMeasureBase::~TimeMeasureBase()
{
  // TODO Auto-generated destructor stub
}


void TimeMeasureBase::startTimer(const char *identifier)
{
  Logger::debug(Logger::TIMEMEASURE, "startTimer called");
  if(first)
  {
    Logger::info(Logger::TIMEMEASURE, "initializing...");
    first = false;
    init();
  }

  TimeMeasureObject* obj = getTimeMeasureObjectByIdentifier(identifier);

  if(obj == NULL)
  {
    //Create a new object:
    TimeMeasureObject newobj(identifier);
    newobj.callCount = 0;
    newobj.totalTime.tv_sec = 0;
    newobj.totalTime.tv_usec = 0;
    newobj.running = false;
    Logger::debug(Logger::TIMEMEASURE, "startTimer: creating new TimeMeasureObject");
    timers.push_back(newobj);

    obj = getTimeMeasureObjectByIdentifier(identifier);
  }

  if(obj->running == true)
  {
    Logger::warn(Logger::TIMEMEASURE, "startTimer called, EVEN if object still running!");
    //Ignore call and return:
    return;
  }

  obj->running = true;
  obj->lastStart = getCurrentTime();
}

void TimeMeasureBase::stopTimer(const char *identifier)
{
  timeval currentTime = getCurrentTime();

  Logger::debug(Logger::TIMEMEASURE, "stopTimer called");

  TimeMeasureObject* obj = getTimeMeasureObjectByIdentifier(identifier);

  if(obj == NULL)
  {
    Logger::warn(Logger::TIMEMEASURE, "stopTimer: Identifier %s not found!!!", identifier);
    return;
  }


  obj->totalTime.tv_sec += currentTime.tv_sec - obj->lastStart.tv_sec;
  obj->totalTime.tv_usec += currentTime.tv_usec - obj->lastStart.tv_usec;
  obj->callCount++;

  obj->running = false;
}



timeval TimeMeasureBase::getTotalTime(const char *identifier)
{
  TimeMeasureObject* obj = getTimeMeasureObjectByIdentifier(identifier);

  if(obj == NULL)
  {
    Logger::warn(Logger::TIMEMEASURE, "getCallCount: Identifier %s not found!!!", identifier);
    return timeval();
  }

  return obj->totalTime;
}



int TimeMeasureBase::getCallCount(const char *identifier)
{
  TimeMeasureObject* obj = getTimeMeasureObjectByIdentifier(identifier);

  if(obj == NULL)
  {
    Logger::warn(Logger::TIMEMEASURE, "getCallCount: Identifier %s not found!!!", identifier);
    return -1;
  }

  return obj->callCount;
}



void TimeMeasureBase::printStatistic()
{
  printf("identifier        | totaltime(ms)    | callcount    | mean duration\n");
  printf("-----------------------------------------------------------------\n");
  for(unsigned i = 0; i < timers.size(); i++)
  {
    float test = timers[i].totalTime.tv_usec;
    float totalTime = timers[i].totalTime.tv_sec*1000 + test/1000.0;
    printf("%25s |%17f |%13d |%f\n", timers[i].identifier, totalTime, (int)timers[i].callCount, totalTime/timers[i].callCount);
  }
}


TimeMeasureObject* TimeMeasureBase::getTimeMeasureObjectByIdentifier(const char* identifier)
{
  for(unsigned i = 0; i < timers.size(); i++)
  {
    if(strcmp(timers[i].identifier, identifier) == 0)
      return &timers[i];
  }

  return NULL;
}


