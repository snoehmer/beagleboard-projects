/*
 * debug.c
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "logger.h"

#include "ConsoleLogger.h"
#include "FileLogger.h"

std::vector<Logger*> Logger::loggers;


void Logger::init()
{
  loggers.push_back(new ConsoleLogger(DEBUG));
  loggers.push_back(new FileLogger(DEBUG, "log/log"));
}


void Logger::getPrefix(const int type, const int level, char* prefix, int maxlen)
{
  char lvl[32];

  switch(level)
  {
    case DEBUG:
      strncpy(lvl, "DEBUG", sizeof(lvl));
      break;
    case INFO:
      strncpy(lvl, "\033[1mINFO\033[0m", sizeof(lvl));
      break;
    case WARN:
      strncpy(lvl, "\033[0;30;43mWARN\033[0m", sizeof(lvl));
      break;
    case ERROR:
      strncpy(lvl, "\033[0;30;41mERROR\033[0m", sizeof(lvl));
      break;
    default:
      strncpy(lvl, "UNDEFINED-LVL", sizeof(lvl));
      break;
  }

  switch(type)
  {
    case SIFT:
      snprintf(prefix, maxlen, "[\033[0;31mSIFT\033[0m        ] %s: ", lvl);
      break;
    case SIFTTEST:
      snprintf(prefix, maxlen, "[\033[0;32mSIFTTEST\033[0m    ] %s: ", lvl);
      break;
    case TIMEMEASURE:
      snprintf(prefix, maxlen, "[\033[0;33mTIMEMEASURE\033[0m ] %s: ", lvl);
      break;
    case DSP:
      snprintf(prefix, maxlen, "[\033[0;34mDSP\033[0m         ] %s: ", lvl);
      break;
    case DMMMANGER:
      snprintf(prefix, maxlen, "[\033[0;35mDMMMANGER\033[0m   ] %s: ", lvl);
      break;
    default:
      snprintf(prefix, maxlen, "[\033[0;31mUNKNOWN\033[0m     ] %s: ", lvl);
      break;
  }

}


void Logger::addToAllLoggers(const int type, const int level, const char* fmt, va_list ap)
{
  unsigned i;

  if(!(type&outputEna))
    return;

  for(i = 0; i < loggers.size(); i++)
  {
    if(level < loggers[i]->dbgLevel)
      continue;

    loggers[i]->add(type, level, fmt, ap);
  }
}

void Logger::debug(const int type, const char* fmt, ...)
{
  va_list ap;

  va_start(ap, fmt); /* Initialize the va_list */

  addToAllLoggers(type, DEBUG, fmt, ap);

  va_end(ap); /* Cleanup the va_list */
}

void Logger::info(const int type, const char* fmt, ...)
{
  va_list ap;

  va_start(ap, fmt); /* Initialize the va_list */

  addToAllLoggers(type, INFO, fmt, ap);

  va_end(ap); /* Cleanup the va_list */
}


void Logger::warn(const int type, const char* fmt, ...)
{
  va_list ap;

  va_start(ap, fmt); /* Initialize the va_list */

  addToAllLoggers(type, WARN, fmt, ap);

  va_end(ap); /* Cleanup the va_list */
}


void Logger::error(const int type, const char* fmt, ...)
{
  va_list ap;

  va_start(ap, fmt); /* Initialize the va_list */

  addToAllLoggers(type, ERROR, fmt, ap);

  va_end(ap); /* Cleanup the va_list */
}
