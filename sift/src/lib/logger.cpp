/*
 * debug.c
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#include <stdio.h>
#include <stdarg.h>


#include "logger.h"

#include "ConsoleLogger.h"

std::vector<Logger*> Logger::loggers;



void Logger::getPrefix(const int type, const int level, char* prefix, int maxlen)
{
  char* lvl;

  switch(level)
  {
    case DEBUG:
      lvl = "DEBUG";
      break;
    case INFO:
      lvl = "INFO";
      break;
    case WARN:
      lvl = "WARN";
      break;
    case ERROR:
      lvl = "ERROR";
      break;
    default:
      lvl = "UNKNOWN-LVL";
      break;
  }

  switch(type)
  {
    case SIFT:
      snprintf(prefix, maxlen, "[SIFT        ] %s: ", lvl);
      break;
    case SIFTTEST:
      snprintf(prefix, maxlen, "[SIFTTEST    ] %s: ", lvl);
      break;
    default:
      snprintf(prefix, maxlen, "[UNKNOWN     ] %s: ", lvl);
      break;
  }

}


void Logger::init()
{
  loggers.push_back(new ConsoleLogger(DEBUG));
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
