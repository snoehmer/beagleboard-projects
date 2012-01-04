/*
 * debug.c
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "Logger.h"

#include "ConsoleLogger.h"
#include "FileLogger.h"

std::vector<Logger*> Logger::loggers;


void Logger::init()
{
  loggers.push_back(new ConsoleLogger(DEBUG));
  //loggers.push_back(new FileLogger(DEBUG, "output/log/log"));
}

void Logger::cleanup()
{
  for(unsigned int i = 0; i < loggers.size(); i++)
  {
    delete loggers[i];
  }

  loggers.clear();
}

/* colors:
 * black - 30
 * red - 31
 * green - 32
 * brown - 33
 * blue - 34
 * magenta - 35
 * cyan - 36
 * lightgray - 37
 */

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
    case MAIN:
      snprintf(prefix, maxlen, "[\033[0;31mMAIN\033[0m        ] %s: ", lvl);
      break;
    case HARRIS:
      snprintf(prefix, maxlen, "[\033[0;32mHARRIS\033[0m      ] %s: ", lvl);
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
    case NCC:
      snprintf(prefix, maxlen, "[\033[0;36mNCC\033[0m         ] %s: ", lvl);
      break;
    case IMG_BITSTR:
      snprintf(prefix, maxlen, "[\033[0;37mIMG_BITSTR\033[0m  ] %s: ", lvl);
      break;
    case NONMAX:
      snprintf(prefix, maxlen, "[\033[0;38mNONMAX\033[0m      ] %s: ", lvl);
      break;
    case GAUSS:
      snprintf(prefix, maxlen, "[\033[0;39mGAUSS\033[0m       ] %s: ", lvl);
      break;
    case RESULT:
      snprintf(prefix, maxlen, "[\033[0;40mRESULT\033[0m      ] %s: ", lvl);
      break;
    case PERFORMANCE:
      snprintf(prefix, maxlen, "[\033[0;41mPERFORMANCE\033[0m ] %s: ", lvl);
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
