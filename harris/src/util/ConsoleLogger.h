/*
 * ConsoleLogger.h
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#ifndef CONSOLELOGGER_H_
#define CONSOLELOGGER_H_

#include "Logger.h"
#include <stdarg.h>

class ConsoleLogger : public Logger
{
  int dbgLevel;

  void add(const int type, const int level, const char* fmt, va_list ap);
public:
  ConsoleLogger(int dbgLevel) : Logger(dbgLevel)
  {

  }

  virtual ~ConsoleLogger()
  {

  }

};

#endif /* CONSOLELOGGER_H_ */
