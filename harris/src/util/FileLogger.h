/*
 * FileLogger.h
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#ifndef FILELOGGER_H_
#define FILELOGGER_H_

#include "Logger.h"
#include <stdarg.h>
#include <stdio.h>

class FileLogger : public Logger
{
  int dbgLevel;
  const char* filename;
  FILE* file;

  void add(const int type, const int level, const char* fmt, va_list ap);
public:
  FileLogger(int dbgLevel, const char* filename) : Logger(dbgLevel)
  {
    this->filename = filename;
    file = fopen(filename, "w");

    if(file == 0)
    {
      printf("couldn't open logfile!!!\n");
      throw 0;
    }
  }

  virtual ~FileLogger()
  {
    if(file)
      fclose(file);
  }

};

#endif /* FILELOGGER_H_ */
