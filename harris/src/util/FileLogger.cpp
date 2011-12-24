/*
 * FileLogger
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#include "FileLogger.h"

#include <stdio.h>
#include <string.h>

void FileLogger::add(const int type, const int level, const char* fmt, va_list ap)
{
  char prefix[51];
  getPrefix(type, level, prefix, sizeof(prefix));

  fprintf(file,"%s", prefix);

  vfprintf(file, fmt, ap); /* Call vprintf */

  fprintf(file, "\n");

  fflush(file);
}
