/*
 * ConsoleLogger.cpp
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#include "ConsoleLogger.h"

#include <stdio.h>
#include <string.h>

void ConsoleLogger::add(const int type, const int level, const char* fmt, va_list ap)
{
  char prefix[31];
  getPrefix(type, level, prefix, sizeof(prefix));

  printf("%s", prefix);

  vprintf(fmt, ap); /* Call vprintf */
  printf("\n");
}
