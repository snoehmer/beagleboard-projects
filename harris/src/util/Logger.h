/*
 * debug.h
 *
 *  Created on: 15.08.2011
 *      Author: tom
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <vector>
#include <stdarg.h>
/**
 * Abstract class Logger
 */
class Logger
{
private:
  static std::vector<Logger*> loggers;
  int dbgLevel;

  static void addToAllLoggers(const int type, const int level, const char* fmt, va_list ap);

  static const int outputEna = 0x80000;


  static const int DEBUG = 0;
  static const int INFO = 1;
  static const int WARN = 2;
  static const int ERROR = 3;



protected:
  virtual void add(const int type, const int level, const char* fmt, va_list ap) = 0;

  Logger(int dbgLevel)  //protected Constructor -> Singleton, to allow also a file output logger (later) ...
  {
    this->dbgLevel = dbgLevel;
  }

  void getPrefix(const int type, const int level, char* prefix, int maxlen);

public:

  static const int MAIN        = 0x00001 | outputEna;
  static const int HARRIS      = 0x00002 | outputEna;
  static const int TIMEMEASURE = 0x00004;// | outputEna;
  static const int DSP         = 0x00008 | outputEna;
  static const int DMMMANGER   = 0x00010;// | outputEna;
  static const int NCC         = 0x00020 | outputEna;
  static const int IMG_BITSTR  = 0x00040 | outputEna;
  static const int NONMAX      = 0x00080 | outputEna;
  static const int GAUSS       = 0x00100 | outputEna;
  static const int RESULT      = 0x00200 | outputEna;
  static const int PERFORMANCE = 0x00400 | outputEna;


  static void debug(const int type, const char* fmt, ...);
  static void info(const int type, const char* fmt, ...);
  static void warn(const int type, const char* fmt, ...);
  static void error(const int type, const char* fmt, ...);

  static void init();
  static void cleanup();


  virtual ~Logger()
  {

  }
};

#endif /* LOGGER_H_ */
