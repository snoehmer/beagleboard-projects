/*
 * Exception.h
 *
 *  Created on: 27.08.2011
 *      Author: tom
 */

#ifndef EXCEPTION_H_
#define EXCEPTION_H_

class Exception
{
  const char* msg;
public:
  Exception(const char* msg)
  {
    this->msg = msg;
  }
  const char* getMessage()
  {
    return msg;
  }

  virtual ~Exception() {}
};

#endif /* EXCEPTION_H_ */
