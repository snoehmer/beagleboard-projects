/*
 * FixedArithmetic.h
 *
 *  Created on: 29.12.2011
 *      Author: sn
 */

#ifndef FIXEDARITHMETIC_H_
#define FIXEDARITHMETIC_H_

#include <math.h>

/**
 * this header file provides some functions for handling
 * fixed point arithmetic, especially on the ARM
 */

class Fixed
{
public:

  inline Fixed()
  {
    q_ = 15;
    value_ = 0;
  }

  inline Fixed(float f, int q = 15)
  {
    q_ = q;
    value_ = (int) (f * (float)(1 << q));
  }

  inline Fixed(int i, int q = 15)
  {
    q_ = q;
    value_ = i * (1 << q);
  }

  virtual ~Fixed() {}

  inline Fixed convert(int q)
  {
    if(q != q_)
    {
      Fixed ret;

      if(q_ > q)
        ret.value_ = value_ << (q_ - q);
      else
        ret.value_ = value_ >> (q - q_);

      ret.q_ = q;

      return ret;
    }
    else
      return *this;
  }

  inline Fixed operator =(Fixed right)
  {
    q_ = right.q_;
    value_ = right.value_;
    return *this;
  }

  inline Fixed operator +(Fixed right)
  {
    Fixed ret = right.convert(q_);
    ret.value_ += value_;
    return ret;
  }

  inline Fixed operator -(Fixed right)
  {
    Fixed ret = right.convert(q_);
    ret.value_ = value_ - ret.value_;
    return ret;
  }

  inline Fixed operator *(Fixed right)
  {
    Fixed ret = right.convert(q_);
    ret.value_ = (int) (((long)value_ * (long)ret.value_) >> q_);
    return ret;
  }

  inline Fixed operator /(Fixed right)
  {
    Fixed ret = right.convert(q_);
    ret.value_ = (int) ((((long)value_) << q_) / ((long)ret.value_));
    return ret;
  }

  inline Fixed operator +=(Fixed right)
  {
    value_ += right.convert(q_).value_;
    return *this;
  }

  inline Fixed operator -=(Fixed right)
  {
    value_ -= right.convert(q_).value_;
    return *this;
  }

  inline Fixed operator *=(Fixed right)
  {
    value_ = (int) (((long)value_ * (long)right.convert(q_).value_) >> q_);
    return *this;
  }

  inline Fixed operator /=(Fixed right)
  {
    value_ = (int) ((((long)value_) << q_) / ((long)right.convert(q_).value_));
    return *this;
  }


  inline Fixed operator =(int right)
  {
    value_ = right * (1 << q_);
    return *this;
  }

  inline Fixed operator +(int right)
  {
    Fixed ret;
    ret.q_ = q_;
    ret.value_ = value_ + right * (1 << q_);
    return ret;
  }

  inline Fixed operator -(int right)
  {
    Fixed ret;
    ret.q_ = q_;
    ret.value_ = value_ - right * (1 << q_);
    return ret;
  }

  inline Fixed operator *(int right)
  {
    Fixed ret;
    ret.q_ = q_;
    ret.value_ = value_ * right;
    return ret;
  }

  inline Fixed operator /(int right)
  {
    Fixed ret;
    ret.q_ = q_;
    ret.value_ = value_ / right;
    return ret;
  }

  inline Fixed operator +=(int right)
  {
    value_ += right * (1 << q_);
    return *this;
  }

  inline Fixed operator -=(int right)
  {
    value_ -= right * (1 << q_);
    return *this;
  }

  inline Fixed operator *=(int right)
  {
    value_ *= right;
    return *this;
  }

  inline Fixed operator /=(int right)
  {
    value_ /= right;
    return *this;
  }


  inline Fixed operator =(float right)
  {
    value_ = (int)(right * (float)(1 << q_));
    return *this;
  }

  inline Fixed operator +(float right)
  {
    //Fixed ret;
    //ret.q_ = q_;
    //ret.value_ = value_ + (int)(right * (float)(1 << q_));
    return *this + Fixed(right);
  }

  inline Fixed operator -(float right)
  {
    //Fixed ret;
    //ret.q_ = q_;
    //ret.value_ = value_ - (int)(right * (float)(1 << q_));
    return *this - Fixed(right);
  }

  inline Fixed operator *(float right)
  {
    return *this * Fixed(right);
  }

  inline Fixed operator /(float right)
  {
    return *this / Fixed(right);
  }

  inline Fixed operator +=(float right)
  {
    *this += Fixed(right);
    return *this;
  }

  inline Fixed operator -=(float right)
  {
    *this -= Fixed(right);
    return *this;
  }

  inline Fixed operator *=(float right)
  {
    *this *= Fixed(right);
    return *this;
  }

  inline Fixed operator /=(float right)
  {
    *this /= Fixed(right);
    return *this;
  }


  inline Fixed square()
  {
    return *this * *this;
  }

  inline Fixed sqrt()
  {
    Fixed ret = *this;
    ret.value_ = ::sqrt(((long)value_) << q_);
    return ret;
  }


  inline int toInt()
  {
    return round(this->toFloat());
  }

  inline float toFloat()
  {
    return ((float) value_) / ((float) (1 << q_));
  }


private:

  int value_;
  int q_;
};

#endif /* FIXEDARITHMETIC_H_ */
