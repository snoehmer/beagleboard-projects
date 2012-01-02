/*
 * FixedArithmetic.h
 *
 *  Created on: 29.12.2011
 *      Author: sn
 */

#ifndef FIXEDARITHMETIC_H_
#define FIXEDARITHMETIC_H_

#include <math.h>

#define FIXED_STD_Q 15

/**
 * this header file provides some functions for handling
 * fixed point arithmetic, especially on the ARM
 */

class Fixed
{
public:

  inline Fixed()
  {
    q_ = FIXED_STD_Q;
    value_ = 0;
  }

  inline Fixed(float f, int q = FIXED_STD_Q)
  {
    q_ = q;
    value_ = (int) (f * (float)(1 << q));
  }

  inline Fixed(int i, int q = FIXED_STD_Q)
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

  inline Fixed operator -()
  {
    Fixed ret;
    ret.value_ = - this->value_;
    ret.q_ = this->q_;
    return ret;
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
    ret.value_ = (int) (((long long)value_ * (long long)ret.value_) >> q_);
    return ret;
  }

  inline Fixed operator /(Fixed right)
  {
    Fixed ret = right.convert(q_);
    ret.value_ = (int) ((((long long)value_) << q_) / ((long long)ret.value_));
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
    value_ = (int) (((long long)value_ * (long long)right.convert(q_).value_) >> q_);
    return *this;
  }

  inline Fixed operator /=(Fixed right)
  {
    value_ = (int) ((((long long)value_) << q_) / ((long long)right.convert(q_).value_));
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

    if(this->value_ >= 0)
      ret.value_ = ::sqrt(((long long)value_) << q_);
    else
      printf("WARNING: FixedArithmetic.h: sqrt of negative number!\n");

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


  // relational operators
  inline bool operator >(Fixed right)
  {
    if(this->value_ > right.convert(this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator <(Fixed right)
  {
    if(this->value_ < right.convert(this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator >=(Fixed right)
  {
    if(this->value_ >= right.convert(this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator <=(Fixed right)
  {
    if(this->value_ <= right.convert(this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator ==(Fixed right)
  {
    if((this->value_ - right.convert(this->q_).value_) <= 1)  // 1 is the smallest possible value (epsilon) for a Fixed
      return true;
    else
      return false;
  }

  inline bool operator !=(Fixed right)
  {
    return !(*this == right);
  }

  inline bool operator >(int right)
  {
    if(this->value_ > Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator <(int right)
  {
    if(this->value_ < Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator >=(int right)
  {
    if(this->value_ >= Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator <=(int right)
  {
    if(this->value_ <= Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator ==(int right)
  {
    if((this->value_ - Fixed(right, this->q_).value_) <= 1)  // 1 is the smallest possible value (epsilon) for a Fixed
      return true;
    else
      return false;
  }

  inline bool operator !=(int right)
  {
    return !(*this == right);
  }

  inline bool operator >(float right)
  {
    if(this->value_ > Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator <(float right)
  {
    if(this->value_ < Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator >=(float right)
  {
    if(this->value_ >= Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator <=(float right)
  {
    if(this->value_ <= Fixed(right, this->q_).value_)
      return true;
    else
      return false;
  }

  inline bool operator ==(float right)
  {
    if((this->value_ - Fixed(right, this->q_).value_) <= 1)  // 1 is the smallest possible value (epsilon) for a Fixed
      return true;
    else
      return false;
  }

  inline bool operator !=(float right)
  {
    return !(*this == right);
  }


  // friend operators
  friend inline Fixed operator +(int left, Fixed right)
  {
    return right + left;
  }

  friend inline Fixed operator -(int left, Fixed right)
  {
    Fixed ret(left, right.q_);
    ret -= right;
    return ret;
  }

  friend inline Fixed operator *(int left, Fixed right)
  {
    return right * left;
  }

  friend inline Fixed operator /(int left, Fixed right)
  {
    Fixed ret(left, right.q_);
    ret /= right;
    return ret;
  }

  friend inline Fixed operator +(float left, Fixed right)
  {
    return right + left;
  }

  friend inline Fixed operator -(float left, Fixed right)
  {
    Fixed ret(left, right.q_);
    ret -= right;
    return ret;
  }

  friend inline Fixed operator *(float left, Fixed right)
  {
    return right * left;
  }

  friend inline Fixed operator /(float left, Fixed right)
  {
    Fixed ret(left, right.q_);
    ret /= right;
    return ret;
  }


  // special scale function to convert a uchar directly to a Fixed
  friend inline Fixed scale_uchar(unsigned int i, unsigned int q = FIXED_STD_Q);
  friend inline Fixed scale_uchar2(unsigned int i, unsigned int q = FIXED_STD_Q);


private:

  int value_;
  int q_;
};


// special scale function to convert a uchar directly to a Fixed
// a 0..255 uchar is converted to a 0..1 Fixed
inline Fixed scale_uchar(unsigned int i, unsigned int q)
{
  Fixed ret;
  ret.q_ = q;

  if(q > 8)
    ret.value_ = i << (q - 8);
  else
    ret.value_ = i >> (8 - q);

  return ret;
}

// special scale function to convert a squared uchar directly to a Fixed
// a (0..255)^2 uchar is converted to a 0..1 Fixed
inline Fixed scale_uchar2(unsigned int i, unsigned int q)
{
  Fixed ret;
  ret.q_ = q;

  if(q > 16)
    ret.value_ = i << (q - 16);
  else
    ret.value_ = i >> (16 - q);

  return ret;
}


#endif /* FIXEDARITHMETIC_H_ */