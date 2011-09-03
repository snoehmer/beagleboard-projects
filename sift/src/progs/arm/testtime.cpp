/*
 * testtime.c
 *
 *  Created on: 16.08.2011
 *      Author: tom
 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <ctime>
#include <sys/time.h>
#include <iostream>

#include "../../lib/arm/SystemTimeMeasure.h"
#include "../../lib/arm/logger.h"

using namespace std;


int main()
{
  //Logger::init();

  TimeMeasureBase& m = *TimeMeasureBase::getInstance();

  m.startTimer("total");
  float i;
  for(int a = 0; a < 20; a++)
  {
    m.startTimer("inloop");

    m.startTimer("A");
    for(i = 0; i < 10000; i++); printf("%f\n", i);
    m.stopTimer("A");

    m.startTimer("B");
    for(i = 0; i < 10000; i++); printf("%f\n", i);
    m.stopTimer("B");

    m.stopTimer("inloop");
  }

  m.stopTimer("total");

  m.printStatistic();
  return 0;
}
