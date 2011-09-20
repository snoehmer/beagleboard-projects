/*
 * dsphellow.cpp
 *
 *  Created on: 28.08.2011
 *      Author: tom
 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <ctime>

#include "../../lib/arm/generic-driver.h"

#include "../../lib/arm/sift.h"
#include "../../lib/arm/logger.h"
#include "../../lib/arm/Dsp.h"
#include "../../lib/arm/TimeMeasureBase.h"

extern DmmManager dmmManager;

#define BUFLEN 20

int main()
{
  Logger::init();

  const struct dsp_uuid helloworld_uuid = { 0x3dac26d0, 0x6d4b, 0x11dd, 0xad, 0x8b, { 0x08, 0x00, 0x20, 0x0c, 0x9a, 0x66 } };

  try
  {
    Dsp& dsp = Dsp::Instance();

    dsp.Init();

    DspNode& node = dsp.CreateNode(helloworld_uuid,"./helloworld.dll64P");

    node.Run();

    int* mydata = (int*)dsp_malloc(sizeof(int)*BUFLEN);

    int i;
    for(i = 0; i < BUFLEN; i++)
    {
      mydata[i] = i;
    }

    //TimeMeasureBase* m = TimeMeasureBase::getInstance();


    (uint32_t)dsp_get_mapped_addr(mydata);

    for(i = 0; i < BUFLEN; i++)
    {
      mydata[i] = i;
    }

    dmm_buffer_begin(dmmManager.GetDMMBuffer(mydata), BUFLEN*sizeof(int));

    node.SendMessage(4, (uint32_t)dsp_get_mapped_addr(mydata), BUFLEN);

    dsp_msg msg = node.GetMessage();


    dmm_buffer_end(dmmManager.GetDMMBuffer(mydata), BUFLEN*sizeof(int));

    for(i = 0; i < 10; i++)
    {
      printf("%3d;", mydata[i]);
    }

    printf("\n");

    dsp_free(mydata);



    /*
    char* input_buffer = (char*)dsp_malloc(21);
    char* output_buffer = (char*)dsp_malloc(21);

    dsp_msg msg;

    msg.cmd = 0;
    msg.arg_1 = (uint32_t) dsp_get_mapped_addr(input_buffer);
    msg.arg_2 = (uint32_t) dsp_get_mapped_addr(output_buffer);

    node.SendMessage(msg);

    strcpy(input_buffer, "bla");

    dmm_buffer_begin(dmmManager.GetDMMBuffer(input_buffer), 21);
    dmm_buffer_begin(dmmManager.GetDMMBuffer(output_buffer), 21);

    node.SendMessage(1, 3, 0);

    msg = node.GetMessage();

    printf("received a message %d\n", msg.cmd);

    dmm_buffer_end(dmmManager.GetDMMBuffer(input_buffer), 21);
    dmm_buffer_end(dmmManager.GetDMMBuffer(output_buffer), 21);


    printf("received msg:%s\n", output_buffer);*/

    node.Terminate();

    dsp.DestroyNode();

    dsp.Destroy();
  }
  catch(DspException &e)
  {
    printf("DspException: %s", e.getMessage());
  }
  return 0;
}
