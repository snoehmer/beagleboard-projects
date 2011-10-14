/*
 * Dsp.cpp
 *
 *  Created on: 26.08.2011
 *      Author: tom
 */



#include "Dsp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include "logger.h"
#include "dmm_buffer.h"
#include "dsp_bridge.h"


static bool done;
static int sig;

Dsp* Dsp::instance = NULL;

Dsp& Dsp::Instance()
{
  if(instance == NULL)
    instance = new Dsp();

  return *instance;
}

Dsp::Dsp()
{
  init = false;
}

Dsp::~Dsp()
{
  Destroy();
}


static void signal_handler(int signal)
{
  done = true;
  sig = signal;
}

void DspNode::Create(void)
{
  dsp_node *node;

  Logger::info(Logger::DSP, "DspNode::Create()");

  if (!dsp_register(dsp_handle, &this->uuid, DSP_DCD_LIBRARYTYPE, this->path))
  {
    Logger::error(Logger::DSP, "dsp_register(dsp_handle, this->uuid, DSP_DCD_LIBRARYTYPE, %s) failed", this->path);;
    throw DspException("failed to register node");
  }

  if (!dsp_register(dsp_handle, &this->uuid, DSP_DCD_NODETYPE, this->path))
  {
    Logger::error(Logger::DSP, "dsp_register(dsp_handle, this->uuid, DSP_DCD_LIBRARYTYPE, %s) failed", this->path);;
    throw DspException("failed to register node");
  }


  if (!dsp_node_allocate(dsp_handle, proc, &this->uuid, NULL, NULL, &node)) {
    Logger::error(Logger::DSP, "dsp node allocate failed");
    throw DspException("dsp node allocate failed");
  }

  if (!dsp_node_create(dsp_handle, node)) {
     Logger::error(Logger::DSP, "dsp node create failed");
     throw DspException("dsp node create failed");
  }

  Logger::info(Logger::DSP, "dsp node created");

  this->node = node;
}

void DspNode::Destroy(void)
{
  Logger::info(Logger::DSP, "DspNode::Destroy()");

  if (node)
  {
    if (!dsp_node_free(dsp_handle, node))
    {
       Logger::error(Logger::DSP, "dsp node free failed");
       throw DspException("dsp node create failed");
    }

    Logger::info(Logger::DSP, "dsp node deleted");
  }
  else
    Logger::warn(Logger::DSP, "DspNode::Destroy() called even if no node was allocated!");
}

void DspNode::Run(void)
{
  Logger::info(Logger::DSP, "DspNode::Run()");
  if (!dsp_node_run(dsp_handle, node)) {
    Logger::error(Logger::DSP, "dsp node run failed");
    throw DspException("dsp node run failed");
  }
}

void DspNode::Terminate(void)
{
  unsigned long int exit_status;


  Logger::info(Logger::DSP, "DspNode::Terminate()");

  if (!dsp_node_terminate(dsp_handle, node, &exit_status)) {
    Logger::error(Logger::DSP, "dsp node terminate failed, exit_status:%d", exit_status);
    throw DspException("dsp node terminate failed");
  }
}

void DspNode::SendMessage(uint32_t cmd, uint32_t arg1, uint32_t arg2, unsigned int timeout)
{
  dsp_msg msg;

  msg.cmd = cmd;
  msg.arg_1 = arg1;
  msg.arg_2 = arg2;

  SendMessage(msg, timeout);
}



void DspNode::SendMessage(dsp_msg msg, unsigned int timeout)
{
  Logger::info(Logger::DSP, "DspNode::SendMessage(cmd=%d, arg_1=%d, arg_2=%d, timeout=%d)", msg.cmd, msg.arg_1, msg.arg_2, timeout);

  bool ret = dsp_node_put_message(this->dsp_handle, this->node, &msg, timeout);

  if(!ret)
  {
    Logger::error(Logger::DSP, "sending message failed");
    throw DspException("sending message failed");
  }
}

dsp_msg DspNode::GetMessage()
{
  Logger::info(Logger::DSP, "DspNode::GetMessage()");

  dsp_msg msg;
  bool ret = dsp_node_get_message(dsp_handle, node, &msg, -1);

  if(!ret)
  {
    Logger::error(Logger::DSP, "getting message failed");
    throw DspException("getting message failed");
  }

  Logger::debug(Logger::DSP, "DspNode::received message(cmd=%d, arg_1=%d, arg_2=%d)", msg.cmd, msg.arg_1, msg.arg_2);
  return msg;
}

void Dsp::Destroy()
{
  Logger::info(Logger::DSP, "Dsp::Destroy()");
  init = false;

  DestroyNode();

  if (proc)
  {
    if (!dsp_detach(dsp_handle, proc))
    {
      Logger::error(Logger::DSP, "dsp detach failed");
      throw DspException("dsp detach failed");
    }
    proc = NULL;
  }

  if (dsp_handle > 0)
  {
    if (dsp_close(dsp_handle) < 0)
    {
      Logger::error(Logger::DSP, "dsp close failed");
      throw DspException("dsp close failed");
    }
    dsp_handle = 0;
  }
}



void Dsp::Init()
{
  if(init)
    Logger::warn(Logger::DSP, "calling Dsp::Init(), even if already initialised");

  signal(SIGINT, signal_handler);


  dsp_handle = dsp_open();

  if (dsp_handle < 0)
  {
    Logger::error(Logger::DSP, "dsp open failed");
    throw DspException("dsp open failed");
  }

  Logger::debug(Logger::DSP, "calling dsp_attach(%d, 0, NULL, &proc)", dsp_handle);
  if (!dsp_attach(dsp_handle, 0, NULL, &proc))
  {
    Logger::error(Logger::DSP, "dsp attach failed");
    throw DspException("dsp attach failed");
  }

  init = true;
}

DspNode& Dsp::CreateNode(dsp_uuid uuid, const char *path)
{
  if(!init)
    Init();

  node = new DspNode(uuid, path, dsp_handle, proc);

  node->Create();

  return *node;
}

void Dsp::DestroyNode()
{
  if(node)
    node->Destroy();

  node = NULL;
}

DmmManager dmmManager;

void* dsp_malloc(size_t n)
{
  Logger::debug(Logger::DSP, "dsp_malloc(%d)", n);

  dmm_buffer* buf = dmm_buffer_new(Dsp::Instance().GetHandle(), Dsp::Instance().GetProc(), DMA_BIDIRECTIONAL);
  dmm_buffer_allocate(buf, n);

  dmmManager.Add(buf);

  return buf->data;
}

void* dsp_memalign(size_t boundary, size_t n)
{
  Logger::debug(Logger::DSP, "dsp_malloc(%d)", n);

  dmm_buffer* buf = dmm_buffer_new(Dsp::Instance().GetHandle(), Dsp::Instance().GetProc(), DMA_BIDIRECTIONAL);

  buf->alignment = boundary;
  dmm_buffer_allocate(buf, n);

  dmmManager.Add(buf);

  return buf->data;
}

void* dsp_realloc(void *ptr, size_t n)
{
  Logger::debug(Logger::DSP, "dsp_realloc(0x%x, %d)", ptr, n);

  //TODO: this is a very stupid implementation!!!!

  dmm_buffer* buf = dmm_buffer_new(Dsp::Instance().GetHandle(), Dsp::Instance().GetProc(), DMA_BIDIRECTIONAL);
  dmm_buffer_allocate(buf, n);

  dmmManager.Add(buf);

  dmm_buffer* old_buf = dmmManager.GetDMMBuffer(ptr);

  memcpy(buf->data, old_buf->data, old_buf->size);

  //free the old buffer!
  dmmManager.Remove(ptr);
  dmm_buffer_free(old_buf);

  return buf->data;
}

void* dsp_calloc(size_t n, size_t size)
{
  Logger::debug(Logger::DSP, "dsp_calloc(%d, %d)", n, size);

  dmm_buffer* buf = dmm_buffer_calloc(Dsp::Instance().GetHandle(), Dsp::Instance().GetProc(), n*size, DMA_BIDIRECTIONAL);

  dmmManager.Add(buf);

  return buf->data;
}

void dsp_free(void* ptr)
{
  Logger::debug(Logger::DSP, "dsp_free(%x)", ptr);

  dmm_buffer* buf = dmmManager.GetDMMBuffer(ptr);

  dmm_buffer_free(buf);

  dmmManager.Remove(ptr);
}

void* dsp_get_mapped_addr(void* ptr)
{
  Logger::debug(Logger::DSP, "dsp_get_mapped_addr(%x)", ptr);

  dmm_buffer* buf = dmmManager.GetDMMBuffer(ptr);

  if(buf == 0)
  {
    Logger::error(Logger::DSP, "dsp_get_mapped_addr(%x): not found!!!", ptr);
    return 0;
  }

  if(buf->map == NULL)
  {
    Logger::debug(Logger::DSP, "dsp_get_mapped_addr(%x): mapping addr", ptr);
    dmm_buffer_map(buf);
  }

  Logger::debug(Logger::DSP, "dsp_get_mapped_addr(%x)=%x", ptr, (int)buf->map + ((int)ptr - (int)buf->data));
  return (void*)((int)buf->map + ((int)ptr - (int)buf->data));
}

int dsp_dmm_buffer_begin(void* ptr)
{
  Logger::debug(Logger::DSP, "dsp_dmm_buffer_begin(%x)", ptr);

  dmm_buffer* buf = dmmManager.GetDMMBuffer(ptr);

  if(buf == 0)
  {
    Logger::error(Logger::DSP, "dsp_dmm_buffer_begin(%x): not found!!!", ptr);
    return -1;
  }

  if(buf->map == NULL)
  {
    Logger::warn(Logger::DSP, "dsp_dmm_buffer_begin(%x) not mapped => mapping addr", ptr);
    dmm_buffer_map(buf);
  }

  dmm_buffer_begin(buf, buf->size);

  return 0;
}

int dsp_dmm_buffer_end(void* ptr)
{
  Logger::debug(Logger::DSP, "dsp_dmm_buffer_end(%x)", ptr);

  dmm_buffer* buf = dmmManager.GetDMMBuffer(ptr);

  if(buf == 0)
  {
    Logger::error(Logger::DSP, "dsp_dmm_buffer_end(%x): not found!!!", ptr);
    return -1;
  }

  if(buf->map == NULL)
  {
    Logger::warn(Logger::DSP, "dsp_dmm_buffer_end(%x) not mapped => mapping addr", ptr);
    dmm_buffer_map(buf);
  }

  dmm_buffer_end(buf, buf->size);

  return 0;
}

int dsp_send_message(uint32_t cmd, uint32_t arg1, uint32_t arg2)
{
  try
  {
    Dsp::Instance().GetNode().SendMessage(cmd, arg1, arg2);
  }
  catch(DspException e)
  {
    return -1;
  }

  return 0;
}

dsp_msg_t dsp_get_message()
{
  try
  {
    //UGLY CAST!
    dsp_msg m = (Dsp::Instance().GetNode().GetMessage());
    dsp_msg_t * mp = (dsp_msg_t*)&m;

    return *(mp);
  }
  catch(DspException)
  {
    dsp_msg_t err;
    err.cmd = -1;
    return err;
  }
}
