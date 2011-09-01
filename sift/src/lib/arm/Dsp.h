/*
 * Dsp.h
 *
 *  Created on: 26.08.2011
 *      Author: tom
 */

#ifndef DSP_H_
#define DSP_H_

#include "dmm_buffer.h"
#include "dsp_bridge.h"
#include "Exception.h"
#include <map>

#include "../common/node.h"

//this functions are just for the vlfeat library. (C - Functions ...)
void* dsp_malloc  (size_t n) ;
void* dsp_realloc (void *ptr, size_t n) ;
void* dsp_calloc  (size_t n, size_t size) ;
void  dsp_free    (void* ptr) ;
void* dsp_get_mapped_addr(void* ptr);
int dsp_dmm_buffer_begin(void* ptr);
int dsp_dmm_buffer_end(void* ptr);
dsp_msg_t dsp_get_message();
int dsp_send_message(uint32_t cmd, uint32_t arg1, uint32_t arg2);


class DspNode;

class DspException : public Exception
{
public:
  DspException(const char* msg) : Exception(msg)
  {
  }
};


class Dsp
{
  static Dsp* instance;
  int dsp_handle;
  void *proc;
  bool init;

  DspNode* node;

  Dsp();

public:

  ~Dsp();

  void Init();
  void Destroy();

  DspNode& CreateNode(dsp_uuid uuid, const char* path);
  void DestroyNode();
  DspNode& GetNode()
  {
    if(!init || node == NULL)
      throw DspException("You have to init the dsp and create a node before calling GetNode!");

    return *node;
  }

  int GetHandle()
  {
    if(!init)
      throw DspException("dsp not initialised");

    return dsp_handle;
  }

  void* GetProc()
  {
    if(!init)
      throw DspException("dsp not initialised");

    return proc;
  }

  static Dsp& Instance();
};

class DspNode
{
  dsp_uuid uuid;
  const char* path;
  int dsp_handle;
  dsp_node* node;
  void* proc;
public:
  DspNode(dsp_uuid uuid, const char* path, int dsp_handle, void* proc)
  {
    this->uuid = uuid;
    this->path = path;
    this->dsp_handle = dsp_handle;
    this->node = NULL;
    this->proc = proc;
  }

  void Create();
  void Destroy();
  void Run();
  void Terminate();

  void SendMessage(uint32_t cmd, uint32_t arg1, uint32_t arg2, unsigned int timeout = -1);
  void SendMessage(dsp_msg msg, unsigned int timeout = -1);
  dsp_msg GetMessage();
};

using namespace std;

class DmmManager
{
  map<void*, dmm_buffer*> allocated_mem;
public:
  void Add(dmm_buffer* buf)
  {
    allocated_mem[buf->data] = buf;
  }

  void Remove(void* addr)
  {
    allocated_mem.erase(addr);
  }

  dmm_buffer* GetDMMBuffer(void* addr)
  {
    //TODO: searching in case of addr points not to the first position
    // of allocated mem...
    return allocated_mem[addr];
  }
};

#endif /* DSP_H_ */
