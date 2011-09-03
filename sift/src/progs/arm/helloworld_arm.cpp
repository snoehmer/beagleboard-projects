/*
 * Copyright (C) 2008-2009 Nokia Corporation.
 *
 * Author: Felipe Contreras <felipe.contreras@nokia.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation
 * version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include "../../lib/arm/dmm_buffer.h"
#include "../../lib/arm/dsp_bridge.h"

static unsigned long input_buffer_size = 0x1000;
static unsigned long output_buffer_size = 0x1000;
static bool done;
static int sig;

static int dsp_handle;
static void *proc;


static void signal_handler(int signal)
{
	done = true;
	sig = signal;
}

static inline struct dsp_node* create_node(void)
{
	struct dsp_node *node;
	const struct dsp_uuid helloworld_uuid = { 0x3dac26d0, 0x6d4b, 0x11dd, 0xad, 0x8b, { 0x08, 0x00, 0x20, 0x0c, 0x9a, 0x66 } };

	if (!dsp_register(dsp_handle, &helloworld_uuid, DSP_DCD_LIBRARYTYPE, "./helloworld.dll64P"))
		return false;

	if (!dsp_register(dsp_handle, &helloworld_uuid, DSP_DCD_NODETYPE, "./helloworld.dll64P"))
		return false;

	if (!dsp_node_allocate(dsp_handle, proc, &helloworld_uuid, NULL, NULL, &node)) {
		return NULL;
	}

	if (!dsp_node_create(dsp_handle, node)) {
		return NULL;
	}

	printf("dsp node created\n");

	return node;
}

static inline bool destroy_node(struct dsp_node *node)
{
	if (node) {
		if (!dsp_node_free(dsp_handle, node)) {
			printf("dsp node free failed\n");
			return false;
		}

		printf("dsp node deleted\n");
	}

	return true;
}

static inline void configure_dsp_node(void *node, struct dmm_buffer *input_buffer, struct dmm_buffer *output_buffer)
{
	struct dsp_msg msg;

	msg.cmd = 0;
	msg.arg_1 = (uint32_t) input_buffer->map;
	msg.arg_2 = (uint32_t) output_buffer->map;
	dsp_node_put_message(dsp_handle, (dsp_node*)node, &msg, -1);
}

static bool run_task(struct dsp_node *node)
{
  char text_send[] = "Hello";
  char text_receive[31];
  
  struct dsp_msg msg;

	struct dmm_buffer *input_buffer;
	struct dmm_buffer *output_buffer;



  // init memory
	input_buffer = dmm_buffer_new(dsp_handle, proc, DMA_TO_DEVICE);
	output_buffer = dmm_buffer_new(dsp_handle, proc, DMA_FROM_DEVICE);

	dmm_buffer_allocate(input_buffer, input_buffer_size);
	dmm_buffer_allocate(output_buffer, output_buffer_size);

  dmm_buffer_map(input_buffer);
	dmm_buffer_map(output_buffer);

	configure_dsp_node(node, input_buffer, output_buffer);

	printf("now sending string \"%s\" to DSP\n", text_send);

  // copy data into input buffer
  memcpy(input_buffer->data, text_send, strlen(text_send) + 1);

	dmm_buffer_begin(input_buffer, input_buffer->size);
	dmm_buffer_begin(output_buffer, output_buffer->size);
	
	// tell DSP to do some work
	msg.cmd = 1;
	msg.arg_1 = input_buffer->size;
	dsp_node_put_message(dsp_handle, node, &msg, -1);
	dsp_node_get_message(dsp_handle, node, &msg, -1);
	
	if(msg.cmd != 2)
	  printf("returned message has wrong command!\n");
	else
	{
	  memcpy(text_receive, output_buffer->data, 31);
	  printf("received string \"%s\" from DSP\n", text_receive);
	  
	  printf("%s\n", text_receive);
	}
	  
	dmm_buffer_end(input_buffer, input_buffer->size);
	dmm_buffer_end(output_buffer, output_buffer->size);

	dmm_buffer_unmap(output_buffer);
	dmm_buffer_unmap(input_buffer);

	dmm_buffer_free(output_buffer);
	dmm_buffer_free(input_buffer);



	return true;
}


int main()
{
	struct dsp_node *node;
	int ret = 0;

	signal(SIGINT, signal_handler);

#ifdef DEBUG
	debug_level = 3;
#endif

	dsp_handle = dsp_open();

	if (dsp_handle < 0) {
		printf("dsp open failed\n");
		return -1;
	}

	if (!dsp_attach(dsp_handle, 0, NULL, &proc)) {
		printf("dsp attach failed\n");
		ret = -1;
		goto leave;
	}

	node = create_node();
	if (!node) {
		printf("dsp node creation failed\n");
		ret = -1;
		goto leave;
	}

  if (!dsp_node_run(dsp_handle, node)) {
    printf("dsp node run failed\n");
    return false;
  }

  printf("dsp node running\n");

	run_task(node);
	run_task(node);
	unsigned long exit_status;
  if (!dsp_node_terminate(dsp_handle, node, &exit_status)) {
    printf("dsp node terminate failed: %lx\n", exit_status);
    return 0;
  }

  printf("dsp node terminated\n");

	destroy_node(node);

leave:
	if (proc) {
		if (!dsp_detach(dsp_handle, proc)) {
			printf("dsp detach failed\n");
			ret = -1;
		}
		proc = NULL;
	}

	if (dsp_handle > 0) {
		if (dsp_close(dsp_handle) < 0) {
			printf("dsp close failed\n");
			return -1;
		}
	}

	return ret;
}

