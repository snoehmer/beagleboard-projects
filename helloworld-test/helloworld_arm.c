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

#include "dmm_buffer.h"
#include "dsp_bridge.h"
#include "log.h"

static unsigned long input_buffer_size = 0x1000;
static unsigned long output_buffer_size = 0x1000;
static bool done;

static int dsp_handle;
static void *proc;

static void signal_handler(int signal)
{
	done = true;
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
		pr_err("dsp node allocate failed");
		return NULL;
	}

	if (!dsp_node_create(dsp_handle, node)) {
		pr_err("dsp node create failed");
		return NULL;
	}

	pr_info("dsp node created");

	return node;
}

static inline bool destroy_node(struct dsp_node *node)
{
	if (node) {
		if (!dsp_node_free(dsp_handle, node)) {
			pr_err("dsp node free failed");
			return false;
		}

		pr_info("dsp node deleted");
	}

	return true;
}

static inline void configure_dsp_node(void *node, struct dmm_buffer *input_buffer, struct dmm_buffer *output_buffer)
{
	struct dsp_msg msg;

	msg.cmd = 0;
	msg.arg_1 = (uint32_t) input_buffer->map;
	msg.arg_2 = (uint32_t) output_buffer->map;
	dsp_node_put_message(dsp_handle, node, &msg, -1);
}

static bool run_task(struct dsp_node *node)
{
  char text_send[] = "Hello";
  char text_receive[31];
  
  unsigned long exit_status;
  
  struct dsp_msg msg;

	struct dmm_buffer *input_buffer;
	struct dmm_buffer *output_buffer;

	if (!dsp_node_run(dsp_handle, node)) {
		pr_err("dsp node run failed");
		return false;
	}

	pr_info("dsp node running");

  // init memory
	input_buffer = dmm_buffer_new(dsp_handle, proc, DMA_TO_DEVICE);
	output_buffer = dmm_buffer_new(dsp_handle, proc, DMA_FROM_DEVICE);

	dmm_buffer_allocate(input_buffer, input_buffer_size);
	dmm_buffer_allocate(output_buffer, output_buffer_size);

  dmm_buffer_map(input_buffer);
	dmm_buffer_map(output_buffer);

	configure_dsp_node(node, input_buffer, output_buffer);

	pr_info("now sending string \"%s\" to DSP", text_send);

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
	  pr_err("returned message has wrong command!");
	else
	{
	  memcpy(text_receive, output_buffer->data, 31);
	  pr_info("received string \"%s\" from DSP", text_receive);
	  
	  printf("%s\n", text_receive);
	}
	  
	dmm_buffer_end(input_buffer, input_buffer->size);
	dmm_buffer_end(output_buffer, output_buffer->size);

	dmm_buffer_unmap(output_buffer);
	dmm_buffer_unmap(input_buffer);

	dmm_buffer_free(output_buffer);
	dmm_buffer_free(input_buffer);

	if (!dsp_node_terminate(dsp_handle, node, &exit_status)) {
		pr_err("dsp node terminate failed: %lx", exit_status);
		return false;
	}

	pr_info("dsp node terminated");

	return true;
}


int main(int argc, const char **argv)
{
	struct dsp_node *node;
	int ret = 0;

	signal(SIGINT, signal_handler);

#ifdef DEBUG
	debug_level = 3;
#endif

	dsp_handle = dsp_open();

	if (dsp_handle < 0) {
		pr_err("dsp open failed");
		return -1;
	}

	if (!dsp_attach(dsp_handle, 0, NULL, &proc)) {
		pr_err("dsp attach failed");
		ret = -1;
		goto leave;
	}

	node = create_node();
	if (!node) {
		pr_err("dsp node creation failed");
		ret = -1;
		goto leave;
	}

	run_task(node);
	
	destroy_node(node);

leave:
	if (proc) {
		if (!dsp_detach(dsp_handle, proc)) {
			pr_err("dsp detach failed");
			ret = -1;
		}
		proc = NULL;
	}

	if (dsp_handle > 0) {
		if (dsp_close(dsp_handle) < 0) {
			pr_err("dsp close failed");
			return -1;
		}
	}

	return ret;
}

