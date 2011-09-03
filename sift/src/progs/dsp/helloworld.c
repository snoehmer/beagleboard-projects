/*
 * Copyright (C) 2008-2009 Nokia Corporation
 * Copyright (C) 2009 Igalia S.L
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

#include <stddef.h>
#include "../../lib/common/node.h"

unsigned int dsp_helloworld_create(void)
{
	return 0x8000;
}

unsigned int dsp_helloworld_delete(void)
{
	return 0x8000;
}

unsigned int dsp_helloworld_execute(void *env)
{
	dsp_msg_t msg;
	char *input;
	char *output;
	unsigned char done = 0;
	unsigned int i, j;
	
	char text[] = " World!";

	while (!done) {
		NODE_getMsg(env, &msg, (unsigned) -1);

		switch (msg.cmd) {
		case 0:
			input = (char *) (msg.arg_1);
			output = (char *) (msg.arg_2);
			break;
		case 1:
			{
				unsigned int size;

				size = (unsigned int) (msg.arg_1);

				BCACHE_inv((void*) input, size, 1);
				
				for(i = 0; input[i]; i++)
				  output[i] = input[i];
				  
				for(j = 0; text[j]; j++)
				  output[i + j] = text[j];
				  
				output[i + j] = 0;
				
				BCACHE_wbInv((void*) output, size, 1);

        msg.cmd = 2;

				NODE_putMsg(env, NULL, &msg, 0);
				break;
			}
    case 4:
    {
      int* data = (int*)msg.arg_1;
      int length = (int)msg.arg_2;
      int i;

      BCACHE_inv((void*) data, length*sizeof(int), 1);

      for(i = 0; i < length; i++)
      {
        data[i] += 10;
      }


      BCACHE_wbInv((void*) data, length*sizeof(int), 1);

      msg.cmd = 42;
      NODE_putMsg(env, NULL, &msg, 0);
      break;
    }
		case 0x80000000:
			done = 1;
			break;
		}
	}

	return 0x8000;
}
