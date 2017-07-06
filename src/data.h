/* vim: set ts=2 sw=2 tw=0 noet :
   
   libcin : Driver for LBNL FastCCD 
   Copyright (c) 2014, Stuart B. Wilkins, Daron Chabot
   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met: 
   
   1. Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer. 
   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution. 
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   
   The views and conclusions contained in the software and documentation are those
   of the authors and should not be interpreted as representing official policies, 
   either expressed or implied, of the FreeBSD Project.

*/
#ifndef __CIN_DATA__H 
#define __CIN_DATA__H

#include "fifo.h"
#include "pthread.h"
#include "descramble.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Definitions */

#define MAX_THREADS             20
#define CIN_DATA_FRAMESTORE_NONE            0       /** Disable framestore mode */
#define CIN_DATA_FRAMESTORE_TRIGGER         1       /** Software triggering mode in framestore */
#define CIN_DATA_FRAMESTORE_SKIP            2      /** Framestore initial skip mode */

/* Datastructures */


typedef struct cin_data_packet {
  unsigned char *data;
  int size;
  struct timespec timestamp;
} cin_data_packet_t;

typedef struct cin_data_proc {
  // Input Function
  void* (*input_get) (void*, int);
  void* (*input_put) (void*, int);
  void* input_args;
  int reader;

  // Output Function
  void* (*output_put) (void*);
  void* (*output_get) (void*);
  void* output_args;

  // Thread Info

  // Parent pointer
  cin_data_t *parent;

} cin_data_proc_t;


/* Templates for functions */

int cin_data_init_port(cin_data_t *cin,
                       char* ipaddr, uint16_t port,
                       char* cin_ipaddr, uint16_t cin_port,
                       int rcvbuf);

/* Thread Handeling */

int cin_data_thread_start(cin_data_threads_t *thread, 
                          pthread_attr_t *attr,
                          void *(*func) (void *),
                          void *arg);
int cin_data_init_buffers(cin_data_t *cin, int packet_buffer_len, int frame_buffer_len);

/* UDP Port handeling */
int cin_data_read(cin_port_t *dp, unsigned char* buffer);
int cin_data_write(cin_port_t *dp, char* buffer, int buffer_len);

/* Threads for processing stream */

void *cin_data_listen_thread(void *args);
void *cin_data_assembler_thread(void *args);
void *cin_data_descramble_thread(void *args);

/* Buffer Routines */

void* cin_data_buffer_push(void *arg);
void cin_data_buffer_pop(void *arg);


int cin_data_send_magic(cin_data_t *cin);
#ifdef __cplusplus
}
#endif

#endif
