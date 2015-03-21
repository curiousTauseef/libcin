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

#include "mbuffer.h"
#include "fifo.h"
#include "pthread.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Definitions */

#define MAX_THREADS             20
#define CIN_DATA_MONITOR_UPDATE 100000 // in usec

/* Datastructures */

typedef struct cin_data_threads {
  pthread_t thread_id;
  int started;
} cin_data_threads_t;

typedef struct cin_data_callbacks {
  void* (*push) (cin_data_frame_t *);
  void* (*pop)  (cin_data_frame_t *);
  cin_data_frame_t *frame;
} cin_data_callbacks_t;

typedef struct cin_data_thread_data {
  /* FIFO Elements */
  fifo *packet_fifo;  
  fifo *frame_fifo;
  fifo *image_fifo;

  /* Image double buffer */
  mbuffer_t *image_dbuffer;

  /* Callbacks Buffer */

  cin_data_callbacks_t callbacks;

  /* Interface */
  struct cin_port* dp; 

  /* Statistics */
  struct timespec framerate;
  unsigned long int dropped_packets;
  unsigned long int mallformed_packets;
  uint16_t last_frame;

  /* Statistics */
  struct cin_data_stats stats;
  pthread_mutex_t stats_mutex;
  pthread_cond_t stats_signal;

} cin_data_thread_data_t;

typedef struct cin_data_packet {
  unsigned char *data;
  int size;
  struct timespec timestamp;
} cin_data_packet_t;

typedef struct cin_data_proc {
  void* (*input_get) (void*, int);
  void* (*input_put) (void*, int);
  void* input_args;
  int reader;

  void* (*output_put) (void*);
  void* (*output_get) (void*);
  void* output_args;
} cin_data_proc_t;


/* Templates for functions */

/* Thread Handeling */

int cin_data_thread_start(cin_data_threads_t *thread, 
                          pthread_attr_t *attr,
                          void *(*func) (void *),
                          void *arg);
int cin_data_init_buffers(int packet_buffer_len, int frame_buffer_len);

/* UDP Port handeling */
int cin_data_read(struct cin_port* dp, unsigned char* buffer);
int cin_data_write(struct cin_port* dp, char* buffer, int buffer_len);

/* Threads for processing stream */

void *cin_data_listen_thread(void *args);
void *cin_data_monitor_thread(void);
void *cin_data_assembler_thread(void *args);
void *cin_data_descramble_thread(void *args);
void *cin_data_monitor_output_thread(void);
void *cin_data_writer_thread(void *args);

/* Buffer Routines */

void* cin_data_buffer_push(void *arg);
void cin_data_buffer_pop(void *arg);

/* Profiling Functions */
struct timespec timespec_diff(struct timespec start, struct timespec end);
struct timeval timeval_diff(struct timeval start, struct timeval end);

#ifdef __cplusplus
}
#endif

#endif
