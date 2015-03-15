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
#ifndef __MBUFFER_H__
#define __MBUFFER_H__

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CIN_DATA_MBUFFER_SIZE 2

#define CIN_DATA_MBUFFER_ERR_NONE       0
#define CIN_DATA_MBUFFER_ERR_MEMORY     1

typedef struct image_mbuffer {
  void* data;
  int active[CIN_DATA_MBUFFER_SIZE];
  int empty[CIN_DATA_MBUFFER_SIZE];
  int write_buffer;
  int read_buffer;
  int size;
  pthread_mutex_t mutex;
  pthread_cond_t signal;
} mbuffer_t;

int mbuffer_init(mbuffer_t *buffer, int data_size);
void* mbuffer_get_write_buffer(mbuffer_t *buffer);
void mbuffer_write_done(mbuffer_t *buffer);
void* mbuffer_get_read_buffer(mbuffer_t *buffer);
void mbuffer_read_done(mbuffer_t *buffer);

#ifdef __cplusplus
}
#endif

#endif
