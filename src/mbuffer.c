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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "mbuffer.h"
#include "cin.h"

int mbuffer_init(mbuffer_t *buffer, int data_size){

  buffer->size = data_size;

  buffer->data = malloc(CIN_DATA_MBUFFER_SIZE * data_size);
  if(!buffer->data){
    return CIN_DATA_MBUFFER_ERR_MEMORY;
  }

  int i;
  for(i=0;i<CIN_DATA_MBUFFER_SIZE;i++){
    buffer->active[i] = 0;
    buffer->empty[i] = 1;
  }

  buffer->write_buffer = 0;
  buffer->read_buffer = 1;

  pthread_mutex_init(&buffer->mutex, NULL);
  pthread_cond_init(&buffer->signal, NULL);
  
  return CIN_DATA_MBUFFER_ERR_NONE;
}

void* mbuffer_get_write_buffer(mbuffer_t *buffer){
  /* Get the current buffer pointer and return */
  void* p;

  pthread_mutex_lock(&buffer->mutex);
  p = buffer->data + (buffer->size * buffer->write_buffer);
  buffer->active[buffer->write_buffer] = 1;
  pthread_mutex_unlock(&buffer->mutex);

  return p;
}

void mbuffer_write_done(mbuffer_t *buffer){
  /* Signal that we are done writing */
  pthread_mutex_lock(&buffer->mutex);

  // Set the current write buffer to active

  buffer->active[buffer->write_buffer] = 0;
  buffer->empty[buffer->write_buffer] = 0;

  // Now switch the write buffer to the read buffer

  buffer->write_buffer ^= buffer->read_buffer;
  buffer->read_buffer ^= buffer->write_buffer;
  buffer->write_buffer ^= buffer->read_buffer;

  pthread_cond_signal(&buffer->signal);
  pthread_mutex_unlock(&buffer->mutex);
}

void* mbuffer_get_read_buffer(mbuffer_t *buffer){
  /* Get the current read buffer */
  void* rtn;

  pthread_mutex_lock(&buffer->mutex);

  while(buffer->empty[buffer->read_buffer]){
    pthread_cond_wait(&buffer->signal, &buffer->mutex);
  }

  // return a pointer to the read buffer
  rtn = buffer->data + (buffer->read_buffer * buffer->size);

  // Set the buffer to active
  buffer->active[buffer->read_buffer] = 1;


  // If we are writing to the buffer then wait
  while(buffer->active[buffer->write_buffer]){
    pthread_cond_wait(&buffer->signal, &buffer->mutex);
  }

  pthread_mutex_unlock(&buffer->mutex);

  return rtn;
}

void mbuffer_read_done(mbuffer_t *buffer){
  /* Signal we are done with the read buffer */
  pthread_mutex_lock(&buffer->mutex);
  buffer->active[buffer->read_buffer] = 0;
  pthread_mutex_unlock(&buffer->mutex);
}

