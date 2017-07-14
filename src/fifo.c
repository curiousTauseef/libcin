/* vim: set ts=2 sw=2 tw=0 noet :
   
   libcin : Driver for LBNL FastCCD 
 *  Copyright (c) 2014, Brookhaven Science Associates, Brookhaven National Laboratory
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
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include "cin.h"
#include "fifo.h"

/* -----------------------------------------------------------------------------------------
 *
 * FIFO Functions
 *
 * -----------------------------------------------------------------------------------------
 */

int fifo_init(fifo *f, int elem_size, long int size, int readers){
  /* Initialize the fifo */
  /* NOTE : if this fails then it causes memory leaks !*/

  f->data = malloc(size * (long int)elem_size);
  if(f->data == NULL){
    return FIFO_ERR_MEMORY;
  } 

  /* set the initial parameters */
  f->head = f->data;
  int i;
  for(i=0;i<FIFO_MAX_READERS;i++){
    f->tail[i] = f->data;
  }
  f->end  = f->data + ((size - 1) * (long int)elem_size);
  f->size = size;
  f->elem_size = elem_size;

  f->full = 0;
  f->readers = readers;
  f->overruns = 0;

  /* Setup mutex */

  pthread_mutex_init(&f->mutex,NULL);
  pthread_cond_init(&f->signal, NULL);

  return FIFO_NOERR;
}

void fifo_flush(fifo *f){
  pthread_mutex_lock(&f->mutex);
  int i;
  for(i=0;i<FIFO_MAX_READERS;i++){
    f->tail[i] = f->head;
  }
  f->full = 0;

  pthread_mutex_unlock(&f->mutex);
}

int fifo_overruns(fifo *f)
{
  int _overruns;
  pthread_mutex_lock(&f->mutex);
  _overruns = f->overruns;
  pthread_mutex_unlock(&f->mutex);

  return _overruns;
}
  

long int fifo_used_bytes(fifo *f){
  long int bytes = 0;

  int i;
  long int used;

  pthread_mutex_lock(&f->mutex);
  for(i=0;i<f->readers;i++){
    if(f->head >= f->tail[i]){
      used = (long int)(f->head - f->tail[i]);  
    } else {
      used = (long int)((f->end - f->tail[i]) + (f->head - f->data));
    }
    if(used > bytes){
      bytes = used;
    }
  }
  pthread_mutex_unlock(&f->mutex);
  return bytes;
}

long int fifo_used_elements(fifo *f){
  return fifo_used_bytes(f) / f->elem_size;
}

double fifo_percent_full(fifo *f){
  long int bytes;
  double percent;
  
  bytes = fifo_used_bytes(f);
  percent = (double)bytes / (double)(f->elem_size * f->size);

  return (percent * 100.0);
}

void *fifo_get_head(fifo *f){
  void *head;

  pthread_mutex_lock(&f->mutex);
  head = f->head;
  pthread_mutex_unlock(&f->mutex);

  return head;
}

void fifo_advance_head(fifo *f){
  /* Increment the head pointet */
  pthread_mutex_lock(&f->mutex);

  /* Check all the tail pointers */

  int i;
  for(i=0;i<f->readers;i++){
    if((f->head == f->end) && (f->tail[i] == f->data)){
      f->full = 1;
      f->overruns++;
      goto cleanup;
    } else if((f->head + f->elem_size) == f->tail[i]){
      f->full = 1;
      f->overruns++;
      goto cleanup;
    }
  }

  if(f->head == f->end){
    f->head = f->data;
    f->full = 0;
  } else {
    f->head += f->elem_size;
    f->full = 0;
  }

cleanup:
  pthread_cond_broadcast(&f->signal);
  pthread_mutex_unlock(&f->mutex);
}

void* fifo_get_tail(fifo *f, int reader){
  /* Return the tail pointer or NULL if the FIFO is empty */

  void* tail;

  pthread_mutex_lock(&f->mutex);
  while(f->tail[reader] == f->head){
    pthread_cond_wait(&f->signal, &f->mutex);
  }

  tail = f->tail[reader];
  pthread_mutex_unlock(&f->mutex);

  return tail;
}

void fifo_advance_tail(fifo *f, int reader){
  /* Return the tail pointer and advance the FIFO */

  pthread_mutex_lock(&f->mutex);

  /* If the head and tail are the same, FIFO is empty */
  if(f->tail[reader] == f->head){
    return;
  }

  if(f->tail[reader] == f->end){
    f->tail[reader] = f->data;
  } else {
    f->tail[reader] += f->elem_size;
  }

  pthread_mutex_unlock(&f->mutex);
}

