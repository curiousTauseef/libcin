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
#ifndef __CIN_DATA_FIFO__H 
#define __CIN_DATA_FIFO__H

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Macro Definitions */

#define FIFO_ERR_MEMORY 1
#define FIFO_NOERR      0

/* FIFO Functions */

void* fifo_get_head(fifo *f);
/* 
 * Return the head of the FIFO element pointed to by f. 
 * This routine will signal that new data is avaliable in
 * the fifo using "pthread_cond_signal"
 */
void* fifo_get_tail(fifo *f, int reader);
/* 
 * Return the tail of the FIFO element pointed to by f.
 * This routine will block until data is avaliable, waiting
 * on the signal sent by "fifo_get_head". If data is on the
 * fifo then it will immediately return
 */
void fifo_advance_head(fifo *f);
/*
 * Advance the head pointer, signalling we are done filling
 * the fifo with an element.
 */
void fifo_advance_tail(fifo *f, int reader);
/*
 * Advance the tail pointer, signalling we have processed a fifo
 * element and this can be returned
 */
int fifo_init(fifo *f, int elem_size, long int size, int readers);
/*
 * Initialize the fifo. The FIFO is of length size with a data
 * structure of length elem_size. 
 */ 
long int fifo_used_bytes(fifo *f);
double fifo_percent_full(fifo *f);
long int fifo_used_elements(fifo *f);
void fifo_flush(fifo *f);
#ifdef __cplusplus
}
#endif

#endif
