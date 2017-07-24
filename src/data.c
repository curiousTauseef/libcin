/* vim: set ts=2 sw=2 tw=0 noet :
   
   libcin : Driver for LBNL FastCCD 
   Copyright (c) 2014, Brookhaven Science Associates, Brookhaven National Laboratory
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
#define _GNU_SOURCE
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>  // For getpid()
#include <syscall.h>
#include <time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <arpa/inet.h>

#include <pthread.h>

#include "cin.h"
#include "fifo.h"
#include "data.h"
#include "descramble.h"
#include "common.h"

/* -------------------------------------------------------------------------------
 *
 * Network functions to read from fabric UDP port
 *
 * -------------------------------------------------------------------------------
 */


int cin_data_read(cin_port_t *dp, unsigned char* buffer){
  /* Read from the UDP stream */
  return recvfrom(dp->sockfd, buffer, 
                 CIN_DATA_MAX_MTU * sizeof(unsigned char), 0,
                 (struct sockaddr*)&dp->sin_cli, 
                 (socklen_t*)&dp->slen);

}

int cin_data_write(cin_port_t *dp, char* buffer, int buffer_len){
  int rtn = sendto(dp->sockfd, buffer, buffer_len, 0,
                   (struct sockaddr*)&dp->sin_srv, sizeof(dp->sin_srv));
  if(rtn == buffer_len){
    return 0;
  } else {
    return -1;
  }
}

/* -------------------------------------------------------------------------------
 *
 * Initialization Functions
 *
 * -------------------------------------------------------------------------------
 */


int cin_data_init(cin_data_t *cin, 
                  char* addr, uint16_t port, char* bind_addr, uint16_t bind_port, int rcvbuf,
                  int packet_buffer_len, int frame_buffer_len,
                  cin_data_callback push_callback, cin_data_callback pop_callback, void *usr_ptr){

  /* Initialize and start all the threads to acquire data */
  /* This does not block, just start threads */
  /* Setup FIFO elements */
  
  /* For DEBUG print out the process id */

  DEBUG_COMMENT("Starting cindata init.\n");
  DEBUG_PRINT("Build sha     = %s\n", cin_build_git_sha);
  DEBUG_PRINT("Build time    = %s\n", cin_build_git_time);
  DEBUG_PRINT("Build version = %s\n", cin_build_version);

  // First open communications .....

  cin->addr       = cin_com_set_string(addr, CIN_DATA_IP);
  cin->bind_addr  = cin_com_set_string(bind_addr, "0.0.0.0");
  cin->port       = cin_com_set_int(port, CIN_DATA_CIN_PORT);
  cin->bind_port  = cin_com_set_int(bind_port, CIN_DATA_BIND_PORT);
  cin->recv_buf   = cin_com_set_int(rcvbuf, CIN_DATA_RCVBUF);

  if(cin_com_init_port(&cin->dp, cin->addr, cin->port, 
                       cin->bind_addr, cin->bind_port, cin->recv_buf)){
    return -1;
  }
  
  /* Initialize buffers */
  int rtn;
  if((rtn = cin_data_init_buffers(cin, packet_buffer_len, frame_buffer_len)) != 0){
    DEBUG_PRINT("Failed to initialize buffers : %d\n", rtn);
    return 1;
  }

  /* Set some defaults */

  cin->mallformed_packets = 0;
  cin->dropped_packets = 0;

  /* Some framesore info */
  cin->framestore_counter = 0;
  cin->framestore_mode = 0;
  cin->framestore_trigger.tv_sec = 0;
  cin->framestore_trigger.tv_nsec = 0;

  /* Setup the descramble map */

  cin->map.map = malloc(sizeof(uint32_t) * CIN_DATA_MAX_STREAM);
  if(!cin->map.map){
    return 1;
  }
  cin->map.overscan = 2;
  cin->map.rows = CIN_DATA_MAX_FRAME_Y;

  /* Setup threads */

  cin->listen_thread.started = 0;
  cin->assembler_thread.started = 0;
  cin->descramble_thread.started = 0;
  pthread_mutex_init(&cin->stats_mutex, NULL);
  pthread_mutex_init(&cin->descramble_mutex, NULL);
  pthread_mutex_init(&cin->framestore_mutex, NULL);

  pthread_barrier_init(&cin->listen_thread.barrier, NULL, 2);
  pthread_barrier_init(&cin->assembler_thread.barrier, NULL, 2);
  pthread_barrier_init(&cin->descramble_thread.barrier, NULL, 2);

  /* Setup connections between processes */
  cin->callbacks.push = (void*)push_callback;
  cin->callbacks.pop = (void*)pop_callback;
  cin->callbacks.frame = malloc(sizeof(cin_data_frame_t));
  cin->callbacks.usr_ptr = usr_ptr;

  cin_data_thread_start(&cin->listen_thread, NULL,
                        (void *)cin_data_listen_thread, 
                        (void *)cin);

  cin_data_thread_start(&cin->assembler_thread, NULL,
                        (void *)cin_data_assembler_thread, 
                        (void *)cin);
  cin_data_thread_start(&cin->descramble_thread, NULL,
                        (void *)cin_data_descramble_thread,
                        (void *)cin);

  // Now wait for threads to start

  DEBUG_COMMENT("Waiting for all threads to signal they have started\n");

  pthread_barrier_wait(&cin->listen_thread.barrier);
  pthread_barrier_destroy(&cin->listen_thread.barrier);
  pthread_barrier_wait(&cin->assembler_thread.barrier);
  pthread_barrier_destroy(&cin->assembler_thread.barrier);
  pthread_barrier_wait(&cin->descramble_thread.barrier);
  pthread_barrier_destroy(&cin->descramble_thread.barrier);

  DEBUG_COMMENT("All threads started\n");

  return 0;
}

void cin_data_destroy(cin_data_t *cin)
{
  // First stop all the threads

  cin_data_stop_threads(cin);

  // TODO cleanup memory by doing free()

}

int cin_data_init_buffers(cin_data_t *cin, int packet_buffer_len, int frame_buffer_len){
  /* Initialize all the buffers */

  long int i;

  /* Packet FIFO */

  cin->packet_fifo = (fifo*)malloc(sizeof(fifo));
  if(fifo_init(cin->packet_fifo, sizeof(cin_data_packet_t), 
     packet_buffer_len)){
    return 1;
  }

  cin_data_packet_t *p = 
    (cin_data_packet_t*)(cin->packet_fifo->data);
  for(i=0;i<cin->packet_fifo->size;i++){
    if(!(p->data = (unsigned char*)malloc(sizeof(unsigned char) * CIN_DATA_MAX_MTU))){
      return 1;
    }
    p++;
  }

  /* Frame FIFO */
  cin->frame_fifo = (fifo*)malloc(sizeof(fifo));
  if(fifo_init(cin->frame_fifo, sizeof(struct cin_data_frame), frame_buffer_len)){
    return 1;
  }

  long int size = CIN_DATA_MAX_STREAM;
  cin_data_frame_t *q = (cin_data_frame_t*)(cin->frame_fifo->data);
  for(i=0;i<cin->frame_fifo->size;i++){
    if(!(q->data = (uint16_t *)malloc(sizeof(uint16_t) * size))){
      return 1;
    }
    q++;
  } 

  DEBUG_PRINT("Initialized FIFO for data of size %d %d\n", packet_buffer_len,
      frame_buffer_len);

  return 0;
}

int cin_data_thread_start(cin_data_threads_t *thread,
                          pthread_attr_t *attr,
                          void *(*func) (void *), 
                          void *arg){
  int rtn;
  rtn = pthread_create(&thread->thread_id, attr, func, arg);
  if(rtn == 0){
    DEBUG_COMMENT("Started thread\n");
    thread->started = 1;
    return 0;
  }else {
    DEBUG_COMMENT("Unable to start thread.\n");
    thread->started = 0;
    return 1;
  }
  return 1;
}

void cin_data_stop_threads(cin_data_t *cin){
  DEBUG_COMMENT("Thread cancel requested\n");

  if(cin->descramble_thread.started){
    pthread_cancel(cin->descramble_thread.thread_id);
    DEBUG_COMMENT("Descramble thread canceled\n");
  }
  if(cin->assembler_thread.started){
    pthread_cancel(cin->assembler_thread.thread_id);
    DEBUG_COMMENT("Assembler thread canceled\n");
  }
  if(cin->listen_thread.started){
    pthread_cancel(cin->listen_thread.thread_id);
    DEBUG_COMMENT("Listener thread canceled\n");
  }

  DEBUG_COMMENT("Waiting for threads to join\n");

  if(cin->descramble_thread.started){
    pthread_join(cin->descramble_thread.thread_id, NULL);
    DEBUG_COMMENT("Descramble thread joined\n");
  }
  if(cin->assembler_thread.started){
    pthread_join(cin->assembler_thread.thread_id, NULL);
    DEBUG_COMMENT("Assembler thread joined\n");
  }
  if(cin->listen_thread.started){
    pthread_join(cin->listen_thread.thread_id, NULL);
    DEBUG_COMMENT("Listener thread joined\n");
  }
}

void cin_data_framestore_trigger_enable(cin_data_t *cin){
  pthread_mutex_lock(&cin->framestore_mutex);
  cin->framestore_counter = 0;
  cin->framestore_mode = CIN_DATA_FRAMESTORE_TRIGGER;
  DEBUG_PRINT("Framestore trigger enabled, mode set to %d.\n", 
      cin->framestore_mode);
  pthread_mutex_unlock(&cin->framestore_mutex);
}

void cin_data_framestore_trigger(cin_data_t *cin, int count){
  struct timespec time;
  clock_gettime(CLOCK_REALTIME, &time);

  pthread_mutex_lock(&cin->framestore_mutex);
  cin->framestore_trigger = time;
  cin->framestore_counter = count;
  cin->framestore_mode = CIN_DATA_FRAMESTORE_TRIGGER;
  DEBUG_PRINT("Framestore trigger at %ld.%09ld mode set to %d.\n", 
      cin->framestore_trigger.tv_sec, cin->framestore_trigger.tv_nsec,
      cin->framestore_mode);
  pthread_mutex_unlock(&cin->framestore_mutex);
}

void cin_data_set_framestore_skip(cin_data_t *cin, int count){
  pthread_mutex_lock(&cin->framestore_mutex);
  if(count != 0){
    cin->framestore_counter = count;
    cin->framestore_mode = CIN_DATA_FRAMESTORE_SKIP;
  } else {
    cin->framestore_mode = CIN_DATA_FRAMESTORE_NONE;
  }
  DEBUG_PRINT("Set framestore counter to %d mode to %d\n", 
      cin->framestore_counter, cin->framestore_mode);
  pthread_mutex_unlock(&cin->framestore_mutex);
}

int cin_data_get_framestore_counter(cin_data_t *cin){
  int count;

  pthread_mutex_lock(&cin->framestore_mutex);
  count = cin->framestore_counter;
  DEBUG_PRINT("Framestore counter is %d\n", cin->framestore_counter);
  pthread_mutex_unlock(&cin->framestore_mutex);

  return count;
}

void cin_data_framestore_disable(cin_data_t *cin){
  pthread_mutex_lock(&cin->framestore_mutex);
  cin->framestore_mode = CIN_DATA_FRAMESTORE_NONE;
  DEBUG_PRINT("Framestore mode set to %d (Disabled)\n", cin->framestore_mode);
  pthread_mutex_unlock(&cin->framestore_mutex);
}
/* -----------------------------------------------------------------------------------------
 *
 * MAIN Thread Routines
 *
 * -----------------------------------------------------------------------------------------
 */

void *cin_data_assembler_thread(void *args){
  
  struct cin_data_packet *buffer = NULL;
  struct cin_data_frame *frame = NULL;
  int this_frame = 0;
  int last_frame = -1;
  int this_packet = 0;
  int last_packet = 0;
  int this_packet_msb = 0;
  int last_packet_msb = 0;
  int last_packet_flag = 0;
  int skipped;

  int buffer_len;
  unsigned char *buffer_p;

  uint16_t *frame_p;
  int i;

  uint64_t *header;
  uint64_t *footer;

  struct timespec last_frame_timestamp = {0,0};
  struct timespec this_frame_timestamp = {0,0};

  int packet_len = CIN_DATA_PACKET_LEN;

  cin_data_t *cin = (cin_data_t*)args;

  pthread_barrier_wait(&cin->assembler_thread.barrier);
  while(1){

    /* Get a packet from the fifo */

    buffer = (cin_data_packet_t*)fifo_get_tail(cin->packet_fifo);

    if(buffer == NULL){
      /* We don't have a frame, continue */
      fifo_advance_tail(cin->packet_fifo);
      continue;
    }

    /* Process a packet. Dump it if it is not 
       of a correct size */

    buffer_p = buffer->data;
    buffer_len = buffer->size - CIN_DATA_UDP_HEADER;

    if(buffer_len > packet_len){
      /* This packet is too big, dump the frame and continue */
      ERROR_COMMENT("Recieved packet too large");

      pthread_mutex_lock(&cin->stats_mutex);
      cin->mallformed_packets++;
      pthread_mutex_unlock(&cin->stats_mutex);

      /* We don't have a frame, continue */
      fifo_advance_tail(cin->packet_fifo);
      continue;
    } else if(buffer_len < 60) {
      // This is a quick hack to remove some bad packets from the end
      // of the data stream. They all say DEAD FOOD .....
      // we don't mark these as mallformed as they are always there
      //DEBUG_COMMENT("Packet too small ... dropping.\n");
      fifo_advance_tail(cin->packet_fifo);
      continue;
    }

    /* Next lets check the magic number of the packet */
    /* to see if it is a valid packet                 */

    header = ((uint64_t *)buffer_p); 
    if((*header & CIN_DATA_MAGIC_PACKET_MASK) != CIN_DATA_MAGIC_PACKET) {
      /* This is not a valid packet ... dump the packet and continue */
      ERROR_COMMENT("Packet does not match magic\n");

      pthread_mutex_lock(&cin->stats_mutex);
      cin->mallformed_packets++;
      pthread_mutex_unlock(&cin->stats_mutex);
  
      fifo_advance_tail(cin->packet_fifo);
      continue;
    }
    
    /* First byte of frame header is the packet no*/
    /* Bytes 7 and 8 are the frame number */ 

    this_packet = *buffer_p; 
    buffer_p += 6;
    this_frame  = (*buffer_p << 8) + *(buffer_p + 1);
    buffer_p += 2;

    // First check if this is the last packet. 
    // By looking for DEAD FOOD

    footer = ((uint64_t*)(buffer->data + (buffer->size - 8)));
    if(*footer == CIN_DATA_TAIL_MAGIC_PACKET){
      // We are the last packet
      last_packet_flag = 1;
      // decrease the buffer by the tail magic packet
      buffer_len = buffer_len - 8;
    }

    if(this_frame != last_frame){
      // We have a new frame, but missed the DEAD FOOD

      if(frame){
        // Note this is repeated below, but is here becuase we won't have
        // pushed out the frame at the end of the last packet
        
        ERROR_PRINT("Missed end of frame magic on frame %d\n", last_frame);
        frame->number = last_frame;
        frame->timestamp = last_frame_timestamp;

        pthread_mutex_lock(&cin->stats_mutex);
        cin->last_frame = last_frame;
        cin->mallformed_packets++;
        pthread_mutex_unlock(&cin->stats_mutex);
       
        fifo_advance_head(cin->frame_fifo);
        frame = NULL;
      }
    } // this_frame != last_frame 

    if(!frame){
      // We don't have a valid frame, get one from the stack

      frame = (cin_data_frame_t*)fifo_get_head(cin->frame_fifo);

      /* Blank the frame */
      memset(frame->data, 0x0, CIN_DATA_MAX_STREAM * 2);

      /* Set all the last frame stuff */

      pthread_mutex_lock(&cin->stats_mutex);
      cin->framerate = timespec_diff(last_frame_timestamp,this_frame_timestamp);
      pthread_mutex_unlock(&cin->stats_mutex);

      last_frame = this_frame;
      last_frame_timestamp  = this_frame_timestamp;
      this_frame_timestamp  = buffer->timestamp;

      last_packet = -1;
      this_packet_msb = 0;
      last_packet_msb = 0;

      // Now check the packet length, if it is not correct reset here.

      if((packet_len != buffer_len) && !last_packet_flag){
        ERROR_PRINT("Packet length does not match max packet value (%d). Setting packet value to (%d)\n", 
            packet_len, buffer_len);
        packet_len = buffer_len;
      }

      DEBUG_PRINT("Assembling frame %d\n", this_frame);

    } // !frame 

    // Account for packet counter being too few bits 
    // If the packet number goes backwards we should add
    // 0x100

    if(this_packet < last_packet){
      this_packet_msb += 0x100;
    }

    if(this_packet == last_packet){
      ERROR_PRINT("Duplicate packet (frame = %d, packet = 0x%x)\n", 
                  this_frame, this_packet + this_packet_msb);
      pthread_mutex_lock(&cin->stats_mutex);
      cin->mallformed_packets++;
      pthread_mutex_unlock(&cin->stats_mutex);
      fifo_advance_tail(cin->packet_fifo);
      continue;
    }

    skipped = (this_packet + this_packet_msb) - (last_packet + last_packet_msb + 1);
    if(skipped > 0){
      pthread_mutex_lock(&cin->stats_mutex);
      cin->dropped_packets += skipped;
      pthread_mutex_unlock(&cin->stats_mutex);

      // Do some bounds checking
      if(((this_packet + this_packet_msb + 1) * packet_len) <
         (CIN_DATA_MAX_STREAM * 2)){

        ERROR_PRINT("Skipped %d packets from frame %d\n", skipped, this_frame);
     
        // Encode skipped packets into the data
        int i;
        frame_p = frame->data;
        frame_p += (last_packet + last_packet_msb + 1) * packet_len / 2;
        for(i=0;i<(skipped * packet_len / 2);i++){
          *frame_p = CIN_DATA_DROPPED_PACKET_VAL;
          frame_p++;
        }
      } else {
        // Out of bounds packet
        ERROR_COMMENT("Packet out of bounds\n");
        pthread_mutex_lock(&cin->stats_mutex);
        cin->mallformed_packets++;
        pthread_mutex_unlock(&cin->stats_mutex);
      }
    }

    // Do some bounds checking
    // Check if the packet number is bigger than the 
    // frame size. If so, skip the packet else swap the
    // endienness and copy to the raw frame

    if(((this_packet + this_packet_msb + 1) * packet_len) < 
       (CIN_DATA_MAX_STREAM * 2)){
      frame_p = frame->data;
      frame_p += (this_packet + this_packet_msb) * packet_len / 2;
      for(i=0;i<buffer_len/2;i++){
        *frame_p = *buffer_p++ << 8;
        *frame_p++ += *buffer_p++;
      }
    } else {
      ERROR_PRINT("Packet count out of bounds (frame = %d, packet = 0x%x)\n", 
                  this_frame, this_packet + this_packet_msb);
      pthread_mutex_lock(&cin->stats_mutex);
      cin->mallformed_packets++;
      pthread_mutex_unlock(&cin->stats_mutex);
      fifo_advance_tail(cin->packet_fifo);
      continue;
    }

    if(last_packet_flag){
      // We just recieved the last packet. 
      // Write the frame number and timestamp
      // and push it onto the fifo
      
      frame->number = this_frame;
      frame->timestamp = this_frame_timestamp;

      pthread_mutex_lock(&cin->stats_mutex);
      cin->last_frame = this_frame;
      pthread_mutex_unlock(&cin->stats_mutex);

      // If we are in a framestore mode only output 
      // if the framestore counter

      int go = 1;

      pthread_mutex_lock(&cin->framestore_mutex);
      if(cin->framestore_mode == CIN_DATA_FRAMESTORE_TRIGGER){
        if(timespec_after(cin->framestore_trigger, this_frame_timestamp) &&
           (cin->framestore_counter != 0)){
          go = 1;
          cin->framestore_counter--;
        } else {
          go = 0;
        }
      } else if(cin->framestore_mode == CIN_DATA_FRAMESTORE_SKIP){
        if(cin->framestore_counter > 0){
          go = 0;
          cin->framestore_counter--;
        } else {
          go = 1;
        }
      }
      pthread_mutex_unlock(&cin->framestore_mutex);

      if(go)
      {
        fifo_advance_head(cin->frame_fifo);
      } else {
        DEBUG_PRINT("Dropped frame %d\n", this_frame);
      }
      frame = NULL;
      last_packet_flag = 0;
    }

    /* Now we are done with the packet, we can advance the fifo */

    fifo_advance_tail(cin->packet_fifo);

    /* Now we can set the last packet to this packet */

    last_packet = this_packet;
    last_packet_msb = this_packet_msb;

  } // while(1)

  pthread_exit(NULL);
}

void cin_data_reset_stats(cin_data_t *cin){

  // Lock all mutexes 
  pthread_mutex_lock(&cin->stats_mutex);
  pthread_mutex_lock(&cin->packet_fifo->mutex);
  pthread_mutex_lock(&cin->frame_fifo->mutex);

  cin->dropped_packets = 0;
  cin->mallformed_packets = 0;
  cin->packet_fifo->overruns = 0;
  cin->frame_fifo->overruns = 0;

  // Unlock all mutexes 
  pthread_mutex_unlock(&cin->stats_mutex);
  pthread_mutex_unlock(&cin->packet_fifo->mutex);
  pthread_mutex_unlock(&cin->frame_fifo->mutex);
}

void cin_data_compute_stats(cin_data_t *cin, cin_data_stats_t *stats){
  double framerate;

  pthread_mutex_lock(&cin->stats_mutex);

  if(cin->last_frame){
    // Compute framerate

    framerate  =  (double)cin->framerate.tv_nsec * 1e-9;
    framerate  += (double)cin->framerate.tv_sec; 
    if(framerate == 0){
      framerate = 0;
    } else {
      framerate = 1 / framerate;
    }
  } else {
    framerate = 0; 
  }

  // TODO This is probably not accurate... it is not all CIN_DATA_MAX_STREAM!

  stats->last_frame = (int)cin->last_frame;
  stats->framerate = framerate;

  stats->packet_percent_full = fifo_percent_full(cin->packet_fifo);
  stats->frame_percent_full = fifo_percent_full(cin->frame_fifo);
  stats->packet_used = fifo_used_elements(cin->packet_fifo);
  stats->frame_used = fifo_used_elements(cin->frame_fifo);
  stats->packet_overruns = fifo_overruns(cin->packet_fifo);
  stats->frame_overruns = fifo_overruns(cin->frame_fifo);
  stats->dropped_packets = cin->dropped_packets;
  stats->mallformed_packets = cin->mallformed_packets;

  // Unlock all mutexes
  pthread_mutex_unlock(&cin->stats_mutex);
}

void cin_data_show_stats(FILE *fp, cin_data_stats_t stats){
  char buffer[256];
  
  sprintf(buffer, "Last frame %-12d", stats.last_frame);
  fprintf(fp, "%-80s\n", buffer);

  sprintf(buffer, "Packet buffer %8.3f%%.", 
          stats.packet_percent_full);
  sprintf(buffer, "%s Image buffer %8.3f%%.",buffer,
          stats.frame_percent_full);
  sprintf(buffer, "%s Spool buffer %8.3f%%.",buffer,
          stats.image_percent_full);
  fprintf(fp, "%-80s\n", buffer);

  sprintf(buffer, "Framerate     = %6.1f s^-1", stats.framerate);
  fprintf(fp, "%-80s\n", buffer);

  sprintf(buffer, "Dropped packets %-11ld : Mallformed packets %-11ld",
          stats.dropped_packets, stats.mallformed_packets);
  fprintf(fp, "%-80s\n", buffer);
}

int cin_data_send_magic(cin_data_t *cin){
  DEBUG_COMMENT("Sending \"DUMMY DATA\" to CIN to initialize fabric comms\n");
  char* dummy = "DUMMY DATA";
  return cin_data_write(&cin->dp, dummy, sizeof(dummy));
}

void *cin_data_listen_thread(void *args){
  
  struct cin_data_packet *buffer = NULL;
  cin_data_t *cin = (cin_data_t *)args;
 
  /* Send a packet to initialize the CIN */
  cin_data_send_magic(cin);

  struct timespec time;

  pthread_barrier_wait(&cin->listen_thread.barrier);

  while(1){
    /* Get the next element in the fifo */
    buffer = (cin_data_packet_t*)fifo_get_head(cin->packet_fifo);
    buffer->size = recvfrom(cin->dp.sockfd, buffer->data, 
                            CIN_DATA_MAX_MTU * sizeof(unsigned char), 0,
                            (struct sockaddr*)&cin->dp.sin_cli, 
                            (socklen_t*)&cin->dp.slen);
    
    if(ioctl(cin->dp.sockfd, SIOCGSTAMPNS, &time) == -1){
      clock_gettime(CLOCK_REALTIME, &time);
    }

    buffer->timestamp = time;

    fifo_advance_head(cin->packet_fifo);
  }

  pthread_exit(NULL);
}

void cin_data_get_descramble_params(cin_data_t *cin, int *rows, int *overscan, int *xsize, int *ysize)
{
  pthread_mutex_lock(&cin->descramble_mutex);
  *overscan = cin->map.overscan;
  *rows = cin->map.rows;
  *xsize = 0;
  *ysize = 0;

  if(xsize != NULL){
    *xsize = cin->map.size_x;
  }
  if(ysize != NULL){
    *ysize = cin->map.size_y;
  }
  pthread_mutex_unlock(&cin->descramble_mutex);

}

int cin_data_set_descramble_params(cin_data_t *cin, int rows, int overscan)
{
  /* This routine sets the descamble parameters */
  int rtn;

  pthread_mutex_lock(&cin->descramble_mutex);

  cin->map.overscan = overscan;
  cin->map.rows = rows;

  rtn = cin_data_descramble_init(&cin->map);
  DEBUG_PRINT("Created new descramble map. Overscan = %d, rows = %d\n", overscan, rows);

  pthread_mutex_unlock(&cin->descramble_mutex);

  return rtn;
}

void* cin_data_descramble_thread(void *args){
  /* This routine gets the next frame and descrambles is */

  cin_data_t *cin = (cin_data_t*)args;

  struct cin_data_frame *frame = NULL;
  struct cin_data_frame *image = NULL;

  int max;
  
  pthread_mutex_lock(&cin->descramble_mutex);
  cin_data_descramble_init(&cin->map);
  pthread_mutex_unlock(&cin->descramble_mutex);

  DEBUG_COMMENT("Initialized descramble map\n");

  pthread_barrier_wait(&cin->descramble_thread.barrier);
  while(1){
    // Get a frame 
    frame = (cin_data_frame_t*)fifo_get_tail(cin->frame_fifo);
    image = (cin_data_frame_t*)cin_data_buffer_push(cin);

    DEBUG_PRINT("Descrambling frame %d\n", frame->number);

    if(image != NULL)
    {
      max = image->size_x * image->size_y;
      
      pthread_mutex_lock(&cin->descramble_mutex);
      cin_data_descramble_frame(&cin->map, image->data, frame->data, max); 
      pthread_mutex_unlock(&cin->descramble_mutex);

      image->timestamp = frame->timestamp;
      image->number = frame->number;
      image->size_x = cin->map.size_x;
      image->size_y = cin->map.size_y;

    }

    // Release the frame and the image
    fifo_advance_tail(cin->frame_fifo);
    cin_data_buffer_pop(cin);
  }

  pthread_exit(NULL);
}


/* -------------------------------------------------------------------------------
 *
 * Routines for sequential access to data
 *
 * -------------------------------------------------------------------------------
 */

cin_data_frame_t* cin_data_buffer_push(cin_data_t *cin)
{
  // Run the callback to load the data frame
  if(cin->callbacks.push != NULL)
  {
    (*cin->callbacks.push)(cin->callbacks.frame, cin->callbacks.usr_ptr);
    // We return a type of cin_data_frame to process
    return cin->callbacks.frame;
  }
  return NULL;
}

void cin_data_buffer_pop(cin_data_t *cin)
{
  if(cin->callbacks.pop != NULL)
  {
    (*cin->callbacks.pop)(cin->callbacks.frame, cin->callbacks.usr_ptr);
  }
  return;
}
