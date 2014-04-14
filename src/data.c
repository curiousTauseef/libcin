#define _GNU_SOURCE
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>  // For getpid()
#include <syscall.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <arpa/inet.h>

#include <pthread.h>

#include "cin.h"
#include "fifo.h"
#include "data.h"
#include "mbuffer.h"
#include "descramble.h"
#include "version.h"

/* -------------------------------------------------------------------------------
 *
 * Thread communication global variables
 *
 * -------------------------------------------------------------------------------
 */

static cin_data_threads_t *threads;
static cin_data_thread_data_t thread_data;

/* -------------------------------------------------------------------------------
 *
 * Network functions to read from fabric UDP port
 *
 * -------------------------------------------------------------------------------
 */

//int cin_init

int cin_data_init_port(struct cin_port* dp, 
                       char* ipaddr, uint16_t port,
                       char* cin_ipaddr, uint16_t cin_port,
                       int rcvbuf) {

  if(ipaddr == NULL){
    dp->cliaddr = "0.0.0.0";
  } else {
    dp->cliaddr = ipaddr;
  }

  if(port == 0){
    dp->cliport = CIN_DATA_PORT;
  } else {
    dp->cliport = port;
  }

  if(cin_ipaddr == NULL){
    dp->srvaddr = CIN_DATA_IP;
  } else {
    dp->srvaddr = cin_ipaddr;
  }

  if(cin_port == 0){
    dp->srvport = CIN_DATA_CTL_PORT;
  } else {
    dp->srvport = cin_port;
  }

  if(rcvbuf == 0){
    dp->rcvbuf = CIN_DATA_RCVBUF;
  } else {
    dp->rcvbuf = rcvbuf;
  }
  dp->rcvbuf = dp->rcvbuf * 1024 * 1024; // Convert to Mb
  
  // Do debur prints

  DEBUG_PRINT("Client address = %s:%d\n", dp->cliaddr, dp->cliport);
  DEBUG_PRINT("Server address = %s:%d\n", dp->srvaddr, dp->srvport);

  dp->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if(dp->sockfd < 0) {
    perror("CIN data port - socket() failed !!!");
    return 1;
  }

  DEBUG_PRINT("Opened socket :%d\n", dp->sockfd);

  int i = 1;
  if(setsockopt(dp->sockfd, SOL_SOCKET, SO_REUSEADDR, \
                (void *)&i, sizeof i) < 0) {
    perror("CIN data port - setsockopt() failed !!!");
    return 1;
  }

  /* initialize CIN (server) and client (us!) sockaddr structs */

  memset(&dp->sin_srv, 0, sizeof(struct sockaddr_in));
  memset(&dp->sin_cli, 0, sizeof(struct sockaddr_in));

  dp->sin_srv.sin_family = AF_INET;
  dp->sin_srv.sin_port = htons(dp->srvport);
  dp->sin_cli.sin_family = AF_INET;
  dp->sin_cli.sin_port = htons(dp->cliport);
  dp->slen = sizeof(struct sockaddr_in);

  if(inet_aton(dp->srvaddr, &dp->sin_srv.sin_addr) == 0) {
    perror("CIN data port - inet_aton() failed!!");
    return 1;
  }

  if(inet_aton(dp->cliaddr, &dp->sin_cli.sin_addr) == 0) {
    perror("CIN data port - inet_aton() failed!!");
    return 1;
  }

  /* Bind to the socket to get CIN traffic */

  if((bind(dp->sockfd, (struct sockaddr *)&dp->sin_cli , sizeof(dp->sin_cli))) == -1){
    perror("CIN data port - cannot bind");
    return 1;
  }

  /* Set the receieve buffers for the socket */

  DEBUG_PRINT("Requesting recieve buffer of %d Mb\n", dp->rcvbuf / (1024*1024));

  if(setsockopt(dp->sockfd, SOL_SOCKET, SO_RCVBUF, 
                &dp->rcvbuf, sizeof(dp->rcvbuf)) == -1){
    perror("CIN data port - unable to set receive buffer :");
  } 

  socklen_t rcvbuf_rb_len = sizeof(dp->rcvbuf_rb);
  if(getsockopt(dp->sockfd, SOL_SOCKET, SO_RCVBUF,
                &dp->rcvbuf_rb, &rcvbuf_rb_len) == -1){
    perror("CIN data port - unable to get receive buffer :");
  }

  DEBUG_PRINT("Recieve buffer = %d Mb\n", dp->rcvbuf_rb / (1024 * 1024));

  int zero = 0;
  if(setsockopt(dp->sockfd, SOL_SOCKET, SO_TIMESTAMP,
                &zero, sizeof(zero)) == -1){
    perror("Unable to set sockopt SO_TIMESTAMP");
  } else {
    DEBUG_COMMENT("Set SO_TIMESTAMP to zero\n");
  }
                
  thread_data.dp = dp;
  return 0;
}

int cin_data_read(struct cin_port* dp, unsigned char* buffer){
  /* Read from the UDP stream */
  return recvfrom(dp->sockfd, buffer, 
                 CIN_DATA_MAX_MTU * sizeof(unsigned char), 0,
                 (struct sockaddr*)&dp->sin_cli, 
                 (socklen_t*)&dp->slen);

}

int cin_data_write(struct cin_port* dp, char* buffer, int buffer_len){
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

int cin_data_init(int mode, int packet_buffer_len, int frame_buffer_len,
                  cin_data_callback push_callback, cin_data_callback pop_callback,
                  void *usr_ptr){
  /* Initialize and start all the threads to acquire data */
  /* This does not block, just start threads */
  /* Setup FIFO elements */
  
  /* For DEBUG print out the process id */

  DEBUG_PRINT("build sha     = %s\n", cin_build_git_sha);
  DEBUG_PRINT("build time    = %s\n", cin_build_git_time);
  DEBUG_PRINT("build version = %s\n", cin_build_version);

  DEBUG_PRINT("PID = %d\n", getpid());
  DEBUG_PRINT("getuid = %d\n", getuid());
  DEBUG_PRINT("geteuid = %d\n", geteuid());

  /* Initialize buffers */
  int rtn;
  if((rtn = cin_data_init_buffers(packet_buffer_len, frame_buffer_len)) != 0){
    DEBUG_PRINT("Failed to initialize buffers : %d\n", rtn);
    return 1;
  }

  /* Set some defaults */

  thread_data.mallformed_packets = 0;
  thread_data.dropped_packets = 0;

  /* Setup the needed mutexes */
  pthread_mutex_init(&thread_data.stats_mutex, NULL);

  /* Setup threads */

  threads = malloc(sizeof(cin_data_threads_t) * MAX_THREADS);
  if(!threads){
    return 1;
  }

  int i;
  for(i=0;i<MAX_THREADS;i++){
    threads[i].started = 0;
  }

  /* Setup connections between processes */

  cin_data_proc_t *listen = malloc(sizeof(cin_data_proc_t));
  listen->output_get = (void*)&fifo_get_head;
  listen->output_put = (void*)&fifo_advance_head;
  listen->output_args = (void*)thread_data.packet_fifo;

  cin_data_proc_t *assemble = malloc(sizeof(cin_data_proc_t));
  assemble->input_get = (void*)&fifo_get_tail;
  assemble->input_put = (void*)&fifo_advance_tail;
  assemble->input_args = (void*)thread_data.packet_fifo;
  assemble->reader = 0;
  assemble->output_get = (void*)&fifo_get_head;
  assemble->output_put = (void*)&fifo_advance_head;
  assemble->output_args = (void*)thread_data.frame_fifo;

  cin_data_proc_t *descramble1 = malloc(sizeof(cin_data_proc_t));
  descramble1->input_get = (void*)&fifo_get_tail;
  descramble1->input_put = (void*)&fifo_advance_tail;
  descramble1->input_args = (void*)thread_data.frame_fifo;
  descramble1->reader = 0;
  
  if(CIN_DATA_MODE_DBL_BUFFER & mode){
    descramble1->output_get = (void*)&mbuffer_get_write_buffer;
    descramble1->output_put = (void*)&mbuffer_write_done;
    descramble1->output_args = (void*)thread_data.image_dbuffer;
  } else {
    thread_data.callbacks.push = (void*)push_callback;
    thread_data.callbacks.pop = (void*)pop_callback;
    thread_data.callbacks.frame = malloc(sizeof(cin_data_frame_t));
    thread_data.callbacks.frame->usr_ptr = usr_ptr;
    descramble1->output_get = (void*)&cin_data_buffer_push;
    descramble1->output_put = (void*)&cin_data_buffer_pop;
    descramble1->output_args = (void*)&thread_data.callbacks;
  }

  cin_data_thread_start(&threads[0], NULL,
                        (void *)cin_data_listen_thread, 
                        (void *)listen);

  cin_data_thread_start(&threads[1], NULL,
                        (void *)cin_data_assembler_thread, 
                        (void *)assemble);
  cin_data_thread_start(&threads[2], NULL,
                        (void *)cin_data_descramble_thread,
                        (void *)descramble1);
#ifdef __AFFINITY__

  /*
   * Try to set the processor AFFINITY to lock the current
   * thread onto the first CPU and then the other three threads
   * onto separate cores. YMMV use carefully.
   */

  int j;
  cpu_set_t cpu_set;

  CPU_ZERO(&cpu_set);
  CPU_SET(1, &cpu_set);
  sched_setaffinity(0, sizeof(cpu_set_t), &cpu_set);

  for(j=0;j<3;j++){
    if(threads[j].started){
      CPU_ZERO(&cpu_set);
      CPU_SET(j + 2, &cpu_set);
      if(!pthread_setaffinity_np(threads[j].thread_id, sizeof(cpu_set_t), &cpu_set)){
        DEBUG_PRINT("Set CUP affinity on thread %d to CPU %d\n", j, j);
      } else {
        DEBUG_COMMENT("Unable to set CPU affinity");
      }
    }
  }

#endif


  return 0;
}

void cin_data_start_monitor_output(void){
    cin_data_thread_start(&threads[6], NULL,
                          (void *)cin_data_monitor_output_thread,
                          NULL);
  DEBUG_COMMENT("Starting monitor ourput\n");
}

void cin_data_stop_monitor_output(void){
  if(threads[6].started){
    pthread_cancel(threads[6].thread_id);
    DEBUG_COMMENT("Sending cancel to monitor thread\n");
  }
}

int cin_data_init_buffers(int packet_buffer_len, int frame_buffer_len){
  /* Initialize all the buffers */

  long int i;

  /* Packet FIFO */

  thread_data.packet_fifo = malloc(sizeof(fifo));
  if(fifo_init(thread_data.packet_fifo, sizeof(cin_data_packet_t), 
     packet_buffer_len, 1)){
    return 1;
  }

  cin_data_packet_t *p = 
    (cin_data_packet_t*)(thread_data.packet_fifo->data);
  for(i=0;i<thread_data.packet_fifo->size;i++){

    if(!(p->data = malloc(sizeof(char) * CIN_DATA_MAX_MTU))){
      return 1;
    }
    p++;
  }

  /* Frame FIFO */
  thread_data.frame_fifo = malloc(sizeof(fifo));
  if(fifo_init(thread_data.frame_fifo, sizeof(struct cin_data_frame), frame_buffer_len, 1)){
    return 1;
  }

  long int size = CIN_DATA_MAX_STREAM;
  cin_data_frame_t *q = (cin_data_frame_t*)(thread_data.frame_fifo->data);
  for(i=0;i<thread_data.frame_fifo->size;i++){
    if(!(q->data = malloc(sizeof(uint16_t) * size))){
      return 1;
    }
    q++;
  } 

  /* Image Output Fifo */

  thread_data.image_fifo = malloc(sizeof(fifo));
  if(fifo_init(thread_data.image_fifo, sizeof(struct cin_data_frame), frame_buffer_len, 1)){
    return 1;
  }
  q = (cin_data_frame_t*)(thread_data.image_fifo->data);
  for(i=0;i<thread_data.image_fifo->size;i++){
    if(!(q->data = malloc(sizeof(uint16_t) * size))){
      return 1;
    }
    q++;
  }

  /* Image Double Buffer */

  thread_data.image_dbuffer = malloc(sizeof(mbuffer_t));
  if(mbuffer_init(thread_data.image_dbuffer, sizeof(cin_data_frame_t))){
    return 1;
  }
  q = (cin_data_frame_t*)thread_data.image_dbuffer->data;
  q[0].data = malloc(sizeof(uint16_t) * size);
  q[1].data = malloc(sizeof(uint16_t) * size);

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

void cin_data_wait_for_threads(void){
  /* This routine waits for the threads to stop 
     NOTE : This blocks until all threads complete */
  int i;
  for(i=0;i<MAX_THREADS;i++){
    if(threads[i].started){
      pthread_join(threads[i].thread_id, NULL);
      DEBUG_PRINT("Thread %d joined\n", i);
    }
  }
}

int cin_data_stop_threads(void){
  int i;
  DEBUG_COMMENT("Cancel Requested\n");
  for(i=0;i<MAX_THREADS;i++){
    if(threads[i].started){
      pthread_cancel(threads[i].thread_id);
      DEBUG_PRINT("Sending cancel to thread: %d\n", i);
    }
  }
  return 0;
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

  struct timeval last_frame_timestamp = {0,0};
  struct timeval this_frame_timestamp = {0,0};

  cin_data_proc_t *proc = (cin_data_proc_t*)args;

  int sid = syscall(SYS_gettid);
  DEBUG_PRINT("Starting assembler thread ppid = %d\n", sid);

  while(1){

    /* Get a packet from the fifo */

    buffer = (cin_data_packet_t*)(*proc->input_get)(proc->input_args, proc->reader);

    if(buffer == NULL){
      /* We don't have a frame, continue */
      continue;
    }

    /* Process a packet. Dump it if it is not 
       of a correct size */
    buffer_p = buffer->data;
    buffer_len = buffer->size - CIN_DATA_UDP_HEADER;
    if(buffer_len > CIN_DATA_PACKET_LEN){
      /* Dump the frame and continue */
      DEBUG_COMMENT("Recieved packet too large");
      (*proc->input_put)(proc->input_args, proc->reader);
      thread_data.mallformed_packets++;
      continue;
    }

    /* Next lets check the magic number of the packet */
    header = ((uint64_t *)buffer_p); 
    if((*header & CIN_DATA_MAGIC_PACKET_MASK) != CIN_DATA_MAGIC_PACKET) {
      /* Dump the packet and continue */
      DEBUG_COMMENT("Packet does not match magic\n");
      (*proc->input_put)(proc->input_args, proc->reader);
      thread_data.mallformed_packets++;
      continue;
    }
    
    /* First byte of frame header is the packet no*/
    /* Bytes 7 and 8 are the frame number */ 

    this_packet = *buffer_p; 
    buffer_p += 6;
    this_frame  = (*buffer_p << 8) + *(buffer_p + 1);
    buffer_p += 2;

    //DEBUG_PRINT("Recieved packet %d from frame %d\n", this_packet, this_frame);

    if(this_frame != last_frame){
      // We have a new frame, but missed the DEAD FOOD

      if(frame){
        // Note this is repeated below, but is here in
        // case we haven't pushed out the frame at the end
        DEBUG_PRINT("Missed end of frame magic on frame %d\n", last_frame);
        frame->number = last_frame;
        frame->timestamp = last_frame_timestamp;
        thread_data.last_frame = last_frame;
        (*proc->output_put)(proc->output_args);
        frame = NULL;
      }
    } // this_frame != last_frame 

    if(!frame){
      // We don't have a valid frame, get one from the stack

      frame = (cin_data_frame_t*)(*proc->output_get)(proc->output_args);

      /* Blank the frame */
      memset(frame->data, 0x0, CIN_DATA_MAX_STREAM * 2);

      /* Set all the last frame stuff */
      last_frame = this_frame;
      last_packet = -1;
      this_packet_msb = 0;
      last_packet_msb = 0;
        
      last_frame_timestamp  = this_frame_timestamp;
      this_frame_timestamp  = buffer->timestamp;
      thread_data.framerate = timeval_diff(last_frame_timestamp,this_frame_timestamp);
    } // !frame 

    if(this_packet < last_packet){
      this_packet_msb += 0x100;
    }

    if(this_packet == last_packet){
      DEBUG_PRINT("Duplicate packet (frame = %d, packet = 0x%x)\n", 
                  this_frame, this_packet + this_packet_msb);
      (*proc->input_put)(proc->input_args, proc->reader);
      thread_data.mallformed_packets++;
      continue;
    }

    skipped = (this_packet + this_packet_msb) - (last_packet + last_packet_msb + 1);
    if(skipped > 0){
      thread_data.dropped_packets += skipped;
      // Do some bounds checking
      if(((this_packet + this_packet_msb + 1) * CIN_DATA_PACKET_LEN) <
         (CIN_DATA_MAX_STREAM * 2)){

        DEBUG_PRINT("Skipped %d packets from frame %d\n", skipped, this_frame);
     
        // Encode skipped packets into the data
        int i;
        frame_p = frame->data;
        frame_p += (last_packet + last_packet_msb + 1) * CIN_DATA_PACKET_LEN / 2;
        for(i=0;i<(skipped * CIN_DATA_PACKET_LEN / 2);i++){
          *frame_p = CIN_DATA_DROPPED_PACKET_VAL;
          frame_p++;
        }
      } else {
        // Out of bounds packet
        DEBUG_COMMENT("Packet out of bounds\n");
      }
    }

    // First check if this is the last packet. 

    footer = ((uint64_t*)(buffer->data + (buffer->size - 8)));
    if(*footer == CIN_DATA_TAIL_MAGIC_PACKET){
      last_packet_flag = 1;
      buffer_len = buffer_len - 8;
    }

    // Do some bounds checking
    // Check if the packet number is bigger than the 
    // frame size. If so, skip the packet. 

    if(((this_packet + this_packet_msb + 1) * CIN_DATA_PACKET_LEN) < 
       (CIN_DATA_MAX_STREAM * 2)){
      // Copy the data and swap the endieness
      frame_p = frame->data;
      frame_p += (this_packet + this_packet_msb) * CIN_DATA_PACKET_LEN / 2;
      for(i=0;i<buffer_len/2;i++){
        *frame_p = *buffer_p++ << 8;
        *frame_p++ += *buffer_p++;
      }
    } else {
      DEBUG_PRINT("Packet count out of bounds (frame = %d, packet = 0x%x)\n", 
                  this_frame, this_packet + this_packet_msb);
      (*proc->input_put)(proc->input_args, proc->reader);
      thread_data.mallformed_packets++;
      continue;
    }

    if(last_packet_flag){
      // We just recieved the last packet. 
      // Write the frame number and timestamp
      // and push it onto the fifo
      
      frame->number = this_frame;
      frame->timestamp = this_frame_timestamp;
      thread_data.last_frame = this_frame;
      (*proc->output_put)(proc->output_args);
      frame = NULL;
      last_packet_flag = 0;
    }

    /* Now we are done with the packet, we can advance the fifo */

    (*proc->input_put)(proc->input_args, proc->reader);

    /* Now we can set the last packet to this packet */

    last_packet = this_packet;
    last_packet_msb = this_packet_msb;

  } // while(1)

  pthread_exit(NULL);
}

void cin_data_compute_stats(cin_data_stats_t *stats){
  double framerate, datarate;

  static unsigned int last_frame = 0;
  //static unsigned long int n = 0; // Note this will rollover .. then this breaks!

  pthread_mutex_lock(&thread_data.stats_mutex);

  if((unsigned int)thread_data.last_frame != last_frame){
    // Compute framerate

    framerate  =  (double)thread_data.framerate.tv_usec * 1e-6;
    framerate  += (double)thread_data.framerate.tv_sec; 
    if(framerate == 0){
      framerate = 0;
    } else {
      framerate = 1 / framerate;
    }
  } else {
    framerate = 0; 
  }

  datarate = framerate * CIN_DATA_MAX_STREAM * sizeof(uint16_t);
  datarate = datarate / (1024 * 1024); // Convert to Mb.s^-1

  last_frame = (int)thread_data.last_frame;
  stats->last_frame = last_frame;

  stats->framerate = framerate;
  stats->datarate = datarate;
  stats->av_framerate = framerate;
  stats->av_datarate = datarate;

  //if(n == 0){
  //  stats->av_framerate = framerate;
  //  stats->av_datarate = datarate;
  //} else {
  //  stats->av_datarate =  ((n * stats->av_datarate + datarate) / (n + 1));
  //  stats->av_framerate = ((n * stats->av_framerate + framerate) / (n + 1));
  //}
  //n++;

  stats->packet_percent_full = fifo_percent_full(thread_data.packet_fifo);
  stats->frame_percent_full = fifo_percent_full(thread_data.frame_fifo);
  stats->image_percent_full = fifo_percent_full(thread_data.image_fifo);
  stats->packet_overruns = thread_data.packet_fifo->overruns;
  stats->frame_overruns = thread_data.frame_fifo->overruns;
  stats->image_overruns = thread_data.image_fifo->overruns;
  stats->dropped_packets = thread_data.dropped_packets;
  stats->mallformed_packets = thread_data.mallformed_packets;

  pthread_mutex_unlock(&thread_data.stats_mutex);

}

void *cin_data_monitor_output_thread(void){

  //fprintf(stderr, "\033[?25l");

  cin_data_stats_t stats;

  while(1){
    cin_data_compute_stats(&stats);
    cin_data_show_stats(stderr, stats);
    fprintf(stderr, "\033[A\033[A\033[A\033[A\033[A"); // Move up 5 lines 

    usleep(CIN_DATA_MONITOR_UPDATE);
  }

  pthread_exit(NULL);
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

  sprintf(buffer, "Framerate     = %6.1f s^-1 : Data Rate     = %10.3f Mb.s^-1",
          stats.framerate, stats.datarate);
  fprintf(fp, "%-80s\n", buffer);

  //sprintf(buffer, "Av. framerate = %6.1f s^-1 : Av. data Rate = %10.3f Mb.s^-1",
  //        stats.av_framerate, stats.av_datarate);
  //fprintf(fp, "%-80s\n", buffer);

  sprintf(buffer, "Dropped packets %-11ld : Mallformed packets %-11ld",
          stats.dropped_packets, stats.mallformed_packets);
  fprintf(fp, "%-80s\n", buffer);
}

int cin_data_send_magic(void){
  char* dummy = "DUMMY DATA";
  return cin_data_write(thread_data.dp, dummy, sizeof(dummy));
}

void *cin_data_listen_thread(void *args){
  
  struct cin_data_packet *buffer = NULL;
  cin_data_proc_t *proc = (cin_data_proc_t*)args;
 
  /* Send a packet to initialize the CIN */
  cin_data_send_magic();

  int sid = syscall(SYS_gettid);
  DEBUG_PRINT("Starting listener thread ppid = %d\n", sid);


#ifdef __REALTIME__

  /*
   * Try so set the UDP listener thread to use realtime
   * scheduling with a high priority.
   */

  struct sched_param fifo_param;
  fifo_param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  if(!pthread_setschedparam(pthread_self(), SCHED_FIFO, &fifo_param)){
    DEBUG_COMMENT("Set realtime priority to UDP listener\n");
  } else {
    DEBUG_COMMENT("Unable to set realtime priority to UDP listener\n");
  }
 
#endif

  // Release root priv.
  if(setuid(getuid())){
    DEBUG_COMMENT("Unable to drop root priv.\n");
  } else {
    DEBUG_COMMENT("Dropped root priv.\n");
  }

  while(1){
    /* Get the next element in the fifo */
    buffer = (cin_data_packet_t*)(*proc->output_get)(proc->output_args);
    buffer->size = recvfrom(thread_data.dp->sockfd, buffer->data, 
                            CIN_DATA_MAX_MTU * sizeof(unsigned char), 0,
                            (struct sockaddr*)&thread_data.dp->sin_cli, 
                            (socklen_t*)&thread_data.dp->slen);
    //buffer->size = cin_data_read(thread_data.dp, buffer->data);
    if(ioctl(thread_data.dp->sockfd, SIOCGSTAMP, &buffer->timestamp) == -1){
      DEBUG_COMMENT("Unable to read packet time");
      //clock_gettime(CLOCK_REALTIME, &buffer->timestamp);
      gettimeofday(&buffer->timestamp, 0);
    }
    (*proc->output_put)(proc->output_args);
  }

  pthread_exit(NULL);
}


void* cin_data_descramble_thread(void *args){
  /* This routine gets the next frame and descrambles is */

  cin_data_proc_t *proc = (cin_data_proc_t*)args;

  struct cin_data_frame *frame = NULL;
  struct cin_data_frame *image = NULL;
  
  int sid = syscall(SYS_gettid);
  DEBUG_PRINT("Starting descrambler thread ppid = %d\n", sid);
 
  descramble_map_t *map;
  map = malloc(sizeof(descramble_map_t));
  if(!map){
    pthread_exit(NULL);
  }
 
  cin_data_descramble_init(map, CIN_DATA_FRAME_HEIGHT, 2);
  DEBUG_COMMENT("Initialized descramble map\n");

  while(1){
    // Get a frame 
    
    frame = (cin_data_frame_t*)(*proc->input_get)(proc->input_args, proc->reader);
    image = (cin_data_frame_t*)(*proc->output_get)(proc->output_args);

    DEBUG_PRINT("Descrambling frame %d\n", frame->number);

    cin_data_descramble_frame(map, image->data, frame->data); 

    image->timestamp = frame->timestamp;
    image->number = frame->number;
    image->size_x = map->size_x;
    image->size_y = map->size_y;

    // Release the frame and the image

    (*proc->input_put)(proc->input_args, proc->reader);
    (*proc->output_put)(proc->output_args);

  }

  pthread_exit(NULL);
}

void* cin_data_writer_thread(void *args){
  /* This routine gets the next frame and descrambles is */
  cin_data_proc_t *proc = (cin_data_proc_t*)args;

  while(1){
    // Get a frame 
    
    cin_data_frame_t *image = (cin_data_frame_t*)(*proc->input_get)(proc->input_args, proc->reader);
    DEBUG_PRINT("image %d\n", image->number);
    // Release the frame and the image

    (*proc->input_put)(proc->input_args, proc->reader);
  }

  pthread_exit(NULL);
}
/* -----------------------------------------------------------------------------------------
 *
 * Routines for benchmarking
 *
 * -----------------------------------------------------------------------------------------
 */

struct timespec timespec_diff(struct timespec start, struct timespec end){
  /* Calculte the difference between two times */
  struct timespec temp;
  if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }
  return temp;
}

struct timeval timeval_diff(struct timeval start, struct timeval end){
  /* Calculte the difference between two times */
  struct timeval temp;
  if ((end.tv_usec-start.tv_usec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_usec = 1000000+end.tv_usec-start.tv_usec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_usec = end.tv_usec-start.tv_usec;
  }
  return temp;
}


/* -------------------------------------------------------------------------------
 *
 * Routines for accessing the image buffer
 *
 * -------------------------------------------------------------------------------
 */

struct cin_data_frame* cin_data_get_next_frame(void){
  /* This routine gets the next frame. This will block until a frame is avaliable */
  struct cin_data_frame *frame = NULL;

  frame = (struct cin_data_frame*)fifo_get_tail(thread_data.image_fifo, 0);
  return frame;
}

void cin_data_release_frame(int free_mem){
  /* Advance the fifo */
  fifo_advance_tail(thread_data.image_fifo, 0);
}

/* -------------------------------------------------------------------------------
 *
 * Routines for accessing the double buffer
 *
 * -------------------------------------------------------------------------------
 */

struct cin_data_frame* cin_data_get_buffered_frame(void){
  /* This routine gets the buffered frame. 
     This will block until a frame is avaliable */
  struct cin_data_frame *frame = NULL;

  frame = (struct cin_data_frame*)mbuffer_get_read_buffer(thread_data.image_dbuffer);
  return frame;
}

void cin_data_release_buffered_frame(void){
  mbuffer_read_done(thread_data.image_dbuffer);
}

/* -------------------------------------------------------------------------------
 *
 * Routines for accessing the statistics
 *
 * -------------------------------------------------------------------------------
 */

struct cin_data_stats cin_data_get_stats(void){
  /* Return the stats on the data */
  struct cin_data_stats s;
  
  pthread_mutex_lock(&thread_data.stats_mutex);
  s = thread_data.stats;
  pthread_mutex_unlock(&thread_data.stats_mutex);

  return s;
}


/* -------------------------------------------------------------------------------
 *
 * Routines for sequential access to data
 *
 * -------------------------------------------------------------------------------
 */

void* cin_data_buffer_push(void *arg){
  void *rtn; 
  cin_data_callbacks_t *callbacks = (cin_data_callbacks_t*)arg;
  
  // Run the callback to load the data frame
  DEBUG_PRINT("Calling callback %p with frame %p\n", callbacks->push, callbacks->frame);
  (*callbacks->push)(callbacks->frame);

  // We return a type of cin_data_frame to process
  rtn = (void *)callbacks->frame;
  return rtn;
}

void cin_data_buffer_pop(void *arg){
  cin_data_callbacks_t *callbacks = (cin_data_callbacks_t*)arg;

  DEBUG_PRINT("Calling callback %p with frame %p\n", callbacks->pop, callbacks->frame); 
  (*callbacks->pop)(callbacks->frame);

  return;
}
