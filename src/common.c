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

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "cin.h"
#include "common.h"

/* -----------------------------------------------------------------------------------------
 *
 * Routines for benchmarking
 *
 * -----------------------------------------------------------------------------------------
 */

int _debug_print_flag = 0;
int _error_print_flag = 0;

void cin_set_debug_print(int debug){
  fprintf(stderr, "%s:%d:%s(): Set DEBUG to %d from %d\n", __FILE__, __LINE__, __func__,
          debug, _debug_print_flag);
  _debug_print_flag = debug;
}

void cin_set_error_print(int error){
  fprintf(stderr, "%s:%d:%s(): Set ERROR to %d from %d\n", __FILE__, __LINE__, __func__,
          error, _error_print_flag);
  _error_print_flag = error;
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

int timespec_after(struct timespec a, struct timespec b){
  /* Calculte if B is afetr A */

  if(b.tv_sec > a.tv_sec){
    return 1;
  }
  if((b.tv_sec == a.tv_sec) && (b.tv_nsec > a.tv_nsec)){
    return 1;
  }
  return 0;
}

int cin_init_port(cin_port_t *port){

  DEBUG_PRINT("Client address = %s:%d\n", port->cliaddr, port->cliport);
  DEBUG_PRINT("Server address = %s:%d\n", port->srvaddr, port->srvport);

  port->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (port->sockfd < 0) {
    ERROR_COMMENT("socket() failed.\n");
      return -1;
  }

  int i = 1;
  if(setsockopt(port->sockfd, SOL_SOCKET, SO_REUSEADDR, 
                (void *)&i, sizeof i) < 0) {
    ERROR_COMMENT("setsockopt() failed.\n");
    return -1;
  }

  // initialize CIN (server) and client (us!) sockaddr structs
  memset(&port->sin_srv, 0, sizeof(struct sockaddr_in));
  memset(&port->sin_cli, 0, sizeof(struct sockaddr_in));

  port->sin_srv.sin_family = AF_INET;
  port->sin_srv.sin_port = htons(port->srvport);
  port->sin_cli.sin_family = AF_INET;
  port->sin_cli.sin_port = htons(port->cliport);
  port->slen = sizeof(struct sockaddr_in);

  if(inet_aton(port->srvaddr, &port->sin_srv.sin_addr) == 0) {
    ERROR_COMMENT("inet_aton() Failed.\n");
    return 1;
  }

  if(inet_aton(port->cliaddr, &port->sin_cli.sin_addr) == 0) {
    ERROR_COMMENT("inet_aton() Failed.\n");
    return 1;
  }

  if(bind(port->sockfd, (struct sockaddr *)&port->sin_cli, sizeof(port->sin_cli))){
    ERROR_COMMENT("Bind failed.\n");
    return -1;
  }

  port->rcvbuf = port->rcvbuf * 1024 * 1024; // Convert to Mb
  DEBUG_PRINT("Requesting recieve buffer of %d Mb (%d) \n", 
      port->rcvbuf / (1024*1024), port->rcvbuf );

  if(setsockopt(port->sockfd, SOL_SOCKET, SO_RCVBUF, 
                &port->rcvbuf, sizeof(port->rcvbuf)) == -1){
    ERROR_COMMENT("CIN data port - unable to set receive buffer :");
  } 

  socklen_t rcvbuf_rb_len = sizeof(port->rcvbuf_rb);
  if(getsockopt(port->sockfd, SOL_SOCKET, SO_RCVBUF,
                &port->rcvbuf_rb, &rcvbuf_rb_len) == -1){
    ERROR_COMMENT("CIN data port - unable to get receive buffer :");
  }

  DEBUG_PRINT("Recieve buffer returns %d Mb (%d)\n",
      port->rcvbuf_rb / (1024*1024), port->rcvbuf_rb );

  if(port->rcvbuf_rb/2 != port->rcvbuf){
    ERROR_PRINT("WARNING : Unable to set RCVBUFFER to size %d Mb (%d bytes)\n",
      port->rcvbuf / (1024*1024), port->rcvbuf );
    ERROR_PRINT("WARNING : RCVBUFFER is set to %d Mb (%d bytes)\n",
      port->rcvbuf_rb / (1024*1024), port->rcvbuf_rb );
  }


  int zero = 0;
  if(setsockopt(port->sockfd, SOL_SOCKET, SO_TIMESTAMP,
                &zero, sizeof(zero)) == -1){
    ERROR_COMMENT("Unable to set sockopt SO_TIMESTAMP");
  } else {
    DEBUG_COMMENT("Set SO_TIMESTAMP to zero\n");
  }

  return 0;
}

