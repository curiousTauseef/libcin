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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "cin.h"
#include "config.h"
#include "common.h"

/* -----------------------------------------------------------------------------------------
 *
 * Routines for configuring the CIN
 *
 * -----------------------------------------------------------------------------------------
 */

int cin_com_set_timing(cin_ctl_t *cin, char *name)
{
  int mode;
  mode = cin_config_find_timing(cin, name);
  if(mode == CIN_CONFIG_ERROR)
  {
    return -1;
  }

  cin_ctl_set_timing_regs(cin, cin->timing[mode].data, cin->timing[mode].data_len);

  return 0;
}

int cin_com_boot(cin_ctl_t *cin_ctl, cin_data_t *cin_data)
{
  // Power cycle the CIN

  if(cin_ctl_pwr(cin_ctl, 0))
  {
    return -1;
  }
  sleep(1);

  if(cin_ctl_pwr(cin_ctl, 1))
  {
    return -1;
  }
  sleep(1);

  // Load the firmware

  if(cin_ctl_load_firmware(cin_ctl))
  {
    return -1;
  }

  // Get the ID of the CIN
  cin_ctl_id_t cin_id;
  if(cin_ctl_get_id(cin_ctl, &cin_id))
  {
    return -1;
  }

  if(cin_com_set_fabric_comms(cin_ctl, cin_data))
  {
    return -1;
  }

  if(mode != NULL)
  {
    if(cin_com_set_timing(cin_ctl, mode)){

    }
  }

  return 0;
}

int cin_com_set_fabric_comms(cin_ctl_t *cin_ctl, cin_data_t *cin_data)
{
  // Set the fabric comms depending on how the cin_data has been setup

  cin_ctl_set_fabric_address(cin_ctl, cin_data->addr);
  cin_data_send_magic(cin_data);

  return 0;
}

/* -----------------------------------------------------------------------------------------
 *
 * Routines for debug printing
 *
 * -----------------------------------------------------------------------------------------
 */

int _debug_print_flag = 0;
int _error_print_flag = 1;

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

/* -----------------------------------------------------------------------------------------
 *
 * IP Communication Routines
 *
 * -----------------------------------------------------------------------------------------
 */

int cin_com_init_port(cin_port_t *port, const char *cin_addr, int cin_port, 
                       const char *bind_addr, int bind_port, int recv_buf)
{

  DEBUG_PRINT("CIN address     = %s:%d\n", cin_addr, cin_port);
  DEBUG_PRINT("Bind to address = %s:%d\n", bind_addr, bind_port);

  // Open Datagram (UDP) Socket
  
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

  // According to convention
  // srv -> CIN 
  // cli -> host computer
  
  memset(&port->sin_srv, 0, sizeof(struct sockaddr_in));
  memset(&port->sin_cli, 0, sizeof(struct sockaddr_in));

  // htons is used to correct the endian for IP comms
  // family is always AF_INET
  
  port->sin_srv.sin_family = AF_INET;
  port->sin_srv.sin_port = htons(cin_port);
  port->sin_cli.sin_family = AF_INET;
  port->sin_cli.sin_port = htons(bind_port);
  port->slen = sizeof(struct sockaddr_in);

  // Use inet_aton to convert from ip4 dot format (string) to address
  
  if(inet_aton(cin_addr, &port->sin_srv.sin_addr) == 0) {
    ERROR_COMMENT("inet_aton() Failed.\n");
    return 1;
  }

  if(inet_aton(bind_addr, &port->sin_cli.sin_addr) == 0) {
    ERROR_COMMENT("inet_aton() Failed.\n");
    return 1;
  }

  // 

  if(bind(port->sockfd, (struct sockaddr *)&port->sin_cli, 
        sizeof(port->sin_cli))){
    ERROR_COMMENT("Bind failed.\n");
    return -1;
  }

  if(recv_buf)
  {
  
    DEBUG_PRINT("Requesting recieve buffer of %d (%d Mb) \n", 
                recv_buf, recv_buf / (1024 * 1024));

    if(setsockopt(port->sockfd, SOL_SOCKET, SO_RCVBUF, 
                  &recv_buf, sizeof(recv_buf)) == -1){
      ERROR_COMMENT("Unable to set receive buffer.");
    } 

    int recv_buf_rb;
    socklen_t rcvbuf_rb_len = sizeof(recv_buf_rb);
    if(getsockopt(port->sockfd, SOL_SOCKET, SO_RCVBUF,
                  &recv_buf_rb, &rcvbuf_rb_len) == -1){
      ERROR_COMMENT("Unable to get receive buffer.");
    }

    DEBUG_PRINT("Recieve buffer returns %d (%d Mb)\n",
        recv_buf_rb, recv_buf_rb / (1024*1024) );

    if(recv_buf_rb/2 != recv_buf){
      ERROR_PRINT("WARNING : Unable to set RCVBUFFER to size %d (%d Mb)\n",
        recv_buf, recv_buf / (1024*1024));
      ERROR_PRINT("WARNING : RCVBUFFER is set to %d Mb (%d bytes)\n",
        recv_buf_rb, recv_buf_rb / (1024*1024));
    }
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

int cin_com_set_int(int val, int def)
{
  if(val == 0)
  {
    return def;
  }

  return val;
}

char *cin_com_set_string(char *val, char *def)
{
  char *_val, *_rtn;
  if(val != NULL)
  {
    _val = val;
  } else {
    _val = def;
  }

  // Now copy

  int slen;
  slen = strlen(_val);
 
  _rtn = malloc(sizeof(char) * slen);
  if(_rtn == NULL)
  {
    return NULL;
  }

  strcpy(_rtn, _val);

  return _rtn;
}
