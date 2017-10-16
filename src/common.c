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

#include "cin_register_map.h"

/* -----------------------------------------------------------------------------------------
 *
 * Routines for configuring the CIN
 *
 * -----------------------------------------------------------------------------------------
 */

int cin_com_set_timing(cin_ctl_t *cin_ctl, cin_data_t *cin_data, int mode)
{
  char _msg[256];
  const char *name;

  // Find the timing mode
  if((mode < 0) || (mode >= cin_ctl->timing_num))
  {
    ERROR_PRINT("Invalid timing mode %d\n", mode);
    snprintf(_msg, 256, "No timing data for mode %d", mode);
    cin_ctl_message(cin_ctl, _msg, CIN_CTL_MSG_MAJOR);
    return CIN_ERROR;
  }

  cin_ctl->current_timing = &cin_ctl->timing[mode];
  name = cin_ctl->timing[mode].name;

  // Now upload timing data to CIN
  DEBUG_PRINT("Uploading timing data from \"%s\".\n", name);
  snprintf(_msg, 256, "Uploading timing data \"%s\"", name);
  cin_ctl_message(cin_ctl, _msg, CIN_CTL_MSG_OK);

  if(cin_ctl_set_timing_regs(cin_ctl, cin_ctl->current_timing->data, 
        cin_ctl->current_timing->data_len) != CIN_OK)
  {
    goto error;
  }

  // Now set the fclk
  if(cin_ctl_set_fclk(cin_ctl, cin_ctl->current_timing->fclk_freq) != CIN_OK)
  {
    goto error;
  }

  if(cin_ctl_write_with_readback(cin_ctl, REG_FRM_SANDBOX_REG0F, 0x5500 | (mode & 0x00FF)) != CIN_OK)
  {
    goto error;
  }

  int _mode;
  if(cin_com_get_timing(cin_ctl, cin_data, &_mode) != CIN_OK)
  {
    goto error;
  }

  DEBUG_PRINT("Timing mode from CIN = %d\n", _mode);

  if(_mode != mode)
  {
    ERROR_COMMENT("Setting timing mode register failed\n");
    goto error;
  }

  snprintf(_msg, 256, "Timing set to \"%s\"", name);
  cin_ctl_message(cin_ctl, _msg, CIN_CTL_MSG_OK);
  return CIN_OK;

error:
  cin_ctl_message(cin_ctl, "Error Uploading Timing", CIN_CTL_MSG_MAJOR);
  return CIN_ERROR;
}

int cin_com_get_timing(cin_ctl_t *cin_ctl, cin_data_t *cin_data, int *mode)
{
  uint16_t val;
  char _msg[256];

  if(cin_ctl_read(cin_ctl, REG_FRM_SANDBOX_REG0F, &val) != CIN_OK)
  {
    goto error;
  }

  DEBUG_PRINT("Timing mode register = 0x%x\n", val);

  if((val & 0xFF00) != 0x5500)
  {
    goto error;
  }

  *mode = val & 0x00FF; 

  DEBUG_PRINT("Timing mode = %d\n", *mode);

  if((*mode < 0) || (*mode >= cin_ctl->timing_num))
  {
    ERROR_PRINT("Invalid timing mode %d\n", *mode);
    snprintf(_msg, 256, "No timing data for mode %d", *mode);
    cin_ctl_message(cin_ctl, _msg, CIN_CTL_MSG_MAJOR);
    goto error;
  }

  cin_ctl->current_timing = &cin_ctl->timing[*mode];

  switch(cin_ctl->current_timing->fclk_freq)
  {
    case CIN_CTL_FCLK_200:
      cin_ctl->fclk_time_factor = 1.0;
      break;
    case CIN_CTL_FCLK_125_C:
      cin_ctl->fclk_time_factor = 125.0 / 200.0;
      break;
    default:
      cin_ctl->fclk_time_factor = 1.0;
      break;
  }

  // Now set the descramble params
  if(cin_data_set_descramble_params(cin_data, cin_ctl->current_timing->rows,
        cin_ctl->current_timing->overscan) != CIN_OK)
  {
    goto error;
  }

  return CIN_OK;

error:
  return CIN_ERROR;
}

int cin_com_boot(cin_ctl_t *cin_ctl, cin_data_t *cin_data, int mode)
{
  // Power cycle the CIN
  
  // Lock Mutex
  pthread_mutex_lock(&cin_ctl->access);

  cin_ctl_message(cin_ctl, "Powering OFF CIN", CIN_CTL_MSG_OK);
  if(cin_ctl_pwr(cin_ctl, 0))
  {
    ERROR_COMMENT("Unable to turn off CIN power\n");
    goto error;
  }
  sleep(1);

  cin_ctl_message(cin_ctl, "Powering ON CIN", CIN_CTL_MSG_OK);
  if(cin_ctl_pwr(cin_ctl, 1))
  {
    ERROR_COMMENT("Unable to turn on CIN power\n");
    goto error;
  }

  sleep(1);

  // Load the firmware

  if(cin_ctl_load_firmware(cin_ctl))
  {
    ERROR_COMMENT("Unable to Upload Firmware\n");
    goto error;
  }

  sleep(1);

  cin_ctl_message(cin_ctl, "Powering ON CIN Front Panel", CIN_CTL_MSG_OK);
  if(cin_ctl_fp_pwr(cin_ctl, 1) != CIN_OK)
  {
    ERROR_COMMENT("Unable to turn on FP power\n");
    goto error;
  }

  sleep(1);

  if(cin_com_set_fabric_comms(cin_ctl, cin_data) != CIN_OK)
  {
    ERROR_COMMENT("Unable to set fabric comms\n");
    goto error;
  }

  if(mode != -1)
  {
    if(cin_com_set_timing(cin_ctl, cin_data, mode) != CIN_OK){
      ERROR_COMMENT("Unable to set timing\n");
      goto error;
    }
  }

  pthread_mutex_unlock(&cin_ctl->access);

  return CIN_OK;

error:
  pthread_mutex_unlock(&cin_ctl->access);
  return CIN_ERROR;
}

int cin_com_set_fabric_comms(cin_ctl_t *cin_ctl, cin_data_t *cin_data)
{
  // Set the fabric comms depending on how the cin_data has been setup
  DEBUG_PRINT("cin_data->addr = %s\n", cin_data->addr);
  cin_ctl_set_fabric_address(cin_ctl, cin_data->addr);
  cin_data_send_magic(cin_data);

  return CIN_OK;
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
  _debug_print_flag = debug;
  DEBUG_COMMENT("Debug output ON\n");
}

void cin_set_error_print(int error){
  _error_print_flag = error;
  ERROR_COMMENT("Error output ON\n");
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
    return CIN_ERROR;
  }
  if((b.tv_sec == a.tv_sec) && (b.tv_nsec > a.tv_nsec)){
    return CIN_ERROR;
  }
  return CIN_OK;
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
      return CIN_ERROR;
  }

  int i = 1;
  if(setsockopt(port->sockfd, SOL_SOCKET, SO_REUSEADDR, 
                (void *)&i, sizeof i) < 0) {
    ERROR_COMMENT("setsockopt() failed.\n");
    return CIN_ERROR;
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
    return CIN_ERROR;
  }

  if(inet_aton(bind_addr, &port->sin_cli.sin_addr) == 0) {
    ERROR_COMMENT("inet_aton() Failed.\n");
    return CIN_ERROR;
  }

  // 

  if(bind(port->sockfd, (struct sockaddr *)&port->sin_cli, 
        sizeof(port->sin_cli))){
    ERROR_COMMENT("Bind failed.\n");
    return CIN_ERROR;
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

  return CIN_OK;
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
