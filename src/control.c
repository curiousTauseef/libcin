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
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include "cin.h"
#include "cin_register_map.h"
#include "cinregisters.h"
#include "control.h"
#include "config.h"
#include "common.h"
#include "fifo.h"
#include "fcric.h"
#include "bias.h"

extern unsigned char cin_config_firmware[];
extern unsigned int cin_config_firmware_len;

/**************************** INITIALIZATION **************************/

int cin_ctl_init(cin_ctl_t *cin, 
                 char *addr, uint16_t port, uint16_t sport, 
                 char *bind_addr, uint16_t bind_port, uint16_t bind_sport) 
{ 
  cin->msg_callback = NULL;
  cin->msg_callback_ptr = NULL;

  // Initialize the config

  cin_config_init(cin);
  cin->current_timing = NULL;

  cin->fclk_time_factor = 125.0 / 200.0;          // 200 MHz 1us
  //cin->fclk_time_factor = 1.0;          // 200 MHz 1us

  cin->addr       = cin_com_set_string(addr, CIN_CTL_IP);
  cin->bind_addr  = cin_com_set_string(bind_addr, "0.0.0.0");
  cin->port       = cin_com_set_int(port, CIN_CTL_CIN_PORT);
  cin->bind_port  = cin_com_set_int(bind_port, CIN_CTL_BIND_PORT);
  cin->sport      = cin_com_set_int(sport, CIN_CTL_FRMW_CIN_PORT);
  cin->bind_sport = cin_com_set_int(bind_sport, CIN_CTL_FRMW_BIND_PORT);

  if(cin_com_init_port(&cin->ctl_port, 
                       cin->addr, cin->port, 
                       cin->bind_addr, cin->bind_port, 0) || 
     cin_com_init_port(&cin->stream_port, 
                       cin->addr, cin->sport, 
                       cin->bind_addr, cin->bind_sport, 0)){
     ERROR_COMMENT("Unable to open commuincations\n");
     return CIN_ERROR;
  }

  DEBUG_COMMENT("Opened ports\n");

  // Now initialize a listener thread for UDP communications

  cin_ctl_listener_t *listener = malloc(sizeof(cin_ctl_listener_t));
  cin->listener = listener;

  if(!listener){
    ERROR_COMMENT("Unable to create listener (MALLOC Failed)\n");
    return CIN_ERROR;
  }

  listener->cp = &(cin->ctl_port);
  if(fifo_init(&listener->ctl_fifo, sizeof(uint32_t), 100)){
    ERROR_COMMENT("Failed to initialize fifo.\n");
    return CIN_ERROR;
  }

  // Initialize the mutex for sequential access
  pthread_mutexattr_init(&cin->access_attr);
  pthread_mutexattr_settype(&cin->access_attr, PTHREAD_MUTEX_RECURSIVE);
  pthread_mutex_init(&cin->access, &cin->access_attr);
  pthread_barrier_init(&listener->barrier, NULL, 2);

  pthread_create(&listener->thread_id, NULL, 
                 cin_ctl_listen_thread, (void*)listener);
  
  // Now wait for the listening thread to start

  DEBUG_COMMENT("Waiting for listening thread to start ....\n");

  pthread_barrier_wait(&listener->barrier);
  pthread_barrier_destroy(&listener->barrier);

  DEBUG_COMMENT("Listening thread has started.\n");

  return CIN_OK;
}

int cin_ctl_destroy(cin_ctl_t *cin){
  DEBUG_COMMENT("Closing comm ports\n");
  cin_ctl_close_ports(cin);
  DEBUG_COMMENT("Goodbye .....\n");
  return CIN_OK;
}

void cin_ctl_message(cin_ctl_t *cin, const char *message, int severity)
{
  if(cin->msg_callback != NULL)
  {
    // Call the callback
    (*cin->msg_callback)(message, severity, cin->msg_callback_ptr);
  }
}

void cin_ctl_set_msg_callback(cin_ctl_t *cin, cin_ctl_msg_callback callback, void *ptr)
{
  cin->msg_callback = callback;
  cin->msg_callback_ptr = ptr;
}

/**************************** UDP Socket ******************************/

uint32_t cin_ctl_get_packet(cin_ctl_t *cin, uint32_t *val){
  long int r;
  struct timeval start, now, diff;

  gettimeofday(&start, NULL);
 
  while(1)
  {
    r = fifo_used_bytes(&cin->listener->ctl_fifo);
    if(r){
      *val = *((uint32_t *)fifo_get_tail(&cin->listener->ctl_fifo));
      fifo_advance_tail(&cin->listener->ctl_fifo);
      return CIN_OK;
    }
    
    gettimeofday(&now, NULL);
    timersub(&now, &start, &diff);
    if(diff.tv_sec > 2)
    {
      ERROR_PRINT("Read timeout after %d seconds\n", 2);
      break;
    }
  }

  return CIN_ERROR;
}

// This is to be run in a different thread

void *cin_ctl_listen_thread(void* args){
  uint32_t *buffer;
  uint32_t val;
  int i;
  cin_ctl_listener_t *data = (cin_ctl_listener_t*)args;

  // Parse the data structure passed to the thread

  DEBUG_COMMENT("Started listener thread.\n");

  pthread_barrier_wait(&data->barrier);
  while(1){
    buffer = (uint32_t*)fifo_get_head(&data->ctl_fifo);
    i = recvfrom(data->cp->sockfd, &val, sizeof(val), 0,
                 (struct sockaddr*)&data->cp->sin_cli,
                 (socklen_t*)&data->cp->slen);
    if(i != -1){
      *buffer = ntohl(val);
#ifdef __DEBUG_STREAM_THREAD__
      DEBUG_PRINT("Received %d bytes value  = %08lX\n", i, (long int)(*buffer));
#endif
      fifo_advance_head(&data->ctl_fifo);
    } else {
      DEBUG_COMMENT("Timeout\n");
    }
  }

  pthread_exit(NULL);
}

int cin_ctl_close_ports(cin_ctl_t *cin) {

  if(cin->ctl_port.sockfd){
    DEBUG_COMMENT("Canceling thread\n");
    pthread_cancel(cin->listener->thread_id);
    pthread_join(cin->listener->thread_id, NULL);
    DEBUG_COMMENT("Thread returned\n");
    close(cin->ctl_port.sockfd); 
    cin->ctl_port.sockfd = 0;
  }

  if(cin->stream_port.sockfd){
    close(cin->stream_port.sockfd); 
    cin->stream_port.sockfd = 0;
  }

  return CIN_OK;
}

/*************************** CIN Read/Write ***************************/

int cin_ctl_write(cin_ctl_t *cin, uint16_t reg, uint16_t val, int wait){

  uint32_t _valwr;
  int rc;
  cin_port_t *cp = &(cin->ctl_port);
  
  // Lock the mutex for access
  pthread_mutex_lock(&cin->access);

  _valwr = ntohl((uint32_t)(reg << 16 | val));
  rc = sendto(cp->sockfd, &_valwr, sizeof(_valwr), 0,
        (struct sockaddr*)&cp->sin_srv,
        sizeof(cp->sin_srv));

  if(wait){
    usleep(CIN_CTL_WRITE_SLEEP);
  }

  // Unlock Mutex for access
  pthread_mutex_unlock(&cin->access);

  if (rc != sizeof(_valwr) ) {
    ERROR_COMMENT("CIN control port - sendto() failure!!\n");
    goto error;
  }

#ifdef __DEBUG_STREAM__ 
  DEBUG_PRINT("Set register 0x%04X to 0x%04X\n", reg, val);
#endif 

  return CIN_OK;
   
error:  
  ERROR_COMMENT("Write error control port\n");
  return CIN_ERROR;
}

int cin_ctl_write_with_readback(cin_ctl_t *cin, uint16_t reg, uint16_t val){
  
  int try = CIN_CTL_MAX_WRITE_TRIES;
  uint16_t _val;

  pthread_mutex_lock(&cin->access);

  while(try){
    if(cin_ctl_write(cin, reg, val, 1) != CIN_OK)
    {
      ERROR_PRINT("Error writing register %x\n", reg);
      continue;
    }

    if(cin_ctl_read(cin, reg, &_val, 1) != CIN_OK)
    {
      ERROR_PRINT("Unable to read register %x\n", reg);
      continue;
    }

#ifdef __DEBUG_STREAM__
    DEBUG_PRINT("try = %d sent = 0x%04X recv = 0x%04X\n", try, val, _val); 
#endif

    if(_val == val){
      // Value correct
      break;
    }
    try--;
  }

  if(_val != val)
  {
    // We failed
    ERROR_PRINT("Unable to set reg 0x%04X after %d tries.\n", reg, CIN_CTL_MAX_WRITE_TRIES);
    pthread_mutex_unlock(&cin->access);
    return CIN_ERROR;
  }
    
  pthread_mutex_unlock(&cin->access);
  return CIN_OK;

}

int cin_ctl_stream_write(cin_ctl_t *cin, unsigned char *val,int size) {
 
  cin_port_t *cp = &(cin->stream_port);
    
  pthread_mutex_lock(&cin->access);

  unsigned char *_val = val;
  int _size = size;
  int _chunk;
  while(_size > 0)
  {
    if(_size > CIN_CTL_STREAM_CHUNK)
    {
      _chunk = CIN_CTL_STREAM_CHUNK;
    } else {
      _chunk = _size;
    }
    if((_chunk != sendto(cp->sockfd, _val, _chunk, 0,
                          (struct sockaddr*)&cp->sin_srv,
                          sizeof(cp->sin_srv))))
    {
      ERROR_PRINT("sendto() failure %d vs %d!! errno = %d\n", _chunk, size, errno);
      goto error;
    }
    _val += _chunk;
    _size -= _chunk;
    usleep(CIN_CTL_STREAM_SLEEP); // For flow control
  }

  pthread_mutex_unlock(&cin->access);

  /*** TODO - implement write verification procedure ***/
  return CIN_OK;  
   
error:   
   ERROR_COMMENT("Write error\n");
   return CIN_ERROR;
}                                      

int cin_ctl_read(cin_ctl_t *cin, uint16_t reg, uint16_t *val, int wait)
{
  int _status;
  uint32_t buf = 0;

#ifdef __DEBUG_STREAM__
  DEBUG_PRINT("Read reg %04X\n", reg);
#endif 

  pthread_mutex_lock(&cin->access);

  int tries = CIN_CTL_MAX_READ_TRIES;
  while(tries)
  {
    fifo_flush(&cin->listener->ctl_fifo);     

    _status = cin_ctl_write(cin, REG_READ_ADDRESS, reg, 0);
    if (_status != CIN_OK)
    {
      goto error;
    }

    if(wait)
    {
      usleep(CIN_CTL_READ_SLEEP);
    }

    _status = cin_ctl_write(cin, REG_COMMAND, CMD_READ_REG, 0);
    if (_status != CIN_OK)
    {
     ERROR_COMMENT("Write Error\n");
      goto error;
    }

    if(cin_ctl_get_packet(cin, &buf) == CIN_OK)
    {
      if((buf >> 16) == reg)
      {
        break;
      } else {
        ERROR_PRINT("Read value is register 0x%04X was expecting 0x%04X (try %d)\n", 
                    (buf >> 16), reg, tries);
      }
    } else {
      ERROR_PRINT("Read timeout on try %d\n", tries);
    }

    tries--;
  }

  if(!tries){ 
    ERROR_PRINT("Failed to read register %04X\n", reg);
    goto error;
  }

#ifdef __DEBUG_STREAM__
  DEBUG_PRINT("Got %04X from %04X\n", buf & 0xFFFF, (buf >> 16));
#endif


  *val = (uint16_t)buf;
  pthread_mutex_unlock(&cin->access);
  return CIN_OK;
    
error:  
   pthread_mutex_unlock(&cin->access);
   return CIN_ERROR;
}

 
/******************* CIN PowerUP/PowerDown *************************/
int cin_ctl_pwr(cin_ctl_t *cin, int pwr){

  int _status;
  uint16_t _val;

  if(pwr)
  {
    DEBUG_COMMENT("Powering ON CIN Board\n");
    _val = CIN_CTL_POWER_ENABLE;
  } else {
    DEBUG_COMMENT("Powering OFF CIN Board\n");
    _val = CIN_CTL_POWER_DISABLE;
  }

  _status=cin_ctl_write_with_readback(cin,REG_PS_ENABLE, _val);
  if (_status != CIN_OK)
  {
    return CIN_ERROR;
  }
  _status=cin_ctl_write(cin,REG_COMMAND, CMD_PS_ENABLE, 0);
  if (_status != CIN_OK)
  {
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_fp_pwr(cin_ctl_t *cin, int pwr){ 

  int _status;
  uint16_t _val;

  _status = cin_ctl_read(cin, REG_PS_ENABLE, &_val, 0);
  if(_status != 0){
    goto error;
  }

  if((_val & CIN_CTL_POWER_ENABLE) == CIN_CTL_POWER_ENABLE){
    // We are powered up do we ever want to turn on FP power?

    if(pwr){
      DEBUG_COMMENT("Powering ON CIN Front Panel Boards\n");
      _val |= CIN_CTL_FP_POWER_ENABLE;
    } else {
      DEBUG_COMMENT("Powering OFF CIN Front Panel Boards\n");
      _val &= ~CIN_CTL_FP_POWER_ENABLE;
    }

    _status=cin_ctl_write_with_readback(cin,REG_PS_ENABLE, _val);
    if(_status != 0){
      goto error;
    }
    _status=cin_ctl_write(cin,REG_COMMAND, CMD_PS_ENABLE, 0);
    if(_status != 0){
      goto error;
    }
  }

  return CIN_OK;
   
error:
   return CIN_ERROR;
}

int cin_ctl_fo_test_pattern(cin_ctl_t *cin, int on_off){
  int _status = CIN_OK;
  uint16_t _val1;
  uint16_t _val2;

  if(on_off){
    _val1 = 0x0001;
    _val2 = 0x0000;
    DEBUG_COMMENT("Enabeling FO Test Pattern\n");
  } else {
    _val1 = 0x0000;
    _val2 = 0xFFFF;
    DEBUG_COMMENT("Disabeling FO Test Pattern\n");
  }

  _status |= cin_ctl_write_with_readback(cin,REG_FCRIC_WRITE0_REG, 0x9E00);
  _status |= cin_ctl_write_with_readback(cin,REG_FCRIC_WRITE1_REG, 0x0000);
  _status |= cin_ctl_write_with_readback(cin,REG_FCRIC_WRITE2_REG, _val1);
  _status |= cin_ctl_write(cin,REG_FRM_COMMAND, CMD_SEND_FCRIC_CONFIG, 0);

  usleep(CIN_CTL_FO_SLEEP); // For flow control

  _status |= cin_ctl_write_with_readback(cin,REG_FCRIC_MASK_REG1, _val2);
  _status |= cin_ctl_write_with_readback(cin,REG_FCRIC_MASK_REG2, _val2);
  _status |= cin_ctl_write_with_readback(cin,REG_FCRIC_MASK_REG3, _val2);

  return _status;
}

/******************* CIN Configuration/Status *************************/

int cin_ctl_load_config(cin_ctl_t *cin,const char *filename){

  int _status;
  uint32_t _regul,_valul;
  char _regstr[12],_valstr[12],_line[1024];

  // We are going to check if bias and clocks are on. 
 
  int _val;
  _status = cin_ctl_get_bias(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read bias status. Refusing to upload config\n");
    return CIN_ERROR;
  }
  if(_val){
    ERROR_COMMENT("Cannot upload with bias on.\n");
    return CIN_ERROR;
  }

  _status = cin_ctl_get_clocks(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read clock status. Refusing to upload config\n");
    return CIN_ERROR;
  }
  if(_val){
    ERROR_COMMENT("Cannot upload with clocks on.\n");
    return CIN_ERROR;
  }

  FILE *file = fopen(filename, "r");
  if(file == NULL){
    ERROR_PRINT("Unable to open file %s\n", filename);
    goto error;
  }

  DEBUG_PRINT("Loading %s\n",filename);
   
  /* Read a line an filter out comments */     

  // Lock the mutex to get exclusive access to the CIN

  pthread_mutex_lock(&cin->access);

  while(fgets(_line,sizeof _line,file) != NULL){ 
    _line[strlen(_line)-1]='\0';   
    //fprintf(stderr, "Line\n");
    if ('#' == _line[0] || '\0' == _line[0]){
      //fprintf(stdout," Ignore line\n"); //DEBUG 
    } else { 
      sscanf (_line,"%s %s",_regstr,_valstr);
      _regul=strtoul(_regstr,NULL,16);
      _valul=strtoul(_valstr,NULL,16);          
      usleep(CIN_CTL_CONFIG_SLEEP);   /*for flow control*/ 
      _status=cin_ctl_write(cin, _regul, _valul, 0);
      if (_status != 0){
        ERROR_COMMENT("Error writing to CIN\n");
        fclose(file);
        goto error;
      }
   }
  }
 
  DEBUG_COMMENT("Done.\n");
  pthread_mutex_unlock(&cin->access);
  
  fclose(file);
  return CIN_OK;
  
error:
  pthread_mutex_unlock(&cin->access);
  return CIN_ERROR;
}

int cin_ctl_load_firmware_file(cin_ctl_t *cin, char *filename){
   
  size_t num_e;
  FILE *file = NULL;
  unsigned char *buffer = NULL;
  size_t buffer_max = 20000000;
  int _status = -1; 

  if((buffer = (unsigned char *)malloc(buffer_max)) == NULL)
  {
    ERROR_COMMENT("Unable to allocate memory for firmware buffer\n");
    goto error;
  }

  file = fopen(filename, "rb");
  if(file == NULL){ 
    ERROR_COMMENT("Failed to open file.\n");
    goto error;
  }

  DEBUG_PRINT("Loading %s\n", filename);

  num_e = fread(buffer, sizeof(unsigned char), buffer_max, file);
  if(num_e == 0)
  {
    ERROR_PRINT("fread error %d %d\n", ferror(file), feof(file));
    goto error;
  }

  fclose(file);

  DEBUG_PRINT("Read %d bytes (max = %d)\n", (int)num_e, (int)buffer_max);

  _status = cin_ctl_load_firmware_data(cin, buffer, (int)num_e);
  return _status;

error:
  if(buffer != NULL)
  {
    free(buffer);
  }
  if(file != NULL)
  {
    fclose(file);
  }
  return CIN_ERROR;
}

int cin_ctl_load_firmware(cin_ctl_t *cin)
{
  return cin_ctl_load_firmware_data(cin, cin_config_firmware, cin_config_firmware_len);
}

int cin_ctl_load_firmware_data(cin_ctl_t *cin, unsigned char *data, int data_len)
{
  int _status;

  cin_ctl_message(cin, "Sending frame FPGA firmware to CIN", CIN_CTL_MSG_OK);
  // Lock the mutex for exclusive access to the CIN

  pthread_mutex_lock(&cin->access);

  DEBUG_COMMENT("Placing CIN in firmware program mode\n");
  _status = cin_ctl_write(cin, REG_COMMAND, CMD_PROGRAM_FRAME, 0); 
  if (_status != CIN_OK){
    ERROR_COMMENT("Failed to program CIN\n");
    goto error;
  }   

  sleep(1);
  DEBUG_PRINT("Starting to upload firmware (%d bytes)\n", data_len);
  _status = cin_ctl_stream_write(cin, data, data_len);
  if (_status != CIN_OK){
    ERROR_COMMENT("Error writing firmware to CIN\n");
    goto error;
  }

  DEBUG_COMMENT("Done.\n");
  
  sleep(1);

  DEBUG_COMMENT("Resetting Frame FPGA\n");

  _status=cin_ctl_write(cin, REG_FRM_RESET, 0x0001, 0);
  if(_status != CIN_OK){
    goto error;
  } 

  sleep(1); 

  _status=cin_ctl_write(cin,REG_FRM_RESET,0x0000, 0);
  if(_status != CIN_OK){
    goto error;
  } 

  sleep(1);
   
  DEBUG_COMMENT("Done.\n");

  uint16_t _fpga_status;
  _status = cin_ctl_get_cfg_fpga_status(cin, &_fpga_status);
  _status |= !(_fpga_status & CIN_CTL_FPGA_STS_CFG);
  if(_status != CIN_OK){
    DEBUG_COMMENT("FPGA Failed to configure.\n");
    goto error;
  }
  
  // Get the ID of the CIN
  cin_ctl_id_t cin_id;
  if(cin_ctl_get_id(cin, &cin_id))
  {
    ERROR_COMMENT("Unable to get ID of CIN\n");
    goto error;
  }
  
  pthread_mutex_unlock(&cin->access);

  DEBUG_COMMENT("FPGA Configured OK\n");

  char _msg[256];
  snprintf(_msg, 256, "Firmware 0x%04X uploaded and FPGA configured", cin_id.fabric_fpga_ver);
  cin_ctl_message(cin, _msg, CIN_CTL_MSG_OK);

  return CIN_OK;
   
error:
  pthread_mutex_unlock(&cin->access);
  cin_ctl_message(cin, "Unable to upload firmware", CIN_CTL_MSG_MAJOR);
  return CIN_ERROR;
}

int cin_ctl_freeze_dco(cin_ctl_t *cin, int freeze){
  int _status;

  _status = cin_ctl_write_with_readback(cin,REG_FCLK_I2C_ADDRESS, 0xB089);

  if(freeze){
    _status |= cin_ctl_write_with_readback(cin,REG_FCLK_I2C_DATA_WR, 0xF010);
  } else {
    _status |= cin_ctl_write_with_readback(cin,REG_FCLK_I2C_DATA_WR, 0xF000);
  }

  _status |= cin_ctl_write(cin,REG_FRM_COMMAND, CMD_FCLK_COMMIT, 0);

  sleep(1);

  if(!freeze){
    // Start the DCO up
    _status |= cin_ctl_write_with_readback(cin,REG_FCLK_I2C_ADDRESS, 0xB087);
    _status |= cin_ctl_write_with_readback(cin,REG_FCLK_I2C_DATA_WR, 0xF040);
    _status |= cin_ctl_write(cin,REG_FRM_COMMAND, CMD_FCLK_COMMIT, 0);
    sleep(1);
  }

  if(!_status){
    DEBUG_PRINT("Set FCLK Freeze/Unfreeze to %d\n", freeze);
  } else {
    ERROR_PRINT("Error setting FCLK Freeze/Unfreeze to %d\n", freeze);
  }
  return _status; 
}

int cin_ctl_set_fclk_regs(cin_ctl_t *cin, int clkfreq)
{
  int _status = CIN_OK;

  DEBUG_PRINT("Setting FCLK DCO to %d\n", clkfreq);

  if(cin_ctl_freeze_dco(cin, 1) != CIN_OK)
  {
    ERROR_COMMENT("Unable to freeze DCO\n");
    return CIN_ERROR;
  }

  usleep(CIN_CTL_DCO_SLEEP);

  int i;
  for(i=0;i<CIN_CTL_FCLK_NUM_REG;i++)
  {
    _status |= cin_ctl_write(cin,REG_FCLK_I2C_ADDRESS, CIN_FCLK_REG[i], 1);
    _status |= cin_ctl_write(cin,REG_FCLK_I2C_DATA_WR, CIN_FCLK_PROGRAM[clkfreq][i], 1);
    _status |= cin_ctl_write(cin,REG_FRM_COMMAND, CMD_FCLK_COMMIT, 1);
    usleep(200000);
  }

  if(_status != CIN_OK)
  {
    ERROR_COMMENT("Unable to set FCLK regs\n");
    return CIN_ERROR;
  }

  if(cin_ctl_freeze_dco(cin, 0) != CIN_OK)
  {
    ERROR_COMMENT("Unable to unfreeze DCO\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_set_fclk(cin_ctl_t *cin, int clkfreq){

  int _status = CIN_OK;

  pthread_mutex_lock(&cin->access);

  switch(clkfreq)
  {
    case CIN_CTL_FCLK_125_C:
      _status = cin_ctl_set_fclk_regs(cin, CIN_FCLK_PROGRAM_125);
      break;

    case CIN_CTL_FCLK_200_C:
      _status = cin_ctl_set_fclk_regs(cin, CIN_FCLK_PROGRAM_200);
      break;

    case CIN_CTL_FCLK_250_C:
      _status = cin_ctl_set_fclk_regs(cin, CIN_FCLK_PROGRAM_250);
      break;

    case CIN_CTL_FCLK_200:
      _status = cin_ctl_write_with_readback(cin, REG_FCLK_I2C_DATA_WR, CMD_FCLK_200);
      break;

    default:
      ERROR_PRINT("Invalid clock frequency %d\n", clkfreq);
      goto error;
      break;
  }

  if(_status != CIN_OK){
    ERROR_COMMENT("Unable to set FCLK frequency.\n");
    goto error;
  }

  // Now verrify that fclk has been set
  
  int _fclk;
  if(cin_ctl_get_fclk(cin, &_fclk) != CIN_OK)
  {
    ERROR_COMMENT("Unable to read FCLK value.\n");
    goto error;
  }

  if(_fclk != clkfreq)
  {
    ERROR_PRINT("Failed to set fclk frequency. Set %d got %d\n", clkfreq, _fclk);
    goto error;
  }

  DEBUG_PRINT("Set FCLK to %d\n", clkfreq);
  pthread_mutex_unlock(&cin->access);
  return CIN_OK;

error:
  pthread_mutex_unlock(&cin->access);
  return CIN_ERROR;
}

int cin_ctl_get_fclk(cin_ctl_t *cin, int *clkfreq)
{
  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cin, REG_FCLK_I2C_DATA_WR, &_val, 0);
  if(_status){
    ERROR_COMMENT("Unable to get FCLK status.\n");
    return CIN_ERROR;
  }

  DEBUG_PRINT("FCLK Status = 0x%04X\n", _val);

  if((_val & 0xF000) == 0xF000){
    /* Not standard clock frequency */
    int i;
    _status = 0;
    uint16_t _reg[CIN_FCLK_READ_N];

    // Get CIN FCLK Registers

    for(i=0;i<CIN_FCLK_READ_N;i++){
      _status |= cin_ctl_write(cin, REG_FCLK_I2C_ADDRESS, CIN_FCLK_READ[i], 0);
      _status |= cin_ctl_write(cin, REG_FRM_COMMAND, CMD_FCLK_COMMIT, 0);
      usleep(CIN_CTL_FCLK_SLEEP);
      _status |= cin_ctl_read(cin, REG_FCLK_I2C_DATA_RD, &_reg[i], 1);
    }

    // Print Reisters to debug stream

    for(i=0;i<CIN_FCLK_READ_N;i++){
      DEBUG_PRINT("FCLK REG 0x%04X = 0x%04X %d\n", CIN_FCLK_READ[i], _reg[i], _status);
    }

    // Calculate HS Divider

    uint16_t _hsd = ((_reg[1] & 0x00E0) >> 5) + 4;
    DEBUG_PRINT("FCLK HS Divider = %d\n", _hsd);

    // Calculate N1 Divider
    
    uint16_t _n1 = ((_reg[1] & 0x001F) << 2) + (_reg[2] & 0x00C0 >> 6);
    if(_n1 % 2) _n1++;
    DEBUG_PRINT("FCLK N1 Divider = %d\n", _n1);

    DEBUG_PRINT("FCLK RFREQ      = %01X%01X.%01X%02X%02X%02X\n", (_reg[2] & 0x000F),
                                                                 (_reg[3] & 0x00F0) >> 4,
                                                                 (_reg[3] & 0x000F),
                                                                 (_reg[4] & 0x00FF),
                                                                 (_reg[5] & 0x00FF),
                                                                 (_reg[6] & 0x00FF));

    if((_hsd == 4) && (_n1 == 8)){

      *clkfreq = CIN_CTL_FCLK_156_C;
      DEBUG_PRINT("FCLK is at 156 MHz (%d)\n", *clkfreq);
      return CIN_OK;

    } else if((_hsd == 4) && (_n1 == 10)){

      *clkfreq = CIN_CTL_FCLK_125_C;
      DEBUG_PRINT("FCLK is at 125 MHz (%d)\n", *clkfreq);
      return CIN_OK;

    } else if((_hsd == 5) && (_n1 == 4)){

      *clkfreq = CIN_CTL_FCLK_250_C;
      DEBUG_PRINT("FCLK is at 150 MHz (%d)\n", *clkfreq);
      return CIN_OK;
        
    } else if((_hsd == 7) && (_n1 == 4)){

      *clkfreq = CIN_CTL_FCLK_200_C;
      DEBUG_PRINT("FCLK is at 200 MHz (%d)\n", *clkfreq);
      return CIN_OK;

    } else {

      ERROR_COMMENT("Not known clk. freq.\n");
      for(i=0;i<CIN_FCLK_READ_N;i++){
        ERROR_PRINT("FCLK REG 0x%04X = 0x%04X\n", CIN_FCLK_READ[i], _reg[i]);
      }
      return CIN_ERROR;
      
    }
  }

  if((_val & CMD_FCLK_125) == CMD_FCLK_125){
    DEBUG_COMMENT("FCLK = 125 MHz\n");
    *clkfreq = CIN_CTL_FCLK_125;
    return CIN_OK;
  } 

  if((_val & CMD_FCLK_200) == CMD_FCLK_200){
    DEBUG_COMMENT("FCLK = 200 MHz\n");
    *clkfreq = CIN_CTL_FCLK_200;
    return CIN_OK;		     
  }

  if((_val & CMD_FCLK_250) == CMD_FCLK_250){
    DEBUG_COMMENT("FCLK = 250 MHz\n");
    *clkfreq = CIN_CTL_FCLK_250;
    return CIN_OK;
  }

  ERROR_PRINT("Recieved unknown clk. freq. 0x%X\n", _val);
  return CIN_ERROR;
}  

int cin_ctl_get_cfg_fpga_status(cin_ctl_t *cin, uint16_t *_val){
      
  int _status = cin_ctl_read(cin,REG_FPGA_STATUS, _val, 0);
  DEBUG_PRINT("CFG FPGA Status  :  0x%04X\n", *_val);

  if(_status){
    ERROR_COMMENT("Unable to read FPGA status\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_get_id(cin_ctl_t *cin, cin_ctl_id_t *val){
  int _status = 0;

  _status  = cin_ctl_read(cin, REG_BOARD_ID, &val->base_board_id, 0);
  _status |= cin_ctl_read(cin,REG_HW_SERIAL_NUM, &val->base_serial_no, 0);
  _status |= cin_ctl_read(cin,REG_FPGA_VERSION, &val->base_fpga_ver, 0);
  _status |= cin_ctl_read(cin,REG_FRM_BOARD_ID, &val->fabric_board_id, 0);
  _status |= cin_ctl_read(cin,REG_FRM_HW_SERIAL_NUM, &val->fabric_serial_no, 0);
  _status |= cin_ctl_read(cin,REG_FRM_FPGA_VERSION, &val->fabric_fpga_ver, 0);
  DEBUG_PRINT("Base Board ID           :  0x%04X\n",val->base_board_id);
  DEBUG_PRINT("Base HW Serial Number   :  0x%04X\n",val->base_serial_no);
  DEBUG_PRINT("Base FPGA Version       :  0x%04X\n",val->base_fpga_ver);
  DEBUG_PRINT("Fabric Board ID         :  0x%04X\n",val->fabric_board_id);
  DEBUG_PRINT("Fabric HW Serial Number :  0x%04X\n",val->fabric_serial_no);
  DEBUG_PRINT("Fabric FPGA Version     :  0x%04X\n",val->fabric_fpga_ver);


  if(_status){
    ERROR_COMMENT("Unable to read CIN ID\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_get_dcm_status(cin_ctl_t *cin, uint16_t *_val){
  int _status;

  _status = cin_ctl_read(cin, REG_DCM_STATUS, _val, 0);
  if(_status){
    ERROR_COMMENT("Unable to read DCM status.\n");
    return CIN_ERROR;
  }

  DEBUG_PRINT("CFG DCM Status   :  0x%04X\n", *_val);
  return CIN_OK; 
}

double cin_ctl_current_calc(uint16_t val){
   
  double _current;
   
  if(val >= 0x8000){
    _current = 0.000000476 * (double)(0x10000 - val) / 0.003;
  } else { 
    _current = 0.000000476 * (double)val / 0.003;
  }
   
  return _current;
}

int cin_ctl_calc_vi_status(cin_ctl_t *cin, 
                           uint16_t vreg, uint16_t ireg, double vfact,
                           cin_ctl_pwr_val_t *vi){
  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cin, vreg, &_val, 0);
  if(_status){
    ERROR_COMMENT("Unable to read voltage.\n");
    return CIN_ERROR;
  }

  vi->v = vfact * _val;
  
  _status = cin_ctl_read(cin, ireg, &_val, 0);
  if(_status){
    ERROR_COMMENT("Unable to read current.\n");
    return CIN_ERROR;
  }

  vi->i = cin_ctl_current_calc(_val);

  return CIN_OK;
}

int cin_ctl_get_power_status(cin_ctl_t *cin, int full,
                             int *pwr, cin_ctl_pwr_mon_t *values){
    
  double _current, _voltage;
  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cin, REG_PS_ENABLE, &_val, 0);
  if(_status){
    ERROR_COMMENT("Unable to read power supply status\n");
    return CIN_ERROR;
  }

  if(!((_val & CIN_CTL_POWER_ENABLE) == CIN_CTL_POWER_ENABLE)) {
    // Power supply is off
    *pwr = 0;
    DEBUG_COMMENT("12V Power Supply is OFF\n");
    return CIN_OK;
  } 
  
  if(_val & CIN_CTL_FP_POWER_ENABLE){
    *pwr = 2;
    DEBUG_COMMENT("12V Power Supply + FP ON\n");
  } else {
    *pwr = 1;
    DEBUG_COMMENT("12V Power Supply ON\n");
  }

  /* ADC == LT4151 */
  _status  = cin_ctl_read(cin, REG_VMON_ADC1_CH1, &_val, 0);
  _voltage = 0.025 * _val;
  _status |= cin_ctl_read(cin, REG_IMON_ADC1_CH0, &_val, 0);
  _current = 0.00002 * _val / 0.003;
  if(_status){
    ERROR_COMMENT("Unable to read ADC1 values.\n");
    return CIN_ERROR;
  }
  values->bus_12v0.v = _voltage;
  values->bus_12v0.i = _current;

  if(full){
    _status  = cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH5, REG_IMON_ADC0_CH5,
                           0.00015258, &values->mgmt_3v3);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH7, REG_IMON_ADC0_CH7,
                           0.00015258, &values->mgmt_2v5);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH2, REG_IMON_ADC0_CH2,
                           0.00007629, &values->mgmt_1v2);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH3, REG_IMON_ADC0_CH3,
                           0.00007629, &values->enet_1v0);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH4, REG_IMON_ADC0_CH4,
                           0.00015258, &values->s3e_3v3);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH8, REG_IMON_ADC0_CH8,
                           0.00015258, &values->gen_3v3);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CH9, REG_IMON_ADC0_CH9,
                           0.00015258, &values->gen_2v5);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CHE, REG_IMON_ADC0_CHE,
                           0.00007629, &values->v6_0v9);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CHB, REG_IMON_ADC0_CHB,
                           0.00007629, &values->v6_1v0);
    _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CHD, REG_IMON_ADC0_CHD,
                           0.00015258, &values->v6_2v5);
  }

  _status |= cin_ctl_calc_vi_status(cin, REG_VMON_ADC0_CHF, REG_IMON_ADC0_CHF,
                         0.00030516, &values->fp);
  if(_status){
    ERROR_COMMENT("Unable to read power values\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

/******************* CIN Control *************************/

int cin_ctl_set_camera_pwr(cin_ctl_t *cin, int val){
  int _status = 0;
  _status |= cin_ctl_set_bias(cin, val);
  _status |= cin_ctl_set_clocks(cin, val);
  return _status;
}

int cin_ctl_get_camera_pwr(cin_ctl_t *cin, int *val){
  int _status = 0;
  int _val1 = 0, _val2 = 0;

  _status |= cin_ctl_get_bias(cin, &_val1);
  _status |= cin_ctl_get_clocks(cin, &_val2);

  if(!_status){
    *val = _val1 | _val2;
  } else {
    *val = -1;
  }

  return _status;
}

/******************* CIN Control *************************/

int cin_ctl_set_bias(cin_ctl_t *cin,int val){

  int _status;
   
  if(val){
    _status = cin_ctl_write_with_readback(cin,REG_BIASCONFIGREGISTER0_REG, 0x0001);
  } else if (val == 0){
    _status = cin_ctl_write_with_readback(cin,REG_BIASCONFIGREGISTER0_REG, 0x0000);
  } 

  if(_status){
    ERROR_COMMENT("Unable to set bias.\n");
    return _status;
  }

  DEBUG_PRINT("Bias state set to %d\n", val);
  return CIN_OK;
}

int cin_ctl_get_bias(cin_ctl_t *cin, int *val){

  int _status;
  uint16_t _val = -1;
  _status = cin_ctl_read(cin, REG_BIASCONFIGREGISTER0_REG, &_val, 0); 
   
  if(_status){
    ERROR_COMMENT("Unable to read bias status\n");
    return _status;
  }
  
  if((_val & 0x0001) == 0x0001){
    *val = 1;
  } else {
    *val = 0;
  }

  DEBUG_PRINT("Bias value is %d\n", *val);
  return CIN_OK;
}

int cin_ctl_set_clocks(cin_ctl_t *cin,int val){

  int _status;   
  uint16_t _val;

  _status = cin_ctl_read(cin, REG_CLOCKCONFIGREGISTER0_REG, &_val, 0); 
  if(_status){
    ERROR_COMMENT("Unable to read clock status\n");
    return _status;
  }
   
  if (val == 1){
    _status = cin_ctl_write_with_readback(cin,REG_CLOCKCONFIGREGISTER0_REG, _val | 0x0001);
  } else if (val == 0){
    _status = cin_ctl_write_with_readback(cin,REG_CLOCKCONFIGREGISTER0_REG, _val & ~0x0001);
  } else {
    ERROR_COMMENT("Illegal Clocks state: Only 0 or 1 allowed\n");
    return CIN_ERROR;
  }

  if(_status){
    ERROR_COMMENT("Unable to set clocks.");
    return _status;
  }

  DEBUG_PRINT("Clocks set to %d\n", val);
  return CIN_OK;
}

int cin_ctl_get_clocks(cin_ctl_t *cin, int *val){

  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cin, REG_CLOCKCONFIGREGISTER0_REG, &_val, 0); 
   
  if(_status){
    ERROR_COMMENT("Unable to read clock status\n");
    return _status;
  }

  if((_val & 0x0001) == 0x0001){
    *val = 1;
  } else {
    *val = 0;
  }

  DEBUG_PRINT("Clock value is %d\n", *val);
  return CIN_OK;
}

int cin_ctl_set_trigger(cin_ctl_t *cin,int val){

  int _status;
  if((val < 0) || (val > 3)){ 
    ERROR_COMMENT("Illegal Trigger state: Only values 0 to 3 allowed\n");
    return CIN_ERROR;
  }

  _status=cin_ctl_write_with_readback(cin,REG_TRIGGERMASK_REG, val);
  if(_status){
    ERROR_PRINT("Unable to set trigger to %d\n", val);
    return _status;
  }

  DEBUG_PRINT("Trigger set to %d\n", val);
  return CIN_OK;
}

int cin_ctl_get_trigger(cin_ctl_t *cin, int *val){

  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cin, REG_TRIGGERMASK_REG, &_val, 0); 
   
  if(_status){
    ERROR_COMMENT("Unable to read trigger status\n");
    return _status;
  }

  *val = (int)_val;
  DEBUG_PRINT("Trigger value is %d\n", *val);
  return CIN_OK;
}

int cin_ctl_get_focus(cin_ctl_t *cin, int *val){
  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cin, REG_CLOCKCONFIGREGISTER0_REG, &_val, 0);
  if(_status){
    ERROR_COMMENT("Unable to read focus status\n");
    return _status;
  }

  if((_val & CIN_CTL_FOCUS_BIT) == CIN_CTL_FOCUS_BIT){
    *val = 1;
  } else {
    *val = 0;
  }

  return CIN_OK;
}

int cin_ctl_set_focus(cin_ctl_t *cin, int val){

  uint16_t _val1;
  int _status;
   
  _status = cin_ctl_read(cin,REG_CLOCKCONFIGREGISTER0_REG, &_val1, 0);
  if(_status){
    ERROR_COMMENT("Unable to read focus bit\n");
    return _status;
  }

  if(val){
    _val1 |= CIN_CTL_FOCUS_BIT;
  } else {
    _val1 &= ~CIN_CTL_FOCUS_BIT;
  } 

  _status = cin_ctl_write_with_readback(cin,REG_CLOCKCONFIGREGISTER0_REG, _val1);

  if(_status){
    ERROR_COMMENT("Unable to write focus bit\n");
    return _status;
  }

  return CIN_OK;
}

int cin_ctl_int_trigger_start(cin_ctl_t *cin, int nimages){
  // Trigger the camera, setting the trigger mode

  int _status = CIN_OK;

  DEBUG_PRINT("Set n exposures to %d\n", nimages);

  pthread_mutex_lock(&cin->access);
  _status |= cin_ctl_write_with_readback(cin, REG_NUMBEROFEXPOSURE_REG, (uint16_t)nimages);
  _status |= cin_ctl_set_focus(cin, 1);
  _status |= cin_ctl_write(cin, REG_FRM_COMMAND, 0x0100, 0);
  pthread_mutex_unlock(&cin->access);

  if(_status != CIN_OK){
    ERROR_COMMENT("Unable to start triggers");
    return CIN_ERROR;
  }

  DEBUG_COMMENT("Trigger sent.\n");
  return CIN_OK;
}
      
int cin_ctl_int_trigger_stop(cin_ctl_t *cin){
  int _status;
  _status = cin_ctl_set_focus(cin, 0);

  if(_status){
    ERROR_COMMENT("Error stopping internal triggers\n");
    return _status;
  }
  DEBUG_COMMENT("Stopped internal triggers\n");
  return CIN_OK;
}

int cin_ctl_ext_trigger_start(cin_ctl_t *cin, int trigger_mode){
  // Trigger the camera, setting the trigger mode

  int _status;

  // First set the trigger mode (internal / external etc.)

  _status = cin_ctl_set_trigger(cin, trigger_mode);
  if(_status){
    ERROR_COMMENT("Unable to set trigger mode\n");
    goto error;
  }

  DEBUG_COMMENT("External triggers set.\n");
  return CIN_OK;

error:
  return _status;
}
      
int cin_ctl_ext_trigger_stop(cin_ctl_t *cin){
  int _status;
  _status = cin_ctl_set_trigger(cin, CIN_CTL_TRIG_INTERNAL);

  if(_status){
    ERROR_COMMENT("Error stopping external triggers\n");
    return _status;
  }
  DEBUG_COMMENT("Stopped external triggers\n");
  return CIN_OK;
}

int cin_ctl_get_triggering(cin_ctl_t *cin, int *trigger){
  // Return if we are triggering. 
  // Check the focus bit and the ext status

  int _status = 0;
  int trig, focus;

  _status = cin_ctl_get_trigger(cin, &trig);
  if(_status){
    return _status;
  } 

  if(trig != CIN_CTL_TRIG_INTERNAL){
    *trigger = 2;
    return CIN_OK;
  }

  _status = cin_ctl_get_focus(cin, &focus);
  if(_status){
    return _status;
  }

  if(focus){
    *trigger = 1;
    return CIN_OK;
  } else {
    *trigger = 0;
    return CIN_OK;
  }

  return CIN_ERROR;

}

int cin_ctl_set_exposure_time(cin_ctl_t *cin,float ftime)
{
  int _status = CIN_OK;
  uint32_t _time;
  uint16_t _msbval,_lsbval;

  ftime = ftime * cin->fclk_time_factor;
  _time = (uint32_t)(ftime * 50000); 

  if(_time == 0)
  {
    _time = 1;
  }
   
  _msbval = (uint32_t)(_time >> 16);
  _lsbval = (uint32_t)(_time & 0xFFFF);

  _status  = cin_ctl_write_with_readback(cin,REG_EXPOSURETIMEMSB_REG,_msbval);
  _status |= cin_ctl_write_with_readback(cin,REG_EXPOSURETIMELSB_REG,_lsbval);
  if(_status){
    ERROR_COMMENT("Unable to set exposure time\n");
    return _status;
  }

  DEBUG_PRINT("Exposure time set to %d (ticks)\n", _time);
  return CIN_OK;
}

int cin_ctl_set_trigger_delay(cin_ctl_t *cin,float ftime){  

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;

  _time=(uint32_t)(ftime * 1000);   //Extract integer from decimal

  _msbval=(uint16_t)(_time >> 16);
  _lsbval=(uint16_t)(_time & 0xFFFF);

  _status  = cin_ctl_write_with_readback(cin,REG_DELAYTOEXPOSUREMSB_REG,_msbval);
  _status |= cin_ctl_write_with_readback(cin,REG_DELAYTOEXPOSURELSB_REG,_lsbval);
  if(_status){
    ERROR_COMMENT("Unable to set trigger delay");
    return _status;
  }

  DEBUG_PRINT("Set trigger delay to %d\n", _time);
  return CIN_OK;
}

int cin_ctl_set_cycle_time(cin_ctl_t *cin,float ftime){

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;
                                       
  ftime = ftime*1000;         //Convert to ms
  _time = (uint32_t)ftime;    //Extract integer from decimal

  _msbval=(uint16_t)(_time >> 16);
  _lsbval=(uint16_t)(_time & 0xFFFF);

  _status  = cin_ctl_write_with_readback(cin,REG_TRIGGERREPETITIONTIMEMSB_REG,_msbval);
  _status |= cin_ctl_write_with_readback(cin,REG_TRIGGERREPETITIONTIMELSB_REG,_lsbval);

  if(_status != CIN_OK){
    ERROR_COMMENT("Unable to set cycle time");
    return CIN_ERROR;
  } 

  DEBUG_PRINT("Cycle time set to %d msec\n", _time);
  return CIN_OK;
}

/******************* Frame Acquisition *************************/
int cin_ctl_frame_count_reset(cin_ctl_t *cin){

  int _status;
  _status = cin_ctl_write(cin,REG_FRM_COMMAND, CMD_RESET_FRAME_COUNT, 0);
  if(_status){
    ERROR_COMMENT("Unable to reset frame counter\n");
    return _status;
  }

  DEBUG_COMMENT("Frame count reset to 0\n");
  return CIN_OK;
}

/* Setting of IP Addresses */

int cin_ctl_set_fabric_address(cin_ctl_t *cin, char *ip){
  DEBUG_PRINT("Setting fabric address to %s\n", ip);
  return cin_ctl_set_address(cin, ip, REG_IF_IP_FAB1B0, REG_IF_IP_FAB1B1);
}

int cin_ctl_set_address(cin_ctl_t *cin, char *ip, uint16_t reg0, uint16_t reg1){

  // First get the address

  struct in_addr addr;
  if(!inet_aton(ip, &addr)){
    ERROR_COMMENT("inet_aton() failed\n");
    return CIN_ERROR;
  }

  uint32_t addr_s = ntohl(addr.s_addr);

  DEBUG_PRINT("Setting IP to %08X\n", addr_s);

  int _status;
  _status  = cin_ctl_write_with_readback(cin, reg0, (uint16_t)addr_s);
  _status |= cin_ctl_write_with_readback(cin, reg1, (uint16_t)(addr_s >> 16));

  if(_status){
    ERROR_COMMENT("Could not set IP.\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

/*******************  MUX Settings for Output **********************/

int cin_ctl_set_mux(cin_ctl_t *cin, int setting){

  int _status = cin_ctl_write_with_readback(cin, REG_TRIGGERSELECT_REG, (uint16_t)setting);
  if(_status){
    ERROR_COMMENT("Failed to write MUX setting\n");
    return CIN_ERROR;
  } 

  DEBUG_PRINT("Set MUX to 0x%X\n", setting);

  return CIN_OK;
}

int cin_ctl_get_mux(cin_ctl_t *cin, int *setting){

  if(cin_ctl_read(cin, REG_TRIGGERSELECT_REG, (uint16_t*)setting, 0) != CIN_OK)
  {
    ERROR_COMMENT("Failed to read MUX setting\n");
    return CIN_ERROR;
  } 

  DEBUG_PRINT("Mux value is 0x%X\n", *setting);
  return CIN_OK;
}

/*******************  Upload values to fCRICs **********************/

int cin_ctl_set_fcric(cin_ctl_t *cin)
{
  // First get the fclk 
  int fclk;

  if(cin_ctl_get_fclk(cin, &fclk) != CIN_OK)
  {
    ERROR_COMMENT("Unable to get fclk status\n");
    return CIN_ERROR;
  }

  if(fclk == CIN_CTL_FCLK_200)
  {
    cin_ctl_message(cin, "Uploading fCRIC config for 200 MHz", CIN_CTL_MSG_OK);
    cin_ctl_set_fcric_regs(cin, cin_config_fcric_200, cin_config_fcric_200_len);
    cin_ctl_message(cin, "fCRICs configured for 200 MHz", CIN_CTL_MSG_OK);
  } else if(fclk == CIN_CTL_FCLK_125_C) { 
    cin_ctl_message(cin, "Uploading fCRIC config for 125 MHz", CIN_CTL_MSG_OK);
    cin_ctl_set_fcric_regs(cin, cin_config_fcric_125, cin_config_fcric_125_len);
    cin_ctl_message(cin, "fCRICs configured for 125 MHz", CIN_CTL_MSG_OK);
  } else {
    ERROR_PRINT("Could not set fCRIC, no data for fclk value %d\n", fclk);
    cin_ctl_message(cin, "Unable to configure fCRIC - Invalid fclk", CIN_CTL_MSG_MINOR);
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_set_fcric_regs(cin_ctl_t *cin, uint16_t *reg, int num_reg)
{
  int _status = CIN_OK;

  pthread_mutex_lock(&cin->access);

  _status |= cin_ctl_write_with_readback(cin, REG_DETECTOR_CONFIG_REG5, 0x0001);
  _status |= cin_ctl_write_with_readback(cin, REG_DETECTOR_CONFIG_REG5, 0x0000);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_MASK_REG1, 0xFFFF);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_MASK_REG2, 0xFFFF);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_MASK_REG3, 0xFFFF);

  int i;
  for(i=0;i<num_reg;i+=2)
  {
    _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE0_REG, 0xA000);
    _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE1_REG, reg[i]);
    _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE2_REG, reg[i+1]);
    _status |= cin_ctl_write(cin, REG_FRM_COMMAND, CMD_SEND_FCRIC_CONFIG, 0);
  }


  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_MASK_REG1, 0x0000);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_MASK_REG2, 0x0000);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_MASK_REG3, 0x0000);

  pthread_mutex_unlock(&cin->access);

  if(_status != CIN_OK)
  {
    ERROR_COMMENT("Failed to write fCRIC data to CIN\n");
    return CIN_ERROR;
  }

  ERROR_PRINT("Uploaded %d values to fCRIC regs\n", num_reg);
  return CIN_OK;
}

/*******************  Control Gain of fCRIC   **********************/

int cin_ctl_set_fcric_gain(cin_ctl_t *cin, int gain){
  uint16_t _gain;
  int _status = CIN_OK;

  if((gain == 0) || (gain == 2) || (gain == 3)){
    _gain = (uint16_t)gain;
  } else {
    ERROR_PRINT("Invalid gain setting %d\n", gain);
    return CIN_ERROR;
  }

  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE0_REG, 0xA000);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE1_REG, 0x0086);
  _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE2_REG, _gain);
  _status |= cin_ctl_write(cin, REG_FRM_COMMAND, CMD_SEND_FCRIC_CONFIG, 1);

  if(_status != CIN_OK){
    ERROR_COMMENT("Unable to set gain settings\n");
    return CIN_ERROR;
  }

  return CIN_OK;

}


/*******************  Set FCRIC to Clamp Mode **********************/


int cin_ctl_set_fcric_clamp(cin_ctl_t *cin, int clamp){
  uint16_t *_onoff;
  int _status = CIN_OK;

  if(clamp == 0){
    _onoff = cin_ctl_fcric_clamp_reg_off;
  } else if(clamp == 1){
    _onoff = cin_ctl_fcric_clamp_reg_on;
  } else {
    ERROR_PRINT("Invalid clamp setting %d\n", clamp);
    return CIN_ERROR;
  }

  int i;
  for(i=0;i<CIN_CTL_FCRIC_NUM_REG;i++){
    _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE0_REG, 0xA000);
    _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE1_REG, cin_ctl_fcric_clamp_reg[i]);
    _status |= cin_ctl_write_with_readback(cin, REG_FCRIC_WRITE2_REG, _onoff[i]);
    _status |= cin_ctl_write(cin, REG_FRM_COMMAND, CMD_SEND_FCRIC_CONFIG, 1);
  }

  if(_status != CIN_OK){
    ERROR_COMMENT("Unable to set clamp settings\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}


/*******************  BIAS Voltage Settings   **********************/

int cin_ctl_get_bias_voltages(cin_ctl_t *cin, float *voltage, uint16_t *regs)
{
  int n;
  uint16_t *_val;

  if(regs == NULL)
  {
    uint16_t __val[CIN_CTL_NUM_BIAS];
    _val = __val;
  } else {
    _val = regs;
  }

  if(cin_ctl_get_bias_regs(cin, _val) != CIN_OK)
  {
    ERROR_COMMENT("Unable to read bias registers\n");
    return CIN_ERROR;
  }

  for(n=0;n<CIN_CTL_NUM_BIAS;n++){
    voltage[n] = (float)(_val[n] & 0x0FFF) * bias_voltage_range[n] / 4096.0;
  } 

  return CIN_OK;
}

int cin_ctl_get_bias_regs(cin_ctl_t *cin, uint16_t *vals)
{
  int n;
  int _status = CIN_OK;

  for(n=0;n<CIN_CTL_NUM_BIAS;n++){
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERADDRESS, CIN_CTL_BIAS_OFFSET + (2 * n), 1);
    _status |= cin_ctl_read(cin, REG_BIASREGISTERDATAOUT, &vals[n], 1);
  }

  if(_status != CIN_OK)
  {
    ERROR_COMMENT("Unable to read bias voltages\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_set_bias_regs(cin_ctl_t * cin, uint16_t *vals, int verify)
{
  int n;
  int _status = 0;
  int _val;

  pthread_mutex_lock(&cin->access);

  if(cin_ctl_get_bias(cin, &_val) != CIN_OK)
  {
    ERROR_COMMENT("Unable to read bias status.\n");
    cin_ctl_message(cin, "Unable to set BIAS", CIN_CTL_MSG_MAJOR);
    goto error;
  }
  if(_val){
    ERROR_COMMENT("Cannot set bias values with BIAS on\n");
    cin_ctl_message(cin, "Unable to set BIAS with BIAS on", CIN_CTL_MSG_MINOR);
    goto error;
  }

  if(cin_ctl_get_clocks(cin, &_val) != CIN_OK)
  {
    ERROR_COMMENT("Unable to read clock status.\n"); 
    cin_ctl_message(cin, "Unable to set BIAS", CIN_CTL_MSG_MAJOR);
    goto error;
  }
  if(_val){
    ERROR_COMMENT("Cannot set bias voltages with CLOCKS on.\n");
    cin_ctl_message(cin, "Unable to set BIAS with CLOCKS on", CIN_CTL_MSG_MINOR);
    goto error;
  }
  
  if(cin_ctl_get_triggering(cin, &_val) != CIN_OK)
  {
    ERROR_COMMENT("Unable to read triggering status.\n"); 
    cin_ctl_message(cin, "Unable to set BIAS", CIN_CTL_MSG_MAJOR);
    goto error;
  }
  if(_val){
    ERROR_COMMENT("Cannot set bias voltages while camera is triggering.\n");
    cin_ctl_message(cin, "Unable to set BIAS while triggering", CIN_CTL_MSG_MINOR);
    goto error;
  }

  _status = CIN_OK;
  DEBUG_COMMENT("Uploading Bias Values\n");
  cin_ctl_message(cin, "Uploading BIAS values to CIN", CIN_CTL_MSG_OK);

  for(n=0;n<CIN_CTL_NUM_BIAS;n++){
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERADDRESS, (2 * n), 1);
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERDATA	, vals[n], 1);
    _status |= cin_ctl_write(cin, REG_FRM_COMMAND, 0x0102, 0);
    usleep(CIN_CTL_BIAS_SLEEP);
  } 

  if(_status != CIN_OK)
  {
    ERROR_COMMENT("Unable to set bias values\n");
    cin_ctl_message(cin, "Unable to set BIAS", CIN_CTL_MSG_MAJOR);
    goto error;
  }

  if(verify)
  {
    DEBUG_COMMENT("Verifying bias values\n");
    cin_ctl_message(cin, "Verifying BIAS values", CIN_CTL_MSG_OK);
    uint16_t _rregs[CIN_CTL_NUM_BIAS];
    if(cin_ctl_get_bias_regs(cin, _rregs) != CIN_OK)
    {
      ERROR_COMMENT("Unable to read bias values\n");
      goto error;
    }

    int i;
    int _err = 0;
    for(i=0;i<CIN_CTL_NUM_BIAS;i++)
    {
      if(_rregs[i] != vals[i])
      {
        DEBUG_PRINT("Validation ERROR at bias value %d (wrote %04x read %04x)\n",
            i, vals[i], _rregs[i]);
        _err++;
      } else {
        DEBUG_PRINT("Validation OK at bias value %d (wrote %04x read %04x)\n",
            i, vals[i], _rregs[i]);
      }
    }
    if(_err)
    {
      cin_ctl_message(cin, "BIAS values failed verification", CIN_CTL_MSG_MAJOR);
      goto error;
    }
    DEBUG_COMMENT("Bias Verified OK\n");
  }

  cin_ctl_message(cin, "BIAS values uploaded OK", CIN_CTL_MSG_OK);
  return CIN_OK;

error:

  pthread_mutex_unlock(&cin->access);
  return CIN_ERROR;
}

int cin_ctl_set_bias_voltages(cin_ctl_t *cin, float *voltage, int verify)
{
  uint16_t _val[CIN_CTL_NUM_BIAS];

  int n;
  for(n=0;n<CIN_CTL_NUM_BIAS;n++)
  {
    _val[n] =  (int)((voltage[n] / bias_voltage_range[n]) * 0x0FFF) & 0x0FFF;
    _val[n] |= ((n << 14) & 0xC000);
  }

  if(cin_ctl_set_bias_regs(cin, _val, verify) != CIN_OK)
  {
    ERROR_COMMENT("Unable to set bias regs\n");
    return CIN_ERROR;
  }

  return CIN_OK;
}

int cin_ctl_upload_bias(cin_ctl_t *cin)
{
  return cin_ctl_set_bias_regs(cin, cin_config_bias, 1);
}

int cin_ctl_set_timing_regs(cin_ctl_t *cin, uint16_t *vals, int vals_len)
{
  int i;

  pthread_mutex_lock(&cin->access);

  DEBUG_PRINT("Writing to timing registers (%d values)\n", vals_len);
  for(i=0;i<vals_len;i++)
  {
    int _status = CIN_OK;
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERADDRESS, i * 2, 1);
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERDATA, vals[i], 1);
    _status |= cin_ctl_write(cin, REG_FRM_COMMAND, CMD_WR_CCD_CLOCK_REG, 0);
    usleep(1000);
    if(_status != CIN_OK)
    {
      ERROR_PRINT("Unable to write %04X to cin (line %d)\n", vals[i], i);
      goto error;
    }
  }

  DEBUG_COMMENT("Done\n");

  if(cin_ctl_write_with_readback(cin,REG_DETECTOR_CONFIG_REG6, 0x0000) != CIN_OK)
  {
    goto error;
  }

  //uint16_t read_vals[600]; /// OUCH! Needs fixing
  //if(cin_ctl_get_timing_regs(cin, read_vals, vals_len) != CIN_OK)
  //{
  //  goto error;
  //}

  pthread_mutex_unlock(&cin->access);
  return CIN_OK;

error:
  pthread_mutex_unlock(&cin->access);
  return CIN_ERROR;
}

int cin_ctl_get_timing_regs(cin_ctl_t *cin, uint16_t *vals, int vals_len)
{
  int i;
  pthread_mutex_lock(&cin->access);

  for(i=0;i<vals_len;i++)
  {
    int _status = 0;
    _status |= cin_ctl_write_with_readback(cin, REG_BIASANDCLOCKREGISTERADDRESS, (2 * i));
    _status |= cin_ctl_read(cin, REG_CLOCKREGISTERDATAOUT, &vals[i], 0);
    DEBUG_PRINT("Reg 0x%04X = 0x%04X\n", 2 * i, vals[i]);
    if(_status)
    {
      ERROR_PRINT("Unable to write %04X to cin (line %d)\n", vals[i], i);
      goto error;
    }
  }

  DEBUG_COMMENT("Done\n");

  pthread_mutex_unlock(&cin->access);
  return CIN_OK;

error:
  pthread_mutex_unlock(&cin->access);
  return CIN_ERROR;
}

/*******************  Register Dump of CIN    **********************/

int cin_ctl_reg_dump(cin_ctl_t *cin, FILE *fp)
{
  fprintf(fp, "-------------------------------------------------------------\n");
  fprintf(fp, "Register Name                            : Register : Value \n");
  fprintf(fp, "-------------------------------------------------------------\n");

  cin_map_t *rmap = cin_reg_map;
  
  int status = CIN_OK;
  while(rmap->name != NULL){
    uint16_t reg = rmap->reg;
    uint16_t val;
    if(!(status |= cin_ctl_read(cin, reg, &val, 0)))
    {
      fprintf(fp, "%-40s :  0x%04X  :  0x%04X\n", rmap->name, reg, val);
    } else {
      fprintf(fp, "%-40s :  0x%04X  : ERROR\n", rmap->name, reg);
    }
    rmap++;
  } 

  fprintf(fp, "-------------------------------------------------------------\n");
  return status;
}

int cin_ctl_bias_dump(cin_ctl_t *cin, FILE *fp)
{
  fprintf(fp, "--------------------------------------------------------------------------------\n");
  fprintf(fp, "Bias Setting Name                        : Register : Value  : Voltage         :\n");
  fprintf(fp, "--------------------------------------------------------------------------------\n");

  int status;
  uint16_t reg[CIN_CTL_NUM_BIAS];
  float val[CIN_CTL_NUM_BIAS];
  status = cin_ctl_get_bias_voltages(cin, val, reg);
  if(status != CIN_OK)
  {
    ERROR_COMMENT("Unable to read bias voltages\n");
    return CIN_ERROR;
  }

  int i;
  for(i=0;i<CIN_CTL_NUM_BIAS;i++)
  {
    fprintf(fp, "%-40s :  0x%04X  :  0x%04X  : % 13.8f :\n", 
        cin_ctl_bias_name[i], 0x030 + (2 * i), reg[i], val[i]);
  } 

  fprintf(fp, "--------------------------------------------------------------------------------\n");

  return CIN_OK;
}
