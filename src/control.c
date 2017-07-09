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
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "cin.h"
#include "cin_register_map.h"
#include "fclk_program.h"
#include "cinregisters.h"
#include "control.h"
#include "config.h"
#include "common.h"
#include "fifo.h"

extern unsigned char firmware[];
extern unsigned firmware_len;
  
/**************************** INITIALIZATION **************************/

int cin_ctl_init(cin_ctl_t *cin, 
                 const char* ipaddr, const char *bind_addr,
                 uint16_t oport, uint16_t iport, 
                 uint16_t soport, uint16_t siport) {
  
  // Initialize the config

  cin_config_init(&(cin->config));


  if(ipaddr == NULL){ 
    cin->ctl_port.srvaddr = CIN_CTL_IP; 
    cin->stream_port.srvaddr = CIN_CTL_IP; 
  } else {
    cin->ctl_port.srvaddr = strdup(ipaddr); 
    cin->stream_port.srvaddr = strdup(ipaddr); 
  }

  if(bind_addr == NULL){ 
    cin->ctl_port.cliaddr = "0.0.0.0";
    cin->stream_port.cliaddr = "0.0.0.0";
  } else {
    cin->ctl_port.cliaddr = strdup(bind_addr); 
    cin->stream_port.cliaddr = strdup(bind_addr); 
  }
   
  if(oport == 0){ 
    cin->ctl_port.srvport = CIN_CTL_SVR_PORT; 
  } else {
    cin->ctl_port.srvport = oport; 
  }

  if(iport == 0){
    cin->ctl_port.cliport = CIN_CTL_CLI_PORT;
  } else {
    cin->ctl_port.cliport = iport;
  }

  if(siport == 0){
    cin->stream_port.srvport = CIN_CTL_SVR_FRMW_PORT;
  } else {
    cin->stream_port.srvport = iport;
  }

  if(soport == 0){
    cin->stream_port.cliport = CIN_CTL_CLI_FRMW_PORT;
  } else {
    cin->stream_port.cliport = iport;
  }
  
  cin->ctl_port.rcvbuf = CIN_CTL_RCVBUF;
  cin->stream_port.rcvbuf = CIN_CTL_RCVBUF;

  if(cin_init_port(&(cin->ctl_port)) || cin_init_port(&(cin->stream_port))){
     ERROR_COMMENT("Unable to open commuincations\n");
     return -1;
  }

  DEBUG_COMMENT("Opened ports\n");

  // Now initialize a listener thread for UDP communications

  cin_ctl_listener_t *listener = malloc(sizeof(cin_ctl_listener_t));
  cin->listener = listener;

  if(!listener){
    ERROR_COMMENT("Unable to create listener (MALLOC Failed)\n");
    return -1;
  }

  listener->cp = &(cin->ctl_port);
  if(fifo_init(&listener->ctl_fifo, sizeof(uint32_t), 100,1)){
    ERROR_COMMENT("Failed to initialize fifo.\n");
    return -1;
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

  // SBW : Do we need to wait for listener to start?
  return(0);
}

int cin_ctl_destroy(cin_ctl_t *cin){
  DEBUG_COMMENT("Closing comm ports\n");
  cin_ctl_close_ports(cin);
  DEBUG_COMMENT("Goodbye .....\n");
  return 0;
}

/**************************** UDP Socket ******************************/


uint32_t cin_ctl_get_packet(cin_ctl_t *cin, uint32_t *val){
  int i;
  long int r;
  
  for(i=0;i<1000;i++){
    r = fifo_used_bytes(&cin->listener->ctl_fifo);
    if(r){
      *val = *((uint32_t *)fifo_get_tail(&cin->listener->ctl_fifo, 0));
      fifo_advance_tail(&cin->listener->ctl_fifo, 0);
      return 0;
    }
    usleep(200);
  }

  return -1;
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
#ifdef __DEBUG_STREAM__
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

  return 0;
}

/*************************** CIN Read/Write ***************************/

int cin_ctl_write(cin_ctl_t *cin, uint16_t reg, uint16_t val, int wait){

  uint32_t _valwr;
  int rc;
  cin_port_t *cp = &(cin->ctl_port);

  if(cp == NULL){
     ERROR_COMMENT("Parameter cp is NULL!");
     goto error;
  }
  
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

  return 0;
   
error:  
  ERROR_COMMENT("Write error control port\n");
  return (-1);
}

int cin_ctl_write_with_readback(cin_ctl_t *cin, uint16_t reg, uint16_t val){
  
  int tries = CIN_CTL_MAX_WRITE_TRIES;

  pthread_mutex_lock(&cin->access);

  int _status;
  while(tries){
    DEBUG_PRINT("try = %d\n", tries);
    _status = cin_ctl_write(cin, reg, val, 1);
    if(_status){
      ERROR_PRINT("Error writing register %x\n", reg);
      goto error;
    }

    uint16_t _val;
    _status = cin_ctl_read(cin, reg, &_val);
    if(_status){
      ERROR_PRINT("Unable to read register %x\n", reg);
      goto error;
    }
    DEBUG_PRINT("sent = 0x%04X recv = 0x%04X\n", val, _val); 
    if(_val == val){
      // Value correct
      break;
    }
    tries--;
  }

  pthread_mutex_unlock(&cin->access);
  return 0;

error:
  pthread_mutex_unlock(&cin->access);
  return _status;
}

int cin_ctl_stream_write(cin_ctl_t *cin, unsigned char *val,int size) {
 
   int rc;
   cin_port_t *cp = &(cin->stream_port);
    
  pthread_mutex_lock(&cin->access);

   rc = sendto(cp->sockfd, val, size, 0,
               (struct sockaddr*)&cp->sin_srv,
               sizeof(cp->sin_srv));

  pthread_mutex_unlock(&cin->access);

   if (rc != size ) {
      ERROR_COMMENT("sendto() failure!!");
      goto error;
   }

   /*** TODO - implement write verification procedure ***/
   return 0;  
   
error:   
   ERROR_COMMENT("Write error");
   return -1;
}                                      

int cin_ctl_read(cin_ctl_t *cin, uint16_t reg, uint16_t *val) {
    
  int _status;
  uint32_t buf = 0;

  pthread_mutex_lock(&cin->access);

  int tries = CIN_CTL_MAX_READ_TRIES;
  while(tries){
    fifo_flush(&cin->listener->ctl_fifo);     

    _status = cin_ctl_write(cin, REG_READ_ADDRESS, reg, 1);
    if (_status){
      goto error;
    }

    _status = cin_ctl_write(cin, REG_COMMAND, CMD_READ_REG, 1);
    if (_status != 0){
      goto error;
    }

    if(!cin_ctl_get_packet(cin, &buf)){
      break;
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

  if((buf >> 16) != reg){
    ERROR_PRINT("Read value is register 0x%04X was expecting 0x%04X\n", 
                (buf >> 16), reg);
    goto error;
  }

  *val = (uint16_t)buf;
  pthread_mutex_unlock(&cin->access);
  return 0;
    
error:  
   ERROR_COMMENT("Read error.\n");
   pthread_mutex_unlock(&cin->access);
   return (-1);
}

 
/******************* CIN PowerUP/PowerDown *************************/
int cin_ctl_pwr(cin_ctl_t *cin, int pwr){

  int _status;
  uint16_t _val;

  if(pwr){
    DEBUG_COMMENT("Powering ON CIN Board\n");
    _val = CIN_CTL_POWER_ENABLE;
  } else {
    DEBUG_COMMENT("Powering OFF CIN Board\n");
    _val = CIN_CTL_POWER_DISABLE;
  }

  _status=cin_ctl_write_with_readback(cin,REG_PS_ENABLE, _val);
  if (_status != 0){
    goto error;
  }
  _status=cin_ctl_write(cin,REG_COMMAND, CMD_PS_ENABLE, 0);
  if (_status != 0){
    goto error;
  }
   
error:
   return _status;
}

int cin_ctl_fp_pwr(cin_ctl_t *cin, int pwr){ 

  int _status;
  uint16_t _val;

  _status = cin_ctl_read(cin, REG_PS_ENABLE, &_val);
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
   
error:
   return _status;
}

int cin_ctl_fo_test_pattern(cin_ctl_t *cin, int on_off){
  int _status;
  uint16_t _val1 = 0x0000;
  uint16_t _val2 = 0xFFFF;

  if(on_off){
    _val1 = 0x0001;
    DEBUG_COMMENT("Powering ON CIN Front Panel Boards\n");
    _val2 = 0x0000;
  } else {
    DEBUG_COMMENT("Powering ON CIN Front Panel Boards\n");
  }

  _status  = cin_ctl_write(cin,CIN_CTL_FO_REG1, 0x9E00, 0);
  _status |= cin_ctl_write(cin,CIN_CTL_FO_REG2, 0x0000, 0);
  _status |= cin_ctl_write(cin,CIN_CTL_FO_REG3, _val1,  0);
  _status |= cin_ctl_write(cin,CIN_CTL_FO_REG4, 0x0105, 0);

  usleep(20000);   /*for flow control*/ 

  _status |= cin_ctl_write(cin,CIN_CTL_FO_REG5, _val2, 0);
  _status |= cin_ctl_write(cin,CIN_CTL_FO_REG6, _val2, 0);
  _status |= cin_ctl_write(cin,CIN_CTL_FO_REG7, _val2, 0);

  return _status;
}

/******************* CIN Configuration/Status *************************/

int cin_ctl_load_config(cin_ctl_t *cin,char *filename){

  int _status;
  uint32_t _regul,_valul;
  char _regstr[12],_valstr[12],_line [1024];

  // We are going to check if bias and clocks are on. 
 
  int _val;
  _status = cin_ctl_get_bias(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read bias status. Refusing to upload config\n");
    return -1;
  }
  if(_val){
    ERROR_COMMENT("Cannot upload with bias on.\n");
    return -1;
  }

  _status = cin_ctl_get_clocks(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read clock status. Refusing to upload config\n");
    return -1;
  }
  if(_val){
    ERROR_COMMENT("Cannot upload with clocks on.\n");
    return -1;
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
      usleep(10000);   /*for flow control*/ 
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
  return 0;
  
error:
  pthread_mutex_unlock(&cin->access);
  return -1;
}

int cin_ctl_load_firmware_file(cin_ctl_t *cin, char *filename){
   
  uint32_t num_e;
  unsigned char buffer[128];
  int _status = -1; 

  // Lock the mutex for exclusive access to the CIN

  pthread_mutex_lock(&cin->access);

  FILE *file= fopen(filename, "rb");
  if(file == NULL){ 
    ERROR_COMMENT("Failed to open file.\n");
    goto error;
  }
            
  DEBUG_PRINT("Loading %s\n", filename);

  _status = cin_ctl_write(cin, REG_COMMAND, CMD_PROGRAM_FRAME, 0); 
  if (_status != 0){
    ERROR_COMMENT("Failed to program CIN\n");
    goto error;
  }   

  sleep(1);
  while ((num_e = fread(buffer,sizeof(char), sizeof(buffer), file)) != 0){    
    _status = cin_ctl_stream_write(cin, buffer, num_e);       
    if (_status != 0){
      ERROR_COMMENT("Error writing firmware to CIN\n");
      fclose(file);
      goto error;
    }
    usleep(200);   /*for UDP flow control*/ 
  }
  fclose(file);

  DEBUG_COMMENT("Done.\n");
  
  sleep(1);

  DEBUG_COMMENT("Resetting Frame FPGA\n");

  _status=cin_ctl_write(cin, REG_FRM_RESET, 0x0001, 0);
  if(_status != 0){
    goto error;
  } 

  sleep(1); 

  _status=cin_ctl_write(cin,REG_FRM_RESET,0x0000, 0);
  if(_status != 0){
    goto error;
  } 

  sleep(1);
  DEBUG_COMMENT("Done.\n");

  uint16_t _fpga_status;
  _status = cin_ctl_get_cfg_fpga_status(cin, &_fpga_status);
  _status |= !(_fpga_status & CIN_CTL_FPGA_STS_CFG);
  if(_status){
    DEBUG_COMMENT("FPGA Failed to configure.\n");
    goto error;
  }
  
  pthread_mutex_unlock(&cin->access);
  DEBUG_COMMENT("FPGA Configured OK\n");
  return 0;
   
error:
  pthread_mutex_unlock(&cin->access);
  return _status;
}

int cin_ctl_load_firmware(cin_ctl_t *cin){
   
  int _status = -1; 

  // Lock the mutex for exclusive access to the CIN

  pthread_mutex_lock(&cin->access);

  _status = cin_ctl_write(cin, REG_COMMAND, CMD_PROGRAM_FRAME, 0); 
  if (_status != 0){
    ERROR_COMMENT("Failed to program CIN\n");
    goto error;
  }   

  sleep(1);
  _status = cin_ctl_stream_write(cin, firmware, firmware_len);       
  if (_status != 0){
    ERROR_COMMENT("Error writing firmware to CIN\n");
    goto error;
  }

  DEBUG_COMMENT("Done.\n");
  
  sleep(1);

  DEBUG_COMMENT("Resetting Frame FPGA\n");

  _status=cin_ctl_write(cin, REG_FRM_RESET, 0x0001, 0);
  if(_status != 0){
    goto error;
  } 

  sleep(1); 

  _status=cin_ctl_write(cin,REG_FRM_RESET,0x0000, 0);
  if(_status != 0){
    goto error;
  } 

  sleep(1);
  DEBUG_COMMENT("Done.\n");

  uint16_t _fpga_status;
  _status = cin_ctl_get_cfg_fpga_status(cin, &_fpga_status);
  _status |= !(_fpga_status & CIN_CTL_FPGA_STS_CFG);
  if(_status){
    DEBUG_COMMENT("FPGA Failed to configure.\n");
    goto error;
  }
  
  pthread_mutex_unlock(&cin->access);
  DEBUG_COMMENT("FPGA Configured OK\n");
  return 0;
   
error:
  pthread_mutex_unlock(&cin->access);
  return _status;
}

int cin_ctl_freeze_dco(cin_ctl_t *cin, int freeze){
  int _status;

  _status = cin_ctl_write(cin,REG_FCLK_I2C_ADDRESS, 0xB089, 1);

  if(freeze){
    _status |= cin_ctl_write(cin,REG_FCLK_I2C_DATA_WR, 0xF010, 1);
  } else {
    _status |= cin_ctl_write(cin,REG_FCLK_I2C_DATA_WR, 0xF000, 1);
  }

  _status |= cin_ctl_write(cin,REG_FRM_COMMAND, CMD_FCLK_COMMIT, 1);

  if(!freeze){
    // Start the DCO up
    _status |= cin_ctl_write(cin,REG_FCLK_I2C_ADDRESS, 0xB087, 1);
    _status |= cin_ctl_write(cin,REG_FCLK_I2C_DATA_WR, 0xF040, 1);
    _status |= cin_ctl_write(cin,REG_FRM_COMMAND, CMD_FCLK_COMMIT, 1);
  }

  if(!_status){
    DEBUG_PRINT("Set FCLK Freeze/Unfreeze to %d\n", freeze);
  } else {
    ERROR_PRINT("Error setting FCLK Freeze/Unfreeze to %d\n", freeze);
  }
  return _status; 
}

int cin_ctl_set_fclk(cin_ctl_t *cin, int clkfreq){

  int _status = 0;

  switch(clkfreq){

    case CIN_CTL_FCLK_125:
      _status = cin_ctl_freeze_dco(cin, 1);
      if(!_status){
        _status  = cin_ctl_write(cin, REG_FCLK_SET0, 0xF002, 0);
        _status |= cin_ctl_write(cin, REG_FCLK_SET1, 0xF042, 0);
        _status |= cin_ctl_write(cin, REG_FCLK_SET2, 0xF0BC, 0);
        _status |= cin_ctl_write(cin, REG_FCLK_SET3, 0xF019, 0);
        _status |= cin_ctl_write(cin, REG_FCLK_SET4, 0xF06D, 0);
        _status |= cin_ctl_write(cin, REG_FCLK_SET5, 0xF08F, 0);
      }
      if(!_status){
        _status = cin_ctl_freeze_dco(cin, 0);
      }
      break;

    case CIN_CTL_FCLK_200:
      _status = cin_ctl_write(cin, REG_FCLK_I2C_DATA_WR, CMD_FCLK_200, 0);
      break;

    default:
      ERROR_PRINT("Invalid clock frequency %d\n", clkfreq);
      return -1;
      break;
  }

  if(_status){
    ERROR_COMMENT("Unable to set FCLK frequency.\n");
    return -1;
  }

  DEBUG_PRINT("Set FCLK to %d\n", clkfreq);
  return 0;
}

int cin_ctl_get_fclk(cin_ctl_t *cin, int *clkfreq){ 

  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cin, REG_FCLK_I2C_DATA_WR, &_val);
  if(_status){
    ERROR_COMMENT("Unable to get FCLK status.\n");
    return _status;
  }

  if((_val & 0xF000) == 0xF000){
    /* Not standard clock frequency */
    int i;
    _status = 0;
    uint16_t _reg[CIN_FCLK_READ_N];

    // Get CIN FCLK Registers

    for(i=0;i<CIN_FCLK_READ_N;i++){
      _status |= cin_ctl_write(cin, REG_FCLK_I2C_ADDRESS, CIN_FCLK_READ[i], 1);
      _status |= cin_ctl_write(cin, REG_FRM_COMMAND, CMD_FCLK_COMMIT, 1);
      _status |= cin_ctl_read(cin, REG_FCLK_I2C_DATA_RD, &_reg[i]);
    }

    // Print Reisters to debug stream

    for(i=0;i<CIN_FCLK_READ_N;i++){
      DEBUG_PRINT("FCLK REG 0x%04X = 0x%04X\n", CIN_FCLK_READ[i], _reg[i]);
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
      return 0;

    } else if((_hsd == 4) && (_n1 == 10)){

      *clkfreq = CIN_CTL_FCLK_125_C;
      return 0;

    } else if((_hsd == 5) && (_n1 == 4)){

      *clkfreq = CIN_CTL_FCLK_250_C;
      return 0;
        
    } else if((_hsd == 7) && (_n1 == 4)){

      *clkfreq = CIN_CTL_FCLK_200_C;
      return 0;

    } else {

      ERROR_COMMENT("Not known clk. freq.\n");
      for(i=0;i<CIN_FCLK_READ_N;i++){
        ERROR_PRINT("FCLK REG 0x%04X = 0x%04X\n", CIN_FCLK_READ[i], _reg[i]);
      }
      return -1;
      
    }
  }

  if((_val & CMD_FCLK_125) == CMD_FCLK_125){
    *clkfreq = CIN_CTL_FCLK_125;
    return 0;
  } 

  if((_val & CMD_FCLK_200) == CMD_FCLK_200){
    *clkfreq = CIN_CTL_FCLK_200;
    return 0;		     
  }

  if((_val & CMD_FCLK_250) == CMD_FCLK_250){
    *clkfreq = CIN_CTL_FCLK_250;
    return 0;
  }

  ERROR_PRINT("Recieved unknown clk. freq. 0x%X\n", _val);
  return -1;
}  

int cin_ctl_get_cfg_fpga_status(cin_ctl_t *cin, uint16_t *_val){
      
  int _status = cin_ctl_read(cin,REG_FPGA_STATUS, _val);
  DEBUG_PRINT("CFG FPGA Status  :  0x%04X\n", *_val);

  if(_status){
    ERROR_COMMENT("Unable to read FPGA status\n");
    return -1;
  }

  return 0;
}

int cin_ctl_get_id(cin_ctl_t *cin, cin_ctl_id_t *val){
  int _status;

  _status  = cin_ctl_read(cin, REG_BOARD_ID, &val->board_id);
  DEBUG_PRINT("CIN Board ID     :  0x%04X\n",val->board_id);

  _status |= cin_ctl_read(cin,REG_HW_SERIAL_NUM, &val->serial_no);
  DEBUG_PRINT("HW Serial Number :  0x%04X\n",val->serial_no);

  _status |= cin_ctl_read(cin,REG_FPGA_VERSION, &val->fpga_ver);
  DEBUG_PRINT("CFG FPGA Version :  0x%04X\n\n",val->fpga_ver);

  if(_status){
    ERROR_COMMENT("Unable to read CIN ID\n");
    return -1;
  }

  return 0;
}

int cin_ctl_get_dcm_status(cin_ctl_t *cin, uint16_t *_val){
  int _status;

  _status = cin_ctl_read(cin, REG_DCM_STATUS, _val);
  if(_status){
    ERROR_COMMENT("Unable to read DCM status.\n");
    return _status;
  }

  DEBUG_PRINT("CFG DCM Status   :  0x%04X\n", *_val);
  return 0; 
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

  _status = cin_ctl_read(cin, vreg, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read voltage.\n");
    return _status;
  }

  vi->v = vfact * _val;
  
  _status = cin_ctl_read(cin, ireg, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read current.\n");
    return _status;
  }

  vi->i = cin_ctl_current_calc(_val);

  return 0;
}

int cin_ctl_get_power_status(cin_ctl_t *cin, int full,
                             int *pwr, cin_ctl_pwr_mon_t *values){
    
  double _current, _voltage;
  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cin, REG_PS_ENABLE, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read power supply status\n");
    return _status;
  }

  if(!((_val & CIN_CTL_POWER_ENABLE) == CIN_CTL_POWER_ENABLE)) {
    // Power supply is off
    *pwr = 0;
    DEBUG_COMMENT("12V Power Supply is OFF\n");
    return 0;
  } 
  
  if(_val & CIN_CTL_FP_POWER_ENABLE){
    *pwr = 2;
    DEBUG_COMMENT("12V Power Supply + FP ON\n");
  } else {
    *pwr = 1;
    DEBUG_COMMENT("12V Power Supply ON\n");
  }

  /* ADC == LT4151 */
  _status  = cin_ctl_read(cin, REG_VMON_ADC1_CH1, &_val);
  _voltage = 0.025 * _val;
  _status |= cin_ctl_read(cin, REG_IMON_ADC1_CH0, &_val);
  _current = 0.00002 * _val / 0.003;
  if(_status){
    ERROR_COMMENT("Unable to read ADC1 values.\n");
    return _status;
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
    return _status;
  }

  return 0;
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
  return 0;
}

int cin_ctl_get_bias(cin_ctl_t *cin, int *val){

  int _status;
  uint16_t _val = -1;
  _status = cin_ctl_read(cin, REG_BIASCONFIGREGISTER0_REG, &_val); 
   
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
  return 0;
}

int cin_ctl_set_clocks(cin_ctl_t *cin,int val){

  int _status;   
  uint16_t _val;

  _status = cin_ctl_read(cin, REG_CLOCKCONFIGREGISTER0_REG, &_val); 
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
    return -1;
  }

  if(_status){
    ERROR_COMMENT("Unable to set clocks.");
    return _status;
  }

  DEBUG_PRINT("Clocks set to %d\n", val);
  return 0;
}

int cin_ctl_get_clocks(cin_ctl_t *cin, int *val){

  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cin, REG_CLOCKCONFIGREGISTER0_REG, &_val); 
   
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
  return 0;
}

int cin_ctl_set_trigger(cin_ctl_t *cin,int val){

  int _status;
  if((val < 0) || (val > 3)){ 
    ERROR_COMMENT("Illegal Trigger state: Only values 0 to 3 allowed\n");
    return -1;
  }

  _status=cin_ctl_write_with_readback(cin,REG_TRIGGERMASK_REG, val);
  if(_status){
    ERROR_PRINT("Unable to set trigger to %d\n", val);
    return _status;
  }

  DEBUG_PRINT("Trigger set to %d\n", val);
  return 0;
}

int cin_ctl_get_trigger(cin_ctl_t *cin, int *val){

  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cin, REG_TRIGGERMASK_REG, &_val); 
   
  if(_status){
    ERROR_COMMENT("Unable to read trigger status\n");
    return _status;
  }

  *val = (int)_val;
  DEBUG_PRINT("Trigger value is %d\n", *val);
  return 0;
}

int cin_ctl_get_focus(cin_ctl_t *cin, int *val){
  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cin, REG_CLOCKCONFIGREGISTER0_REG, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read focus status\n");
    return _status;
  }

  if((_val & CIN_CTL_FOCUS_BIT) == CIN_CTL_FOCUS_BIT){
    *val = 1;
  } else {
    *val = 0;
  }

  return 0;
}

int cin_ctl_set_focus(cin_ctl_t *cin, int val){

  uint16_t _val1;
  int _status;
   
  _status = cin_ctl_read(cin,REG_CLOCKCONFIGREGISTER0_REG, &_val1);
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

  return 0;
}

int cin_ctl_int_trigger_start(cin_ctl_t *cin, int nimages){
  // Trigger the camera, setting the trigger mode

  int _status = 0;

  DEBUG_PRINT("Set n exposures to %d\n", nimages);

  _status |= cin_ctl_write_with_readback(cin, REG_NUMBEROFEXPOSURE_REG, (uint16_t)nimages);
  _status |= cin_ctl_set_focus(cin, 1);
  _status |= cin_ctl_write(cin, REG_FRM_COMMAND, 0x0100, 0);

  if(_status){
    ERROR_COMMENT("Unable to start triggers");
  }

  DEBUG_COMMENT("Trigger sent.\n");
  return _status;
}
      
int cin_ctl_int_trigger_stop(cin_ctl_t *cin){
  int _status;
  _status = cin_ctl_set_focus(cin, 0);

  if(_status){
    ERROR_COMMENT("Error stopping internal triggers\n");
    return _status;
  }
  DEBUG_COMMENT("Stopped internal triggers\n");
  return 0;
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
  return 0;

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
  return 0;
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
    return 0;
  }

  _status = cin_ctl_get_focus(cin, &focus);
  if(_status){
    return _status;
  }

  if(focus){
    *trigger = 1;
    return 0;
  } else {
    *trigger = 0;
    return 0;
  }

  return -1;

}

int cin_ctl_set_exposure_time(cin_ctl_t *cin,float ftime){

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;

  ftime = ftime * 1e5;
  _time = (uint32_t)ftime;  //Extract integer from decimal
   
  _msbval = (uint32_t)(_time >> 16);
  _lsbval = (uint32_t)(_time & 0xFFFF);

  _status  = cin_ctl_write_with_readback(cin,REG_EXPOSURETIMEMSB_REG,_msbval);
  _status |= cin_ctl_write_with_readback(cin,REG_EXPOSURETIMELSB_REG,_lsbval);
  if(_status){
    ERROR_COMMENT("Unable to set exposure time\n");
    return _status;
  }

  DEBUG_PRINT("Exposure time set to %d (10this us)\n", _time);
  return 0;
}

int cin_ctl_set_trigger_delay(cin_ctl_t *cin,float ftime){  

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;

  _time=(uint32_t)ftime;   //Extract integer from decimal

  _msbval=(uint16_t)(_time >> 16);
  _lsbval=(uint16_t)(_time & 0xFFFF);

  _status  = cin_ctl_write_with_readback(cin,REG_DELAYTOEXPOSUREMSB_REG,_msbval);
  _status |= cin_ctl_write_with_readback(cin,REG_DELAYTOEXPOSURELSB_REG,_lsbval);
  if(_status){
    ERROR_COMMENT("Unable to set trigger delay");
    return _status;
  }

  DEBUG_PRINT("Set trigger delay to %d\n", _time);
  return 0;
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

  if(_status){
    ERROR_COMMENT("Unable to set cycle time");
    return _status;
  } 

  DEBUG_PRINT("Cycle time set to %d msec\n", _time);
  return 0;
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
  return 0;
}

/* Setting of IP Addresses */

int cin_ctl_set_fabric_address(cin_ctl_t *cin, char *ip){
  return cin_ctl_set_address(cin, ip, REG_IF_IP_FAB1B0, REG_IF_IP_FAB1B1);
}

int cin_ctl_set_address(cin_ctl_t *cin, char *ip, uint16_t reg0, uint16_t reg1){

  // First get the address

  struct in_addr addr;
  if(!inet_aton(ip, &addr)){
    ERROR_COMMENT("inet_aton() failed\n");
    return -1;
  }

  uint32_t addr_s = ntohl(addr.s_addr);

  DEBUG_PRINT("Setting IP to %08X\n", addr_s);

  int _status;
  _status  = cin_ctl_write_with_readback(cin, reg0, (uint16_t)addr_s);
  _status |= cin_ctl_write_with_readback(cin, reg1, (uint16_t)(addr_s >> 16));

  if(_status){
    ERROR_COMMENT("Could not set IP.\n");
    return -1;
  }

  return 0;
}

/*******************  MUX Settings for Output **********************/

int cin_ctl_set_mux(cin_ctl_t *cin, int setting){

  int _status = cin_ctl_write_with_readback(cin, REG_TRIGGERSELECT_REG, (uint16_t)setting);
  if(_status){
    ERROR_COMMENT("Failed to write MUX setting\n");
    return -1;
  } 

  DEBUG_PRINT("Set MUX to 0x%X\n", setting);

  return 0;
}

int cin_ctl_get_mux(cin_ctl_t *cin, int *setting){

  int _status = cin_ctl_read(cin, REG_TRIGGERSELECT_REG, (uint16_t*)setting);
  if(_status){
    ERROR_COMMENT("Failed to read MUX setting\n");
    return -1;
  } 

  DEBUG_PRINT("Mux value is 0x%X\n", *setting);

  return 0;
}

/*******************  Control Gain of fCRIC   **********************/

int cin_ctl_set_fcric_gain(cin_ctl_t *cin, int gain){
  uint16_t _gain;
  int _status = 0;

  if((gain == 0) || (gain == 2) || (gain == 3)){
    _gain = (uint16_t)gain;
  } else {
    ERROR_PRINT("Invalid gain setting %d\n", gain);
    return -1;
  }

  _status |= cin_ctl_write(cin, REG_FCRIC_WRITE0_REG, 0xA000, 1);
  _status |= cin_ctl_write(cin, REG_FCRIC_WRITE1_REG, 0x0086, 1);
  _status |= cin_ctl_write(cin, REG_FCRIC_WRITE2_REG, _gain, 1);
  _status |= cin_ctl_write(cin, REG_FRM_COMMAND, 0x0105, 1);

  if(_status){
    ERROR_COMMENT("Unable to set gain settings\n");
    return _status;
  }

  return 0;

}


/*******************  Set FCRIC to Clamp Mode **********************/


int cin_ctl_set_fcric_clamp(cin_ctl_t *cin, int clamp){
  uint16_t *_onoff;
  int _status = 0;

  if(clamp == 0){
    _onoff = fcric_clamp_reg_off;
  } else if(clamp == 1){
    _onoff = fcric_clamp_reg_on;
  } else {
    ERROR_PRINT("Invalid clamp setting %d\n", clamp);
    return -1;
  }

  int i;
  for(i=0;i<NUM_CLAMP_REG;i++){
    _status |= cin_ctl_write(cin, REG_FCRIC_WRITE0_REG, 0xA000, 1);
    _status |= cin_ctl_write(cin, REG_FCRIC_WRITE1_REG, fcric_clamp_reg[i], 1);
    _status |= cin_ctl_write(cin, REG_FCRIC_WRITE2_REG, _onoff[i], 1);
    _status |= cin_ctl_write(cin, REG_FRM_COMMAND, 0x0105, 1);
  }

  if(_status){
    ERROR_COMMENT("Unable to set clamp settings\n");
    return _status;
  }

  return 0;

}


/*******************  BIAS Voltage Settings   **********************/

int cin_ctl_get_bias_voltages(cin_ctl_t *cin, float *voltage){

  int n;
  int _status = 0;
  uint16_t _val;

  for(n=0;n<NUM_BIAS_VOLTAGE;n++){
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERADDRESS, 0x0030 + (2 * n), 1);
    _status |= cin_ctl_read(cin, REG_BIASREGISTERDATAOUT, &_val);
    voltage[n] = (float)(_val & 0x0FFF) * bias_voltage_range[n] / 4096.0;
  } 

  return _status;
}

int cin_ctl_set_bias_voltages(cin_ctl_t *cin, float *voltage){

  int n;
  int _status = 0;
  int _val;

  _status = cin_ctl_get_bias(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read bias status.\n");
    return -1;
  }
  if(_val){
    ERROR_COMMENT("Cannot set bias values with BIAS on\n");
    return -1;
  }

  _status = cin_ctl_get_clocks(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read clock status.\n"); 
    return -1;
  }
  if(_val){
    ERROR_COMMENT("Cannot set bias voltages with CLOCKS on.\n");
    return -1;
  }
  
  _status = cin_ctl_get_triggering(cin, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read triggering status.\n"); 
    return -1;
  }
  if(_val){
    ERROR_COMMENT("Cannot set bias voltages while camera is triggering.\n");
    return -1;
  }

  _status = 0;
  for(n=0;n<NUM_BIAS_VOLTAGE;n++){
    _val =  (int)((voltage[n] / bias_voltage_range[n]) * 0x0FFF) & 0x0FFF;
    _val |= ((n << 14) & 0xC000);
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERADDRESS, (2 * n), 1);
    _status |= cin_ctl_write(cin, REG_BIASANDCLOCKREGISTERDATA	, _val, 1);
    _status |= cin_ctl_write(cin, REG_FRM_COMMAND, 0x0102, 1);
  } 

  return _status;
}

/*******************  Register Dump of CIN    **********************/

int cin_ctl_reg_dump(cin_ctl_t *cin, FILE *fp)
{
  fprintf(fp, "-------------------------------------------------------------\n");
  fprintf(fp, "Register Name                            : Register : Value \n");
  fprintf(fp, "-------------------------------------------------------------\n");

  cin_map_t *rmap = cin_reg_map;
  
  int status = 0;
  while(rmap->name != NULL){
    uint16_t reg = rmap->reg;
    uint16_t val;
    if(!(status |= cin_ctl_read(cin, reg, &val)))
    {
      fprintf(fp, "%-40s :  0x%04X  :  0x%04X\n", rmap->name, reg, val);
    } else {
      fprintf(fp, "%-40s :  0x%04X  : ERROR\n", rmap->name, reg);
    }
    rmap++;
  } 

  return status;
}
