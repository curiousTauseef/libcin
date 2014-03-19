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
#include "cin_api.h"

/**************************** UDP Socket ******************************/

int cin_ctl_init_port(struct cin_port* cp, char* ipaddr, 
                      uint16_t port) {
  if(ipaddr == 0){ 
    cp->srvaddr = CIN_CTL_IP; 
  } else {
    cp->srvaddr = strndup(ipaddr, strlen(ipaddr)); 
  }
   
  if(port == 0){ 
      cp->srvport = CIN_CTL_PORT; 
  } else {
     cp->srvport = port; 
  }

  cp->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (cp->sockfd < 0) {
    ERROR_COMMENT("CIN control port - socket() failed !!!\n");
      // Should this return (-1) ?
  }

  int i = 1;
  if(setsockopt(cp->sockfd, SOL_SOCKET, SO_REUSEADDR, 
                (void *)&i, sizeof i) < 0) {
    ERROR_COMMENT("CIN control port - setsockopt() failed !!!\n");
    return -1;
  }
    
  // default timeout of 1s
  cp->tv.tv_sec  = 1;
  cp->tv.tv_usec = 0;
  if(setsockopt(cp->sockfd, SOL_SOCKET, SO_RCVTIMEO,
     (void*)&cp->tv, sizeof(struct timeval)) < 0){
     ERROR_COMMENT("setsockopt(timeout)\n");
     return -1;
  }

   // initialize CIN (server) and client (us!) sockaddr structs
   memset(&cp->sin_srv, 0, sizeof(struct sockaddr_in));
   memset(&cp->sin_cli, 0, sizeof(struct sockaddr_in));
   cp->sin_srv.sin_family = AF_INET;
   cp->sin_srv.sin_port = htons(cp->srvport);
   cp->slen = sizeof(struct sockaddr_in);
   if(inet_aton(cp->srvaddr, &cp->sin_srv.sin_addr) == 0) {
      ERROR_COMMENT("CIN control port - inet_aton() failed!!");
      return -1;
   }

   return 0;
}


int cin_ctl_close_port(struct cin_port* cp) {
  if(cp->sockfd){
    close(cp->sockfd); 
  }
  return 0;
}

/*************************** CIN Read/Write ***************************/

int cin_ctl_write(struct cin_port* cp, uint16_t reg, uint16_t val){

   uint32_t _valwr;
   int rc;

   if(cp == NULL){
      ERROR_COMMENT("Parameter cp is NULL!");
      goto error;
   }
   
   _valwr = ntohl((uint32_t)(reg << 16 | val));
   rc = sendto(cp->sockfd, &_valwr, sizeof(_valwr), 0,
         (struct sockaddr*)&cp->sin_srv,
         sizeof(cp->sin_srv));
   if (rc != sizeof(_valwr) ) {
      ERROR_COMMENT("CIN control port - sendto( ) failure!!");
      goto error;
   }


   return 0;
   
error:  
   ERROR_COMMENT("Write error control port");
   return (-1);
}

int cin_ctl_stream_write(struct cin_port* cp, char *val,int size) {
 
   int rc;
    
   if (cp == NULL){
      ERROR_COMMENT("Parameter cp is NULL!");
      goto error;
   }
   
   rc = sendto(cp->sockfd, val, size, 0,
               (struct sockaddr*)&cp->sin_srv,
               sizeof(cp->sin_srv));
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

int cin_ctl_read(struct cin_port* cp, uint16_t reg, uint16_t *val) {
    
  int _status;
  uint32_t buf = 0;
  ssize_t n;

  _status=cin_ctl_write(cp, REG_READ_ADDRESS, reg);
  if (_status){
    goto error;
  }

  usleep(100000); // YF Hard coded sleep - how to best handle this?

  _status=cin_ctl_write(cp, REG_COMMAND, CMD_READ_REG);
  if (_status != 0){
    goto error;
  }

  n = recvfrom(cp->sockfd, (void*)&buf, sizeof(buf), 0,
               (struct sockaddr*)&cp->sin_cli, 
               (socklen_t*)&cp->slen);

  if (n != sizeof(buf)) {
    goto error;
  }

  *val = (uint16_t)ntohl(buf);
  return 0;
    
error:  
   ERROR_COMMENT("Read error.");
   return (-1);
}

 
/******************* CIN PowerUP/PowerDown *************************/
int cin_pwr(struct cin_port* cp, int pwr){

  int _status;
  uint16_t _val;

  if(pwr){
    DEBUG_COMMENT("Powering ON CIN Board");
    _val = 0x000F;
  } else {
    DEBUG_COMMENT("Powering OFF CIN Board");
    _val = 0x0000;
  }

  _status=cin_ctl_write(cp,REG_PS_ENABLE, _val);
  if (_status != 0){
    goto error;
  }
  _status=cin_ctl_write(cp,REG_COMMAND, CMD_PS_ENABLE);
  if (_status != 0){
    goto error;
  }
   
error:
   return _status;
}

int cin_on(struct cin_port* cp) {
  return cin_pwr(cp, 1);
}

int cin_off(struct cin_port* cp) {
  return cin_pwr(cp, 0);
}

int cin_fp_pwr(struct cin_port* cp, int pwr){ 

  int _status;
  uint16_t _val;

  if(pwr){
    DEBUG_COMMENT("Powering ON CIN Front Panel Boards");
    _val = 0x003F;
  } else {
    DEBUG_COMMENT("Powering OFF CIN Front Panel Boards");
    _val = 0x0000;
  }

  _status=cin_ctl_write(cp,REG_PS_ENABLE, _val);
  if(_status != 0){
    goto error;
  }
  _status=cin_ctl_write(cp,REG_COMMAND, CMD_PS_ENABLE);
  if(_status != 0){
    goto error;
  }
   
error:
   return _status;
}

int cin_fp_off(struct cin_port* cp){
  return cin_fp_pwr(cp, 1);
}
int cin_fp_on(struct cin_port* cp){
  return cin_fp_pwr(cp, 0);
}

/******************* CIN Configuration/Status *************************/

int cin_ctl_load_config(struct cin_port* cp,char *filename){

  int _status;
  uint32_t _regul,_valul;
  char _regstr[12],_valstr[12],_line [1024];

  FILE *file = fopen(filename, "r");
  if(file == NULL){
    ERROR_PRINT("Unable to open file %s\n", filename);
    goto error;
  }

  DEBUG_PRINT("Loading CIN configuration from %s\n",filename);
   
  /* Read a line an filter out comments */     

  while(fgets(_line,sizeof _line,file)!= NULL){ 
    _line[strlen(_line)-1]='\0';   

    if ('#' == _line[0] || '\0' == _line[0]){
      //fprintf(stdout," Ignore line\n"); //DEBUG 
    } else { 
      sscanf (_line,"%s %s",_regstr,_valstr);
      _regul=strtoul(_regstr,NULL,16);
      _valul=strtoul(_valstr,NULL,16);          
      usleep(10000);   /*for flow control*/ 
      _status=cin_ctl_write(cp,_regul,_valul);
      if (_status != 0){
        ERROR_COMMENT("Error writing to CIN\n");
        goto error;
      }
   }
  }
  
  DEBUG_COMMENT("CIN configuration loaded!\n");
  fclose(file);
  return 0;
  
error:
  return -1;
}

int cin_ctl_load_firmware(struct cin_port* cp,struct cin_port* dcp,char *filename){
   
  uint32_t num_e;
  int _status; 
  char buffer[128];     
   
  FILE *file= fopen(filename, "rb");

  if(file == NULL){ 
    return -1;
  }
               
  DEBUG_PRINT("Loading CIN FPGA firmware %s\n",filename);

  _status = cin_ctl_write(cp,REG_COMMAND,CMD_PROGRAM_FRAME); 
  if (_status != 0){
    ERROR_COMMENT("Failed to program CIN\n");
    goto error;
  }   
  
  sleep(1);
      
  while ((num_e = fread(buffer,sizeof(char), sizeof(buffer), file)) != 0){    
    _status = cin_ctl_stream_write(dcp, buffer, num_e);       
    if (_status != 0){
      ERROR_COMMENT("Error writing firmware to CIN\n");
      goto error;
    }
    usleep(500);   /*for UDP flow control*/ 
  }

  sleep(1);
  
  _status=cin_ctl_write(cp,REG_FRM_RESET,0x0001);
  if(_status != 0){
    goto error;
  } 
 
  _status=cin_ctl_write(cp,REG_FRM_RESET,0x0000);
  if(_status != 0){
    goto error;
  } 
      
  DEBUG_COMMENT("CIN FPGA configuration loaded!!\n"); 
  fclose(file);
  return 0;
   
error:
   return _status;
}

int cin_ctl_set_fclk(struct cin_port* cp, int clkfreq){

  int _status;
   
  if (clkfreq == CIN_CTL_FCLK_125){
    _status = cin_ctl_write(cp,REG_FCLK_I2C_DATA_WR, CMD_FCLK_125);
  } else if (clkfreq == CIN_CTL_FCLK_200){
    _status = cin_ctl_write(cp,REG_FCLK_I2C_DATA_WR, CMD_FCLK_200);
  } else if (clkfreq == CIN_CTL_FCLK_250){
    _status = cin_ctl_write(cp,REG_FCLK_I2C_DATA_WR, CMD_FCLK_250);
  } else {
    ERROR_COMMENT("Invalid clkfreq.\n");
    return -1;
  }

  if(_status){
    ERROR_COMMENT("Unable to set FCLK frequency.\n");
    return _status;
  }

  DEBUG_PRINT("Set FCLK to %d\n", clkfreq);
  return 0;
}

int cin_ctl_get_fclk(struct cin_port* cp, int *clkfreq){ 

  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cp, REG_FCLK_I2C_DATA_WR, &_val);
  if(_status){
    ERROR_COMMENT("Unable to get FCLK status.\n");
    return _status;
  }

  if(_val & CIN_CTL_FCLK_125){
    DEBUG_COMMENT("FCLK Frequency = 125 MHz\n");
    *clkfreq = CIN_CTL_FCLK_125;
    return 0;
  } 

  if(_val & CIN_CTL_FCLK_200){
    DEBUG_COMMENT("FCLK Frequency = 200 MHz\n");
    *clkfreq = CIN_CTL_FCLK_200;
    return 0;		     
  }

  if(_val & CIN_CTL_FCLK_250){
    DEBUG_COMMENT("FCLK Frequency = 250 MHz\n");
    *clkfreq = CIN_CTL_FCLK_250;
    return 0;
  }

  ERROR_COMMENT("Recieved unknown clk. freq.\n");
  return -1;
}  

int cin_ctl_get_cfg_fpga_status(struct cin_port* cp, 
                                cin_ctl_fpga_status_t *_val){
      
  int _status; 

  //# get Status Registers
  _status  = cin_ctl_read(cp, REG_BOARD_ID, &_val->board_id);
  DEBUG_PRINT("CIN Board ID     :  %04X\n",_val->board_id);

  _status |= cin_ctl_read(cp,REG_HW_SERIAL_NUM, &_val->serial_no);
  DEBUG_PRINT("HW Serial Number :  %04X\n",_val->serial_no);

  _status |= cin_ctl_read(cp,REG_FPGA_VERSION, &_val->fpga_ver);
  DEBUG_PRINT("CFG FPGA Version :  %04X\n\n",_val->fpga_ver);

  _status |= cin_ctl_read(cp,REG_FPGA_STATUS, &_val->fpga_status);
  DEBUG_PRINT("CFG FPGA Status  :  %04X\n",_val->fpga_status);

  if(_status){
    ERROR_COMMENT("Unable to read FPGA status\n");
    return -1;
  }

  return 0;
}
/*
      # FPGA Status
      # 15 == FRM DONE
      # 14 == NOT FRM BUSY
      # 13 == NOT FRM INIT B
      # 12 >> 4 == 0
      # 3 >>0 == FP Config Control 3 == PS Interlock
  */
  // _val= cin_ctl_read(cp,REG_FPGA_STATUS);   
  /*
   if(_val == 0xFFFF){
      ERROR_COMMENT("\n  ERROR:No Input\n\n");
      return -1;   
   }
   else if((_val & 0x8000)==0x8000){
      DEBUG_COMMENT("Frame FPGA Configuration Done\n"); 
      return 0;
   }
   else{
      ERROR_COMMENT("Frame FPGA NOT Configured\n");
      return -1;
   }
   if((_val & 0x0008)==0x0008){
      DEBUG_COMMENT("FP Power Supply Unlocked\n"); 
   }
   else{
      DEBUG_COMMENT("FP Power Supply Locked\n");
   }
  */

int cin_ctl_get_dcm_status(struct cin_port* cp, uint16_t *_val){
  int _status;

  _status = cin_ctl_read(cp, REG_DCM_STATUS, _val);
  if(_status){
    ERROR_COMMENT("Unable to read DCM status.\n");
    return _status;
  }

  DEBUG_PRINT("CFG DCM Status   :  %04X\n", *_val);
  return 0; 
/*
   # DCM Status
   # 15 == 0
   # 14 >> 8 == CONF SW
   # 7 == ATCA 48V Alarm
   # 6 == tx2 src ready
   # 5 == tx1 src ready
   # 4 == DCM STATUS2
   # 3 == DCM STATUS1
   # 2 == DCM STATUS0
   # 1 == DCM PSDONE
   # 0 == DCM LOCKED
   */

  // SBW : THIS MUST BE AN ERROR
  // if((_val != 0x0800)==0x0000){
  //    DEBUG_COMMENT("  ** FP Power Supply Interlock Overide Enabled\n");  
  // }
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

int cin_ctl_calc_vi_status(struct cin_port* cp, 
                           uint16_t vreg, uint16_t ireg, double vfact,
                           cin_ctl_pwr_val_t *vi){
  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cp, vreg, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read voltage.\n");
    return _status;
  }

  vi->v = vfact * _val;
  
  _status = cin_ctl_read(cp, ireg, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read current.\n");
    return _status;
  }

  vi->i = cin_ctl_current_calc(_val);

  return 0;
}

int cin_ctl_get_power_status(struct cin_port* cp, 
                             int *pwr, cin_ctl_pwr_mon_t *values){
    
  double _current, _voltage;
  uint16_t _val;
  int _status;

  _status = cin_ctl_read(cp, REG_PS_ENABLE, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read power supply status\n");
    return _status;
  }
      
  if(!(_val & 0x0001)) {
    // Power supply is off
    *pwr = 0;
    DEBUG_COMMENT("12V Power Supply is OFF\n");
    return 0;
  } else {
    DEBUG_COMMENT("12V Power Supply in ON\n");
  }

  /* ADC == LT4151 */
  _status  = cin_ctl_read(cp, REG_VMON_ADC1_CH1, &_val);
  _voltage = 0.025 * _val;
  _status |= cin_ctl_read(cp, REG_IMON_ADC1_CH0, &_val);
  _current = 0.00002 * _val / 0.003;
  if(_status){
    ERROR_COMMENT("Unable to read ADC1 values.\n");
    return _status;
  }
  values->bus_12v0.v = _voltage;
  values->bus_12v0.i = _current;

  _status  = cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH5, REG_IMON_ADC0_CH5,
                         0.00015258, &values->mgmt_3v3);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH7, REG_IMON_ADC0_CH7,
                         0.00015258, &values->mgmt_2v5);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH2, REG_IMON_ADC0_CH2,
                         0.00007629, &values->mgmt_1v2);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH3, REG_IMON_ADC0_CH3,
                         0.00007629, &values->enet_1v0);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH4, REG_IMON_ADC0_CH4,
                         0.00015258, &values->s3e_3v3);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH8, REG_IMON_ADC0_CH8,
                         0.00015258, &values->gen_3v3);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CH9, REG_IMON_ADC0_CH9,
                         0.00015258, &values->gen_2v5);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CHE, REG_IMON_ADC0_CHE,
                         0.00007629, &values->v6_0v9);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CHB, REG_IMON_ADC0_CHB,
                         0.00007629, &values->v6_1v0);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CHD, REG_IMON_ADC0_CHD,
                         0.00015258, &values->v6_2v5);
  _status |= cin_ctl_calc_vi_status(cp, REG_VMON_ADC0_CHF, REG_IMON_ADC0_CHF,
                         0.00030516, &values->fp);
  if(_status){
    ERROR_COMMENT("Unable to read power values\n");
    return _status;
  }

  cin_ctl_display_pwr(stdout, values);
  return 0;
}

void cin_ctl_display_pwr(FILE *out, cin_ctl_pwr_mon_t *values){
  fprintf(out,"CIN Power values :\n\n");
  cin_ctl_display_pwr_line(out,"  V12P_BUS Power   :", values->bus_12v0);
  fprintf(out,"\n");
  cin_ctl_display_pwr_line(out,"  V3P3_MGMT Power  :", values->mgmt_3v3);
  cin_ctl_display_pwr_line(out,"  V2P5_MGMT Power  :", values->mgmt_2v5);
  cin_ctl_display_pwr_line(out,"  V1P2_MGMT Power  :", values->mgmt_1v2);
  cin_ctl_display_pwr_line(out,"  V1P0_ENET Power  :", values->enet_1v0);
  fprintf(out,"\n");
  cin_ctl_display_pwr_line(out,"  V3P3_S3E Power   :", values->s3e_3v3);
  cin_ctl_display_pwr_line(out,"  V3P3_GEN Power   :", values->gen_3v3);
  cin_ctl_display_pwr_line(out,"  V2P5_GEN Power   :", values->gen_2v5);
  fprintf(out,"\n");
  cin_ctl_display_pwr_line(out,"  V0P9_V6 Power    :", values->v6_0v9);
  cin_ctl_display_pwr_line(out,"  V1P0_V6 Power    :", values->v6_1v0);
  cin_ctl_display_pwr_line(out,"  V2P5_V6 Power    :", values->v6_2v5);
  fprintf(out,"\n");
  cin_ctl_display_pwr_line(out,"  V_FP Power       :", values->fp);
}

void cin_ctl_display_pwr_line(FILE *out,const char* msg, cin_ctl_pwr_val_t val){
  fprintf(out,"%s %0.2f V @ %0.2f A \n", msg, val.v, val.i);
}

/******************* CIN Control *************************/

int cin_ctl_set_bias(struct cin_port* cp,int val){

  int _status;
   
  if (val == 1){
    _status = cin_ctl_write(cp,REG_BIASCONFIGREGISTER0_REG, 0x0001);
  } else if (val == 0){
    _status = cin_ctl_write(cp,REG_BIASCONFIGREGISTER0_REG, 0x0000);
  } else {
    ERROR_COMMENT("Illegal Bias state: Only 0 or 1 allowed\n");
    return -1;
  }

  if(_status){
    ERROR_COMMENT("Unable to set bias.\n");
    return _status;
  }

  DEBUG_PRINT("Bias state set to %d\n", val);
  return 0;
}

int cin_ctl_set_clocks(struct cin_port* cp,int val){

  int _status;   
   
  if (val == 1){
    _status = cin_ctl_write(cp,REG_CLOCKCONFIGREGISTER0_REG, 0x0001);
  } else if (val == 0){
    _status = cin_ctl_write(cp,REG_CLOCKCONFIGREGISTER0_REG, 0x0000);
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

int cin_ctl_set_trigger(struct cin_port* cp,int val){

  int _status;
  if((val < 0) || (val > 3)){ 
    ERROR_COMMENT("Illegal Trigger state: Only values 0 to 3 allowed\n");
    return -1;
  }

  _status=cin_ctl_write(cp,REG_TRIGGERMASK_REG, val);
  if(_status){
    ERROR_PRINT("Unable to set trigger to %d\n", val);
    return _status;
  }

  DEBUG_PRINT("Trigger set to %d\n", val);
  return 0;
}

int cin_ctl_get_trigger(struct cin_port* cp, int *val){

  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cp, REG_TRIGGERMASK_REG, &_val); 
   
  if(_status){
    ERROR_COMMENT("Unable to read trigger status\n");
    return _status;
  }

  *val = (int)_val;
  DEBUG_PRINT("Trigger value is %d\n", *val);
  return 0;
}

int cin_ctl_set_focus(struct cin_port* cp, int val){

  uint16_t _val1;
  int _status;
   
  _status = cin_ctl_read(cp,REG_CLOCKCONFIGREGISTER0_REG, &_val1);
  if(_status){
    ERROR_COMMENT("Unable to read focus bit\n");
    return _status;
  }

  if(val){
    _val1 |= CIN_CTL_FOCUS_BIT;
  } else {
    _val1 &= ~CIN_CTL_FOCUS_BIT;
  } 

  _status = cin_ctl_write(cp,REG_CLOCKCONFIGREGISTER0_REG, _val1);
  if(_status){
    ERROR_COMMENT("Unable to write focus bit\n");
    return _status;
  }

  return 0;
}

int cin_clt_trigger_start(struct cin_port* cp, int nimages, int trigger_mode){
  // Trigger the camera, setting the trigger mode

  int _status;

  // First set the trigger mode (internal / external etc.)

  _status = cin_ctl_set_trigger(cp, trigger_mode);
  if(_status){
    ERROR_COMMENT("Unable to set trigger mode\n");
    goto error;
  }

  _status  = cin_ctl_write(cp, REG_NUMBEROFEXPOSURE_REG, (uint16_t)nimages);
  _status |= cin_ctl_write(cp, REG_FRM_COMMAND, 0x0100);
  if(_status){
    ERROR_COMMENT("Unable to start triggers");
    goto error;
  }

  if(nimages != 1){
    _status = cin_ctl_set_focus(cp, 1);
    if(_status){
      ERROR_COMMENT("Unable to set focus bit.\n");
      goto error;
    }
  }

  DEBUG_COMMENT("Triggers started");
  return 0;

error:
  return _status;
}
      
int cin_ctl_trigger_stop(struct cin_port* cp){
  int _status;
  _status  = cin_ctl_set_focus(cp, 0);
  _status |= cin_ctl_set_trigger(cp, CIN_CTL_TRIG_INTERNAL);

  if(_status){
    ERROR_COMMENT("Error stopping triggers");
    return _status;
  }
  DEBUG_COMMENT("Stopped Triggers");
  return 0;
}

int cin_ctl_set_exposure_time(struct cin_port* cp,float ftime){

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;

  ftime = ftime * 1e5;
  _time = (uint32_t)ftime;  //Extract integer from decimal
   
  _msbval = (uint32_t)(_time >> 16);
  _lsbval = (uint32_t)(_time & 0xFFFF);

  _status  = cin_ctl_write(cp,REG_EXPOSURETIMEMSB_REG,_msbval);
  _status |= cin_ctl_write(cp,REG_EXPOSURETIMELSB_REG,_lsbval);
  if(_status){
    ERROR_COMMENT("Unable to set exposure time");
    return _status;
  }

  DEBUG_PRINT("Exposure time set to %d (10this us)\n", _time);
  return 0;
}

int cin_ctl_set_trigger_delay(struct cin_port* cp,float ftime){  

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;

  _time=(uint32_t)ftime;   //Extract integer from decimal

  _msbval=(uint16_t)(_time >> 16);
  _lsbval=(uint16_t)(_time & 0xFFFF);

  _status  = cin_ctl_write(cp,REG_DELAYTOEXPOSUREMSB_REG,_msbval);
  _status |= cin_ctl_write(cp,REG_DELAYTOEXPOSURELSB_REG,_lsbval);
  if(_status){
    ERROR_COMMENT("Unable to set trigger delay");
    return _status;
  }

  DEBUG_PRINT("Set trigger delay to %d\n", _time);
  return 0;
}

int cin_ctl_set_cycle_time(struct cin_port* cp,float ftime){

  int _status;
  uint32_t _time;
  uint16_t _msbval,_lsbval;
                                       
  ftime = ftime*1000;         //Convert to ms
  _time = (uint32_t)ftime;    //Extract integer from decimal

  _msbval=(uint16_t)(_time >> 16);
  _lsbval=(uint16_t)(_time & 0xFFFF);

  _status  = cin_ctl_write(cp,REG_TRIGGERREPETITIONTIMEMSB_REG,_msbval);
  _status |= cin_ctl_write(cp,REG_TRIGGERREPETITIONTIMELSB_REG,_lsbval);

  if(_status){
    ERROR_COMMENT("Unable to set cycle time");
    return _status;
  } 

  DEBUG_PRINT("Cycle time set to %d msec\n", _time);
  return 0;
}

/******************* Frame Acquisition *************************/
int cin_ctl_frame_count_reset(struct cin_port* cp){

  int _status;
  _status = cin_ctl_write(cp,REG_FRM_COMMAND, CMD_RESET_FRAME_COUNT);
  if(_status){
    ERROR_COMMENT("Unable to reset frame counter\n");
    return _status;
  }

  DEBUG_COMMENT("Frame count reset to 0\n");
  return 0;
}

