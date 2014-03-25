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
#include "control.h"
#include "fifo.h"

/**************************** UDP Socket ******************************/

int cin_ctl_init_port(struct cin_port* cp, char* ipaddr, 
                      uint16_t oport, uint16_t iport) {
  if(ipaddr == 0){ 
    cp->srvaddr = CIN_CTL_IP; 
  } else {
    cp->srvaddr = strndup(ipaddr, strlen(ipaddr)); 
  }
   
  if(oport == 0){ 
    cp->srvport = CIN_CTL_SVR_PORT; 
  } else {
    cp->srvport = oport; 
  }

  if(iport == 0){
    cp->cliport = CIN_CTL_CLI_PORT;
  } else {
    cp->cliport = iport;
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
    
  // initialize CIN (server) and client (us!) sockaddr structs
  memset(&cp->sin_srv, 0, sizeof(struct sockaddr_in));
  cp->sin_srv.sin_family = AF_INET;
  cp->sin_srv.sin_port = htons(cp->srvport);
  cp->slen = sizeof(struct sockaddr_in);
  if(inet_aton(cp->srvaddr, &cp->sin_srv.sin_addr) == 0) {
    ERROR_COMMENT("CIN control port - inet_aton() failed!!");
    return -1;

  }
  memset(&cp->sin_cli, 0, sizeof(struct sockaddr_in));
  cp->sin_cli.sin_family = AF_INET;
  cp->sin_cli.sin_port = htons(cp->cliport);
  cp->sin_cli.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(cp->sockfd, (struct sockaddr *)&cp->sin_cli, sizeof(cp->sin_cli))){
    ERROR_COMMENT("Bind failed.\n");
    return -1;
  }

  // Now initialize a listener thread for UDP communications

  cin_ctl_listener_t *listener = malloc(sizeof(cin_ctl_listener_t));
    if(!listener){
    ERROR_COMMENT("Malloc failed.\n");
    return -1;
  }

  listener->cp = cp;
  if(fifo_init(&listener->ctl_fifo, sizeof(uint32_t), 100,1)){
    ERROR_COMMENT("Failed to initialize fifo.\n");
    return -1;
  }

  pthread_create(&listener->thread_id, NULL, 
                 cin_ctl_listen_thread, (void*)listener);

  cp->listener = listener;
  return 0;
}

uint32_t cin_ctl_get_packet(struct cin_port *cp, uint32_t *val){
  int i;
  long int r;
  for(i=0;i<1000;i++){
    r = fifo_used_bytes(&cp->listener->ctl_fifo);
    if(r){
      *val = *((uint32_t *)fifo_get_tail(&cp->listener->ctl_fifo, 0));
      fifo_advance_tail(&cp->listener->ctl_fifo, 0);
      return 0;
    }
    usleep(10);
  }

  return -1;
}

// This is to be run in a different thread

void *cin_ctl_listen_thread(void* args){
  uint32_t *buffer;
  uint32_t val;
  struct cin_port* cp;
  fifo *ctl_fifo;
  int i;

  // Parse the data structure passed to the thread

  ctl_fifo = &((cin_ctl_listener_t*)args)->ctl_fifo;
  cp       = ((cin_ctl_listener_t*)args)->cp;

  DEBUG_COMMENT("Started listener thread.\n");

  while(1){
    buffer = (uint32_t*)fifo_get_head(ctl_fifo);
    i = recvfrom(cp->sockfd, &val, sizeof(val), 0,
                 (struct sockaddr*)&cp->sin_cli,
                 (socklen_t*)&cp->slen);
    if(i != -1){
      *buffer = ntohl(val);
#ifdef __DEBUG_STREAM__
      DEBUG_PRINT("Received %d bytes value  = %08lX\n", i, (long int)(*buffer));
#endif
      fifo_advance_head(ctl_fifo);
    }
  }

  pthread_exit(NULL);
}

int cin_ctl_close_port(struct cin_port* cp) {
  if(cp->sockfd){
    DEBUG_COMMENT("Canceling thread\n");
    pthread_cancel(cp->listener->thread_id);
    pthread_join(cp->listener->thread_id, NULL);
    DEBUG_COMMENT("Thread returned\n");
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
      ERROR_COMMENT("CIN control port - sendto() failure!!");
      goto error;
   }
#ifdef __DEBUG_STREAM__ 
  DEBUG_PRINT("Set register %04X to %04X\n", reg, val);
#endif 

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

  _status=cin_ctl_write(cp, REG_READ_ADDRESS, reg);
  if (_status){
    goto error;
  }

  //usleep(100000); // Hard coded sleep (0.1 s)

  _status=cin_ctl_write(cp, REG_COMMAND, CMD_READ_REG);
  if (_status != 0){
    goto error;
  }

  if(cin_ctl_get_packet(cp, &buf)){
    ERROR_PRINT("Failed to read register %04X\n", reg);
    goto error;
  }

#ifdef __DEBUG_STREAM__
  DEBUG_PRINT("Got %04X from %04X\n", buf & 0xFFFF, (buf >> 16));
#endif

  if((buf >> 16) != reg){
    ERROR_PRINT("Read value is register %04X was expecting %04X\n", 
                (buf >> 16), reg);
    goto error;
  }

  *val = (uint16_t)buf;

  return 0;
    
error:  
   ERROR_COMMENT("Read error.\n");
   return (-1);
}

 
/******************* CIN PowerUP/PowerDown *************************/
int cin_pwr(struct cin_port* cp, int pwr){

  int _status;
  uint16_t _val;

  if(pwr){
    DEBUG_COMMENT("Powering ON CIN Board\n");
    _val = 0x000F;
  } else {
    DEBUG_COMMENT("Powering OFF CIN Board\n");
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
    DEBUG_COMMENT("Powering ON CIN Front Panel Boards\n");
    _val = 0x003F;
  } else {
    DEBUG_COMMENT("Powering OFF CIN Front Panel Boards\n");
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
  return cin_fp_pwr(cp, 0);
}
int cin_fp_on(struct cin_port* cp){
  return cin_fp_pwr(cp, 1);
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

  DEBUG_PRINT("Loading %s\n",filename);
   
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
        fclose(file);
        goto error;
      }
   }
  }
  
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
            
  fseek(file, 0L, SEEK_END);
  long size = ftell(file);
  fseek(file, 0L, SEEK_SET);

  DEBUG_PRINT("Loading %s\n", filename);

  _status = cin_ctl_write(cp,REG_COMMAND,CMD_PROGRAM_FRAME); 
  if (_status != 0){
    ERROR_COMMENT("Failed to program CIN\n");
    goto error;
  }   

  sleep(1);
  //int count = 0;
  //size = size / sizeof(buffer); // Number of writes
  fprintf(stderr, "Loading Firmware ......... ");
  while ((num_e = fread(buffer,sizeof(char), sizeof(buffer), file)) != 0){    
    _status = cin_ctl_stream_write(dcp, buffer, num_e);       
    if (_status != 0){
      ERROR_COMMENT("Error writing firmware to CIN\n");
      fclose(file);
      goto error;
    }
    // int i;
    // for(i=0;i<(count * 50/ size);i++){
    //   fprintf(stderr, "#");
    // }
    // for(;i<50;i++){
    //   fprintf(stderr, ".");
    // }
    // fprintf(stderr, "]\r");
    // count++;
    usleep(2000);   /*for UDP flow control*/ 
  }
  fclose(file);

  fprintf(stderr, "Done.\n");
  
  sleep(1);

  fprintf(stderr, "Resetting Frame FPGA ..... ");

  _status=cin_ctl_write(cp,REG_FRM_RESET,0x0001);
  if(_status != 0){
    goto error;
  } 
  sleep(1); 
  _status=cin_ctl_write(cp,REG_FRM_RESET,0x0000);
  if(_status != 0){
    goto error;
  } 

  sleep(1);
  fprintf(stderr, "Done.\n");
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

void cin_ctl_display_fpga_status(FILE *out, cin_ctl_fpga_status_t *_val){
  fprintf(out, "FPGA Status :\n\n");
  fprintf(out, "  Board ID         : %04X\n", _val->board_id);
  fprintf(out, "  Serial Number    : %04X\n", _val->serial_no);
  fprintf(out, "  CFG FPGA Ver.    : %04X\n", _val->fpga_ver);
  fprintf(out, "  CFG FPGA Status  : %04X\n\n", _val->fpga_status);
  if(_val->fpga_status & CIN_CTL_FPGA_STS_CFG){
    fprintf(out, "  ** Frame FPGA Configuration Done\n");
  } else {
    fprintf(out, "  ** Frame FPGA NOT CONFIGURED!\n");
  }
  if(_val->fpga_status & CIN_CTL_FPGA_STS_FP_PWR){
    fprintf(out, "  ** FP Power Supply Unlocked\n");
  } else {
    fprintf(out, "  ** FP Power Supply Locked Off\n");
  }
}

int cin_ctl_get_dcm_status(struct cin_port* cp, uint16_t *_val){
  int _status;

  _status = cin_ctl_read(cp, REG_DCM_STATUS, _val);
  if(_status){
    ERROR_COMMENT("Unable to read DCM status.\n");
    return _status;
  }

  DEBUG_PRINT("CFG DCM Status   :  %04X\n", *_val);
  return 0; 
}

void cin_ctl_display_dcm_status(FILE *out, uint16_t *_val){
  fprintf(out, "CFG DCM Status :\n\n");
  fprintf(out, "  DCM Status       : %04X\n\n", *_val);
  if(*_val & CIN_CTL_DCM_STS_ATCA){
    fprintf(out, "  ** ATCA 48V Alarm Active\n");
  } else {
    fprintf(out, "  ** ATCA 48V OK\n");
  }
  if(*_val & CIN_CTL_DCM_STS_LOCKED){
    fprintf(out, "  ** CFG Clock DCM Locked\n");
  } else {
    fprintf(out, "  ** CFG Clock DCM NOT Locked\n");
  }
  if(*_val & CIN_CTL_DCM_STS_OVERIDE){
    fprintf(out, "  ** FP Power Supply Interlock Overide Enabled\n");
  } else {
    fprintf(out, "  ** FP Power Supply Interlock Active\n");
  }
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

int cin_ctl_get_focus(struct cin_port* cp, int *val){
  int _status;
  uint16_t _val;
  _status = cin_ctl_read(cp, REG_CLOCKCONFIGREGISTER0_REG, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read focus status\n");
    return _status;
  }

  if(_val & CIN_CTL_FOCUS_BIT){
    *val = 1;
  } else {
    *val = 0;
  }

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

int cin_ctl_int_trigger_start(struct cin_port* cp, int nimages){
  // Trigger the camera, setting the trigger mode

  int _status;

  DEBUG_PRINT("Set n exposures to %d\n", nimages);
  _status  = cin_ctl_write(cp, REG_NUMBEROFEXPOSURE_REG, (uint16_t)nimages);
  _status |= cin_ctl_write(cp, REG_FRM_COMMAND, 0x0100);
  if(_status){
    ERROR_COMMENT("Unable to start triggers");
    goto error;
  }

  DEBUG_COMMENT("Trigger sent.\n");
  return 0;

error:
  return _status;
}
      
int cin_ctl_int_trigger_stop(struct cin_port* cp){
  int _status;
  _status = cin_ctl_set_trigger(cp, CIN_CTL_TRIG_INTERNAL);

  if(_status){
    ERROR_COMMENT("Error stopping internal triggers\n");
    return _status;
  }
  DEBUG_COMMENT("Stopped internal triggers\n");
  return 0;
}

int cin_ctl_ext_trigger_start(struct cin_port* cp, int trigger_mode){
  // Trigger the camera, setting the trigger mode

  int _status;

  // First set the trigger mode (internal / external etc.)

  _status = cin_ctl_set_trigger(cp, trigger_mode);
  if(_status){
    ERROR_COMMENT("Unable to set trigger mode\n");
    goto error;
  }

  DEBUG_COMMENT("External triggers set.\n");
  return 0;

error:
  return _status;
}
      
int cin_ctl_ext_trigger_stop(struct cin_port* cp){
  int _status;
  _status = cin_ctl_set_trigger(cp, CIN_CTL_TRIG_INTERNAL);

  if(_status){
    ERROR_COMMENT("Error stopping external triggers\n");
    return _status;
  }
  DEBUG_COMMENT("Stopped external triggers\n");
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

/* Setting of IP Addresses */

int cin_ctl_set_fabric_address(struct cin_port* cp, char *ip){
  return cin_ctl_set_address(cp, ip, REG_IF_IP_FAB1B0, REG_IF_IP_FAB1B1);
}

int cin_ctl_set_address(struct cin_port* cp, char *ip, uint16_t reg0, uint16_t reg1){

  // First get the address

  struct in_addr addr;
  if(!inet_aton(ip, &addr)){
    ERROR_COMMENT("inet_aton() failed\n");
    return -1;
  }

  uint32_t addr_s = ntohl(addr.s_addr);

  DEBUG_PRINT("Setting IP to %08X\n", addr_s);

  int _status;
  _status  = cin_ctl_write(cp, reg0, (uint16_t)addr_s);
  _status |= cin_ctl_write(cp, reg1, (uint16_t)(addr_s >> 16));

  if(_status){
    ERROR_COMMENT("Could not set IP.\n");
    return -1;
  }

  return 0;
}

/*******************  MUX Settings for Output **********************/

int cin_ctl_set_mux(struct cin_port *cp, int setting){

  // First read the mux register

  uint16_t _val;
  int _status = cin_ctl_read(cp, REG_TRIGGERSELECT_REG, &_val);
  if(_status){
    ERROR_COMMENT("Unable to read MUX register\n");
    return -1;
  }

  if((setting & 0x000F) == 0x0){
    _val = (_val & 0xF0) | setting;
  } else {
    _val = (_val & 0x0F) | setting;
  }

  _status = cin_ctl_write(cp, REG_TRIGGERSELECT_REG, _val);
  if(_status){
    ERROR_COMMENT("Failed to write MUX setting\n");
    return -1;
  } 

  return 0;
}
