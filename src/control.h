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
#ifndef __CIN_API_H__
#define __CIN_API_H__

#include "cin.h"
#include "fifo.h"

float bias_voltage_range[] = {
  9.0, -9.0, 9.0, -9.0, 9.0, -9.0, 9.0, -9.0, 9.0, -9.0, 
  9.0, -9.0, 99.0, 5.0, -15.0, -25.0, -10.0, -5.1, 0.0, 0.0
};

char cin_ctl_bias_name[][20] = {
  "POSH",     "NEGH",      "POSRG",    "NEGRG",    
  "POSSW",    "NEGSW",     "POSV",     "NEGV",    
  "POSTG",    "NEGTG",     "POSVF",    "NEGVF",
  "NEDGE",    "OTG",       "VDDR",     "VDD_OUT",
  "BUF_BASE", "BUF_DELTA", "SPARE1",   "SPARE2"
};

#define CIN_CTL_FCLK_NUM_REG            6
#define CIN_CTL_FCRIC_NUM_REG           8

uint16_t cin_ctl_fcric_clamp_reg[CIN_CTL_FCRIC_NUM_REG] =     
{ 
  0x0048, 0x0049, 0x0050, 0x0051, 0x0058, 0x0059, 0x005A, 0x005B
};
uint16_t cin_ctl_fcric_clamp_reg_on[CIN_CTL_FCRIC_NUM_REG] =  
{ 
  0x0001, 0x00FF, 0x0001, 0x00FF, 0x00FF, 0x0001, 0x00FF, 0x0001
};
uint16_t cin_ctl_fcric_clamp_reg_off[CIN_CTL_FCRIC_NUM_REG] = 
{ 
  0x00C7, 0x004C, 0x00B4, 0x0002, 0x0001, 0x004C, 0x0064, 0x005B
};

uint16_t cin_ctl_fcric_regs[] = {0x821D, 0x821E, 0x821F, 0x8001};
uint16_t cin_ctl_bias_timing_regs[] = {0x8200, 0x8201, 0x8001};

uint16_t CIN_FCLK_REG[] = 
{ 
    0xB007, 0xB008, 0xB009, 0xB00A, 0xB00B, 0xB00C 
};

#define CIN_FCLK_PROGRAM_125        0
#define CIN_FCLK_PROGRAM_180        1
#define CIN_FCLK_PROGRAM_200        2
#define CIN_FCLK_PROGRAM_250        3

uint16_t CIN_FCLK_PROGRAM[][CIN_CTL_FCLK_NUM_REG] = 
{
  { 0xF002, 0xF042, 0xF0BC, 0xF019, 0xF06D, 0xF08F }, // 125 MHz
  { 0xF060, 0xF0C2, 0xF0C1, 0xF0B9, 0xF08A, 0xF0EF }, // 180 MHz
  { 0xF060, 0xF0C3, 0xF010, 0xF023, 0xF07D, 0xF0ED }, // 200 MHz
  { 0xF020, 0xF0C2, 0xF0BC, 0xF019, 0xF06D, 0xF08F }, // 250 MHz
};

#define CIN_FCLK_READ_N 7
uint16_t CIN_FCLK_READ[] = 
{ 
  0xB189, 0xB107, 0xB108, 0xB109, 0xB10A, 0xB10B, 0xB10C 
};

void *cin_ctl_listen_thread(void* args);
uint32_t cin_ctl_get_packet(cin_ctl_t *cin, uint32_t *val);
int cin_ctl_init_port(cin_port_t *cp);
int cin_ctl_close_ports(cin_ctl_t *cin);
int cin_ctl_set_address(cin_ctl_t *cin, char *ip, uint16_t reg0, uint16_t reg1);
int cin_ctl_freeze_dco(cin_ctl_t *cin, int freeze);
double cin_ctl_current_calc(uint16_t val);
int cin_ctl_calc_vi_status(cin_ctl_t *cin, 
                           uint16_t vreg, uint16_t ireg, double vfact,
                           cin_ctl_pwr_val_t *vi);

int cin_ctl_set_fclk_regs(cin_ctl_t *cin, int clkfreq);
int cin_ctl_freeze_dco(cin_ctl_t *cin, int freeze);
void cin_ctl_message(cin_ctl_t *cin, const char *message, int severity);
#endif /* ifndef __CIN_API_H__ */
