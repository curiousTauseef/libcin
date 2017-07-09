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

float bias_voltage_range[NUM_BIAS_VOLTAGE] = {
  9.0, -9.0, 9.0, -9.0, 9.0, -9.0, 9.0, -9.0, 9.0, -9.0, 
  9.0, -9.0, 99.0, 5.0, -15.0, -25.0, -10.0, -5.1, 0.0, 0.0
};

#define NUM_CLAMP_REG 8
uint16_t fcric_clamp_reg[NUM_CLAMP_REG] =     { 0x0048, 0x0049, 0x0050, 0x0051, 
                                                0x0058, 0x0059, 0x005A, 0x005B};
uint16_t fcric_clamp_reg_on[NUM_CLAMP_REG] =  { 0x0001, 0x00FF, 0x0001, 0x00FF, 
                                                0x00FF, 0x0001, 0x00FF, 0x0001};
uint16_t fcric_clamp_reg_off[NUM_CLAMP_REG] = { 0x00C7, 0x004C, 0x00B4, 0x0002, 
                                                0x0001, 0x004C, 0x0064, 0x005B};

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

#endif /* ifndef __CIN_API_H__ */
