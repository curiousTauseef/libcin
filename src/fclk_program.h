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
#ifndef _FCLK_PROGRAM
#define _FCLK_PROGRAM 

#define CIN_FCLK_NUM_REG 6

uint16_t CIN_FCLK_REG[] = { 0xB007, 0xB008, 0xB009, 0xB00A, 0xB00B, 0xB00C };

uint16_t CIN_FCLK_PROGRAM[][CIN_FCLK_NUM_REG] = {
  { 0xF002, 0xF042, 0xF0BC, 0xF019, 0xF06D, 0xF08F },
  { 0xF060, 0xF0C3, 0xF010, 0xF023, 0xF07D, 0xF0ED },
  { 0xF020, 0xF0C2, 0xF0BC, 0xF019, 0xF06D, 0xF08F },
  { 0xF060, 0xF0C2, 0xF0C1, 0xF0B9, 0xF08A, 0xF0EF }
};

#define CIN_FCLK_READ_N 7
uint16_t CIN_FCLK_READ[] = { 0xB189, 0xB107, 0xB108, 0xB109, 0xB10A, 0xB10B, 0x10C };

#endif
