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
#include <bsd/string.h>
#include <unistd.h>

#include "cin.h"
#include "config.h"

#include "timing.h"

int cin_config_find_timing(cin_ctl_t *cin, char *name)
{
  int i;
  for(i=0;i<cin->timing_num;i++)
  {
    if(!strcmp(cin->timing[i].name, name))
    {
      DEBUG_PRINT("Found match %d (\"%s\")\n", 
          i, cin->timing[i].name);
      return i;
    }
  }
  return CIN_ERROR;
}

int cin_config_init(cin_ctl_t *cin)
{
  // Initialize built in timing modes

  DEBUG_COMMENT("Configuring default timing modes\n");
  // Defualt timing

  strcpy(cin->timing[0].name, "125MHz_TIMING");
  cin->timing[0].data = cin_config_125_timing;
  cin->timing[0].data_len = cin_config_125_timing_len;
  cin->timing[0].rows = 1920;
  cin->timing[0].cols = 960;
  cin->timing[0].overscan = 0;
  cin->timing[0].fclk_freq = CIN_CTL_FCLK_125_C;
  cin->timing[0].framestore = 0;

  strcpy(cin->timing[1].name, "125MHz_TIMING_FS");
  cin->timing[1].data = cin_config_125_timing_fs;
  cin->timing[1].data_len = cin_config_125_timing_fs_len;
  cin->timing[1].rows = 960;
  cin->timing[1].cols = 960;
  cin->timing[1].overscan = 0;
  cin->timing[1].fclk_freq = CIN_CTL_FCLK_125_C;
  cin->timing[1].framestore = 1;

  cin->timing_num = 2;

  DEBUG_PRINT("Configured %d timing modes.\n", cin->timing_num);

  return CIN_OK;
}
