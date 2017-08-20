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

#include "cin.h"
#include "config.h"

#include "timing.h"

int cin_config_get_current_timing_name(cin_ctl_t *cin, char **name)
{
  if(cin->current_timing != NULL)
  {
    *name = cin->current_timing->name;
    return CIN_OK;
  }

  return CIN_ERROR;
}

int cin_config_get_timing_name(cin_ctl_t *cin, int num, char **name)
{
  if((num < cin->timing_num) && (num >= 0))
  {
    *name = cin->timing[num].name;
    return CIN_OK;
  }

  return CIN_ERROR;
}

int cin_config_find_timing(cin_ctl_t *cin, const char *name)
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

  cin->current_timing = NULL;

  strcpy(cin->timing[0].name, "200MHz_TIMING_GOLD");
  cin->timing[0].data = cin_config_200_full_gold;
  cin->timing[0].data_len = cin_config_200_full_gold_len;
  cin->timing[0].rows = 1920;
  cin->timing[0].cols = 960;
  cin->timing[0].overscan = 2;
  cin->timing[0].fclk_freq = CIN_CTL_FCLK_200;
  cin->timing[0].framestore = 0;

  strcpy(cin->timing[1].name, "125MHz_TIMING");
  cin->timing[1].data = cin_config_125_timing;
  cin->timing[1].data_len = cin_config_125_timing_len;
  cin->timing[1].rows = 1920;
  cin->timing[1].cols = 960;
  cin->timing[1].overscan = 0;
  cin->timing[1].fclk_freq = CIN_CTL_FCLK_125_C;
  cin->timing[1].framestore = 0;

  strcpy(cin->timing[2].name, "125MHz_TIMING_FS");
  cin->timing[2].data = cin_config_125_timing_fs;
  cin->timing[2].data_len = cin_config_125_timing_fs_len;
  cin->timing[2].rows = 960;
  cin->timing[2].cols = 960;
  cin->timing[2].overscan = 0;
  cin->timing[2].fclk_freq = CIN_CTL_FCLK_125_C;
  cin->timing[2].framestore = 1;

  strcpy(cin->timing[3].name, "200MHz_TIMING_LCLS_FS");
  cin->timing[3].data = cin_config_200_lcls_fs;
  cin->timing[3].data_len = cin_config_200_lcls_fs_len;
  cin->timing[3].rows = 960;
  cin->timing[3].cols = 960;
  cin->timing[3].overscan = 0;
  cin->timing[3].fclk_freq = CIN_CTL_FCLK_200;
  cin->timing[3].framestore = 1;

  cin->timing_num = 4;

  DEBUG_PRINT("Configured %d timing modes.\n", cin->timing_num);
  
  int i;
  for(i=cin->timing_num;i<CIN_CONFIG_MAX_TIMING_MODES;i++)
  {
    strcpy(cin->timing[i].name, "");
  }
  for(i=0;i<CIN_CONFIG_MAX_TIMING_MODES;i++)
  {
    DEBUG_PRINT("Timing mode % 2d name = \"%s\"\n", i, cin->timing[i].name);
  }
  return CIN_OK;
}
