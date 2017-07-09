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
#include <libconfig.h>

#include "cin.h"
#include "config.h"

void cin_config_init(cin_ctl_config_t *config){

  config->overscan = 0;
  config->columns = 960;
  config->fclk = 200;
 
  strcpy(config->firmware_filename, "");
}

int cin_config_read_regdata(config_t *cfg, const char *name, uint16_t data[][2], int *len){
  config_setting_t *setting = config_lookup(cfg, name);

  if(setting != NULL){
    int count = config_setting_length(setting);
    int i;

    DEBUG_PRINT("Data length for %s = %d\n", name, count);
   
    if(count > CIN_CONFIG_MAX_DATA){
      ERROR_PRINT("Reg data for %s it too large\n", name);
      return -1;
    }

    *len = count;
    
    for(i=0;i<count;i++){
      int _reg = 0, _val = 0;
      config_setting_t *_element = config_setting_get_elem(setting, i);
      if(_element != NULL){
        config_setting_lookup_int(_element, "reg", &_reg);
        config_setting_lookup_int(_element, "val", &_val);
        data[i][0] = (uint16_t)_reg;
        data[i][1] = (uint16_t)_val;
      } else {
        ERROR_PRINT("Error on %s element %d\n", name, i);
      }
    }
  } else {
    ERROR_PRINT("Unable to find %s in config file\n", name);
    return -1;
  }
  return 0;
}

int cin_config_read_file(cin_ctl_t *cin, const char *file){
  config_t cfg;
  cin_ctl_config_t *config = &(cin->config);

  config_init(&cfg);

  if(!config_read_file(&cfg, file)){
    ERROR_PRINT("Unable to parse config file %s\n", file);
    ERROR_PRINT("%s:%d - %s\n", config_error_file(&cfg),
                                config_error_line(&cfg), config_error_text(&cfg));
    config_destroy(&cfg);
    return -1;
  }
  const char *name;
  if(!config_lookup_string(&cfg, "name", &name)){
    ERROR_PRINT("No name in config file %s\n", file);
    strcpy(config->name, "");
  } else {
    strncat(config->name, name, CIN_CONFIG_MAX_STRING);
  }

  //const char *_path;
  //char global_path[CIN_CONFIG_MAX_FILENAME];
  //if(!config_lookup_string(&cfg, "global_path", &_path)){
  //  DEBUG_COMMENT("No global path in config file\n");
  //  strcpy(global_path, "");
  //} else {
  //  strlcpy(global_path, _path, CIN_CONFIG_MAX_FILENAME);
  //  if(global_path[strlen(global_path)-1] != '/'){
  //    strlcat(global_path, "/", CIN_CONFIG_MAX_FILENAME);
  //  }
  //}
  //DEBUG_PRINT("Global path set to %s\n", global_path);

  config_lookup_int(&cfg, "timing.overscan", &config->overscan);
  config_lookup_int(&cfg, "timing.columns", &config->columns);
  config_lookup_int(&cfg, "timing.fclk", &config->fclk);
  DEBUG_PRINT("timing overscan = %d\n", config->overscan);
  DEBUG_PRINT("timing columns  = %d\n", config->columns);
  DEBUG_PRINT("timing fclk     = %d\n", config->fclk);
  if(cin_config_read_regdata(&cfg, "timing.data", config->timing, &(config->timing_len))){
    goto error;
  }
  if(cin_config_read_regdata(&cfg, "fcric.data", config->fcric, &(config->fcric_len))){
    goto error;
  }
  if(cin_config_read_regdata(&cfg, "bias.data", config->bias, &(config->bias_len))){
    goto error;
  }

  //int i;
  //for(i=0;i<config->timing_len;i++){
  //  DEBUG_PRINT("%s data reg = 0x%04X val = 0x%04X\n", "timing.data", config->timing[i][0], config->timing[i][1]
  //      );
  //}

    // Loop through all posibilities
    
    //for(i = 0; i < count; ++i){
    //  config_setting_t *_element = config_setting_get_elem(, i);
    //  if(!config_setting_lookup_string(_element, "name", &name)){
    //    continue;
    //  }

    //  if(strcmp(name, config_name)){
    //    DEBUG_PRINT("Skipping config : %s\n", name);
    //    continue;
    //  }

    //  DEBUG_PRINT("Reading Config : %s\n", name);

    //  if(!(config_setting_lookup_int(_element, "columns", &config->columns) &&
    //       config_setting_lookup_int(_element, "overscan", &config->overscan) &&
    //       config_setting_lookup_int(_element, "fclk", &config->fclk) &&
    //       config_setting_lookup_string(_element, "firmware", &_firmware) &&
    //       config_setting_lookup_string(_element, "bias", &_bias) &&
    //       config_setting_lookup_string(_element, "clocks", &_clocks) &&
    //       config_setting_lookup_string(_element, "fcric", &_fcric))){
    //    ERROR_PRINT("Error in config \"%s\" in file %s\n", name, file);
    //    continue;
    //  }
    // 
    //  char _buf[CIN_MAX_FILENAME];
    //  strlcpy(_buf, global_path, CIN_MAX_FILENAME);
    //  strlcat(_buf, _firmware, CIN_MAX_FILENAME);
    //  strlcpy(config->firmware_filename, _buf, CIN_MAX_FILENAME);

    //  strlcpy(_buf, global_path, CIN_MAX_FILENAME);
    //  strlcat(_buf, _bias, CIN_MAX_FILENAME);
    //  strlcpy(config->bias_filename, _buf, CIN_MAX_FILENAME);

    //  strlcpy(_buf, global_path, CIN_MAX_FILENAME);
    //  strlcat(_buf, _clocks, CIN_MAX_FILENAME);
    //  strlcpy(config->clocks_filename, _buf, CIN_MAX_FILENAME);

    //  strlcpy(_buf, global_path, CIN_MAX_FILENAME);
    //  strlcat(_buf, _fcric, CIN_MAX_FILENAME);
    //  strlcpy(config->fcric_filename, _buf, CIN_MAX_FILENAME);

    //  DEBUG_PRINT("Overscan     = %d\n", config->overscan);
    //  DEBUG_PRINT("Columns      = %d\n", config->columns);
    //  DEBUG_PRINT("Firmware     = %s\n", config->firmware_filename);
    //  DEBUG_PRINT("Clocks       = %s\n", config->clocks_filename);
    //  DEBUG_PRINT("FCRIC        = %s\n", config->fcric_filename);
    //  DEBUG_PRINT("Bias         = %s\n", config->bias_filename);
    //  DEBUG_PRINT("FCLOCK       = %d\n", config->fclk);

  config_destroy(&cfg);
  return 0;

error:
  config_destroy(&cfg);
  return -1;

}
