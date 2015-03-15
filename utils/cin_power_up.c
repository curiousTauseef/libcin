/* vim: set ts=2 sw=2 tw=0 noet expandtab :
   
   libcin : Driver for LBNL FastCCD 
   Copyright (c) 2014, Stuart B. Wilkins, Daron Chabot
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
#include <stdio.h>
#include <unistd.h> /* for sleep() */
#include <string.h>
#include <stdlib.h>
  
#include "cin.h"

int main(){

	/*Set directory for CIN configuration files*/ 
	char fpga_config_dir[]="/home/swilkins/Repos/lbl-fastccds/CIN_1kFSCCD_BINARY/";

	/*Set CIN FPGA configuration file*/   
	char fpga_configfile[]="top_frame_fpga-v1019j.bit";

	/*Set CIN configuration file*/ 
  char cin_config_dir[] = "/home/swilkins/Repos/lbl-fastccds/CameraConfigFiles/";

	char cin_configfile_waveform[] = "2013_Nov_30-200MHz_CCD_timing.txt";

	/*Create Path to files*/
	char cin_fpga_config[1024];
	char cin_waveform_config[1024];
	sprintf(cin_fpga_config,"%s%s", fpga_config_dir,fpga_configfile);
	sprintf(cin_waveform_config,"%s%s", cin_config_dir,cin_configfile_waveform);

	/*Set control ports*/	
	struct cin_port cp[2];
	
	cin_ctl_init_port(&cp[0], 0, CIN_CTL_SVR_PORT, CIN_CTL_CLI_PORT);
	cin_ctl_init_port(&cp[1], 0, CIN_CTL_SVR_FRMW_PORT, CIN_CTL_CLI_FRMW_PORT);

  fprintf(stderr, "Powering OFF CIN ............................");
	cin_ctl_pwr(&cp[0], 0);
	sleep(2);
  fprintf(stderr, " DONE\n");  

  fprintf(stderr, "Powering ON CIN .............................");
	cin_ctl_pwr(&cp[0], 1);
	sleep(2);
  fprintf(stderr, " DONE\n");

  fprintf(stderr, "Powering ON CIN Front Panel .................");
	cin_ctl_fp_pwr(&cp[0], 1);
	sleep(2);
  fprintf(stderr, " DONE\n");

  //fprintf(stderr, "\n");
  //cin_ctl_fpga_status_t fpga_status;
  //cin_ctl_get_cfg_fpga_status(&cp[0], &fpga_status);
  //cin_ctl_display_fpga_status(stderr, &fpga_status);
  //fprintf(stderr, "\n");
  //uint16_t dcm;
  //cin_ctl_get_dcm_status(&cp[0], &dcm);
  //cin_ctl_display_dcm_status(stderr, &dcm);
  //fprintf(stderr, "\n");

  int tries = 5;
  while(tries){
    fprintf(stderr, "Loading Firmware ............................");
		if(cin_ctl_load_firmware(&cp[0],&cp[1],cin_fpga_config)){
      fprintf(stderr, "FAILED.\n");
    } else {
      break;
    }
    tries--;
  }
  
  if(tries){  
    fprintf(stderr, " DONE\n");
  }

  fprintf(stderr, "Setting fabric IP address ...................");
  cin_ctl_set_fabric_address(&cp[0], "10.23.5.127");
  fprintf(stderr, " DONE\n");

  //fprintf(stderr, "\n");
  //cin_ctl_get_cfg_fpga_status(&cp[0], &fpga_status);
  //cin_ctl_display_fpga_status(stderr, &fpga_status);
  //cin_ctl_get_dcm_status(&cp[0], &dcm);
  //cin_ctl_display_dcm_status(stderr, &dcm);
  //fprintf(stderr, "\n");

  // This appears to be broken
  //int fclk;
	//cin_ctl_get_fclk(&cp[0], &fclk);			

  cin_ctl_pwr_mon_t pwr_values;
  int pwr;
  cin_ctl_get_power_status(&cp[0], &pwr, &pwr_values);
  fprintf(stderr, "\n");
  cin_ctl_display_pwr(stderr, &pwr_values);
  fprintf(stderr, "\n");

/************************* FCCD Configuration **************************/	

  fprintf(stderr, "Loading FPGA Clock Configuration ............");
  cin_ctl_load_config(&cp[0],cin_waveform_config);		//Load FCCD clock configuration
  fprintf(stderr, " DONE\n");

	//sleep(3);
	
	//cin_load_config(&cp[0],cin_fcric_config);		//Load CIN fcric Configuration
	//sleep(3);
	
	//cin_load_config(&cp[0],cin_bias_config);		//Load FCCD bias Configuration
	//sleep(3);

/**********************************************************************/		

  cin_ctl_close_port(&cp[0]);
  cin_ctl_close_port(&cp[1]);

  fprintf(stderr, "\n\n");
	return 0;				
}
