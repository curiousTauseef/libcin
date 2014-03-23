#include <stdio.h>
#include <unistd.h> /* for sleep() */
#include <string.h>
#include <stdlib.h>

#include "cin.h"

int main(){

	/*Set directory for CIN configuration files*/ 
	char fpga_config_dir[]="/home/swilkins/Repos/lbl-fastccds/BINARY/CIN_1kFSCCD/";

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

  fprintf(stderr, "Powering OFF CIN.\n");
	cin_off(&cp[0]);
	sleep(5);

  fprintf(stderr, "Powering ON CIN.\n");
	cin_on(&cp[0]);
	sleep(5);

  fprintf(stderr, "Powering ON CIN Front Panel.\n");
	cin_fp_on(&cp[0]);
	sleep(5);

  fprintf(stderr, "\n");
  cin_ctl_fpga_status_t fpga_status;
  cin_ctl_get_cfg_fpga_status(&cp[0], &fpga_status);
  cin_ctl_display_fpga_status(stderr, &fpga_status);
  fprintf(stderr, "\n");
  uint16_t dcm;
  cin_ctl_get_dcm_status(&cp[0], &dcm);
  cin_ctl_display_dcm_status(stderr, &dcm);
  fprintf(stderr, "\n");

	cin_ctl_load_firmware(&cp[0],&cp[1],cin_fpga_config);	
	sleep(5);

  fprintf(stderr, "Setting fabric IP address\n");
  cin_ctl_set_fabric_address(&cp[0], "10.23.5.127");

  fprintf(stderr, "\n");
	cin_ctl_get_cfg_fpga_status(&cp[0], &fpga_status);
  cin_ctl_display_fpga_status(stderr, &fpga_status);
  cin_ctl_get_dcm_status(&cp[0], &dcm);
  cin_ctl_display_dcm_status(stderr, &dcm);
  fprintf(stderr, "\n");

  // This appears to be broken
  //int fclk;
	//cin_ctl_get_fclk(&cp[0], &fclk);			

  cin_ctl_pwr_mon_t pwr_values;
  int pwr;
	cin_ctl_get_power_status(&cp[0], &pwr, &pwr_values);
  cin_ctl_display_pwr(stderr, &pwr_values);

/************************* FCCD Configuration **************************/	

	cin_ctl_load_config(&cp[0],cin_waveform_config);		//Load FCCD clock configuration
	//sleep(3);
	
	//cin_load_config(&cp[0],cin_fcric_config);		//Load CIN fcric Configuration
	//sleep(3);
	
	//cin_load_config(&cp[0],cin_bias_config);		//Load FCCD bias Configuration
	//sleep(3);

/**********************************************************************/		

  // Do some setup. Set the focus bit

  cin_ctl_set_focus(&cp[0], 1);
  cin_ctl_set_mux(&cp[0], 0, CIN_CTL_MUX1_TRIGMON);
  cin_ctl_set_mux(&cp[0], 1, CIN_CTL_MUX2_CONVERT);
  // int focus;
  // cin_ctl_get_focus(&cp[0], &focus);

  cin_ctl_close_port(&cp[0]);
  cin_ctl_close_port(&cp[1]);


	return 0;				
}
