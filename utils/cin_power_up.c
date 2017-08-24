#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"


void msg_callback(const char *message, int severity, void *ptr)
{
  if(severity == CIN_CTL_MSG_OK)
  {
    fprintf(stderr, "--- %s\n", message);
  } else if(severity == CIN_CTL_MSG_MINOR){
    fprintf(stderr, "*** %s\n", message);
  } else if(severity == CIN_CTL_MSG_MAJOR){
    fprintf(stderr, "!!! %s\n", message);
  }
}


int main(int argc, char *argv[])
{
  cin_ctl_t cin_ctl;
  cin_data_t cin_data;

  cin_set_debug_print(0);
  cin_set_error_print(1);

  if(argc != 2)
  {
    fprintf(stderr, "usage : cin_power_up [timing name]\n\n");
    return -1;
  }

  if(cin_ctl_init(&cin_ctl, NULL, 0, 0, NULL, 0, 0))
  {
    fprintf(stderr,"Unable to initialize CIN control library\n");
    return -1;
  }

  if(cin_data_init(&cin_data, NULL, 0, NULL, 0, 0, 0, 0, 
        NULL, NULL, NULL))
  {
    fprintf(stderr,"Unable to initialize CIN data library\n");
    return -1;
  }

  cin_ctl_set_msg_callback(&cin_ctl, &msg_callback, NULL);

  // Set name
  int mode;
  if((mode = cin_config_find_timing(&cin_ctl, argv[1])) == CIN_ERROR)
  {
    fprintf(stderr, "Unable to find timing mode \"%s\"\n", argv[1]);
    goto error;
  }

  if(cin_com_boot(&cin_ctl, &cin_data, mode) != CIN_OK)
  {
    fprintf(stderr,"Unable to boot CIN\n");
    goto error;
  }

  cin_ctl_set_cycle_time(&cin_ctl, 1.0);
  cin_ctl_set_exposure_time(&cin_ctl, 0.0);
  //cin_ctl_int_trigger_start(&cin_ctl, 0);

  cin_ctl_destroy(&cin_ctl);
  cin_data_destroy(&cin_data);
  return 0;

error:
  cin_ctl_destroy(&cin_ctl);
  cin_data_destroy(&cin_data);
  return 128;
}
