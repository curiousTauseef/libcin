#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[])
{
  cin_ctl_t cin_ctl;
  cin_data_t cin_data;
  cin_set_debug_print(1);
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

  if(cin_com_boot(&cin_ctl, &cin_data, argv[1]) != CIN_OK)
  {
    fprintf(stderr,"Unable to boot CIN\n");
    goto error;
  }

  cin_ctl_set_cycle_time(&cin_ctl, 1.0);
  cin_ctl_set_exposure_time(&cin_ctl, 0.001);
  cin_ctl_int_trigger_start(&cin_ctl, 0);

  cin_ctl_destroy(&cin_ctl);
  cin_data_destroy(&cin_data);
  return 0;

error:
  cin_ctl_destroy(&cin_ctl);
  cin_data_destroy(&cin_data);
  return 128;
}
