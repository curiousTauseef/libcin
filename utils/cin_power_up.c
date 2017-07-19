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

  if(cin_ctl_init(&cin_ctl, NULL, 0, 0, NULL, 0, 0))
  {
    perror("Unable to initialize CIN control library");
    return -1;
  }

  if(cin_data_init(&cin_data, "10.23.4.207", 0, NULL, 0, 0, 0, 0, 
        NULL, NULL, NULL))
  {
    perror("Unable to initialize CIN data library");
    return -1;
  }

  if(cin_com_boot(&cin_ctl, &cin_data, "default") != CIN_OK)
  {
    perror("Unable to boot CIN\n");
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
