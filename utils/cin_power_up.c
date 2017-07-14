#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[])
{
  cin_ctl_t cin;
  cin_set_debug_print(1);
  cin_set_error_print(1);

  if(cin_ctl_init(&cin, NULL, NULL, 0, 0, 0, 0))
  {
    perror("Unable to initialize CIN port");
    return -1;
  }

  if(cin_ctl_pwr(&cin, 0))
  {
    return -1;
  }
  sleep(1);
  if(cin_ctl_pwr(&cin, 1))
  {
    return -1;
  }
  sleep(1);

  if(cin_ctl_load_firmware(&cin))
  {
    return -1;
  }

  //if(cin_ctl_set_fabric_address(&cin, "10.23.4.207"))
  //{
  //  return -1;
  //}

  cin_ctl_set_fclk(&cin, CIN_CTL_FCLK_200);
  int fclk;
  cin_ctl_get_fclk(&cin, &fclk);

  if(cin_ctl_set_timing_regs(&cin, cin_config_timing, cin_config_timing_len))
  {
    return -1;
  }

  cin_ctl_id_t cin_id;
  if(cin_ctl_get_id(&cin, &cin_id))
  {
    return -1;
  }
  cin_ctl_set_cycle_time(&cin, 1);
  cin_ctl_set_exposure_time(&cin, 0.001);
  cin_ctl_int_trigger_start(&cin, 0);

  return 0;
}
