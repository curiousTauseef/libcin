#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[])
{
  int status;
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
  sleep(2);
  if(cin_ctl_pwr(&cin, 1))
  {
    return -1;
  }
  sleep(2);

  if(cin_ctl_load_firmware(&cin))
  {
    return -1;
  }

  return 0;
}
