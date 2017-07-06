#include <stdlib.h>
#include <stdio.h>

#include "cin.h"

int main(int argc, char *argv[]){
  cin_set_debug_print(1);
  cin_set_error_print(1);

  cin_ctl_t cin_ctl;
  cin_data_t cin_data;

  cin_ctl_init(&cin_ctl, NULL, 0, 0, 0, 0);
  cin_data_init(&cin_data, 50, 50, 
      NULL, 0, NULL, 0, 1024,
      NULL, NULL, NULL);

  DEBUG_PRINT("Git SHA           = %s\n", cin_build_git_sha);
  DEBUG_PRINT("Git Time          = %s\n", cin_build_git_time);
  DEBUG_PRINT("Git Build Version = %s\n", cin_build_version);

  cin_data_stop_threads(&cin_data);
  exit(0);
}
