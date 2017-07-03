#include <stdlib.h>
#include <stdio.h>

#include "cin.h"

int main(int argc, char *argv[]){
  cin_set_debug_print(1);
  cin_set_error_print(1);

  cin_ctl_t cin;

  int val = cin_config_read_file(&cin, argv[1]);

  DEBUG_PRINT("config return val = %d\n", val);

  exit(0);
}
