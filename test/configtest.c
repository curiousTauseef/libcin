#include <stdlib.h>
#include <stdio.h>

#include "cin.h"
#include "config.h"

int main(int argc, char *argv[]){
  cin_set_debug_print(1);
  cin_set_error_print(1);

  camera_config config;

  int val = cin_read_config_file(argv[1], "default", &config);

  DEBUG_PRINT("config return val = %d\n", val);

  exit(0);
}
