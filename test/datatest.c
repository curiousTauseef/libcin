#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "cin.h"

int main(int argc, char *argv[]){
  cin_set_debug_print(1);
  cin_set_error_print(1);

  cin_data_t cin_data;

  cin_data_init(&cin_data, NULL, NULL, 0, 0, 0, 
                           1000, 1000, NULL, NULL, NULL);

  while(1)
  {
    sleep(1);
  }

  cin_data_destroy(&cin_data);
  exit(0);
}
