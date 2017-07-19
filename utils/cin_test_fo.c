#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[])
{

  int onoff = 0;

  if(argc != 2)
  {
    ERROR_COMMENT("Incorrect usage\n");
    return 127;
  }

  if(strncmp(argv[1], "on", strlen(argv[1])) == 0)
  {
    onoff = 1;
  } else if(strncmp(argv[1], "off", strlen(argv[1])) == 0) {
    onoff = 0;
  } else {
    ERROR_COMMENT("Command should be [on/off]\n");
    return 127;
  }

  cin_ctl_t cin;

  if(cin_ctl_init(&cin, NULL, 0, 0, NULL, 0, 0)){
    perror("Unable to initialize CIN port");
    return -1;
  }

  if(cin_ctl_fo_test_pattern(&cin, onoff) != CIN_OK)
  {
    ERROR_COMMENT("Unable to set FO Test Pattern\n");
    return 127;
  }

  cin_ctl_destroy(&cin);

  fprintf(stderr, "\n");
  return 0;
}
