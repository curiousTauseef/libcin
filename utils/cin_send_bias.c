#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[])
{

  cin_ctl_t cin;
  cin_set_debug_print(1);
  cin_set_error_print(1);

  if(cin_ctl_init(&cin, NULL, 0, 0, NULL, 0, 0)){
    perror("Unable to initialize CIN port");
    return -1;
  }

  if(cin_ctl_upload_bias(&cin) != CIN_OK)
  {
    ERROR_COMMENT("Unable to upload BIAS\n");
    return 127;
  }

  cin_ctl_destroy(&cin);

  fprintf(stderr, "\n");
  return 0;
}
