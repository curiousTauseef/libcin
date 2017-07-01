#include <stdlib.h>
#include <stdio.h>

#include "cin.h"

int main(int argc, char *argv[]){
  cin_set_debug_print(1);
  cin_set_error_print(1);
  DEBUG_PRINT("Git SHA           = %s\n", cin_build_git_sha);
  DEBUG_PRINT("Git Time          = %s\n", cin_build_git_time);
  DEBUG_PRINT("Git Build Version = %s\n", cin_build_version);
  exit(0);
}
