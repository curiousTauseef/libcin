#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[]){

  cin_ctl_t cin;

  if(cin_ctl_init(&cin, NULL, 0, 0, 0, 0)){
    perror("Unable to initialize CIN port");
    return -1;
  }

  time_t rawtime;
  struct tm *info;
  char buf[80];
  time(&rawtime);
  info = localtime(&rawtime);
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", info);

  fprintf(stdout, "\n");
  fprintf(stdout, "cinregdump : Started at %s\n\n", buf);

  int status = cin_ctl_reg_dump(&cin, stdout);

  cin_ctl_close_ports(&cin);

  return status;
}
