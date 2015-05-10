#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"

int main(int argc, char *argv[]){

  struct cin_port cp;

  if(cin_ctl_init_port(&cp, 0, CIN_CTL_SVR_PORT, CIN_CTL_CLI_PORT)){
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
  fprintf(stdout, "%s\n\n", buf);

  float voltages[20];
  int n;
  for(n=0;n<20;n++){
    voltages[n] = 0;
  }

  int status = cin_ctl_set_bias_voltages(&cp, voltages);

  cin_ctl_close_port(&cp);

  return status;
}
