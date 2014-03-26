#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>

#include "cin.h"
#include "cinregisters.h"

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

  fprintf(stdout, "cinregdump : Started at %s\n\n", buf);
  fprintf(stdout, "Register Name                            : Register : Value \n");
  fprintf(stdout, "-------------------------------------------------------------\n");
  cin_map_t *rmap = cin_reg_map;
  while(rmap->name != NULL){
    uint16_t reg = rmap->reg;
    uint16_t val;
    if(!cin_ctl_read(&cp, reg, &val)){
      fprintf(stdout, "%-40s :  0x%04X  :  0x%04X\n", rmap->name, reg, val);
    } else {
      fprintf(stdout, "%-40s :  0x%04X  : ERROR\n", rmap->name, reg);
    }
    rmap++;
  }

  return 0;
}
