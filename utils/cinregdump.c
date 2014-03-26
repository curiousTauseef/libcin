#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "cin.h"
#include "cinregisters.h"

int main(int argc, char *argv[]){

  struct cin_port cp;

  if(cin_ctl_init_port(&cp, 0, CIN_CTL_SVR_PORT, CIN_CTL_CLI_PORT)){
    perror("Unable to initialize port");
    return -1;
  }

  cin_map_t *rmap = cin_reg_map;
  while(rmap->name != NULL){
    uint16_t reg = rmap->reg;
    uint16_t val;
    if(!cin_ctl_read(&cp, reg, &val)){
      fprintf(stderr, "%-40s :   0x%04X   =   0x%04X\n", rmap->name, reg, val);
    }
    rmap++;
  }

  return 0;
}
