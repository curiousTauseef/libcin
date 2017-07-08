#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "cin.h"

uint16_t fcric_regs[] = {0x821D, 0x821E, 0x821F, 0x8001};

char *strstrip(char *s)
{
	size_t size;
	char *end;

	size = strlen(s);

	if (!size)
		return s;

	end = s + size - 1;
	while (end >= s && isspace(*end))
		end--;
	*(end + 1) = '\0';

	while (*s && isspace(*s))
		s++;

	return s;
}

void print_usage(void)
{
  fprintf(stderr, "%s:\n\n", "convert_config");
}

int read_file(const char *filename, uint16_t vals[][2], int *n_vals)
{
  FILE *fp;
  char _line[256];
  char _regstr[128], _valstr[128];

  if((fp = fopen(filename, "r")) == NULL)
  {
    return 1;
  }

  int n=0;
  while(fgets(_line,sizeof _line,fp) != NULL){ 
    _line[strlen(_line)-1]='\0';   
    char *__line = strstrip(_line);
    if ('#' == __line[0] || '\0' == __line[0]){
      fprintf(stdout," --- Ignore line : %s\n", __line); //DEBUG 
    } else { 
      sscanf (__line,"%4s %4s",_regstr,_valstr);
      vals[n][0]=strtoul(_regstr,NULL,16);
      vals[n][1]=strtoul(_valstr,NULL,16);          
      n++;
    }
  }

  *n_vals = n;
  return 0;
}
 
int parse_fcric_data(uint16_t vals[][2], int n_vals, uint16_t *pvals, int *n_pvals)
{
  // Check length
  if(n_vals != 164)
  {
    fprintf(stderr, "--- FCRIC File is of correct length\n");
    return -1;
  }
  

  fprintf(stderr, "--- Parsing data (%d values)\n", n_vals);
  // Check for reset
  if((vals[0][0] != 0x8105) || (vals[0][1] != 0x0001)) return -1;
  if((vals[1][0] != 0x8105) || (vals[1][1] != 0x0000)) return -1;
  if((vals[2][0] != 0x8211) || (vals[2][1] != 0xFFFF)) return -1;
  if((vals[3][0] != 0x8212) || (vals[3][1] != 0xFFFF)) return -1;
  if((vals[4][0] != 0x8213) || (vals[4][1] != 0xFFFF)) return -1;

  fprintf(stderr, "--- Header OK\n");

  int i;
  int x = 0;
  for(i=0;i<156;i++){
    int j = i + 5;
    if(vals[j][0] != fcric_regs[i % 4]){
      fprintf(stderr, "!!! Error at line %d (%04X,%04X)\n", j,
          vals[j][0], fcric_regs[i % 4]);
      return -1;
    } else {
      if(((i % 4) == 0) && (vals[j][1] != 0xA000)){
        fprintf(stderr, "!!! Error at line %d\n", j);
      } 
      if(((i % 4) == 3) && (vals[j][1] != 0x0105)){
        fprintf(stderr, "!!! Error at line %d\n", j);
      }
      if(((i % 4) == 1) || ((i % 4) == 2))
      {
        pvals[x] = vals[j][1];
        x++;
      }
    }
  }

  if((vals[161][0] != 0x8211) || (vals[161][1] != 0x0000)) return -1;
  if((vals[162][0] != 0x8212) || (vals[162][1] != 0x0000)) return -1;
  if((vals[163][0] != 0x8213) || (vals[163][1] != 0x0000)) return -1;

  *n_pvals = x;
  fprintf(stderr, "--- FCRIC File Parsed OK - %d values\n", x);
  return 0;

}

void dump_data(FILE *fp, const char *name, uint16_t *val, int n_val)
{
  int i;

  fprintf(fp, "const uint16_t %s[] = {\n", name);
  for(i=0;i<n_val-1;i++)
  {
    if((i % 8) == 0)
    {
      fprintf(fp, "  ");
    }
    fprintf(fp, "0x%04X, ", val[i]);
    if((i % 8) == 7)
    {
      fprintf(fp, "\n");
    }
  }

  fprintf(fp, "0x%04X\n}\n", val[i]);
  fprintf(fp, "const int %s_len = %d\n", name, i+1);
}

int main(int argc, char *argv[])
{
  int c;
  int errflg = 0;
  int bflag = 0;
  int fflag = 0;
  int tflag = 0;

  opterr = 0;
  while ((c = getopt(argc, argv, "bft")) != -1)
  {
    switch(c){
    case 'b':
      if(fflag || tflag){
        errflg++;
      } else {
        bflag++;
      }
      break;
    case 'f':
      if(bflag || tflag){
        errflg++;
      } else {
        fflag++;
      }
      break;
    case 't':
      if(bflag || fflag){
        errflg++;
      } else {
        tflag++;
      }
      break;
    case '?':
      fprintf(stderr, "Unrecognized option: -%c\n\n", optopt);
      errflg++;
    }
  }

  if(!(bflag || fflag || tflag)){
    errflg++;
  }

  if((argc - optind) != 2){
    errflg++;
  }

  if(errflg){
    print_usage();
    return 127;
  }

  // Now process file
  
  if(fflag){
    uint16_t vals[1000][2];
    uint16_t pvals[1000];
    int n_vals;
    int n_pvals;
    read_file(argv[optind], vals, &n_vals);
    parse_fcric_data(vals, n_vals, pvals, &n_pvals);
    dump_data(stdout, "fcric_200", pvals, n_pvals);
  }

  return 0;
}

