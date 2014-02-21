#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <tiffio.h>
#include <signal.h>

#include "cin.h"

void int_handler(int dummy){
  cin_data_stop_threads();
  fprintf(stderr, "\n");
  cin_data_show_stats();
  fprintf(stderr, "\n\n");
  exit(0);
}

int main(int argc, char *argv[]){

  /* Command line options */
  int c;

  /* For writing out */
  TIFF *tfp;
  int j;
  uint16_t *p;
  FILE *fp = NULL;
  char filename[256];
  struct cin_port port;

  uint16_t buffer[CIN_DATA_FRAME_WIDTH * CIN_DATA_FRAME_HEIGHT];
  uint16_t frame_number;

  char *output_path = NULL;
  int show_monitor = 0;
  int tiff_output = 1;
  int chunk_size = 1;

  while((c = getopt(argc, argv, "hrmd:c")) != -1){
    switch(c){
      case 'r':
        tiff_output =0;
        break;
      case 'm':
        show_monitor = 1;
        break;
      case 'd':
        output_path = optarg;
        break;
      case 'c':
        chunk_size = atoi(optarg);
        break;
      case 'h':
      default:
        fprintf(stderr,"cindump : Dump data from CIN\n\n");
        fprintf(stderr,"\t -r : Write files in raw (uint16_t format)\n");
        fprintf(stderr,"\t -m : Show status monitor\n");
        fprintf(stderr,"\t -d : Specify output directory\n");
        fprintf(stderr,"\t -c : Chunk size (for raw format)\n");
        fprintf(stderr,"\n");
        exit(1);
        break;
    }
  }

  fprintf(stderr, "\n\n\n\n");

  if(cin_init_data_port(&port, "10.23.5.1", 0, NULL, 0, 1000)){
    exit(1);
  }

  /* Start the main routine */
  if(cin_data_init(CIN_DATA_MODE_PUSH_PULL, 20000, 2000)){
    exit(1);
  }

  signal(SIGINT, int_handler);

  sleep(15);

  if(show_monitor){
    cin_data_start_monitor_output();
  } 

  while(1){
    // Load the buffer
    cin_data_load_frame(buffer, &frame_number);

    if(tiff_output){
      if(output_path){
        sprintf(filename, "%s/frame%08d.tif", output_path, frame_number);
      } else {
        sprintf(filename, "frame%08d.tif", frame_number);
      }

      tfp = TIFFOpen(filename, "w");

      TIFFSetField(tfp, TIFFTAG_IMAGEWIDTH, CIN_DATA_FRAME_WIDTH);
      TIFFSetField(tfp, TIFFTAG_BITSPERSAMPLE, 16);
      TIFFSetField(tfp, TIFFTAG_SAMPLESPERPIXEL, 1);
      TIFFSetField(tfp, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
      TIFFSetField(tfp, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
      TIFFSetField(tfp, TIFFTAG_ORIENTATION, ORIENTATION_BOTLEFT);

      p = buffer;
      for(j=0;j<CIN_DATA_FRAME_HEIGHT;j++){
        TIFFWriteScanline(tfp, p, j, 0);
        p += CIN_DATA_FRAME_WIDTH;
      }

      TIFFClose(tfp);

    } else {
      if(!fp || !(frame_number % chunk_size)){
        if(fp){
          fclose(fp);
        }
        if(output_path){
          sprintf(filename, "%s/frame%08d.bin", output_path, frame_number);
        } else {
          sprintf(filename, "frame%08d.bin", frame_number);
        }
        fp = fopen(filename, "w");
      }
      fwrite(buffer, sizeof(uint16_t),
             CIN_DATA_FRAME_HEIGHT * CIN_DATA_FRAME_WIDTH, fp);
    }

  }

  cin_data_wait_for_threads();

  cin_data_show_stats();

  fprintf(stderr, "\n\n");

  return(0);
}

