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

void write_image(cin_data_frame_t *frame)
{
  char filename[256];
  TIFF *tfp;
  //if(output_path){
  //  sprintf(filename, "%s/frame%08d.tif", output_path, frame_number);
  //} else {
  sprintf(filename, "frame%08d.tif", frame->number);
  //}

  tfp = TIFFOpen(filename, "w");

  TIFFSetField(tfp, TIFFTAG_IMAGEWIDTH, CIN_DATA_FRAME_WIDTH);
  TIFFSetField(tfp, TIFFTAG_BITSPERSAMPLE, 16);
  TIFFSetField(tfp, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tfp, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tfp, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tfp, TIFFTAG_ORIENTATION, ORIENTATION_BOTLEFT);

  uint16_t *p = frame->data;
  int j;
  for(j=0;j<CIN_DATA_FRAME_HEIGHT;j++){
    TIFFWriteScanline(tfp, p, j, 0);
    p += CIN_DATA_FRAME_WIDTH;
  }

  TIFFClose(tfp);
  
  free(frame->data); // Very important!

  return;
}

void allocate_image(cin_data_frame_t *frame)
{
  // Allocate memory
  while(!(frame->data = malloc(CIN_DATA_FRAME_WIDTH * CIN_DATA_FRAME_HEIGHT * sizeof(uint16_t))))
  {
    sleep(1);
    fprintf(stderr, "Failed to allocate memory ... sleeping to wait for data.\n");
  }
  return;
}

int main(int argc, char *argv[]){

  char *output_path = NULL;
  int show_monitor = 0;

  int c;
  while((c = getopt(argc, argv, "hmd:")) != -1){
    switch(c){
      case 'm':
        show_monitor = 1;
        break;
      case 'd':
        output_path = optarg;
        break;
      case 'h':
      default:
        fprintf(stderr,"cindump : Dump data from CIN\n\n");
        fprintf(stderr,"\t -m : Show status monitor\n");
        fprintf(stderr,"\t -d : Specify output directory\n");
        fprintf(stderr,"\n");
        exit(1);
        break;
    }
  }

  fprintf(stderr, "\n\n\n\n");

  struct cin_port port;
  if(cin_data_init_port(&port, NULL, 0, NULL, 0, 1000))
  {
    exit(1);
  }

  /* Start the main routine */
  if(cin_data_init(CIN_DATA_MODE_CALLBACK, 20000, 2000,
                   allocate_image, write_image))
  {
    exit(1);
  }

  signal(SIGINT, int_handler); 

  sleep(5);

  if(show_monitor){
    cin_data_start_monitor_output();
  } 

  while(1){
    sleep(10);
  }

  cin_data_wait_for_threads();

  cin_data_show_stats();

  fprintf(stderr, "\n\n");

  return(0);
}

