/* vim: set ts=2 sw=2 tw=0 noet :
   
   libcin : Driver for LBNL FastCCD 
   Copyright (c) 2014, Stuart B. Wilkins, Daron Chabot
   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met: 
   
   1. Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer. 
   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution. 
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   
   The views and conclusions contained in the software and documentation are those
   of the authors and should not be interpreted as representing official policies, 
   either expressed or implied, of the FreeBSD Project.

*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "cin.h"
#include "descramble.h"
#include "descramble_map.h"


int cin_data_descramble_init(descramble_map_t *map){

  // Do some bounds checking
  
  if((map->rows <= 0) || (map->rows > CIN_DATA_MAX_FRAME_Y)){
    ERROR_PRINT("Invalid number of rows %d\n", map->rows);
    return -1;
  }

  if(map->overscan < 0){
    ERROR_PRINT("Invalid overscan value %d\n", map->overscan);
    return -1;
  }

  DEBUG_PRINT("Creating descramble map for %d rows with overscan of %d\n", map->rows, map->overscan);

  // map->ChanMap is 1 to 96 on top, -1 to -96 on bottom
  // on the top half, fCRIC=6, GRP=0 is the lowest order output
  // on the bottom half, swap

  int i;
  uint16_t ChanMap[CIN_DATA_CCD_COLS * 2]; // Actual chanelmap
  uint16_t TopBot[CIN_DATA_CCD_COLS * 2];  // Definition of top or bottom
  for(i=0; i<48; i++){
      ChanMap[MapCol[i]]=16*(6-MapCric[i])+MapAddr[i];
      ChanMap[MapCol[i]+4]=16*(9-MapCric[i])+MapAddr[i];
      ChanMap[MapCol[i]+8]=ChanMap[MapCol[i]];
      ChanMap[MapCol[i]+12]=ChanMap[MapCol[i]+4];
      TopBot[MapCol[i]]=0; TopBot[MapCol[i]+4]=0;
      TopBot[MapCol[i]+8]=1; TopBot[MapCol[i]+12]=1;
  }

  int CCDreg = CIN_DATA_CCD_COLS_PER_CHAN + map->overscan;
  int CCDsizeX = CIN_DATA_CCD_COLS * CCDreg;
  int CCDsizeY = map->rows / 2;

  for(i=0; i<192; i++){
    ChanMap[i] = ChanMap[i] * CCDreg;
  }

  uint32_t* Map_p = map->map;
  memset(Map_p, 0, CIN_DATA_MAX_STREAM);

  int y, x;
  for(y = 0; y < CCDsizeY; y++){
    for(x = 0; x < CCDreg; x++){
      for(i = 0; i < 192; i++){
        uint32_t pIndex;
        uint32_t xdex = ChanMap[i] + (CCDreg - x - 1);
        if(TopBot[i] == 0 ){
          pIndex = xdex + (y * CCDsizeX);
        } else {
          pIndex = (CCDsizeX - xdex - 1) + ((CCDsizeY * 2 - y - 1) * CCDsizeX);
        }
        // Check if pIndex is in bounds
        if(pIndex >= CIN_DATA_MAX_STREAM){
          pIndex = 0;
          ERROR_PRINT("Map contains invalid value %ld\n", (long int)pIndex);
        }
        *(Map_p++) = pIndex;
      } // i
    } // x
  } // y

  map->size_x = CCDsizeX;
  map->size_y = CCDsizeY * 2;

  DEBUG_PRINT("Map initialized for image of %d x %d.\n", CCDsizeX, CCDsizeY);

  return 0;
}

int cin_data_descramble_frame(descramble_map_t *map, uint16_t *out, uint16_t *in){

  // Actually descramble the frame

  uint16_t *in_p = in;
  uint32_t *map_p = map->map;

  // Flush the pipeline

  in_p += CIN_DATA_PIPELINE_FLUSH;

  // Descramble (copy) into output block

  int i;
  for(i = 0;i < (map->size_x * map->size_y); i++){
    out[*(map_p++)] = *(in_p++);
  }
  
  return 0;
}
