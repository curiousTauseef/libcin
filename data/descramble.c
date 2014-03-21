#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "cin.h"
#include "descramble.h"

void cin_data_descramble_init(descramble_map_t *map){

    // Pixel Mapping Table
    map->MapCol[0]=129;      map->MapCric[0]=4;       map->MapAddr[0]=0;
    map->MapCol[1]=145;      map->MapCric[1]=4;       map->MapAddr[1]=1;
    map->MapCol[2]=161;      map->MapCric[2]=4;       map->MapAddr[2]=2;
    map->MapCol[3]=177;      map->MapCric[3]=4;       map->MapAddr[3]=3;
    map->MapCol[4]=0;        map->MapCric[4]=4;       map->MapAddr[4]=4;
    map->MapCol[5]=16;       map->MapCric[5]=4;       map->MapAddr[5]=5;
    map->MapCol[6]=32;       map->MapCric[6]=4;       map->MapAddr[6]=6;
    map->MapCol[7]=48;       map->MapCric[7]=4;       map->MapAddr[7]=7;
    map->MapCol[8]=64;       map->MapCric[8]=4;       map->MapAddr[8]=8;
    map->MapCol[9]=80;       map->MapCric[9]=4;       map->MapAddr[9]=9;
    map->MapCol[10]=96;      map->MapCric[10]=4;      map->MapAddr[10]=10;
    map->MapCol[11]=112;     map->MapCric[11]=4;      map->MapAddr[11]=11;
    map->MapCol[12]=128;     map->MapCric[12]=4;      map->MapAddr[12]=12;
    map->MapCol[13]=144;     map->MapCric[13]=4;      map->MapAddr[13]=13;
    map->MapCol[14]=160;     map->MapCric[14]=4;      map->MapAddr[14]=14;
    map->MapCol[15]=176;     map->MapCric[15]=4;      map->MapAddr[15]=15;

    map->MapCol[16]=66;      map->MapCric[16]=5;      map->MapAddr[16]=0;
    map->MapCol[17]=82;      map->MapCric[17]=5;      map->MapAddr[17]=1;
    map->MapCol[18]=98;      map->MapCric[18]=5;      map->MapAddr[18]=2;
    map->MapCol[19]=114;     map->MapCric[19]=5;      map->MapAddr[19]=3;
    map->MapCol[20]=130;     map->MapCric[20]=5;      map->MapAddr[20]=4;
    map->MapCol[21]=146; 	   map->MapCric[21]=5;      map->MapAddr[21]=5;
    map->MapCol[22]=162;     map->MapCric[22]=5;      map->MapAddr[22]=6;
    map->MapCol[23]=178;     map->MapCric[23]=5;      map->MapAddr[23]=7;
    map->MapCol[24]=1;       map->MapCric[24]=5;      map->MapAddr[24]=8;
    map->MapCol[25]=17;      map->MapCric[25]=5;      map->MapAddr[25]=9;
    map->MapCol[26]=33;      map->MapCric[26]=5;      map->MapAddr[26]=10;
    map->MapCol[27]=49;      map->MapCric[27]=5;      map->MapAddr[27]=11;
    map->MapCol[28]=65;      map->MapCric[28]=5;      map->MapAddr[28]=12;
    map->MapCol[29]=81;      map->MapCric[29]=5;      map->MapAddr[29]=13;
    map->MapCol[30]=97;      map->MapCric[30]=5;      map->MapAddr[30]=14;
    map->MapCol[31]=113;     map->MapCric[31]=5;      map->MapAddr[31]=15;

    map->MapCol[32]=3;       map->MapCric[32]=6;      map->MapAddr[32]=0;
    map->MapCol[33]=19;      map->MapCric[33]=6;      map->MapAddr[33]=1;
    map->MapCol[34]=35;      map->MapCric[34]=6;      map->MapAddr[34]=2;
    map->MapCol[35]=51;      map->MapCric[35]=6;      map->MapAddr[35]=3;
    map->MapCol[36]=67;      map->MapCric[36]=6;      map->MapAddr[36]=4;
    map->MapCol[37]=83;      map->MapCric[37]=6;      map->MapAddr[37]=5;
    map->MapCol[38]=99;      map->MapCric[38]=6;      map->MapAddr[38]=6;
    map->MapCol[39]=115;     map->MapCric[39]=6;      map->MapAddr[39]=7;
    map->MapCol[40]=131;     map->MapCric[40]=6;      map->MapAddr[40]=8;
    map->MapCol[41]=147;     map->MapCric[41]=6;      map->MapAddr[41]=9;
    map->MapCol[42]=163;     map->MapCric[42]=6;      map->MapAddr[42]=10;
    map->MapCol[43]=179;     map->MapCric[43]=6;      map->MapAddr[43]=11;
    map->MapCol[44]=2;       map->MapCric[44]=6;      map->MapAddr[44]=12;
    map->MapCol[45]=18;      map->MapCric[45]=6;      map->MapAddr[45]=13;
    map->MapCol[46]=34;      map->MapCric[46]=6;      map->MapAddr[46]=14;
    map->MapCol[47]=50;      map->MapCric[47]=6;      map->MapAddr[47]=15;

    // ----------
    // map->ChanMap is 1 to 96 on top, -1 to -96 on bottom
    // on the top half, fCRIC=6, GRP=0 is the lowest order output
    // on the bottom half, swap

    int i;
    for(i=0; i<48; i++){
        map->ChanMap[map->MapCol[i]]=16*(6-map->MapCric[i])+map->MapAddr[i];
        map->ChanMap[map->MapCol[i]+4]=16*(9-map->MapCric[i])+map->MapAddr[i];
        map->ChanMap[map->MapCol[i]+8]=map->ChanMap[map->MapCol[i]];
        map->ChanMap[map->MapCol[i]+12]=map->ChanMap[map->MapCol[i]+4];
        map->TopBot[map->MapCol[i]]=0; map->TopBot[map->MapCol[i]+4]=0;
        map->TopBot[map->MapCol[i]+8]=1; map->TopBot[map->MapCol[i]+12]=1;
    }

    for(i=0; i<192; i++){
      map->ChanMap[i] = map->ChanMap[i] * 10; // CCDreg
    }

    return;
}

int cin_data_descramble_frame(descramble_map_t *map, uint16_t *out, uint16_t *in){

  int CCDcols = 96;
  int OverScan = 0;
  int CCDreg = OverScan + 10;
  int CCDroc = CCDreg-1;
  int CCDsizeX = CCDcols * CCDreg;
  int CCDsizeY = 480 * 2;
  int YTOT = 2 * CCDsizeY;

  // the first 7 fcric converts (ktr=7*192*2) are from the stale pipeline data.
  //Simply skip over them.  (ktr is bytes)

  int ktr = 1344; //7 * 192 * 2; //num_flush_pix;

  // since the pipeline is not flushed, we read one more 
  // row than is displayed on each side.

  int y, x, i;

  for(y = 0; y < CCDsizeY; y++){
    for(x = 0; x < CCDreg; x++){
      for(i = 0; i < 192; i++){
        int xdex = map->ChanMap[i] + (CCDroc - x);
        int pIndex;
        if(map->TopBot[i] == 0 ){
          pIndex =  (xdex+y*CCDsizeX);
        } else {
          pIndex = ((CCDsizeX-xdex-1)+((YTOT-y-1)*CCDsizeX));
        }
        //DEBUG_PRINT("y = %d , x = %d, i = %d, pIndex = %d, ktr = %d\n", 
        //            y,x,i, pIndex, ktr);
        out[pIndex] = in[ktr];
        ktr ++;
      } // i
    } // x
  } // y

  return 0;
}
