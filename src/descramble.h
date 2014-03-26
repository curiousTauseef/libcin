#ifndef _DESCRAMBLE_H
#define _DESCRAMBLE_H

/*
 *   Copyright (c) 2014, Stuart Wilkins
 *   All rights reserved.
 *   
 *   Redistribution and use in source and binary forms, with 
 *   or without modification, are permitted provided that the 
 *   following conditions are met:
 *   
 *     1. Redistributions of source code must retain the above 
 *     copyright notice, this list of conditions and the 
 *     following disclaimer.
 *   
 *     2. Redistributions in binary form must reproduce the 
 *     above copyright notice, this list of conditions and the 
 *     following disclaimer in the documentation and/or other 
 *     materials provided with the distribution.
 *   
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
 *   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


typedef struct {

  uint16_t ChanMap[192]; // Actual chanelmap
  uint16_t TopBot[192];  // Definition of top or bottom
  uint16_t MapCol[48];   // Column Map
  uint16_t MapCric[48];  // fCRIC Map
  uint16_t MapAddr[48];  // Address Map

  uint32_t *Map;
  int      size_x; // COLS
  int      size_y; // ROWS

} descramble_map_t;

// Function prototypes

int cin_data_descramble_init(descramble_map_t *map, 
                             int ccd_size_rows, int overscan);
int cin_data_descramble_frame(descramble_map_t *map, 
                              uint16_t *out, uint16_t *in);

#endif