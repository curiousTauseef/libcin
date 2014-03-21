#ifndef _DESCRAMBLE_H
#define _DESCRAMBLE_H

typedef struct {

  uint16_t ChanMap[192]; // Actual chanelmap
  uint16_t TopBot[192];  // Definition of top or bottom
  uint16_t MapCol[48];   // Column Map
  uint16_t MapCric[48];  // fCRIC Map
  uint16_t MapAddr[48];  // Address Map

} descramble_map_t;

// Function prototypes

void cin_data_descramble_init(descramble_map_t *map);
int cin_data_descramble_frame(descramble_map_t *map, uint16_t *out, uint16_t *in);

#endif
