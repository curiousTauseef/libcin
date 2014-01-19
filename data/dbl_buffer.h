#ifndef __DBL_BUFFER_H__
#define __DBL_BUFFER_H__

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CIN_DATA_DBL_BUFFER_SIZE 2

typedef struct image_dbl_buffer {
  uint16_t *image[CIN_DATA_DBL_BUFFER_SIZE];
  int used[CIN_DATA_DBL_BUFFER_SIZE];
  int active;
} dbl_buffer_t;

#ifdef __cplusplus
}
#endif

#endif