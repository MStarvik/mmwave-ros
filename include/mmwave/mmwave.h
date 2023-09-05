#pragma once

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mmwave_object {
  float x;
  float y;
  float z;
  float vel;
  float snr;
  float noise;
} mmwave_object;

FILE *mmwave_open(const char *dev);
void mmwave_close(FILE *fp);
uint8_t *mmwave_read(FILE *fp, int *len);
mmwave_object *mmwave_decode(uint8_t *data, int len, int *num_obj);

#ifdef __cplusplus
}
#endif
