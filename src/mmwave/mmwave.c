#include "mmwave/mmwave.h"
#include "serial.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define MAGIC_WORD 0x708050603040102

enum mmwave_tlv_type {
  TLV_NULL = 0,
  TLV_DETECTED_POINTS,
  TLV_RANGE_PROFILE,
  TLV_NOISE_PROFILE,
  TLV_AZIMUTH_STATIC_HEAT_MAP,
  TLV_RANGE_DOPPLER_HEAT_MAP,
  TLV_STATS,
  TLV_DETECTED_POINTS_SIDE_INFO,
  TLV_MAX
};

struct mmwave_header {
  uint32_t version;
  uint32_t total_packet_len;
  uint32_t platform;
  uint32_t frame_num;
  uint32_t time_cputime;
  uint32_t num_detected_obj;
  uint32_t num_tlvs;
  uint32_t subframe_num;
};

struct mmwave_detected_obj {
  float x;
  float y;
  float z;
  float vel;
};

struct mmwave_side_info {
  int16_t snr;
  int16_t noise;
};

struct mmwave_tlv_header {
  uint32_t type;
  uint32_t length;
};

FILE *mmwave_open(const char *dev) {
  int fd = serial_open(dev);
  if (fd < 0) {
    return NULL;
  }

  serial_config(fd, B921600, 0, true);

  FILE *fp = fdopen(fd, "r");
  if (fp == NULL) {
    close(fd);
    return NULL;
  }

  return fp;
}

void mmwave_close(FILE *fp) {
  int fd = fileno(fp);
  fclose(fp);
  close(fd);
}

uint8_t *mmwave_read(FILE *fp, int *len) {
  static uint8_t buf[1024];

  uint64_t word = 0;
  for (size_t i = 0; i < sizeof(buf); i++) {
    int c = fgetc(fp);
    if (c == EOF) {
      return NULL;
    }

    buf[i] = c;

    word = (word >> 8) | ((uint64_t)c << 56);
    if (word == MAGIC_WORD) {
      *len = i + 1;
      return buf;
    }
  }

  return NULL;
}

mmwave_object *mmwave_decode(uint8_t *data, int len, int *num_obj) {
  static mmwave_object buf[32];

  struct mmwave_header *header = (struct mmwave_header *)data;
  data += sizeof(struct mmwave_header);

  if (header->total_packet_len != (uint32_t)len) {
    *num_obj = 0;
    return buf;
  }

  *num_obj = header->num_detected_obj;

  for (size_t i = 0; i < header->num_tlvs; i++) {
    struct mmwave_tlv_header *tlv_header = (struct mmwave_tlv_header *)data;
    data += sizeof(struct mmwave_tlv_header);

    if (tlv_header->type == TLV_DETECTED_POINTS) {
      struct mmwave_detected_obj *obj = (struct mmwave_detected_obj *)(data);
      for (size_t j = 0; j < header->num_detected_obj; j++) {
        buf[j].x = obj->x;
        buf[j].y = obj->y;
        buf[j].z = obj->z;
        obj++;
      }
    } else if (tlv_header->type == TLV_DETECTED_POINTS_SIDE_INFO) {
      struct mmwave_side_info *info = (struct mmwave_side_info *)(data);
      for (size_t j = 0; j < header->num_detected_obj; j++) {
        buf[j].snr = (float)info->snr / 10.0;
        buf[j].noise = (float)info->noise / 10.0;
        info++;
      }
    }

    data += tlv_header->length;
  }

  return buf;
}
