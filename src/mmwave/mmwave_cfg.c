#include "mmwave/mmwave_cfg.h"
#include "serial.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static FILE *mmwave_cfg_open(const char *dev) {
  int fd = serial_open(dev);
  if (fd < 0) {
    return NULL;
  }

  serial_config(fd, B115200, 0, false);

  FILE *fp = fdopen(fd, "r+");
  if (fp == NULL) {
    serial_close(fd);
    return NULL;
  }

  return fp;
}

static void mmwave_cfg_close(FILE *fp) {
  int fd = fileno(fp);
  fclose(fp);
  serial_close(fd);
}

static void mmwave_cfg_write(FILE *fp, const char *cmd) {
  fprintf(fp, "%s", cmd);
  fflush(fp);
}

static int mmwave_cfg_flush(FILE *fp) {
  int ret = -1;
  char *line = NULL;
  size_t len = 0;
  ssize_t nread;

  while ((nread = getline(&line, &len, fp)) != -1) {
    if (!strncmp("Done", line, 4)) {
      ret = 0;
      break;
    }
  }

  nread = getdelim(&line, &len, '>', fp);
  free(line);

  return ret;
}

int mmwave_cfg_start(const char *dev) {
  FILE *fp = mmwave_cfg_open(dev);
  if (fp == NULL) {
    return -1;
  }
  
  mmwave_cfg_write(fp, "sensorStart 0\n");
  int ret = mmwave_cfg_flush(fp);
  
  mmwave_cfg_close(fp);
  
  return ret;
}

int mmwave_cfg_stop(const char *dev) {
  FILE *fp = mmwave_cfg_open(dev);
  if (fp == NULL) {
    return -1;
  }
  
  mmwave_cfg_write(fp, "sensorStop\n");
  int ret = mmwave_cfg_flush(fp);
  
  mmwave_cfg_close(fp);
  
  return ret;
}

int mmwave_cfg_write_file(const char *dev, const char *cfg) {
  FILE *dev_fp = mmwave_cfg_open(dev);
  if (dev_fp == NULL) {
    return -1;
  }

  FILE *cfg_fp = fopen(cfg, "r");
  if (cfg_fp == NULL) {
    mmwave_cfg_close(dev_fp);
    return -1;
  }

  char *line = NULL;
  size_t len = 0;
  ssize_t nread;

  while ((nread = getline(&line, &len, cfg_fp)) != -1) {
    if (line[0] == '%') {
      continue;
    }
    mmwave_cfg_write(dev_fp, line);
    if (mmwave_cfg_flush(dev_fp)) {
      free(line);
      mmwave_cfg_close(dev_fp);
      fclose(cfg_fp);
      return -1;
    }
  }

  free(line);
  mmwave_cfg_close(dev_fp);
  fclose(cfg_fp);

  return 0;
}


