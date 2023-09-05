#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int serial_open(const char *path);
int serial_close(int fd);
int serial_config(int fd, int speed, int parity, bool blocking);

#ifdef __cplusplus
}
#endif
