#include "serial.h"

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int serial_open(const char *path) {
  return open(path, O_RDWR | O_NOCTTY | O_SYNC);
}

int serial_close(int fd) {
  return close(fd);
}

int serial_config(int fd, int speed, int parity, bool blocking) {
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    return -1;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK; // disable break processing
  tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
  tty.c_oflag = 0; // no remapping, no delays
  tty.c_cc[VMIN] = 0; // read doesn't block
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = blocking;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    return -1;
  }
  return 0;
}