#ifndef __ARDUINO_SERIAL_H_____
#define __ARDUINO_SERIAL_H_____

#include <stdint.h>

int serialport_init(const char* serialport, int baud);
int serialport_writebyte(int fd, uint8_t b);
int serialport_write(int fd, const char* str);
int serialport_read_until(int fd, char* buf, char until);

#endif
