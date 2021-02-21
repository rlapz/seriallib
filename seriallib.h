#ifndef SERIALLIB_H
#define SERIALLIB_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


int serial_open(const char* port, unsigned int baud,
		int parity, int stopbits, int bytesize, int vmin, int vtime);
int serial_write(int* portdesc, unsigned char* data, size_t len);
int serial_read(int* portdesc, unsigned char* buffer, size_t len);
void serial_close(int* portdesc);

#endif
