/* MIT License
 *
 * Copyright (c) 2021 Arthur Lapz (rLapz)
 *
 * See LICENSE file for license details
 */
#ifndef SERIALLIB_H
#define SERIALLIB_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


int serial_open(const char *portdesc,
		unsigned int baud, int parity, int stopbits, int bytesize,
		unsigned char vmin, unsigned char vtime);
ssize_t serial_write(int *portdesc, unsigned char *data, size_t len);
ssize_t serial_read(int *portdesc, unsigned char *buffer, size_t len);
void serial_flush(int *portdesc);
void serial_close(int *portdesc);

#endif
