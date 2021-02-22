#include "seriallib.h"

struct termios tty;

int serial_open(const char* portdesc, unsigned int baud,
		int parity, int stopbits, int bytesize, int vmin, int vtime)
{
	int serialport = open(portdesc, O_RDWR);
    if (serialport < 0) {
        printf("Error %i : %s from open: %s\n", errno, portdesc, strerror(errno));
		return -1;
    }
    if (tcgetattr(serialport, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return -1;
    }

	// set baudrate
	switch (baud) {
		case 9600:
			cfsetispeed(&tty, B9600);
			cfsetospeed(&tty, B9600);
			break;
		case 19200:
			cfsetispeed(&tty, B19200);
			cfsetospeed(&tty, B19200);
			break;
		case 38400:
			cfsetispeed(&tty, B38400);
			cfsetospeed(&tty, B38400);
			break;
		case 115200:
			cfsetispeed(&tty, B115200);
			cfsetospeed(&tty, B115200);
			break;
		default:
			cfsetispeed(&tty, B9600);
			cfsetospeed(&tty, B9600);
	}

    tty.c_cflag &= ~PARENB; // clear parity bit, disabling parity (most common)
	if (parity > 0)
		tty.c_cflag |= PARENB;  // set parity bit, enabling parity

	switch (stopbits) {
		case 1:
			tty.c_cflag &= ~CSTOPB; // clear stop field, only one stop bit used in communication (most common)
			break;
		case 2:
			tty.c_cflag |= CSTOPB; // set stop field, two stop bits used in communication
			break;
		case 8:
			tty.c_cflag |= CS8; // set stop field, eight stop bits used in communication
			break;
		default:
			tty.c_cflag |= CS8; // set stop field, eight stop bits used in communication
	}

	tty.c_cflag &= ~CSIZE; // clear all the size bits
	switch (bytesize) {
		case 5:
    		tty.c_cflag |= CS5; // 5 bits per byte
			break;
		case 6:
    		tty.c_cflag |= CS6; // 6 bits per byte
			break;
		case 7:
    		tty.c_cflag |= CS7; // 7 bits per byte
			break;
		case 8:
    		tty.c_cflag |= CS8; // 8 bits per byte (most common)
			break;
		default:
    		tty.c_cflag |= CS8; // 8 bits per byte (most common)
	}

    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    //tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control

    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable Cannonical mode
    tty.c_lflag &= ~ECHO; // Disable Echo
    tty.c_lflag &= ~ECHOE; // Disable Erasure
    tty.c_lflag &= ~ECHONL; // Disable New-line Echo

    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow control
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (newline chars)
	tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return

	tty.c_cc[VTIME] = vtime; // wait for up to 1s, returning as soon as any data is received.
	tty.c_cc[VMIN] = vmin;

	if (tcsetattr(serialport, TCSANOW, &tty) != 0){
		printf("Error %i from tsetattr: %s\n", errno, strerror(errno));
		return -1;
	}

	return serialport;
}

int serial_write(int* portdesc, unsigned char* data, size_t len)
{
	return write(*portdesc, data, len);
}

int serial_read(int* portdesc, unsigned char* data, size_t len)
{
	unsigned char buffer[len];
	memset(&buffer, '\0', len);
	int bytesize = read(*portdesc, buffer, len);
	memcpy(data, buffer, len);
	return bytesize;
}

void serial_flush(int* portdesc)
{
	sleep(2);
	tcflush(*portdesc, TCIOFLUSH);
}

void serial_close(int* portdesc)
{
	close(*portdesc);
}
