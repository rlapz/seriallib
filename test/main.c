#include "../seriallib.h"

int
main(void)
{
	int desc = serial_open("/dev/ttyUSB0", 9600, 1, 8, 1, 1, 1);

	close(desc);
	return 0;
}

