#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <netinet/in.h>

#define APP_NAME	"serialtest"

#define VERSION_MAJOR	0
#define VERSION_MINOR	1

void print_usage(void)
{
	printf("%s %d.%d\n", APP_NAME, VERSION_MAJOR, VERSION_MINOR);
}

const char    hexlookup[] = {"0123456789ABCDEF"};

void hex_to_ascii(const unsigned char *source, char *dest, unsigned int source_length)
{
    unsigned int i;
    unsigned char temp;

  for (i = 0; i < source_length; i++) {
        temp = source[i];
        temp >>= 4;
        dest[i*2] = hexlookup[temp];

        temp = source[i];
        temp &= 0x0f;
        dest[(i*2)+1] = hexlookup[temp];
    }
}

int ascii_to_hex(const char *source, int source_len, char *dest)
{
	int i, pos=0;

	if((source_len > 1) && (source_len%2))
		return -1;		/* Error */

	for(i=0; i<source_len; i++){
		dest[pos] = 0;

		if(source_len > 1){
			if(((source[i] <= 'F') && (source[i] >= 'A')) || ((source[i] <= 'f') && (source[i] >= 'a'))){
				dest[pos] = (toupper(source[i]) - 'A' + 0x0a) << 4;
			}else if((source[i] <= '9') && (source[i] >= '0')){
				dest[pos] = (source[i] - '0') << 4;
			}else
				return -1;	/* Error */

			i++;
		}

		if(((source[i] <= 'F') && (source[i] >= 'A')) || ((source[i] <= 'f') && (source[i] >= 'a'))){
			dest[pos] |= (toupper(source[i]) - 'A' + 0x0a) & 0x0f;
		}else if((source[i] <= '9') && (source[i] >= '0')){
			dest[pos] |= (source[i] - '0') & 0x0f;
		}else
			return -1;	/* Error */

		pos++;
	}

	return pos;
}

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	printf ("error %d from tcgetattr\n", errno);
        	return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
        	printf ("error %d from tcsetattr\n", errno);
        	return -1;
        }

        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	printf ("error %d from tggetattr\n", errno);
        	return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        	printf ("error %d setting term attributes\n", errno);
}

int main(int argc, const char* argv[])
{
	if(argc == 1){
		print_usage();
		return 0;
	}

	int fd = open (argv[1], O_RDWR | O_NOCTTY | O_SYNC);

	if (fd < 0)
	{
		printf ("error %d opening %s: %s\n", errno, argv[1], strerror (errno));
		return -1;
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 1);                // set no blocking

	char buf [100];
	int n;

	while(1){
		char tchar;
		int count=0;

		while(1){
			n = read (fd, &tchar, 1);

			if(n && (tchar == 0x02))
				break;
		}

		count=0;
		do{
			n = read (fd, &tchar, 1);

			if(n){
				if(tchar != 0x03)
					buf[count++] = tchar;
				else
					break;
			}
		}while(count < sizeof(buf));

		for(int floatcount=0; floatcount<10; floatcount++){
			char binbuff[sizeof(float)];
			ascii_to_hex((const char*)&buf[floatcount*sizeof(float)*2], sizeof(float)*2, binbuff);

			float fval=0;
			char *cval = (char*)&fval;
			for(int bytecount=0; bytecount<sizeof(float); bytecount++){
				cval[bytecount] = binbuff[bytecount];
			}

			printf("[%12.2f] ", fval);
		}
/*
		for(int intcount=0; intcount<; intcount++){
			char binbuff[sizeof(uint16_t)];
			ascii_to_hex((const char*)&buf[intcount*sizeof(uint16_t)*2], sizeof(uint16_t)*2, binbuff);

			uint16_t ival=0;
			char *cval = (char*)&ival;
			for(int bytecount=0; bytecount<sizeof(uint16_t); bytecount++){
				cval[bytecount] = binbuff[bytecount];
			}
			if(ival >= 0)
				printf("[ %6d] ", ival);
			else
				printf("[%6d] ", ival);
		}
*/

		printf("\n");

	}

	close(fd);

	return 0;
}
