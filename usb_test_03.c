	#include <stdio.h>   /* Standard input/output definitions */
	#include <string.h>  /* String function definitions */
	#include <unistd.h>  /* UNIX standard function definitions */
	#include <fcntl.h>   /* File control definitions */
	#include <errno.h>   /* Error number definitions */
	#include <termios.h> /* POSIX terminal control definitions */
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <sys/select.h>

  //===============================================================
  //   CONSTANT
  //===============================================================

	static const speed_t baudrate = B9600;

  //===============================================================
  //   GLOBAL VARIABLES
  //===============================================================

	static int fd; /* File descriptor for the port */

  //===============================================================
  //   METHODS
  //===============================================================

	int serialport_init(const char *serialPort)
	{
		/*
		* Try to open the port...
		*/
		fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);

		if (fd == -1)
		{
			perror("couldn't open port");
			return -1;
		}
		else
		{
			fcntl(fd, F_SETFL, 0);
		}

		/*
		* Get the current options for the port...
		*/

		struct termios options;

		if(tcgetattr(fd, &options) < 0)
		{
			perror("couldn't read attributes");
			return -2;
		}

		/*
		* Set the baud rates to 9600...
		*/

		cfsetispeed(&options, baudrate);
		cfsetospeed(&options, baudrate);
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		options.c_oflag &= ~OPOST; //raw output

		/*
		* Enable the receiver and set local mode...
		*/

		options.c_cflag |= (CLOCAL | CREAD);

		/*
		* Set the new options for the port...
		*/

		tcsetattr(fd, TCSANOW, &options);
		if(tcsetattr(fd, TCSAFLUSH, &options) < 0)
		{
			perror("couldn't write new attributes");
			return -3;
		}
	}

	void serialport_writebyte(const unsigned char data)
	{
		int n = write(fd,&data,1);
		if(n < 0)
		{
			fputs("write() of 1 bytes failed!\n", stderr);
		}
	}

	unsigned char serialport_readbyte_withtimout()
	{
		fd_set set;
		struct timeval timeout;
		int rv;
		char buff[100];
		int len = 100;

		FD_ZERO(&set); /* clear the set */
		FD_SET(fd, &set); /* add our file descriptor to the set */

		timeout.tv_sec = 0;
		timeout.tv_usec = 10000;

		rv = select(fd + 1, &set, NULL, NULL, &timeout);
		if(rv == -1)
			perror("select"); /* an error accured */
		else if(rv == 0)
			printf("timeout"); /* a timeout occured */
		else
		{
			return serialport_readbyte();
		}
		return 0;
	}

	unsigned char serialport_readbyte()
	{
		unsigned char data = 0;
		int n = read(fd,&data,1);
		if(n < 0)
		{
			fputs("read() of 1 bytes failed!\n", stderr);
		}
		return data;
	}

	int serialport_flush(int fd)
	{
		sleep(2); //required to make flush work, for some reason
		return tcflush(fd, TCIOFLUSH);
	}

  //===============================================================
  //   MAIN
  //===============================================================

	int main(int argc, char *argv[])
	{
		char *serialPort;
		if(argc == 2)
		{
			serialPort = argv[1];
		}
		else
		{
			//try the default one
			serialPort = "/dev/ttyACM0";
			if(serialport_init(serialPort) != 0)
			{
				serialPort = "/dev/ttyACM1";
				if(serialport_init(serialPort) != 0)
				{
					return -1;
				}
			}
		}
		serialport_flush(fd);

		unsigned char data;
		int more = 1;

		while(more == 1)
		{
			printf( "Enter a value : \n");
			data = getchar();
			if (data == 57) //enter 9 in ascii
			{
				more = 0;
			}
			else
			{
				serialport_writebyte(data);

				if(data == 50)
				{
						usleep(10000); //wait the arduino write in pipe
						printf( "Pot value : %d\n", serialport_readbyte());
				}
			}
		}

		close(fd);

		return 0;
	}
