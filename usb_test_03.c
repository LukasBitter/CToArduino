	#include <stdio.h>   /* Standard input/output definitions */
	#include <string.h>  /* String function definitions */
	#include <unistd.h>  /* UNIX standard function definitions */
	#include <fcntl.h>   /* File control definitions */
	#include <errno.h>   /* Error number definitions */
	#include <termios.h> /* POSIX terminal control definitions */

  //===============================================================
  //   CONSTANT
  //===============================================================

	static const speed_t baudrate = B9600;
	static const char *serialPort = "/dev/ttyACM1";

  //===============================================================
  //   GLOBAL VARIABLES
  //===============================================================

	static int fd; /* File descriptor for the port */

  //===============================================================
  //   METHODS
  //===============================================================

	void serialport_init()
	{
		/*
		* Try to open the port...
		*/
		fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);

		if (fd == -1)
		{
			perror("couldn't open port");
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

	void main()
	{
		serialport_init();

		serialport_flush(fd);

		unsigned char data;
		int more = 1;

		while(true)
		{
			serialport_writebyte(serialport_readbyte());
		}

		close(fd);
	}
