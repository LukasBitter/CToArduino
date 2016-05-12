	#include <stdio.h>   /* Standard input/output definitions */
	#include <string.h>  /* String function definitions */
	#include <unistd.h>  /* UNIX standard function definitions */
	#include <fcntl.h>   /* File control definitions */
	#include <errno.h>   /* Error number definitions */
	#include <termios.h> /* POSIX terminal control definitions */


	void main()
	{
		static int fd;
		int baudrate = 9600; 
		
		fd = serialport_init("/dev/ttyACM1", baudrate);
		if( fd==-1 ) error("couldn't open port");
		
		serialport_flush(fd);
		
		unsigned char data;
		int more = 1;
		
		while(more == 1){
			printf( "Enter a value : ");
			data = getchar();
			if (data == 9){
				more = 0;
			}
			else{
				int n = serialport_writebyte(fd, &data);
				if(n < 0)
				fputs("write() of 8 bytes failed!\n", stderr);
			}
		}

		close(fd);

	}
	
	int serialport_init(const char* serialport, int baud)
	{
		//int fd; /* File descriptor for the port */

		int fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd == -1)
		{
			/*
	* Could not open the port.
	*/

			perror("open_port: Unable to open /dev/ttyACM0 - ");
		}
		else
		fcntl(fd, F_SETFL, 0);

		struct termios options;

		/*
	* Get the current options for the port...
	*/

		if(tcgetattr(fd, &options) < 0){
			perror("serialport_init:Couldn't get erm attributes");
		}

		/*
	* Set the baud rates to 9600...
	*/

		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
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
		if(tcsetattr(fd, TCSAFLUSH, &options) < 0){
			perror("init_serialport: Couldn't set term attributes");
		}
		
		return fd;
	}
	
	int serialport_writebyte( int fd, unsigned char* b)
	{
		int n = write(fd,&b,8);
		if( n!=1)
		return -1;
		return 0;
	}
	
	int serialport_flush(int fd)
	{
		sleep(2); //required to make flush work, for some reason
		return tcflush(fd, TCIOFLUSH);
	}
