    #include <stdio.h>   /* Standard input/output definitions */
    #include <string.h>  /* String function definitions */
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */


void main()
{
      int fd; /* File descriptor for the port */


      fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
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
    unsigned char data;
    int more = 1;
    while(more == 1){
	printf( "Enter a value : ");
	data = getchar();
	if (data == 57){
		more = 0;
	}
	else{
		int n = write(fd, &data, 8);
		if(n < 0)
			fputs("write() of 8 bytes failed!\n", stderr);
	}
    }

    close(fd);

}
