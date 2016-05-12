	/*
	 * The following license idents are currently accepted as indicating free
	 * software modules
	 *
	 *	"GPL"				[GNU Public License v2 or later]
	 *	"GPL v2"			[GNU Public License v2]
	 *	"GPL and additional rights"	[GNU Public License v2 rights and more]
	 *	"Dual BSD/GPL"			[GNU Public License v2 or BSD license choice]
	 *	"Dual MIT/GPL"			[GNU Public License v2 or MIT license choice]
	 *	"Dual MPL/GPL"			[GNU Public License v2 or Mozilla license choice]
	 *
	 * The following other idents are available
	 *
	 *	"Proprietary"			[Non free products]
	 *
	 * There are dual licensed components, but when running with Linux it is the
	 * GPL that is relevant so this is a non issue. Similarly LGPL linked with GPL
	 * is a GPL combined work.
	 *
	 * This exists for several reasons
	 * 1.	So modinfo can show license info for users wanting to vet their setup
	 *	is free
	 * 2.	So the community can ignore bug reports including proprietary modules
	 * 3.	So vendors can do likewise based on their own policies
	 */

		#include <linux/moduleparam.h>
		#include <linux/stat.h>
		#include <linux/module.h>	/* Needed by all modules */
		#include <linux/kernel.h>	/* Needed for KERN_INFO */
		#include <linux/init.h>		/* Needed for the macros */
		#include <linux/fs.h>     /* Needed for read write to dev */
		#include <asm/uaccess.h>	/* Needed for put_user */
		#include <stdio.h>   /* Standard input/output definitions */
		#include <string.h>  /* String function definitions */
		#include <unistd.h>  /* UNIX standard function definitions */
		#include <fcntl.h>   /* File control definitions */
		#include <errno.h>   /* Error number definitions */
		#include <termios.h> /* POSIX terminal control definitions */

		#define DRIVER_AUTHOR "Nils Ryter <nils.ryter@he-arc.ch> and Lukas Bitter <lukas.bitter@he-arc.ch>"
		#define DRIVER_DESC   "Send ascii code to arduino"
		#define SUCCESS 0
		#define DEVICE_NAME "arduinodev"	/* Dev name as it appears in /proc/devices   */
		#define BUF_LEN 80		/* Max length of the message from the device */

		MODULE_LICENSE("GPL");
		MODULE_AUTHOR(DRIVER_AUTHOR);	/* Who wrote this module? */
		MODULE_DESCRIPTION(DRIVER_DESC);	/* What does this module do */

		//===============================================================
		//   CONSTANT
		//===============================================================

		static const speed_t baudrate = B9600;

	  //===============================================================
	  //   IMPORT ARGS FROM CONSOLE
	  //===============================================================

	  static char *arduinoSerialPort = "/dev/ttyACM0";

	  module_param(arduinoSerialPort, charp, 0000);
	  MODULE_PARM_DESC(arduinoSerialPort, "The arduino serial port");

		//===============================================================
		//   GLOBAL VARIABLES
		//===============================================================

		static int fd; /* File descriptor for the port */

		//===============================================================
		//   PROTOTYPE OF METHODS
		//===============================================================

		// TODO put in a .h file
		int init_module(void);
		void cleanup_module(void);
		static int device_open(struct inode *, struct file *);
		static int device_release(struct inode *, struct file *);
		static ssize_t device_read(struct file *, char *, size_t, loff_t *);
		static ssize_t device_write(struct file *, const char *, size_t, loff_t *);

		//===============================================================
		//   COMMUNICATION SERIAL
		//===============================================================

		void serialport_init()
		{
			/*
			* Try to open the port...
			*/
			fd = open(arduinoSerialPort, O_RDWR | O_NOCTTY | O_NDELAY);

			if (fd == -1)
			{
				printk(KERN_ALERT "couldn't open port.\n");
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
				printk(KERN_ALERT "couldn't read attributes.\n");
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
				printk(KERN_ALERT "couldn't write new attributes.\n");
			}
		}

		void serialport_writebyte(const unsigned char data)
		{
			int n = write(fd,&data,1);
			if(n < 0)
			{
				printk(KERN_ALERT "write() of 1 bytes failed!\n");
			}
		}

		unsigned char serialport_readbyte()
		{
			unsigned char data = 0;
			int n = read(fd,&data,1);
			if(n < 0)
			{
				printk(KERN_ALERT "read() of 1 bytes failed!\n");
			}
			return data;
		}

		int serialport_flush(int fd)
		{
			sleep(2); //required to make flush work, for some reason
			return tcflush(fd, TCIOFLUSH);
		}

	  //===============================================================
	  //   REGISTER TO DEV FILE
	  //===============================================================

		/* Major number assigned to our device driver */
		static int Major;
		/* Is device open? Used to prevent multiple access to device */
		static int Device_Open = 0;

		//descriptor of valid operations on module
		static struct file_operations fops = {
			.read = device_read,
			.write = device_write,
			.open = device_open,
			.release = device_release
		};

	  /*
	  *  This module uses /dev/testdevice.  The MODULE_SUPPORTED_DEVICE macro might
	  *  be used in the future to help automatic configuration of modules, but is
	  *  currently unused other than for documentation purposes.
	  */
	  MODULE_SUPPORTED_DEVICE("testdevice");

	  /*
	   * Called when a process tries to open the device file, like
	   * "cat /dev/mycharfile"
	   */
	  static int device_open(struct inode *inode, struct file *file)
	  {
	  	if (Device_Open)
	  		return -EBUSY;

	  	Device_Open++;
	  	try_module_get(THIS_MODULE);

	  	return SUCCESS;
	  }

	  /*
	   * Called when a process closes the device file.
	   */
	  static int device_release(struct inode *inode, struct file *file)
	  {
	  	Device_Open--;		/* We're now ready for our next caller */

	  	/*
	  	 * Decrement the usage count, or else once you opened the file, you'll
	  	 * never get get rid of the module.
	  	 */
	  	module_put(THIS_MODULE);

	  	return 0;
	  }

	  /*
	   * Called when a process, which already opened the dev file, attempts to
	   * read from it.
	   */
	  static ssize_t device_read(struct file *filp,	/* see include/linux/fs.h   */
	  			   char *buffer,	/* buffer to fill with data */
	  			   size_t length,	/* length of the buffer     */
	  			   loff_t * offset)
	  {
			/*
			 * The buffer is in the user data segment, not the kernel
			 * segment so "*" assignment won't work.  We have to use
			 * put_user which copies data from the kernel data segment to
			 * the user data segment.
			 */
			put_user(serialport_readbyte(), buffer++);

	  	/* Most read functions return the number of bytes put into the buffer */
			return 1;
	  }

	  /*
	   * Called when a process writes to dev file: echo "1" > /dev/arduino
	   */
	  static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t * off)
	  {
			for(int i = off; i < len; ++i)
			{
				printk(KERN_INFO "Yey, you send %d to arduino.\n", buff[i]);
				serialport_writebyte(buff[i]);
			}
	  	return SUCCESS;
	  }

		//===============================================================
		//   MODULE ENTER/EXIT POINT
		//===============================================================

		/*
		 * This function is called when the module is loaded
		 */
		int __init init_module(void)
		{
	    Major = register_chrdev(0, DEVICE_NAME, &fops);

			if (Major < 0) {
			  printk(KERN_ALERT "Registering char device failed with %d\n", Major);
			  return Major;
			}

	  	printk(KERN_INFO "Please enter this command to create /dev file\n");
	  	printk(KERN_INFO "'mknod /dev/%s c %d 0'.\n", DEVICE_NAME, Major);
	  	printk(KERN_INFO "Remove the device file and module when done.\n");

			//Init serial port
			serialport_init();
			serialport_flush(fd);

			return SUCCESS;
		}

		/*
		 * This function is called when the module is unloaded
		 */
		void __exit cleanup_module(void)
		{
			//Close serial port
			close(fd);
		 	//Unregister the device
			unregister_chrdev(Major, DEVICE_NAME);
		}
