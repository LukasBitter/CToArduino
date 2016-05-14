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
		#include <asm/segment.h> /* Needed for read write to dev */
		#include <linux/buffer_head.h> /* Needed for read write to dev */

		#define DRIVER_AUTHOR "Nils Ryter <nils.ryter@he-arc.ch> and Lukas Bitter <lukas.bitter@he-arc.ch>"
		#define DRIVER_DESC   "Send ascii code to arduino"
		#define SUCCESS 0
		#define DEVICE_NAME "arduinodev"	/* Dev name as it appears in /proc/devices   */
		#define BUF_LEN 80		/* Max length of the message from the device */

		MODULE_LICENSE("GPL");
		MODULE_AUTHOR(DRIVER_AUTHOR);	/* Who wrote this module? */
		MODULE_DESCRIPTION(DRIVER_DESC);	/* What does this module do */

	  //===============================================================
	  //   IMPORT ARGS FROM CONSOLE
	  //===============================================================

	  static char *arduinoSerialPort = "/dev/ttyACM0";

	  module_param(arduinoSerialPort, charp, 0000);
	  MODULE_PARM_DESC(arduinoSerialPort, "The arduino serial port");

		//===============================================================
		//   GLOBAL VARIABLES
		//===============================================================

		static struct file* fd; /* File descriptor for the port */
		/* The msg the device will give when asked */
		static char msg[BUF_LEN];
		static char *msg_Ptr;

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
		void serialport_writebyte(unsigned char data);
		void serialport_readbyte(void);
		struct file* file_open(const char* path, int flags, int rights);
		void file_close(struct file* file);
		int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
		int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size);
		int file_sync(struct file* file);

		//===============================================================
		//   COMMUNICATION SERIAL
		//===============================================================

		void serialport_writebyte(unsigned char data)
		{
			file_write(fd, 0, &data, 1);

			printk(KERN_INFO "write() of 1 bytes %d!\n", data);
		}

		void serialport_readbyte()
		{
			unsigned char data;
			file_read(fd, 0, &data, 1);

			printk(KERN_INFO "read() of 1 bytes %d!\n", data);
		}

		//===============================================================
		//   KERNEL FILE MANAGEMENT
		//
		//	 source : http://stackoverflow.com/questions/1184274/how-to-read-write-files-within-a-linux-kernel-module
		//===============================================================

		/*
		*	Opening a file (similar to open)
		*/
		struct file* file_open(const char* path, int flags, int rights)
		{
		    struct file* filp = NULL;
		    mm_segment_t oldfs;
		    int err = 0;

		    oldfs = get_fs();
		    set_fs(get_ds());
		    filp = filp_open(path, flags, rights);
		    set_fs(oldfs);
		    if(IS_ERR(filp))
				{
		        err = PTR_ERR(filp);
		        return NULL;
		    }
		    return filp;
		}

		/*
		*	Close a file (similar to close)
		*/
		void file_close(struct file* file)
		{
		    filp_close(file, NULL);
		}

		/*
		*	Reading data from a file (similar to pread)
		*/
		int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size)
		{
	    mm_segment_t oldfs;
	    int ret;

	    oldfs = get_fs();
	    set_fs(get_ds());

	    ret = vfs_read(file, data, size, &offset);

	    set_fs(oldfs);
	    return ret;
		}

		/*
		* Writing data to a file (similar to pwrite)
		*/
		int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size)
		{
			mm_segment_t oldfs;
			int ret;

			oldfs = get_fs();
			set_fs(get_ds());

			ret = vfs_write(file, data, size, &offset);

			set_fs(oldfs);
			return ret;
		}

		/*
		* Syncing changes a file (similar to fsync)
		*/
		int file_sync(struct file* file)
		{
	    vfs_fsync(file, 0);
	    return 0;
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
  		put_user(*(msg_Ptr++), buffer++);

	  	/* Most read functions return the number of bytes put into the buffer */
			return 1;
	  }

	  /*
	   * Called when a process writes to dev file: echo "1" > /dev/arduino
	   */
	  static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t * off)
	  {
			int i = *off;
			for(; i < len; ++i)
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

			if (Major < 0)
			{
			  printk(KERN_ALERT "Registering char device failed with %d\n", Major);
			  return Major;
			}

	  	printk(KERN_INFO "Please enter this command to create /dev file\n");
	  	printk(KERN_INFO "'mknod /dev/%s c %d 0'.\n", DEVICE_NAME, Major);
	  	printk(KERN_INFO "Remove the device file and module when done.\n");

			//Init serial port
			fd = file_open("/root/toto", O_APPEND, 0);

			if (fd == NULL)
			{
				printk(KERN_ALERT "couldn't open port.\n");
			}

			return SUCCESS;
		}

		/*
		 * This function is called when the module is unloaded
		 */
		void __exit cleanup_module(void)
		{
			//Close serial port
			file_close(fd);
		 	//Unregister the device
			unregister_chrdev(Major, DEVICE_NAME);
		}
