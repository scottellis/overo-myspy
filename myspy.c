/*
  myspy.c - Messing around with OMAP3 SPI
 
  Copyright Scott Ellis, 2010
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#define DEFAULT_BUS_SPEED 3000000

static int bus_speed = DEFAULT_BUS_SPEED;
module_param(bus_speed, int, S_IRUGO);
MODULE_PARM_DESC(bus_speed, "SPI bus speed in Hz");

#define SPI_BUFF_SIZE 128

struct myspy_dev {
	dev_t devt;
	spinlock_t spi_lock;
	struct semaphore sem;
	struct cdev cdev;
	struct class *class;
	struct spi_device *spi_device;	
	u8 *rx_buff;
	u8 *tx_buff;
	char *user_buff;
};

static struct myspy_dev myspy_dev;


static void myspy_complete(void *arg)
{
	complete(arg);
}

static ssize_t myspy_sync(struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = myspy_complete;
	message->context = &done;

	spin_lock_irq(&myspy_dev.spi_lock);

	if (myspy_dev.spi_device == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(myspy_dev.spi_device, message);

	spin_unlock_irq(&myspy_dev.spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}

	return status;
}

static ssize_t myspy_sync_write(size_t len)
{
	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof(struct spi_transfer));
	t.tx_buf = myspy_dev.tx_buff;
	t.rx_buf = myspy_dev.rx_buff;
	t.len = len;

	/* override the bus speed if needed */
	if (myspy_dev.spi_device->max_speed_hz != bus_speed)
		t.speed_hz = bus_speed;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return myspy_sync(&m);
}

/*
  Invokes a synchronous duplex SPI operation. 

  I'm just playing with a I/O expander that implements a write before read
  protocol when you want to do reads. So when we want to read two bytes,
  we actually have to clock 4 bytes, two bytes for the write to tell it
  which register and two bytes to receive the data. 
  Writes to this device are straightforward.

  If you are just using this code for reference, you'll want to replace 
  everything   in this function with the logic for your own device.
*/
#define DEVICE_ADDRESS 0x20
#define ADDRESS_SHIFT 0x01
#define READ_BIT 0x01
#define IODIRA		0x00
#define GPIOA		0x12
static ssize_t myspy_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	ssize_t	status;
	int tx_len, rx_len;
	
	if (count > 32)
		return -EMSGSIZE;

	if (down_interruptible(&myspy_dev.sem)) 
		return -ERESTARTSYS;

	memset(myspy_dev.user_buff, 0, SPI_BUFF_SIZE);

	if (copy_from_user(myspy_dev.user_buff, buf, count)) {
		up(&myspy_dev.sem);
		return -EFAULT;
	}

	memset(myspy_dev.tx_buff, 0, 16);

	/* initialize rx so we know if its valid */
	memset(myspy_dev.rx_buff, 0x33, 16);

	myspy_dev.tx_buff[0] = DEVICE_ADDRESS << ADDRESS_SHIFT;

	if (!strncmp(myspy_dev.user_buff, "read-config", 
					strlen("read-config"))) {
		myspy_dev.tx_buff[0] |= READ_BIT;
		myspy_dev.tx_buff[1] = IODIRA;		
		tx_len = 4;
		rx_len = 2;
	}
	else if (!strncmp(myspy_dev.user_buff, "set-config-out", 
					strlen("set-config-out"))) {
		myspy_dev.tx_buff[1] = IODIRA;
		myspy_dev.tx_buff[2] = 0x00;
		myspy_dev.tx_buff[3] = 0x00;
		tx_len = 4;
		rx_len = 0;
	}
	else if (!strncmp(myspy_dev.user_buff, "set-config-in", 
					strlen("set-config-in"))) {
		myspy_dev.tx_buff[1] = IODIRA;
		myspy_dev.tx_buff[2] = 0xff;
		myspy_dev.tx_buff[3] = 0xff;
		tx_len = 4;
		rx_len = 0;
	}
	else if (!strncmp(myspy_dev.user_buff, "read-io", 
					strlen("read-io"))) {
		myspy_dev.tx_buff[0] |= READ_BIT;
		myspy_dev.tx_buff[1] = GPIOA;
		tx_len = 4;
		rx_len = 2;		
	}
	else if (!strncmp(myspy_dev.user_buff, "write-io-on", 
					strlen("write-io-on"))) {
		myspy_dev.tx_buff[1] = GPIOA;
		myspy_dev.tx_buff[2] = 0xff;
		myspy_dev.tx_buff[3] = 0xff;
		tx_len = 4;
		rx_len = 0;		
	}
	else if (!strncmp(myspy_dev.user_buff, "write-io-off", 
					strlen("write-io-off"))) {
		myspy_dev.tx_buff[1] = GPIOA;
		myspy_dev.tx_buff[2] = 0x00;
		myspy_dev.tx_buff[3] = 0x00;
		tx_len = 4;
		rx_len = 0;		
	}
	else {
		printk(KERN_ALERT "Unknown command %s\n", myspy_dev.user_buff);
		tx_len = 0;
		rx_len = 0;
	}

	if (tx_len > 0) {
		status = myspy_sync_write(tx_len);

		if (status != tx_len) 
			printk(KERN_ALERT "myspy_sync_write(%d) returned %d\n", 
				tx_len, status);
		
		if (rx_len > 0) 
			printk(KERN_ALERT "rx_buff: %02X %02X\n",
				myspy_dev.rx_buff[2], myspy_dev.rx_buff[3]);
	}

	up(&myspy_dev.sem);

	return count;
}

static int myspy_open(struct inode *inode, struct file *filp)
{	
	int status = 0;

	if (down_interruptible(&myspy_dev.sem))
		return -ERESTARTSYS;

	if (!myspy_dev.tx_buff) {
		myspy_dev.tx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
		if (!myspy_dev.tx_buff) 
			status = -ENOMEM;
	}

	if (!myspy_dev.rx_buff) {
		myspy_dev.rx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
		if (!myspy_dev.rx_buff) 
			status = -ENOMEM;
	}

	if (!myspy_dev.user_buff) {
		myspy_dev.user_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
		if (!myspy_dev.user_buff) 
			status = -ENOMEM;
	}	

	up(&myspy_dev.sem);

	return status;
}

static int myspy_probe(struct spi_device *spi_device)
{
	myspy_dev.spi_device = spi_device;
	spi_set_drvdata(spi_device, &myspy_dev);	
	
	printk(KERN_ALERT 
		"myspy_probe(): SPI[%d] max_speed_hz %d Hz  bus_speed %d Hz\n", 
		spi_device->chip_select, 
		spi_device->max_speed_hz, 
		bus_speed);

	return 0;
}

static int myspy_remove(struct spi_device *spi_device)
{
	printk(KERN_ALERT "inside myspy_remove()\n");

	spin_lock_irq(&myspy_dev.spi_lock);
	myspy_dev.spi_device = NULL;
	spi_set_drvdata(spi_device, NULL);
	spin_unlock_irq(&myspy_dev.spi_lock);

	return 0;
}

static int __init add_myspy_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	int status;
	char buff[64];

	spi_master = spi_busnum_to_master(1);

	if (!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(1) returned NULL\n");
		printk(KERN_ALERT "Missing modprobe omap2_mcspi?\n");
		return -1;
	}

	spi_device = spi_alloc_device(spi_master);

	if (!spi_device) {
		printk(KERN_ALERT "spi_alloc_device() failed\n");
		status = -1;
		goto add_myspy_done;
	}

	/* choose your chip select */
	spi_device->chip_select = 0;

	/* first check if the bus already knows about us */
	snprintf(buff, sizeof(buff), "%s.%u", dev_name(&spi_device->master->dev),
			spi_device->chip_select);

	if (bus_find_device_by_name(spi_device->dev.bus, NULL, buff)) {
		/* 
		We are already registered, nothing to do, just free
		the spi_device. Crashes without a patched 
		omap2_mcspi_cleanup() 
		*/
		spi_dev_put(spi_device);
		status = 0;
	} else {
		spi_device->max_speed_hz = 100000;
		spi_device->mode = SPI_MODE_0;
		spi_device->bits_per_word = 8;
		spi_device->irq = -1;
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, "myspy", SPI_NAME_SIZE);

		status = spi_add_device(spi_device);
		if (status < 0) {	
			/* crashes without a patched omap2_mcspi_cleanup() */	
			spi_dev_put(spi_device);
			printk(KERN_ALERT "spi_add_device() failed: %d\n", status);		
		}	
	}

add_myspy_done:

	put_device(&spi_master->dev);

	return status;
}

static struct spi_driver myspy_spi = {
	.driver = {
		.name =		"myspy",
		.owner =	THIS_MODULE,
	},
	.probe =	myspy_probe,
	.remove =	__devexit_p(myspy_remove),	
};

static int __init myspy_init_spi(void)
{
	int error;

	error = spi_register_driver(&myspy_spi);
	if (error < 0) {
		printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		return -1;
	}

	error = add_myspy_to_bus();
	if (error < 0) {
		printk(KERN_ALERT "add_myspy_to_bus() failed\n");
		spi_unregister_driver(&myspy_spi);		
	}

	return error;
}

static const struct file_operations myspy_fops = {
	.owner =	THIS_MODULE,
	.write =	myspy_write,
	.open =		myspy_open,	
};


static int __init myspy_init_cdev(void)
{
	int error;

	myspy_dev.devt = MKDEV(0, 0);

	error = alloc_chrdev_region(&myspy_dev.devt, 0, 1, "myspy");
	if (error < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: %d \n", 
			error);
		return -1;
	}

	cdev_init(&myspy_dev.cdev, &myspy_fops);
	myspy_dev.cdev.owner = THIS_MODULE;
	
	error = cdev_add(&myspy_dev.cdev, myspy_dev.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: %d\n", error);
		unregister_chrdev_region(myspy_dev.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init myspy_init_class(void)
{
	myspy_dev.class = class_create(THIS_MODULE, "myspy");

	if (!myspy_dev.class) {
		printk(KERN_ALERT "class_create() failed\n");
		return -1;
	}

	if (!device_create(myspy_dev.class, NULL, myspy_dev.devt, NULL, "myspy")) {
		printk(KERN_ALERT "device_create(..., myspy) failed\n");
		class_destroy(myspy_dev.class);
		return -1;
	}

	return 0;
}
static int __init myspy_init(void)
{
	memset(&myspy_dev, 0, sizeof(struct myspy_dev));

	spin_lock_init(&myspy_dev.spi_lock);
	sema_init(&myspy_dev.sem, 1);

	if (myspy_init_cdev() < 0) 
		goto fail_1;
	
	if (myspy_init_class() < 0)  
		goto fail_2;

	if (myspy_init_spi() < 0) 
		goto fail_3;

	return 0;

fail_3:
	device_destroy(myspy_dev.class, myspy_dev.devt);
	class_destroy(myspy_dev.class);

fail_2:
	cdev_del(&myspy_dev.cdev);
	unregister_chrdev_region(myspy_dev.devt, 1);

fail_1:
	return -1;
}

static void __exit myspy_exit(void)
{
	spi_unregister_driver(&myspy_spi);

	device_destroy(myspy_dev.class, myspy_dev.devt);
	class_destroy(myspy_dev.class);

	cdev_del(&myspy_dev.cdev);
	unregister_chrdev_region(myspy_dev.devt, 1);

	if (myspy_dev.rx_buff)
		kfree(myspy_dev.rx_buff);

	if (myspy_dev.tx_buff)
		kfree(myspy_dev.tx_buff);

	if (myspy_dev.user_buff)
		kfree(myspy_dev.user_buff);
}

module_init(myspy_init);
module_exit(myspy_exit);

MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("SPI OMAP3 experimental driver");
MODULE_LICENSE("GPL");

