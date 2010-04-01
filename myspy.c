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
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#define SPI_BUFF_SIZE 128

struct myspy_data {
	dev_t devt;
	struct cdev cdev;
	spinlock_t spi_lock;
	struct spi_device *spi_device;
	struct mutex buf_lock;
	u8 *rx_buff;
	u8 *tx_buff;
	char *user_buff;
};

static struct myspy_data myspy_data;


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

	spin_lock_irq(&myspy_data.spi_lock);

	if (myspy_data.spi_device == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(myspy_data.spi_device, message);

	spin_unlock_irq(&myspy_data.spi_lock);

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
	t.tx_buf = myspy_data.tx_buff;
	t.rx_buf = myspy_data.rx_buff;
	t.len = len;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return myspy_sync(&m);
}

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

	mutex_lock(&myspy_data.buf_lock);

	memset(myspy_data.user_buff, 0, SPI_BUFF_SIZE);

	if (copy_from_user(myspy_data.user_buff, buf, count)) {
		mutex_unlock(&myspy_data.buf_lock);
		return -EFAULT;
	}

	memset(myspy_data.tx_buff, 0, 16);

	/* initialize rx so we know if its valid */
	memset(myspy_data.rx_buff, 0x33, 16);

	myspy_data.tx_buff[0] = DEVICE_ADDRESS << ADDRESS_SHIFT;

	if (!strncmp(myspy_data.user_buff, "read-config", strlen("read-config"))) {
		myspy_data.tx_buff[0] |= READ_BIT;
		myspy_data.tx_buff[1] = IODIRA;		
		tx_len = 4;
		rx_len = 2;
	}
	else if (!strncmp(myspy_data.user_buff, "set-config-out", strlen("set-config-out"))) {
		myspy_data.tx_buff[1] = IODIRA;
		myspy_data.tx_buff[2] = 0x00;
		myspy_data.tx_buff[3] = 0x00;
		tx_len = 4;
		rx_len = 0;
	}
	else if (!strncmp(myspy_data.user_buff, "set-config-in", strlen("set-config-in"))) {
		myspy_data.tx_buff[1] = IODIRA;
		myspy_data.tx_buff[2] = 0xff;
		myspy_data.tx_buff[3] = 0xff;
		tx_len = 4;
		rx_len = 0;
	}
	else if (!strncmp(myspy_data.user_buff, "read-io", strlen("read-io"))) {
		myspy_data.tx_buff[0] |= READ_BIT;
		myspy_data.tx_buff[1] = GPIOA;
		tx_len = 4;
		rx_len = 2;		
	}
	else if (!strncmp(myspy_data.user_buff, "write-io-on", strlen("write-io-on"))) {
		myspy_data.tx_buff[1] = GPIOA;
		myspy_data.tx_buff[2] = 0xff;
		myspy_data.tx_buff[3] = 0xff;
		tx_len = 4;
		rx_len = 0;		
	}
	else if (!strncmp(myspy_data.user_buff, "write-io-off", strlen("write-io-off"))) {
		myspy_data.tx_buff[1] = GPIOA;
		myspy_data.tx_buff[2] = 0x00;
		myspy_data.tx_buff[3] = 0x00;
		tx_len = 4;
		rx_len = 0;		
	}
	else {
		printk(KERN_ALERT "Unknown command %s\n", myspy_data.user_buff);
		tx_len = 0;
		rx_len = 0;
	}

	if (tx_len > 0) {
		status = myspy_sync_write(tx_len);

		if (status != tx_len) 
			printk(KERN_ALERT "myspy_sync_write(%d) returned %d\n", tx_len, status);
		
		if (rx_len > 0) 
			printk(KERN_ALERT "rx_buff: %02X %02X\n",
				myspy_data.rx_buff[2], myspy_data.rx_buff[3]);
	}

	mutex_unlock(&myspy_data.buf_lock);

	return count;
}

static int myspy_open(struct inode *inode, struct file *filp)
{	
	int status = 0;

	mutex_lock(&myspy_data.buf_lock);

	if (!myspy_data.tx_buff) {
		myspy_data.tx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
		if (!myspy_data.tx_buff) 
			status = -ENOMEM;
	}

	if (!myspy_data.rx_buff) {
		myspy_data.rx_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
		if (!myspy_data.rx_buff) 
			status = -ENOMEM;
	}

	if (!myspy_data.user_buff) {
		myspy_data.user_buff = kmalloc(SPI_BUFF_SIZE, GFP_KERNEL);
		if (!myspy_data.user_buff) 
			status = -ENOMEM;
	}	

	mutex_unlock(&myspy_data.buf_lock);

	return status;
}

static int myspy_probe(struct spi_device *spi_device)
{
	printk(KERN_ALERT "inside myspy_probe()\n");

	myspy_data.spi_device = spi_device;
	spi_set_drvdata(spi_device, &myspy_data);	
	
	return 0;
}

static int myspy_remove(struct spi_device *spi_device)
{
	printk(KERN_ALERT "inside myspy_remove()\n");

	spin_lock_irq(&myspy_data.spi_lock);
	myspy_data.spi_device = NULL;
	spi_set_drvdata(spi_device, NULL);
	spin_unlock_irq(&myspy_data.spi_lock);

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

	if (spi_device) {
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

static int __init myspy_spi_setup(void)
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

static int __init myspy_cdev_setup(void)
{
	int error;

	myspy_data.devt = MKDEV(0, 0);

	if ((error = alloc_chrdev_region(&myspy_data.devt, 0, 1, "myspy")) < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: error = %d \n", 
			error);
		return -1;
	}

	cdev_init(&myspy_data.cdev, &myspy_fops);
	myspy_data.cdev.owner = THIS_MODULE;
	myspy_data.cdev.ops = &myspy_fops;

	error = cdev_add(&myspy_data.cdev, myspy_data.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: error = %d\n", error);
		unregister_chrdev_region(myspy_data.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init myspy_init(void)
{
	spin_lock_init(&myspy_data.spi_lock);
	mutex_init(&myspy_data.buf_lock);

	if (myspy_cdev_setup() < 0) {
		printk(KERN_ALERT "myspy_cdev_setup() failed\n");
		goto fail_1;
	}
	
	if (myspy_spi_setup() < 0) {
		printk(KERN_ALERT "myspy_spi_setup() failed\n");
		goto fail_2;
	}

	printk(KERN_ALERT "Verify : mknod /dev/myspy c %d %d\n", 
		MAJOR(myspy_data.devt), MINOR(myspy_data.devt));

	return 0;

fail_2:
	cdev_del(&myspy_data.cdev);
	unregister_chrdev_region(myspy_data.devt, 1);

fail_1:
	return -1;
}

static void __exit myspy_exit(void)
{
	spi_unregister_driver(&myspy_spi);
	cdev_del(&myspy_data.cdev);
	unregister_chrdev_region(myspy_data.devt, 1);

	mutex_lock(&myspy_data.buf_lock);

	if (myspy_data.rx_buff)
		kfree(myspy_data.rx_buff);

	if (myspy_data.tx_buff)
		kfree(myspy_data.tx_buff);

	if (myspy_data.user_buff)
		kfree(myspy_data.user_buff);

	mutex_unlock(&myspy_data.buf_lock);
}

module_init(myspy_init);
module_exit(myspy_exit);

MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("SPI OMAP3 experimental driver");
MODULE_LICENSE("GPL");

