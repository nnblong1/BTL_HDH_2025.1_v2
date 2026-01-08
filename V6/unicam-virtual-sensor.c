// SPDX-License-Identifier: GPL-2.0-only
/*
 * Standalone Virtual Unicam Driver
 * Creates its own platform device - no device tree needed!
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#define DRIVER_NAME "standalone-unicam"
#define DEVICE_NAME "unicam0"

/* Frame Configuration */
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define BYTES_PER_PIXEL 2
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)

/* DMA Buffer Configuration */
#define DMA_BUFFER_SIZE (16 * 1024 * 1024)
#define MAX_FRAMES (DMA_BUFFER_SIZE / FRAME_SIZE)

/* Frame rate: 30 FPS */
#define FRAME_INTERVAL_MS 33

/* ============================================================================
 * DEVICE STRUCTURE
 * ============================================================================ */
struct unicam_device {
	struct device *dev;
	
	/* Virtual Sensor */
	struct task_struct *frame_thread;
	bool virtual_streaming;
	
	/* DMA Memory */
	void *virt_addr;
	dma_addr_t bus_addr;
	size_t buf_size;
	
	/* Frame Tracking */
	atomic_t frame_count;
	u32 current_write_offset;
	wait_queue_head_t frame_wait;
	spinlock_t lock;
	
	/* Device Node */
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *char_device;
	
	bool streaming;
};

static struct unicam_device *g_unicam_dev;
static struct platform_device *g_pdev;

/* ============================================================================
 * FRAME GENERATOR
 * ============================================================================ */
static void generate_test_pattern(u16 *buffer, int width, int height, u32 frame_num)
{
	int x, y;
	
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			u16 value;
			
			if (y < height / 4) {
				/* Top: Horizontal gradient + frame counter */
				value = (x * 1023 / width) + (frame_num % 256);
			} else if (y < height / 2) {
				/* Middle: Vertical gradient */
				value = (y * 1023 / height);
			} else if (y < 3 * height / 4) {
				/* Lower middle: Checkerboard */
				value = ((x / 32 + y / 32) % 2) ? 1023 : 0;
			} else {
				/* Bottom: Frame counter */
				value = (frame_num % 1024);
			}
			
			buffer[y * width + x] = value;
		}
	}
}

static int virtual_sensor_thread(void *data)
{
	struct unicam_device *dev = data;
	u32 frame_num = 0;
	
	pr_info("Virtual sensor thread started\n");
	
	while (!kthread_should_stop()) {
		if (!dev->virtual_streaming) {
			msleep(100);
			continue;
		}
		
		/* Generate frame */
		u16 *frame_ptr = (u16 *)((u8 *)dev->virt_addr + dev->current_write_offset);
		generate_test_pattern(frame_ptr, FRAME_WIDTH, FRAME_HEIGHT, frame_num);
		
		/* Update write pointer */
		spin_lock(&dev->lock);
		
		dev->current_write_offset += FRAME_SIZE;
		if (dev->current_write_offset >= dev->buf_size) {
			dev->current_write_offset = 0;
		}
		
		frame_num++;
		atomic_inc(&dev->frame_count);
		
		/* Update buffer header */
		if (dev->virt_addr) {
			u32 *header = (u32 *)dev->virt_addr;
			header[0] = frame_num;
			header[1] = dev->current_write_offset;
		}
		
		spin_unlock(&dev->lock);
		
		if (frame_num <= 5) {
			pr_info("✓ Frame %u generated (offset: 0x%x)\n", 
					frame_num, dev->current_write_offset);
		}
		
		wake_up_interruptible(&dev->frame_wait);
		msleep(FRAME_INTERVAL_MS);
	}
	
	pr_info("Virtual sensor thread stopped\n");
	return 0;
}

/* ============================================================================
 * CHARACTER DEVICE OPERATIONS
 * ============================================================================ */
static int unicam_open(struct inode *inode, struct file *filp)
{
	if (!g_unicam_dev)
		return -ENODEV;
	
	filp->private_data = g_unicam_dev;
	pr_info("Device opened\n");
	return 0;
}

static int unicam_release(struct inode *inode, struct file *filp)
{
	pr_info("Device closed\n");
	return 0;
}

static ssize_t unicam_read(struct file *filp, char __user *buf,
						   size_t count, loff_t *ppos)
{
	struct unicam_device *dev = filp->private_data;
	u32 frame_info[2];
	
	if (count < sizeof(frame_info))
		return -EINVAL;
	
	frame_info[0] = atomic_read(&dev->frame_count);
	frame_info[1] = dev->current_write_offset;
	
	if (copy_to_user(buf, frame_info, sizeof(frame_info)))
		return -EFAULT;
	
	return sizeof(frame_info);
}

static int unicam_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct unicam_device *dev = filp->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long start = vma->vm_start;
	unsigned long offset = 0;
	void *pos;
	
	if (size > dev->buf_size)
		return -EINVAL;
	
	/* Set VM flags */
	vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	
	/* Map vmalloc memory page by page */
	pos = dev->virt_addr;
	
	while (size > 0) {
		unsigned long pfn;
		struct page *page;
		
		page = vmalloc_to_page(pos);
		if (!page) {
			pr_err("vmalloc_to_page failed at offset %lu\n", offset);
			return -EFAULT;
		}
		
		pfn = page_to_pfn(page);
		
		if (remap_pfn_range(vma, start, pfn, PAGE_SIZE, vma->vm_page_prot)) {
			pr_err("remap_pfn_range failed at offset %lu\n", offset);
			return -EAGAIN;
		}
		
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		offset += PAGE_SIZE;
		
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			break;
	}
	
	pr_info("Mapped %lu bytes to userspace\n", vma->vm_end - vma->vm_start);
	return 0;
}

static unsigned int unicam_poll(struct file *filp, poll_table *wait)
{
	struct unicam_device *dev = filp->private_data;
	poll_wait(filp, &dev->frame_wait, wait);
	return POLLIN | POLLRDNORM;
}

#define UNICAM_IOC_MAGIC 'U'
#define UNICAM_IOC_START_STREAM  _IO(UNICAM_IOC_MAGIC, 1)
#define UNICAM_IOC_STOP_STREAM   _IO(UNICAM_IOC_MAGIC, 2)
#define UNICAM_IOC_GET_BUFFER    _IOR(UNICAM_IOC_MAGIC, 3, unsigned long)

static long unicam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct unicam_device *dev = filp->private_data;
	
	switch (cmd) {
	case UNICAM_IOC_START_STREAM:
		if (dev->streaming) {
			pr_info("Already streaming\n");
			return 0;
		}
		
		pr_info("Starting virtual streaming...\n");
		dev->virtual_streaming = true;
		dev->streaming = true;
		atomic_set(&dev->frame_count, 0);
		dev->current_write_offset = 0;
		
		pr_info("✓✓✓ STREAMING STARTED ✓✓✓\n");
		break;

	case UNICAM_IOC_STOP_STREAM:
		if (!dev->streaming) {
			pr_info("Not streaming\n");
			return 0;
		}
		
		pr_info("Stopping streaming...\n");
		dev->virtual_streaming = false;
		dev->streaming = false;
		pr_info("✓ Streaming stopped\n");
		break;
		
	case UNICAM_IOC_GET_BUFFER:
		if (copy_to_user((void __user *)arg, &dev->buf_size,
						 sizeof(dev->buf_size)))
			return -EFAULT;
		break;
		
	default:
		return -ENOTTY;
	}
	
	return 0;
}

static const struct file_operations unicam_fops = {
	.owner = THIS_MODULE,
	.open = unicam_open,
	.release = unicam_release,
	.read = unicam_read,
	.mmap = unicam_mmap,
	.poll = unicam_poll,
	.unlocked_ioctl = unicam_ioctl,
};

/* ============================================================================
 * PLATFORM DEVICE CREATION & CLEANUP
 * ============================================================================ */
static int create_virtual_device(void)
{
	struct unicam_device *dev;
	int ret;
	
	pr_info("Creating standalone virtual device...\n");
	
	/* Allocate device structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	
	g_unicam_dev = dev;
	
	/* Create a dummy platform device for DMA allocation */
	g_pdev = platform_device_alloc(DRIVER_NAME, -1);
	if (!g_pdev) {
		ret = -ENOMEM;
		goto err_free_dev;
	}
	
	ret = platform_device_add(g_pdev);
	if (ret) {
		platform_device_put(g_pdev);
		goto err_free_dev;
	}
	
	dev->dev = &g_pdev->dev;
	
	/* Initialize synchronization */
	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->frame_wait);
	atomic_set(&dev->frame_count, 0);
	dev->current_write_offset = 0;
	dev->virtual_streaming = false;
	
	/* Set DMA mask */
	ret = dma_set_mask_and_coherent(dev->dev, DMA_BIT_MASK(32));
	if (ret) {
		pr_err("Failed to set DMA mask\n");
		goto err_pdev;
	}
	
	/* Allocate buffer using vmalloc for large allocations */
	dev->buf_size = DMA_BUFFER_SIZE;
	dev->virt_addr = vmalloc(dev->buf_size);
	if (!dev->virt_addr) {
		pr_err("Failed to allocate buffer\n");
		ret = -ENOMEM;
		goto err_pdev;
	}
	
	memset(dev->virt_addr, 0, dev->buf_size);
	/* For vmalloc, we don't have a single physical address */
	dev->bus_addr = 0;
	
	pr_info("✓ Buffer allocated: %zu bytes at virt=%p (vmalloc)\n",
			dev->buf_size, dev->virt_addr);
	pr_info("  Max frames: %d (%dx%d)\n", MAX_FRAMES, FRAME_WIDTH, FRAME_HEIGHT);
	
	/* Create frame generation thread */
	dev->frame_thread = kthread_create(virtual_sensor_thread, dev, "virtual-sensor");
	if (IS_ERR(dev->frame_thread)) {
		pr_err("Failed to create thread\n");
		ret = PTR_ERR(dev->frame_thread);
		goto err_buffer;
	}
	wake_up_process(dev->frame_thread);
	pr_info("✓ Frame thread created\n");
	
	/* Create character device */
	ret = alloc_chrdev_region(&dev->devt, 0, 1, DEVICE_NAME);
	if (ret) {
		pr_err("Failed to allocate chrdev region\n");
		goto err_thread;
	}
	
	cdev_init(&dev->cdev, &unicam_fops);
	dev->cdev.owner = THIS_MODULE;
	
	ret = cdev_add(&dev->cdev, dev->devt, 1);
	if (ret) {
		pr_err("Failed to add cdev\n");
		goto err_chrdev;
	}
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
	dev->class = class_create(DRIVER_NAME);
#else
	dev->class = class_create(THIS_MODULE, DRIVER_NAME);
#endif
	
	if (IS_ERR(dev->class)) {
		pr_err("Failed to create class\n");
		ret = PTR_ERR(dev->class);
		goto err_cdev;
	}
	
	dev->char_device = device_create(dev->class, dev->dev, dev->devt,
									 NULL, DEVICE_NAME);
	if (IS_ERR(dev->char_device)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(dev->char_device);
		goto err_class;
	}
	
	pr_info("\n");
	pr_info("╔════════════════════════════════════════════╗\n");
	pr_info("║  Standalone Virtual Unicam Ready!          ║\n");
	pr_info("╠════════════════════════════════════════════╣\n");
	pr_info("║  Device: /dev/%s                      ║\n", DEVICE_NAME);
	pr_info("║  Mode:   SOFTWARE SIMULATION               ║\n");
	pr_info("║  Frame:  %dx%d @ 30 FPS                 ║\n", FRAME_WIDTH, FRAME_HEIGHT);
	pr_info("║  Buffer: %d MB (%d frames)            ║\n",
			(int)(DMA_BUFFER_SIZE / (1024*1024)), MAX_FRAMES);
	pr_info("╚════════════════════════════════════════════╝\n");
	pr_info("\n");
	
	return 0;

err_class:
	class_destroy(dev->class);
err_cdev:
	cdev_del(&dev->cdev);
err_chrdev:
	unregister_chrdev_region(dev->devt, 1);
err_thread:
	kthread_stop(dev->frame_thread);
err_buffer:
	vfree(dev->virt_addr);
err_pdev:
	platform_device_unregister(g_pdev);
err_free_dev:
	kfree(dev);
	g_unicam_dev = NULL;
	
	return ret;
}

static void destroy_virtual_device(void)
{
	struct unicam_device *dev = g_unicam_dev;
	
	if (!dev)
		return;
	
	pr_info("Removing standalone virtual device...\n");
	
	/* Stop streaming */
	if (dev->streaming) {
		dev->virtual_streaming = false;
		dev->streaming = false;
	}
	
	/* Cleanup character device */
	if (dev->char_device)
		device_destroy(dev->class, dev->devt);
	if (dev->class)
		class_destroy(dev->class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->devt, 1);
	
	/* Stop thread */
	if (dev->frame_thread)
		kthread_stop(dev->frame_thread);
	
	/* Free buffer */
	if (dev->virt_addr)
		vfree(dev->virt_addr);
	
	/* Remove platform device */
	if (g_pdev)
		platform_device_unregister(g_pdev);
	
	kfree(dev);
	g_unicam_dev = NULL;
	
	pr_info("✓ Cleanup complete\n");
}

/* ============================================================================
 * MODULE INIT/EXIT
 * ============================================================================ */
static int __init unicam_init(void)
{
	pr_info("=== Standalone Virtual Unicam Driver ===\n");
	return create_virtual_device();
}

static void __exit unicam_exit(void)
{
	destroy_virtual_device();
	pr_info("Driver unloaded\n");
}

module_init(unicam_init);
module_exit(unicam_exit);

MODULE_AUTHOR("Embedded Vision Systems");
MODULE_DESCRIPTION("Standalone Virtual Unicam - No Device Tree Required");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("3.0");