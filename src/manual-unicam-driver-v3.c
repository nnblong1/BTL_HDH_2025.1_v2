// SPDX-License-Identifier: GPL-2.0-only
/*
 * Manual DMA Driver for BCM2711 Unicam and IMX219 Camera Sensor
 * Raspberry Pi 4B - High Performance Zero-Copy Image Capture
 * 
 * FIXED: Hardware initialization moved to probe() for proper interrupt setup
 */

/* ============================================================================
 * PHẦN 1: INCLUDES
 * ============================================================================ */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include "bcm2835-unicam-regs.h"
#include "imx219-regs.h"

#define DRIVER_NAME "manual-unicam"
#define DEVICE_NAME "unicam0"

/* Frame Configuration for Mode 7 (640x480) */
#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define BYTES_PER_PIXEL 2  /* RAW10 unpacked to 16-bit */
#define FRAME_SIZE (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)

/* DMA Buffer Configuration */
#define DMA_BUFFER_SIZE    (16 * 1024 * 1024)  /* 16MB continuous buffer */
#define MAX_FRAMES         (DMA_BUFFER_SIZE / FRAME_SIZE)

/* ============================================================================
 * PHẦN 2: DEBUG MACROS
 * ============================================================================ */
#define unicam_dbg(dev, fmt, ...) \
	dev_dbg(dev, "%s: " fmt, __func__, ##__VA_ARGS__)

#define unicam_info(dev, fmt, ...) \
	dev_info(dev, "%s: " fmt, __func__, ##__VA_ARGS__)

#define unicam_warn(dev, fmt, ...) \
	dev_warn(dev, "%s: " fmt, __func__, ##__VA_ARGS__)

#define unicam_err(dev, fmt, ...) \
	dev_err(dev, "%s: " fmt, __func__, ##__VA_ARGS__)

/* ============================================================================
 * PHẦN 3: DRIVER PRIVATE DATA STRUCTURE
 * ============================================================================ */
struct unicam_device {
	struct device *dev;
	struct platform_device *pdev;
	
	/* Hardware Resources */
	void __iomem *reg_base;
	int irq;
	struct clk *clock_lp;
	struct clk *clock_vpu;
	
	/* Sensor Power Management */
	struct clk *sensor_clk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator *avdd_regulator;
	struct regulator *dovdd_regulator;
	struct regulator *dvdd_regulator;
	bool sensor_powered;
	
	/* I2C Sensor Interface */
	struct i2c_client *sensor_client;
	
	/* DMA Memory Management */
	void *virt_addr;
	dma_addr_t bus_addr;
	size_t buf_size;
	
	/* Frame Tracking */
	atomic_t frame_count;
	u32 current_write_ptr;
	wait_queue_head_t frame_wait;
	spinlock_t lock;
	
	/* Device Node */
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *char_device;
	
	/* Status Flags */
	bool streaming;
	bool initialized;
};

/* ============================================================================
 * PHẦN 4: FORWARD DECLARATIONS
 * ============================================================================ */
static int imx219_write_reg(struct i2c_client *client, u16 reg, u8 val);
static int imx219_read_reg(struct i2c_client *client, u16 reg, u8 *val);
static int imx219_configure_mode7(struct unicam_device *dev);
static int imx219_stop_streaming(struct unicam_device *dev);
static int imx219_start_streaming(struct unicam_device *dev);

/* Global device pointer for char device ops */
static struct unicam_device *g_unicam_dev;

/* ============================================================================
 * PHẦN 5: REGISTER ACCESS FUNCTIONS
 * ============================================================================ */
static inline u32 unicam_reg_read(struct unicam_device *dev, u32 offset)
{
	return readl(dev->reg_base + offset);
}

static inline void unicam_reg_write(struct unicam_device *dev, u32 offset, u32 value)
{
	writel(value, dev->reg_base + offset);
}

/* ============================================================================
 * PHẦN 6: I2C COMMUNICATION FUNCTIONS
 * ============================================================================ */
static int imx219_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = buf,
	};
	
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;
	
	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		dev_err(&client->dev, "Failed to write reg 0x%04x\n", reg);
		return -EIO;
	}
	
	return 0;
}

static int imx219_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	u8 buf[2];
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};
	
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	
	dev_dbg(&client->dev, "Reading register 0x%04x...\n", reg);
	
	ret = i2c_transfer(client->adapter, msgs, 2);
	
	if (ret != 2) {
		dev_err(&client->dev, "Failed to read reg 0x%04x (ret=%d)\n", reg, ret);
		return -EIO;
	}
	
	dev_dbg(&client->dev, "Register 0x%04x = 0x%02x\n", reg, *val);
	
	return 0;
}

/* ============================================================================
 * PHẦN 7: POWER MANAGEMENT FUNCTIONS
 * ============================================================================ */
static int imx219_power_on(struct unicam_device *dev)
{
	int ret;
	
	if (dev->sensor_powered) {
		unicam_info(dev->dev, "Sensor already powered on\n");
		return 0;
	}
	
	unicam_info(dev->dev, "=== Powering ON IMX219 Sensor ===\n");
	
	/* Step 1: Assert reset */
	if (dev->reset_gpio) {
		gpiod_set_value_cansleep(dev->reset_gpio, 0);
		unicam_info(dev->dev, "Reset asserted (GPIO LOW)\n");
	}
	
	if (dev->pwdn_gpio) {
		gpiod_set_value_cansleep(dev->pwdn_gpio, 0);
		unicam_info(dev->dev, "Power-down deasserted\n");
	}
	
	/* Step 2: Enable power supplies */
	if (dev->avdd_regulator) {
		ret = regulator_enable(dev->avdd_regulator);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable AVDD: %d\n", ret);
			return ret;
		}
		unicam_info(dev->dev, "AVDD enabled\n");
	}
	
	if (dev->dovdd_regulator) {
		ret = regulator_enable(dev->dovdd_regulator);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable DOVDD: %d\n", ret);
			goto err_dovdd;
		}
		unicam_info(dev->dev, "DOVDD enabled\n");
	}
	
	if (dev->dvdd_regulator) {
		ret = regulator_enable(dev->dvdd_regulator);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable DVDD: %d\n", ret);
			goto err_dvdd;
		}
		unicam_info(dev->dev, "DVDD enabled\n");
	}
	
	usleep_range(1000, 2000);
	
	/* Step 3: Enable sensor clock */
	if (dev->sensor_clk) {
		ret = clk_prepare_enable(dev->sensor_clk);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable sensor clock: %d\n", ret);
			goto err_clk;
		}
		unicam_info(dev->dev, "✓ Sensor clock enabled: %lu Hz\n",
					clk_get_rate(dev->sensor_clk));
	} else {
		unicam_warn(dev->dev, "No sensor clock - may not work!\n");
	}
	
	usleep_range(1000, 2000);
	
	/* Step 4: Release reset */
	if (dev->reset_gpio) {
		gpiod_set_value_cansleep(dev->reset_gpio, 1);
		unicam_info(dev->dev, "✓ Reset released (GPIO HIGH)\n");
	}
	
	/* Step 5: Wait for sensor initialization */
	msleep(10);
	
	dev->sensor_powered = true;
	unicam_info(dev->dev, "✓✓✓ Sensor powered ON - I2C ready! ✓✓✓\n");
	
	return 0;

err_clk:
	if (dev->dvdd_regulator)
		regulator_disable(dev->dvdd_regulator);
err_dvdd:
	if (dev->dovdd_regulator)
		regulator_disable(dev->dovdd_regulator);
err_dovdd:
	if (dev->avdd_regulator)
		regulator_disable(dev->avdd_regulator);
	
	return ret;
}

static void imx219_power_off(struct unicam_device *dev)
{
	if (!dev->sensor_powered) {
		return;
	}
	
	unicam_info(dev->dev, "Powering OFF sensor\n");
	
	/* Stop streaming if active */
	if (dev->streaming && dev->sensor_client) {
		imx219_write_reg(dev->sensor_client,
						 IMX219_REG_MODE_SELECT,
						 IMX219_MODE_STANDBY);
	}
	
	/* Assert reset */
	if (dev->reset_gpio) {
		gpiod_set_value_cansleep(dev->reset_gpio, 0);
	}
	
	/* Disable clock */
	if (dev->sensor_clk) {
		clk_disable_unprepare(dev->sensor_clk);
	}
	
	/* Disable power supplies */
	if (dev->dvdd_regulator)
		regulator_disable(dev->dvdd_regulator);
	if (dev->dovdd_regulator)
		regulator_disable(dev->dovdd_regulator);
	if (dev->avdd_regulator)
		regulator_disable(dev->avdd_regulator);
	
	if (dev->pwdn_gpio) {
		gpiod_set_value_cansleep(dev->pwdn_gpio, 1);
	}
	
	dev->sensor_powered = false;
	unicam_info(dev->dev, "✓ Sensor powered OFF\n");
}

/* ============================================================================
 * PHẦN 8: SENSOR CONFIGURATION
 * ============================================================================ */
static int imx219_configure_mode7(struct unicam_device *dev)
{
	struct i2c_client *client = dev->sensor_client;
	int i, ret;
	u8 model_id_h, model_id_l;
	
	unicam_info(dev->dev, "Configuring IMX219 sensor for Mode 7\n");
	
	ret = imx219_read_reg(client, IMX219_REG_MODEL_ID_H, &model_id_h);
	if (ret)
		return ret;
	
	ret = imx219_read_reg(client, IMX219_REG_MODEL_ID_L, &model_id_l);
	if (ret)
		return ret;
	
	if ((model_id_h << 8 | model_id_l) != IMX219_MODEL_ID) {
		unicam_err(dev->dev, "Invalid sensor model ID: 0x%02x%02x\n",
				   model_id_h, model_id_l);
		return -ENODEV;
	}
	
	unicam_info(dev->dev, "IMX219 sensor detected (ID: 0x%04x)\n",
				model_id_h << 8 | model_id_l);
	
	for (i = 0; i < IMX219_MODE7_REG_COUNT; i++) {
		ret = imx219_write_reg(client,
							   imx219_mode7_640x480[i].addr,
							   imx219_mode7_640x480[i].val);
		if (ret) {
			unicam_err(dev->dev, "Failed to write register %d\n", i);
			return ret;
		}
		usleep_range(100, 200);
	}
	
	unicam_info(dev->dev, "IMX219 Mode 7 configuration complete\n");
	return 0;
}

static int imx219_stop_streaming(struct unicam_device *dev)
{
	return imx219_write_reg(dev->sensor_client,
							IMX219_REG_MODE_SELECT,
							IMX219_MODE_STANDBY);
}

static int imx219_start_streaming(struct unicam_device *dev)
{
	int ret;
	
	unicam_info(dev->dev, "Starting sensor streaming\n");
	
	ret = imx219_write_reg(dev->sensor_client,
						   IMX219_REG_MODE_SELECT,
						   IMX219_MODE_STREAMING);
	
	if (ret) {
		unicam_err(dev->dev, "Failed to start streaming\n");
		return ret;
	}
	
	msleep(100);
	unicam_info(dev->dev, "✓ Sensor streaming started\n");
	
	return 0;
}

/* ============================================================================
 * PHẦN 9: UNICAM HARDWARE FUNCTIONS
 * ============================================================================ */

/*
 * Initialize Unicam Hardware - CALLED ONLY WHEN STARTING STREAM
 * Registers are already configured in probe(), just enable controller
 */
static int unicam_hw_init(struct unicam_device *dev)
{
	u32 val;
	
	dev_info(dev->dev, "unicam_hw_init: Starting Unicam controller\n");
	
	/* Registers already configured in probe, just enable controller */
	
	/* Clear any pending interrupts */
	unicam_reg_write(dev, UNICAM_ISTA, 0xFFFFFFFF);
	
	/* Reset write pointer */
	unicam_reg_write(dev, UNICAM_IBWP, (u32)dev->bus_addr);
	
	/* Enable Unicam: ENABLE + MEM mode */
	val = (1 << 0) | (1 << 1);  /* UNICAM_CTRL_ENABLE | UNICAM_CTRL_MEM */
	unicam_reg_write(dev, UNICAM_CTRL, val);
	
	/* Verify registers */
	dev_info(dev->dev, "Register Status:\n");
	dev_info(dev->dev, "  UNICAM_CTRL = 0x%08x\n", unicam_reg_read(dev, UNICAM_CTRL));
	dev_info(dev->dev, "  UNICAM_ICTL = 0x%08x\n", unicam_reg_read(dev, UNICAM_ICTL));
	dev_info(dev->dev, "  UNICAM_IBSA0 = 0x%08x\n", unicam_reg_read(dev, UNICAM_IBSA0));
	dev_info(dev->dev, "  UNICAM_IBEA0 = 0x%08x\n", unicam_reg_read(dev, UNICAM_IBEA0));
	
	dev_info(dev->dev, "✓ Unicam controller enabled\n");
	
	return 0;
}

static void unicam_hw_disable(struct unicam_device *dev)
{
	/* Disable core */
	unicam_reg_write(dev, UNICAM_CTRL, 0);
	
	/* Clear interrupts */
	unicam_reg_write(dev, UNICAM_ISTA, 0xFFFFFFFF);
}

/* ============================================================================
 * PHẦN 10: INTERRUPT SERVICE ROUTINE - IMPROVED
 * ============================================================================ */
static irqreturn_t unicam_isr(int irq, void *dev_id)
{
	struct unicam_device *dev = dev_id;
	u32 status, ctrl;
	unsigned long flags;
	static int isr_call_count = 0;
	
	/* Read status register */
	status = unicam_reg_read(dev, UNICAM_ISTA);
	ctrl = unicam_reg_read(dev, UNICAM_CTRL);
	
	/* Debug: Log first 10 ISR calls regardless of status */
	if (isr_call_count < 10) {
		dev_info(dev->dev, "[ISR #%d] Called! status=0x%08x, ctrl=0x%08x\n",
				 isr_call_count, status, ctrl);
		isr_call_count++;
	}
	
	/* Check if this is our interrupt */
	if (!status) {
		return IRQ_NONE;
	}
	
	spin_lock_irqsave(&dev->lock, flags);
	
	/* Frame End interrupt (bit 0) */
	if (status & 0x00000001) {
		u32 frame_num = atomic_inc_return(&dev->frame_count);
		u32 write_ptr = unicam_reg_read(dev, UNICAM_IBWP);
		
		/* Log first 5 frames */
		if (frame_num <= 5) {
			dev_info(dev->dev, "✓✓✓ FRAME %u CAPTURED! write_ptr=0x%08x ✓✓✓\n",
					 frame_num, write_ptr);
		}
		
		/* Update buffer header */
		if (dev->virt_addr) {
			u32 *header = (u32 *)dev->virt_addr;
			header[0] = frame_num;
			header[1] = write_ptr;
		}
		
		/* Wake up any waiting processes */
		wake_up_interruptible(&dev->frame_wait);
		
		/* Clear the interrupt */
		unicam_reg_write(dev, UNICAM_ISTA, 0x00000001);
	}
	
	/* Frame Start interrupt (bit 1) - optional */
	if (status & 0x00000002) {
		dev_info(dev->dev, "Frame Start interrupt\n");
		unicam_reg_write(dev, UNICAM_ISTA, 0x00000002);
	}
	
	spin_unlock_irqrestore(&dev->lock, flags);
	
	return IRQ_HANDLED;
}

/* ============================================================================
 * PHẦN 11: CHARACTER DEVICE OPERATIONS
 * ============================================================================ */

static int unicam_open(struct inode *inode, struct file *filp)
{
	struct unicam_device *dev = g_unicam_dev;
	
	if (!dev)
		return -ENODEV;
	
	filp->private_data = dev;
	
	unicam_info(dev->dev, "Device opened\n");
	
	return 0;
}

static int unicam_release(struct inode *inode, struct file *filp)
{
	struct unicam_device *dev = filp->private_data;
	
	unicam_info(dev->dev, "Device closed\n");
	
	return 0;
}

static ssize_t unicam_read(struct file *filp, char __user *buf,
						   size_t count, loff_t *ppos)
{
	struct unicam_device *dev = filp->private_data;
	u32 frame_info[2];
	
	if (count < sizeof(frame_info))
		return -EINVAL;
	
	/* Return frame count and current write pointer */
	frame_info[0] = atomic_read(&dev->frame_count);
	frame_info[1] = dev->current_write_ptr;
	
	if (copy_to_user(buf, frame_info, sizeof(frame_info)))
		return -EFAULT;
	
	return sizeof(frame_info);
}

static int unicam_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct unicam_device *dev = filp->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret;
	
	if (size > dev->buf_size) {
		unicam_err(dev->dev, "mmap size too large: %lu > %zu\n",
				   size, dev->buf_size);
		return -EINVAL;
	}
	
	/* Map DMA buffer to user space */
	ret = dma_mmap_coherent(dev->dev, vma, dev->virt_addr,
							dev->bus_addr, size);
	
	if (ret) {
		unicam_err(dev->dev, "mmap failed: %d\n", ret);
		return ret;
	}
	
	unicam_info(dev->dev, "Mapped %lu bytes to user space\n", size);
	
	return 0;
}

static unsigned int unicam_poll(struct file *filp, poll_table *wait)
{
	struct unicam_device *dev = filp->private_data;
	
	poll_wait(filp, &dev->frame_wait, wait);
	
	/* Data is always available in circular buffer */
	return POLLIN | POLLRDNORM;
}

/* IOCTL Commands */
#define UNICAM_IOC_MAGIC 'U'
#define UNICAM_IOC_START_STREAM  _IO(UNICAM_IOC_MAGIC, 1)
#define UNICAM_IOC_STOP_STREAM   _IO(UNICAM_IOC_MAGIC, 2)
#define UNICAM_IOC_GET_BUFFER    _IOR(UNICAM_IOC_MAGIC, 3, unsigned long)

static long unicam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct unicam_device *dev = filp->private_data;
	int ret = 0;
	
	switch (cmd) {
	case UNICAM_IOC_START_STREAM:
		if (dev->streaming) {
			unicam_info(dev->dev, "Already streaming\n");
			break;
		}
		
		unicam_info(dev->dev, "Waiting for sensor to settle...\n");
		msleep(200);
		
		/* Configure sensor (Mode 7) */
		ret = imx219_configure_mode7(dev);
		if (ret) {
			unicam_err(dev->dev, "Failed to configure sensor\n");
			break;
		}
		
		/* Enable Unicam controller */
		ret = unicam_hw_init(dev);
		if (ret) {
			unicam_err(dev->dev, "Failed to init hardware\n");
			break;
		}
		
		/* Start sensor streaming */
		ret = imx219_start_streaming(dev);
		if (ret) {
			unicam_hw_disable(dev);
			unicam_err(dev->dev, "Failed to start sensor streaming\n");
			break;
		}
		
		dev->streaming = true;
		atomic_set(&dev->frame_count, 0);
		
		unicam_info(dev->dev, "✓✓✓ STREAMING STARTED SUCCESSFULLY ✓✓✓\n");
		break;

	case UNICAM_IOC_STOP_STREAM:
		if (!dev->streaming) {
			unicam_info(dev->dev, "Not streaming\n");
			break;
		}
		
		/* Stop sensor */
		imx219_stop_streaming(dev);
		
		/* Disable hardware */
		unicam_hw_disable(dev);
		
		dev->streaming = false;
		
		unicam_info(dev->dev, "Streaming stopped\n");
		break;
		
	case UNICAM_IOC_GET_BUFFER:
		if (copy_to_user((void __user *)arg, &dev->buf_size,
						 sizeof(dev->buf_size)))
			return -EFAULT;
		break;
		
	default:
		return -ENOTTY;
	}
	
	return ret;
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
 * PHẦN 12: I2C ADAPTER DETECTION
 * ============================================================================ */
static struct i2c_adapter *unicam_find_i2c_adapter(struct platform_device *pdev)
{
	struct device_node *i2c_node;
	struct i2c_adapter *adapter;
	int i;
	
	/* Method 1: Try to get I2C bus from device tree */
	i2c_node = of_parse_phandle(pdev->dev.of_node, "i2c-bus", 0);
	if (i2c_node) {
		dev_info(&pdev->dev, "Found I2C bus in device tree\n");
		adapter = of_find_i2c_adapter_by_node(i2c_node);
		of_node_put(i2c_node);
		if (adapter)
			return adapter;
	}
	
	/* Method 2: Look for CSI camera I2C bus */
	dev_info(&pdev->dev, "Scanning for camera I2C adapter...\n");
	
	int common_buses[] = {10, 0, 1, 11};
	
	for (i = 0; i < ARRAY_SIZE(common_buses); i++) {
		adapter = i2c_get_adapter(common_buses[i]);
		if (adapter) {
			dev_info(&pdev->dev, "Found I2C adapter at bus %d\n",
					 common_buses[i]);
			return adapter;
		}
	}
	
	/* Method 3: Scan all available I2C adapters */
	dev_info(&pdev->dev, "Scanning all I2C adapters (0-15)...\n");
	for (i = 0; i < 16; i++) {
		adapter = i2c_get_adapter(i);
		if (adapter) {
			dev_info(&pdev->dev, "Found I2C adapter at bus %d\n", i);
			return adapter;
		}
	}
	
	return NULL;
}

/* ============================================================================
 * PHẦN 13: PLATFORM DRIVER PROBE - WITH HARDWARE INIT
 * ============================================================================ */

static int unicam_probe(struct platform_device *pdev)
{
	struct unicam_device *dev;
	struct resource *res;
	struct i2c_adapter *adapter;
	struct i2c_board_info board_info = {
		.type = "imx219",
		.addr = IMX219_I2C_ADDR,
	};
	int ret;
	u8 model_id_h, model_id_l;
	
	dev_info(&pdev->dev, "=== Probing manual Unicam driver ===\n");
	
	/* Allocate device structure */
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	
	dev->dev = &pdev->dev;
	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);
	g_unicam_dev = dev;
	
	/* Initialize synchronization primitives */
	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->frame_wait);
	atomic_set(&dev->frame_count, 0);
	
	/* Get memory resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENODEV;
	}
	
	/* Map registers */
	dev->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->reg_base)) {
		dev_err(&pdev->dev, "Failed to map registers\n");
		return PTR_ERR(dev->reg_base);
	}
	
	dev_info(&pdev->dev, "✓ Registers mapped at %p (phys: 0x%08llx)\n",
			 dev->reg_base, (u64)res->start);
	
	/* Get IRQ */
	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		return dev->irq;
	}
	
	/* Request IRQ */
	ret = devm_request_irq(&pdev->dev, dev->irq, unicam_isr,
						   IRQF_SHARED, DRIVER_NAME, dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n",
				dev->irq, ret);
		return ret;
	}
	
	dev_info(&pdev->dev, "✓ IRQ %d registered\n", dev->irq);
	
	/* Get clocks */
	dev->clock_lp = devm_clk_get(&pdev->dev, "lp");
	if (IS_ERR(dev->clock_lp)) {
		dev_err(&pdev->dev, "Failed to get LP clock\n");
		return PTR_ERR(dev->clock_lp);
	}
	
	dev->clock_vpu = devm_clk_get(&pdev->dev, "vpu");
	if (IS_ERR(dev->clock_vpu)) {
		dev_err(&pdev->dev, "Failed to get VPU clock\n");
		return PTR_ERR(dev->clock_vpu);
	}
	
	/* Enable clocks */
	ret = clk_prepare_enable(dev->clock_lp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable LP clock\n");
		return ret;
	}
	
	ret = clk_prepare_enable(dev->clock_vpu);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable VPU clock\n");
		clk_disable_unprepare(dev->clock_lp);
		return ret;
	}
	
	dev_info(&pdev->dev, "✓ Clocks enabled (LP: %lu Hz, VPU: %lu Hz)\n",
			 clk_get_rate(dev->clock_lp),
			 clk_get_rate(dev->clock_vpu));

	/* Get sensor clock */
	dev->sensor_clk = devm_clk_get_optional(&pdev->dev, "xclk");
	if (IS_ERR(dev->sensor_clk)) {
		dev->sensor_clk = devm_clk_get_optional(&pdev->dev, "cam");
	}
	if (!IS_ERR_OR_NULL(dev->sensor_clk)) {
		clk_set_rate(dev->sensor_clk, 24000000);
		dev_info(&pdev->dev, "✓ Sensor clock: %lu Hz\n",
				 clk_get_rate(dev->sensor_clk));
	} else {
		dev->sensor_clk = NULL;
		dev_warn(&pdev->dev, "Sensor clock not found\n");
	}
	
	/* Get reset GPIO */
	dev->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
											   GPIOD_OUT_LOW);
	if (IS_ERR(dev->reset_gpio)) {
		dev_err(&pdev->dev, "Failed to get reset GPIO: %ld\n",
				PTR_ERR(dev->reset_gpio));
		ret = PTR_ERR(dev->reset_gpio);
		goto err_clocks;
	}
	
	if (dev->reset_gpio) {
		dev_info(&pdev->dev, "✓ Reset GPIO acquired\n");
	}
	
	/* Get optional GPIOs and regulators */
	dev->pwdn_gpio = devm_gpiod_get_optional(&pdev->dev, "pwdn",
											  GPIOD_OUT_LOW);
	if (IS_ERR(dev->pwdn_gpio)) {
		dev->pwdn_gpio = NULL;
	}
	
	dev->avdd_regulator = devm_regulator_get_optional(&pdev->dev, "avdd");
	if (IS_ERR(dev->avdd_regulator)) {
		dev_info(&pdev->dev, "AVDD regulator not found (may be always-on)\n");
		dev->avdd_regulator = NULL;
	}
	
	dev->dovdd_regulator = devm_regulator_get_optional(&pdev->dev, "dovdd");
	if (IS_ERR(dev->dovdd_regulator)) {
		dev->dovdd_regulator = NULL;
	}
	
	dev->dvdd_regulator = devm_regulator_get_optional(&pdev->dev, "dvdd");
	if (IS_ERR(dev->dvdd_regulator)) {
		dev->dvdd_regulator = NULL;
	}
	
	dev->sensor_powered = false;

	/* Set DMA mask */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set DMA mask to 32-bit\n");
		goto err_clocks;
	}

	/* Allocate DMA buffer */
	dev->buf_size = DMA_BUFFER_SIZE;
	dev->virt_addr = dma_alloc_coherent(&pdev->dev, dev->buf_size,
										&dev->bus_addr, GFP_KERNEL);
	if (!dev->virt_addr) {
		dev_err(&pdev->dev, "Failed to allocate DMA buffer\n");
		ret = -ENOMEM;
		goto err_clocks;
	}
	
	dev_info(&pdev->dev, "✓ DMA buffer allocated: %zu bytes\n"
			 "  Virtual: %p\n"
			 "  Bus:     0x%08llx\n",
			 dev->buf_size, dev->virt_addr, (u64)dev->bus_addr);
	
	/* Find I2C adapter for camera */
	dev_info(&pdev->dev, "Looking for camera I2C adapter...\n");
	adapter = unicam_find_i2c_adapter(pdev);
	
	if (!adapter) {
		dev_err(&pdev->dev, "✗ Failed to find I2C adapter\n");
		dev_warn(&pdev->dev, "Continuing without I2C sensor access\n");
		dev->sensor_client = NULL;
		goto skip_i2c;
	}
	
	dev_info(&pdev->dev, "✓ Using I2C adapter: %s\n", adapter->name);
	
	/* Create I2C client for sensor */
	dev->sensor_client = i2c_new_client_device(adapter, &board_info);
	i2c_put_adapter(adapter);
	
	if (IS_ERR(dev->sensor_client)) {
		dev_err(&pdev->dev, "✗ Failed to create I2C client\n");
		dev_warn(&pdev->dev, "Continuing without sensor access\n");
		dev->sensor_client = NULL;
		goto skip_i2c;
	}
	
	dev_info(&pdev->dev, "✓ I2C sensor client created (addr: 0x%02x)\n",
			 dev->sensor_client->addr);

	/* Power ON sensor before I2C communication */
	dev_info(&pdev->dev, "Attempting to power on sensor...\n");
	ret = imx219_power_on(dev);
	if (ret) {
		dev_err(&pdev->dev, "✗ Failed to power on sensor: %d\n", ret);
		i2c_unregister_device(dev->sensor_client);
		dev->sensor_client = NULL;
		goto skip_i2c;
	}
	
	dev_info(&pdev->dev, "Waiting for sensor to stabilize...\n");
	msleep(100);
	
	/* Verify sensor is responding */
	dev_info(&pdev->dev, "Reading sensor model ID...\n");
	ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODEL_ID_H, &model_id_h);
	if (ret == 0) {
		ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODEL_ID_L, &model_id_l);
	}
	
	if (ret == 0) {
		u16 model_id = (model_id_h << 8) | model_id_l;
		dev_info(&pdev->dev, "✓✓✓ IMX219 detected! Model ID: 0x%04x ✓✓✓\n", 
				 model_id);
	} else {
		dev_warn(&pdev->dev, "⚠ Failed to read sensor model ID\n");
		dev_warn(&pdev->dev, "⚠ Continuing without sensor verification\n");
	}

skip_i2c:
	/* Create character device */
	dev_info(&pdev->dev, "Creating character device...\n");
	
	ret = alloc_chrdev_region(&dev->devt, 0, 1, DEVICE_NAME);
	if (ret) {
		dev_err(&pdev->dev, "✗ Failed to allocate char device region: %d\n", ret);
		goto err_i2c;
	}
	
	dev_info(&pdev->dev, "✓ Character device region allocated: %d:%d\n",
			 MAJOR(dev->devt), MINOR(dev->devt));
	
	cdev_init(&dev->cdev, &unicam_fops);
	dev->cdev.owner = THIS_MODULE;
	
	ret = cdev_add(&dev->cdev, dev->devt, 1);
	if (ret) {
		dev_err(&pdev->dev, "✗ Failed to add char device: %d\n", ret);
		goto err_chrdev;
	}
	
	dev_info(&pdev->dev, "✓ Character device added\n");
	
	/* Create device class */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
	dev->class = class_create(DRIVER_NAME);
#else
	dev->class = class_create(THIS_MODULE, DRIVER_NAME);
#endif
	
	if (IS_ERR(dev->class)) {
		dev_err(&pdev->dev, "✗ Failed to create device class: %ld\n",
				PTR_ERR(dev->class));
		ret = PTR_ERR(dev->class);
		goto err_cdev;
	}
	
	dev_info(&pdev->dev, "✓ Device class created\n");
	
	/* Create device node */
	dev->char_device = device_create(dev->class, &pdev->dev, dev->devt,
									 NULL, DEVICE_NAME);
	if (IS_ERR(dev->char_device)) {
		dev_err(&pdev->dev, "✗ Failed to create device node: %ld\n",
				PTR_ERR(dev->char_device));
		ret = PTR_ERR(dev->char_device);
		goto err_class;
	}
	
	dev_info(&pdev->dev, "✓ Device node created: /dev/%s\n", DEVICE_NAME);
	
	/* === HARDWARE INITIALIZATION IN PROBE === */
	dev_info(&pdev->dev, "Initializing Unicam registers...\n");
	
	/* Reset controller */
	unicam_reg_write(dev, UNICAM_CTRL, 0);
	msleep(10);
	
	/* Configure CSI-2 for 2 lanes */
	unicam_reg_write(dev, UNICAM_ANA, 0x00000002);
	
	/* Set DMA buffer addresses */
	unicam_reg_write(dev, UNICAM_IBSA0, (u32)dev->bus_addr);
	unicam_reg_write(dev, UNICAM_IBEA0, (u32)(dev->bus_addr + dev->buf_size));
	
	/* Configure image size (640x480) */
	unicam_reg_write(dev, UNICAM_IBLS, FRAME_WIDTH * 2);  /* Line stride */
	unicam_reg_write(dev, UNICAM_IHWIN, FRAME_WIDTH);
	unicam_reg_write(dev, UNICAM_IVWIN, FRAME_HEIGHT);
	
	/* Enable Frame End interrupt */
	unicam_reg_write(dev, UNICAM_ICTL, 0x00000001);  /* FE interrupt */
	
	/* Clear any pending interrupts */
	unicam_reg_write(dev, UNICAM_ISTA, 0xFFFFFFFF);
	
	dev_info(&pdev->dev, "✓ Unicam registers initialized\n");
	/* === END HARDWARE INITIALIZATION === */
	
	dev->initialized = true;
	
	dev_info(&pdev->dev, "\n");
	dev_info(&pdev->dev, "╔════════════════════════════════════════════╗\n");
	dev_info(&pdev->dev, "║  Manual Unicam Driver Ready!               ║\n");
	dev_info(&pdev->dev, "╠════════════════════════════════════════════╣\n");
	dev_info(&pdev->dev, "║  Device: /dev/%s                      ║\n", DEVICE_NAME);
	dev_info(&pdev->dev, "║  Buffer: %d MB (%d frames)            ║\n",
			 DMA_BUFFER_SIZE / (1024*1024), MAX_FRAMES);
	dev_info(&pdev->dev, "║  I2C:    %s                        ║\n",
			 dev->sensor_client ? "Connected" : "Not available");
	dev_info(&pdev->dev, "╚════════════════════════════════════════════╝\n");
	dev_info(&pdev->dev, "\n");
	
	/* Test interrupt configuration */
	dev_info(&pdev->dev, "Testing interrupt configuration...\n");
	dev_info(&pdev->dev, "  IRQ number: %d\n", dev->irq);
	dev_info(&pdev->dev, "  UNICAM_ICTL: 0x%08x\n", 
			 unicam_reg_read(dev, UNICAM_ICTL));
	
	/* Check if IRQ line is shared */
	ret = request_irq(dev->irq, NULL, IRQF_PROBE_SHARED, "test", NULL);
	if (ret == -EBUSY) {
		dev_info(&pdev->dev, "  IRQ is shared (OK)\n");
	} else {
		dev_warn(&pdev->dev, "  IRQ probe returned: %d\n", ret);
	}
	
	return 0;

err_class:
	class_destroy(dev->class);
err_cdev:
	cdev_del(&dev->cdev);
err_chrdev:
	unregister_chrdev_region(dev->devt, 1);
err_i2c:
	if (dev->sensor_powered) {
		imx219_power_off(dev);
	}
	if (dev->sensor_client)
		i2c_unregister_device(dev->sensor_client);
	dma_free_coherent(&pdev->dev, dev->buf_size,
					  dev->virt_addr, dev->bus_addr);
err_clocks:
	clk_disable_unprepare(dev->clock_vpu);
	clk_disable_unprepare(dev->clock_lp);
	
	return ret;
}

/* ============================================================================
 * PHẦN 14: PLATFORM DRIVER REMOVE
 * ============================================================================ */

static void unicam_remove(struct platform_device *pdev)
{
	struct unicam_device *dev = platform_get_drvdata(pdev);
	
	dev_info(&pdev->dev, "Removing manual Unicam driver\n");
	
	/* Stop streaming if active */
	if (dev->streaming) {
		if (dev->sensor_client)
			imx219_stop_streaming(dev);
		unicam_hw_disable(dev);
	}
	
	/* Power off sensor */
	if (dev->sensor_powered) {
		imx219_power_off(dev);
	}
	
	/* Cleanup device */
	if (dev->char_device)
		device_destroy(dev->class, dev->devt);
	if (dev->class)
		class_destroy(dev->class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->devt, 1);
	
	/* Cleanup I2C */
	if (dev->sensor_client)
		i2c_unregister_device(dev->sensor_client);
	
	/* Free DMA buffer */
	dma_free_coherent(&pdev->dev, dev->buf_size,
					  dev->virt_addr, dev->bus_addr);
	
	/* Disable clocks */
	clk_disable_unprepare(dev->clock_vpu);
	clk_disable_unprepare(dev->clock_lp);
	
	g_unicam_dev = NULL;
	
	dev_info(&pdev->dev, "Manual Unicam driver removed\n");
}

/* ============================================================================
 * PHẦN 15: DEVICE TREE MATCHING & DRIVER REGISTRATION
 * ============================================================================ */

static const struct of_device_id unicam_of_match[] = {
	{ .compatible = "vendor,manual-unicam" },
	{ }
};
MODULE_DEVICE_TABLE(of, unicam_of_match);

static struct platform_driver unicam_driver = {
	.probe = unicam_probe,
	.remove = unicam_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = unicam_of_match,
	},
};

module_platform_driver(unicam_driver);

MODULE_AUTHOR("Embedded Vision Systems");
MODULE_DESCRIPTION("Manual DMA Driver for BCM2711 Unicam and IMX219");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.2")
