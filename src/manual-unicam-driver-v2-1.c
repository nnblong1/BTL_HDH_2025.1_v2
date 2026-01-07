// SPDX-License-Identifier: GPL-2.0-only
/*
 * Manual DMA Driver for BCM2711 Unicam and IMX219 Camera Sensor
 * Raspberry Pi 4B - High Performance Zero-Copy Image Capture
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

/* CRITICAL: D-PHY and Timing Parameters */
#define MIPI_CSI2_LANES    2  /* IMX219 uses 2 lanes */
#define LINK_FREQ_HZ       456000000UL  /* 456 MHz - MUST match sensor PLL */

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
	atomic_t frame_ready;
	
	/* Device Node */
	dev_t devt;
	struct cdev cdev;
	struct class *class;
	struct device *char_device;
	
	/* Status Flags */
	bool streaming;
	bool initialized;
	bool hw_ready;
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
 * PHẦN 6: I2C COMMUNICATION FUNCTIONS - WITH RETRY LOGIC
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
	int ret, retries = 3;
	
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;
	
	/* CRITICAL: Retry on NACK - sensor may need time to wake up */
	while (retries--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			return 0;
		
		if (ret == -ENXIO || ret == -EIO) {
			usleep_range(1000, 2000);
			continue;
		}
		
		dev_err(&client->dev, "Failed to write reg 0x%04x: %d\n", reg, ret);
		return ret;
	}
	
	return -EIO;
}

static int imx219_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	u8 buf[2];
	int ret, retries = 3;
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
	
	while (retries--) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			return 0;
		
		if (ret == -ENXIO || ret == -EIO) {
			usleep_range(1000, 2000);
			continue;
		}
		
		dev_err(&client->dev, "Failed to read reg 0x%04x: %d\n", reg, ret);
		return -EIO;
	}
	
	return -EIO;
}

/* ============================================================================
 * PHẦN 7: POWER MANAGEMENT - FIXED SEQUENCING
 * ============================================================================ */
static int imx219_power_on(struct unicam_device *dev)
{
	int ret;
	
	if (dev->sensor_powered) {
		unicam_info(dev->dev, "Sensor already powered on\n");
		return 0;
	}
	
	unicam_info(dev->dev, "=== Powering ON IMX219 Sensor ===\n");
	
	/* Step 1: Assert reset and power-down */
	if (dev->reset_gpio) {
		gpiod_set_value_cansleep(dev->reset_gpio, 0);
		unicam_info(dev->dev, "✓ Reset asserted (XCLR LOW)\n");
	}
	
	if (dev->pwdn_gpio) {
		gpiod_set_value_cansleep(dev->pwdn_gpio, 0);
		unicam_info(dev->dev, "✓ Power-down deasserted\n");
	}
	
	/* Step 2: Enable power supplies in sequence */
	if (dev->dvdd_regulator) {
		ret = regulator_enable(dev->dvdd_regulator);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable DVDD: %d\n", ret);
			return ret;
		}
		unicam_info(dev->dev, "✓ DVDD enabled (1.2V core)\n");
		usleep_range(500, 1000);
	}
	
	if (dev->avdd_regulator) {
		ret = regulator_enable(dev->avdd_regulator);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable AVDD: %d\n", ret);
			goto err_avdd;
		}
		unicam_info(dev->dev, "✓ AVDD enabled (2.8V analog)\n");
		usleep_range(500, 1000);
	}
	
	if (dev->dovdd_regulator) {
		ret = regulator_enable(dev->dovdd_regulator);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable DOVDD: %d\n", ret);
			goto err_dovdd;
		}
		unicam_info(dev->dev, "✓ DOVDD enabled (1.8V I/O)\n");
		usleep_range(500, 1000);
	}
	
	/* CRITICAL: Wait for power rail stabilization (IMX219 spec: >1ms) */
	msleep(5);
	
	/* Step 3: Enable sensor clock BEFORE releasing reset */
	if (dev->sensor_clk) {
		ret = clk_prepare_enable(dev->sensor_clk);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable sensor clock: %d\n", ret);
			goto err_clk;
		}
		unicam_info(dev->dev, "✓ Sensor clock enabled: %lu Hz\n",
					clk_get_rate(dev->sensor_clk));
		
		/* CRITICAL: Wait for clock to stabilize */
		usleep_range(1000, 2000);
	} else {
		unicam_warn(dev->dev, "⚠ No sensor clock - may not work!\n");
	}
	
	/* Step 4: Release reset ONLY after power and clock are stable */
	/* CRITICAL: IMX219 datasheet requires XCLR HIGH at least 0.5µs after power stable */
	if (dev->reset_gpio) {
		gpiod_set_value_cansleep(dev->reset_gpio, 1);
		unicam_info(dev->dev, "✓ Reset released (XCLR HIGH)\n");
	}
	
	/* Step 5: Wait for sensor internal initialization */
	/* IMX219 requires ~10ms after XCLR release before I2C is ready */
	msleep(20);
	
	dev->sensor_powered = true;
	unicam_info(dev->dev, "✓✓✓ Sensor powered ON - I2C ready! ✓✓✓\n");
	
	return 0;

err_clk:
	if (dev->dovdd_regulator)
		regulator_disable(dev->dovdd_regulator);
err_dovdd:
	if (dev->avdd_regulator)
		regulator_disable(dev->avdd_regulator);
err_avdd:
	if (dev->dvdd_regulator)
		regulator_disable(dev->dvdd_regulator);
	
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
	
	/* Disable power supplies in reverse order */
	if (dev->dovdd_regulator)
		regulator_disable(dev->dovdd_regulator);
	if (dev->avdd_regulator)
		regulator_disable(dev->avdd_regulator);
	if (dev->dvdd_regulator)
		regulator_disable(dev->dvdd_regulator);
	
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
	
	/* Verify sensor is responding */
	ret = imx219_read_reg(client, IMX219_REG_MODEL_ID_H, &model_id_h);
	if (ret) {
		unicam_err(dev->dev, "Cannot read sensor - check power/I2C!\n");
		return ret;
	}
	
	ret = imx219_read_reg(client, IMX219_REG_MODEL_ID_L, &model_id_l);
	if (ret)
		return ret;
	
	if ((model_id_h << 8 | model_id_l) != IMX219_MODEL_ID) {
		unicam_err(dev->dev, "Invalid sensor model ID: 0x%02x%02x\n",
				   model_id_h, model_id_l);
		return -ENODEV;
	}
	
	unicam_info(dev->dev, "✓ IMX219 sensor verified (ID: 0x%04x)\n",
				model_id_h << 8 | model_id_l);
	
	/* Write Mode 7 configuration */
	for (i = 0; i < IMX219_MODE7_REG_COUNT; i++) {
		ret = imx219_write_reg(client,
							   imx219_mode7_640x480[i].addr,
							   imx219_mode7_640x480[i].val);
		if (ret) {
			unicam_err(dev->dev, "Failed to write register %d (0x%04x)\n", 
					   i, imx219_mode7_640x480[i].addr);
			return ret;
		}
		
		/* Small delay between register writes */
		if ((i % 10) == 0)
			usleep_range(100, 200);
	}
	
	unicam_info(dev->dev, "✓ IMX219 Mode 7 configuration complete\n");
	return 0;
}

static int imx219_start_streaming(struct unicam_device *dev)
{
	int ret;
	u8 mode;
	
	unicam_info(dev->dev, "Starting sensor streaming\n");
	
	/* Verify sensor is in standby before starting */
	ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODE_SELECT, &mode);
	if (ret == 0) {
		unicam_info(dev->dev, "Current mode: 0x%02x\n", mode);
		if (mode != IMX219_MODE_STANDBY) {
			unicam_warn(dev->dev, "Sensor not in standby, forcing standby\n");
			imx219_write_reg(dev->sensor_client, IMX219_REG_MODE_SELECT, 
						   IMX219_MODE_STANDBY);
			msleep(50);
		}
	}
	
	/* Start streaming */
	ret = imx219_write_reg(dev->sensor_client,
						   IMX219_REG_MODE_SELECT,
						   IMX219_MODE_STREAMING);
	
	if (ret) {
		unicam_err(dev->dev, "Failed to start streaming\n");
		return ret;
	}
	
	/* CRITICAL: Wait for sensor to start transmitting */
	msleep(200);
	
	/* Verify streaming started */
	ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODE_SELECT, &mode);
	if (ret == 0) {
		if (mode == IMX219_MODE_STREAMING) {
			unicam_info(dev->dev, "✓✓✓ Sensor streaming confirmed! ✓✓✓\n");
			
			/* Read some status registers for debug */
			u8 val1, val2;
			imx219_read_reg(dev->sensor_client, 0x01e0, &val1);
			imx219_read_reg(dev->sensor_client, 0x0740, &val2);
			unicam_info(dev->dev, "Sensor status 0x01e0=0x%02x, 0x0740=0x%02x\n",
						val1, val2);
		} else {
			unicam_err(dev->dev, "✗ Sensor NOT streaming (mode=0x%02x)\n", mode);
			return -EIO;
		}
	}
	
	return 0;
}

static int imx219_stop_streaming(struct unicam_device *dev)
{
	int ret;
	u8 mode;
	
	unicam_info(dev->dev, "Stopping sensor streaming\n");
	
	/* Stop sensor */
	ret = imx219_write_reg(dev->sensor_client,
						   IMX219_REG_MODE_SELECT,
						   IMX219_MODE_STANDBY);
	
	if (ret == 0) {
		/* Wait for sensor to stop completely */
		msleep(100);
		
		/* Verify sensor stopped */
		ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODE_SELECT, &mode);
		if (ret == 0) {
			if (mode == IMX219_MODE_STANDBY) {
				unicam_info(dev->dev, "✓ Sensor confirmed in standby mode\n");
			} else {
				unicam_warn(dev->dev, "⚠ Sensor still in mode: 0x%02x\n", mode);
			}
		}
	} else {
		unicam_err(dev->dev, "Failed to stop sensor: %d\n", ret);
	}
	
	return ret;
}

/* ============================================================================
 * PHẦN 9: UNICAM HARDWARE FUNCTIONS - CRITICAL FIXES
 * ============================================================================ */

/*
 * Configure D-PHY and prepare Unicam for capture
 */
static int unicam_hw_configure(struct unicam_device *dev)
{
	u32 val;
	
	dev_info(dev->dev, "=== Configuring Unicam Hardware ===\n");
	
	/* Step 1: Reset and disable controller */
	unicam_reg_write(dev, UNICAM_CTRL, 0);
	unicam_reg_write(dev, UNICAM_ISTA, 0xFFFFFFFF);  /* Clear all interrupts */
	msleep(10);
	
	/* Step 2: Configure CSI-2 D-PHY for 2 lanes */
	/* CRITICAL FIX: ANA register bits:
	 * Bit 0-1: Number of lanes (0=1, 1=2, 2=3, 3=4)
	 * Bit 2:   CSI mode (0=CSI2, 1=CCP2)
	 * Bit 4:   Force clock
	 * Bit 5:   Power down analog
	 */
	val = (MIPI_CSI2_LANES - 1) << 0;  /* Number of lanes */
	val |= (0 << 2);                    /* CSI2 mode */
	val |= (0 << 4);                    /* Don't force clock */
	val |= (0 << 5);                    /* Don't power down analog */
	unicam_reg_write(dev, UNICAM_ANA, val);
	dev_info(dev->dev, "✓ Configured ANA=0x%08x for %d CSI-2 lanes\n", 
			 val, MIPI_CSI2_LANES);
	
	/* Step 3: Configure DMA addresses */
	unicam_reg_write(dev, UNICAM_IBSA0, (u32)dev->bus_addr);
	unicam_reg_write(dev, UNICAM_IBEA0, (u32)(dev->bus_addr + dev->buf_size));
	dev_info(dev->dev, "✓ DMA buffer: 0x%08llx - 0x%08llx\n",
			 (u64)dev->bus_addr, (u64)(dev->bus_addr + dev->buf_size));
	
	/* Step 4: Configure image format */
	unicam_reg_write(dev, UNICAM_IBLS, FRAME_WIDTH * BYTES_PER_PIXEL);  /* Bytes per line */
	unicam_reg_write(dev, UNICAM_IHWIN, FRAME_WIDTH);
	unicam_reg_write(dev, UNICAM_IVWIN, FRAME_HEIGHT);
	dev_info(dev->dev, "✓ Image size: %dx%d, Bytes per line: %d\n", 
			 FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH * BYTES_PER_PIXEL);
	
	/* Step 5: Configure data type for RAW10 (0x2B) */
	unicam_reg_write(dev, UNICAM_IDI0, 0x2B2B2B2B);
	
	/* Step 6: Configure interrupts */
	/* Enable Frame End (FE) interrupt - bit 0 */
	/* Enable Frame Start (FS) interrupt - bit 1 for debugging */
	val = (1 << 0) | (1 << 1);
	unicam_reg_write(dev, UNICAM_ICTL, val);
	dev_info(dev->dev, "✓ Interrupts enabled: FE + FS (ICTL=0x%08x)\n", val);
	
	/* Step 7: Clear any pending interrupts */
	unicam_reg_write(dev, UNICAM_ISTA, 0xFFFFFFFF);
	
	/* Step 8: Configure packet filtering - accept all on VC0 */
	unicam_reg_write(dev, UNICAM_CLK, 0x00000000);  /* Virtual channel 0 */
	
	/* Step 9: Configure DPHY timing - CRITICAL! */
	/* DPHY timing configuration from Raspberry Pi firmware */
	unicam_reg_write(dev, UNICAM_DPHY_TIMING0, 0x0c000c00);
	unicam_reg_write(dev, UNICAM_DPHY_TIMING1, 0x00000000);
	
	dev->hw_ready = true;
	dev_info(dev->dev, "✓✓✓ Unicam hardware configured ✓✓✓\n");
	
	/* Dump register values for debug */
	dev_info(dev->dev, "Register dump after configuration:\n");
	dev_info(dev->dev, "  CTRL:  0x%08x\n", unicam_reg_read(dev, UNICAM_CTRL));
	dev_info(dev->dev, "  STA:   0x%08x\n", unicam_reg_read(dev, UNICAM_STA));
	dev_info(dev->dev, "  ANA:   0x%08x\n", unicam_reg_read(dev, UNICAM_ANA));
	dev_info(dev->dev, "  ICTL:  0x%08x\n", unicam_reg_read(dev, UNICAM_ICTL));
	dev_info(dev->dev, "  IBSA0: 0x%08x\n", unicam_reg_read(dev, UNICAM_IBSA0));
	
	return 0;
}

/*
 * Enable Unicam controller - call this AFTER sensor starts streaming
 */
static int unicam_hw_enable(struct unicam_device *dev)
{
	u32 val;
	int timeout = 100;  /* 100ms timeout */
	
	dev_info(dev->dev, "Enabling Unicam controller...\n");
	
	if (!dev->hw_ready) {
		dev_err(dev->dev, "Hardware not configured!\n");
		return -EINVAL;
	}
	
	/* Reset write pointer */
	unicam_reg_write(dev, UNICAM_IBWP, (u32)dev->bus_addr);
	
	/* Enable Unicam: ENABLE (bit 0) + MEM mode (bit 1) */
	val = (1 << 0) | (1 << 1);
	unicam_reg_write(dev, UNICAM_CTRL, val);
	
	dev_info(dev->dev, "✓ Unicam controller enabled (CTRL=0x%08x)\n", val);
	
	/* Wait for D-PHY to lock (check bit 8 of STA) */
	while (timeout-- > 0) {
		u32 sta = unicam_reg_read(dev, UNICAM_STA);
		if (sta & (1 << 8)) {  /* Check D-PHY lock bit */
			dev_info(dev->dev, "✓ D-PHY locked after %dms (STA=0x%08x)\n", 
					 100 - timeout, sta);
			break;
		}
		if ((timeout % 20) == 0) {
			dev_info(dev->dev, "Waiting for D-PHY lock... STA=0x%08x\n", sta);
		}
		msleep(1);
	}
	
	if (timeout <= 0) {
		dev_err(dev->dev, "✗ D-PHY failed to lock!\n");
		
		/* Dump all registers for debugging */
		dev_info(dev->dev, "=== Register Dump (D-PHY not locked) ===\n");
		dev_info(dev->dev, "CTRL:  0x%08x\n", unicam_reg_read(dev, UNICAM_CTRL));
		dev_info(dev->dev, "STA:   0x%08x\n", unicam_reg_read(dev, UNICAM_STA));
		dev_info(dev->dev, "ANA:   0x%08x\n", unicam_reg_read(dev, UNICAM_ANA));
		dev_info(dev->dev, "ICTL:  0x%08x\n", unicam_reg_read(dev, UNICAM_ICTL));
		dev_info(dev->dev, "ISTA:  0x%08x\n", unicam_reg_read(dev, UNICAM_ISTA));
		dev_info(dev->dev, "IBWP:  0x%08x\n", unicam_reg_read(dev, UNICAM_IBWP));
		dev_info(dev->dev, "DPHY_TIMING0: 0x%08x\n", unicam_reg_read(dev, UNICAM_DPHY_TIMING0));
		dev_info(dev->dev, "DPHY_TIMING1: 0x%08x\n", unicam_reg_read(dev, UNICAM_DPHY_TIMING1));
		
		unicam_hw_disable(dev);
		return -ETIMEDOUT;
	}
	
	dev_info(dev->dev, "✓ Unicam controller ENABLED and ready for frames\n");
	return 0;
}

static void unicam_hw_disable(struct unicam_device *dev)
{
	dev_info(dev->dev, "Disabling Unicam controller\n");
	
	/* Disable interrupts first */
	unicam_reg_write(dev, UNICAM_ICTL, 0);
	
	/* Disable core */
	unicam_reg_write(dev, UNICAM_CTRL, 0);
	
	/* Clear interrupts */
	unicam_reg_write(dev, UNICAM_ISTA, 0xFFFFFFFF);
	
	/* Reset buffer pointers */
	unicam_reg_write(dev, UNICAM_IBWP, (u32)dev->bus_addr);
	
	/* Reset frame counters */
	atomic_set(&dev->frame_count, 0);
	atomic_set(&dev->frame_ready, 0);
	
	dev->hw_ready = false;
	dev_info(dev->dev, "✓ Unicam fully disabled\n");
}

/* ============================================================================
 * PHẦN 10: INTERRUPT SERVICE ROUTINE
 * ============================================================================ */
static irqreturn_t unicam_isr(int irq, void *dev_id)
{
	struct unicam_device *dev = dev_id;
	u32 status, sta, ctrl;
	unsigned long flags;
	
	/* Read all relevant registers */
	status = unicam_reg_read(dev, UNICAM_ISTA);
	sta = unicam_reg_read(dev, UNICAM_STA);
	ctrl = unicam_reg_read(dev, UNICAM_CTRL);
	
	/* Check if this is our interrupt */
	if (!status) {
		return IRQ_NONE;
	}
	
	/* Debug: log first 5 interrupts */
	static int isr_count = 0;
	if (isr_count < 5) {
		dev_info(dev->dev, "ISR[%d]: ISTA=0x%08x, STA=0x%08x, CTRL=0x%08x\n",
				 isr_count, status, sta, ctrl);
		isr_count++;
	}
	
	spin_lock_irqsave(&dev->lock, flags);
	
	/* Frame End interrupt (bit 0) */
	if (status & 0x00000001) {
		u32 frame_num = atomic_inc_return(&dev->frame_count);
		u32 write_ptr = unicam_reg_read(dev, UNICAM_IBWP);
		
		dev_info(dev->dev, "✓ FRAME %u CAPTURED! IBWP=0x%08x\n",
				 frame_num, write_ptr);
		
		dev->current_write_ptr = write_ptr;
		atomic_set(&dev->frame_ready, 1);
		
		/* Wake up waiting processes */
		wake_up_interruptible(&dev->frame_wait);
	}
	
	/* Frame Start interrupt (bit 1) */
	if (status & 0x00000002) {
		dev_dbg(dev->dev, "Frame Start interrupt\n");
	}
	
	/* Clear all handled interrupts */
	unicam_reg_write(dev, UNICAM_ISTA, status);
	
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
	int ret;
	u32 frame_info[2];
	
	if (count < sizeof(frame_info))
		return -EINVAL;
	
	/* Wait for frame with timeout (5 seconds) */
	ret = wait_event_interruptible_timeout(dev->frame_wait,
			atomic_read(&dev->frame_ready) > 0,
			msecs_to_jiffies(5000));
	
	if (ret == 0) {
		return -ETIMEDOUT;  /* Timeout */
	} else if (ret < 0) {
		return ret;  /* Interrupted */
	}
	
	atomic_set(&dev->frame_ready, 0);
	
	/* Return frame information */
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
	unsigned int mask = 0;
	
	poll_wait(filp, &dev->frame_wait, wait);
	
	/* Data is available if frames have been captured */
	if (atomic_read(&dev->frame_ready))
		mask |= POLLIN | POLLRDNORM;
	
	return mask;
}

/* IOCTL Commands */
#define UNICAM_IOC_MAGIC 'U'
#define UNICAM_IOC_START_STREAM  _IO(UNICAM_IOC_MAGIC, 1)
#define UNICAM_IOC_STOP_STREAM   _IO(UNICAM_IOC_MAGIC, 2)
#define UNICAM_IOC_GET_BUFFER    _IOR(UNICAM_IOC_MAGIC, 3, unsigned long)
#define UNICAM_IOC_GET_STATUS    _IOR(UNICAM_IOC_MAGIC, 4, unsigned long)

static long unicam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct unicam_device *dev = filp->private_data;
	int ret = 0;
	u32 status_info[8];
	
	switch (cmd) {
	case UNICAM_IOC_START_STREAM:
		if (dev->streaming) {
				unicam_info(dev->dev, "Already streaming\n");
			break;
		}
		
		if (!dev->sensor_client) {
			unicam_err(dev->dev, "No sensor available\n");
			return -ENODEV;
		}
		
		/* CRITICAL: Configure hardware BEFORE starting sensor */
		ret = unicam_hw_configure(dev);
		if (ret) {
			unicam_err(dev->dev, "Failed to configure hardware\n");
			break;
		}
		
		/* Configure sensor (Mode 7) */
		unicam_info(dev->dev, "Configuring sensor (Mode 7)\n");
		ret = imx219_configure_mode7(dev);
		if (ret) {
			unicam_err(dev->dev, "Failed to configure sensor\n");
			break;
		}
		
		/* Enable Unicam receiver BEFORE Sensor starts streaming */
		unicam_info(dev->dev, "Enabling Unicam receiver\n");
		ret = unicam_hw_enable(dev);
		if (ret) {
			unicam_err(dev->dev, "Failed to enable Unicam\n");
			break;
		}
		
		/* Start sensor streaming */
		msleep(50); // Wait for Unicam to be ready
		ret = imx219_start_streaming(dev);
		if (ret) {
			unicam_hw_disable(dev); // Cleanup if sensor fails
			unicam_err(dev->dev, "Failed to start sensor streaming\n");
			break;
		}
		
		dev->streaming = true;
		atomic_set(&dev->frame_count, 0);
		atomic_set(&dev->frame_ready, 0);
		
		unicam_info(dev->dev, "\n");
		unicam_info(dev->dev, "╔════════════════════════════════════════════╗\n");
		unicam_info(dev->dev, "║  STREAMING STARTED SUCCESSFULLY!           ║\n");
		unicam_info(dev->dev, "║  Waiting for frames...                     ║\n");
		unicam_info(dev->dev, "╚════════════════════════════════════════════╝\n");
		break;

	case UNICAM_IOC_STOP_STREAM:
		if (!dev->streaming) {
			unicam_info(dev->dev, "Not streaming\n");
			break;
		}
		
		/* Stop sensor first */
		if (dev->sensor_client) {
			imx219_stop_streaming(dev);
		}
		
		/* Then disable hardware */
		unicam_hw_disable(dev);
		
		dev->streaming = false;
		
		unicam_info(dev->dev, "✓ Streaming stopped (Total frames: %d)\n",
					atomic_read(&dev->frame_count));
		break;
		
	case UNICAM_IOC_GET_BUFFER:
		if (copy_to_user((void __user *)arg, &dev->buf_size,
						 sizeof(dev->buf_size)))
			return -EFAULT;
		break;
	
	case UNICAM_IOC_GET_STATUS:
		/* Return comprehensive hardware status */
		status_info[0] = atomic_read(&dev->frame_count);
		status_info[1] = unicam_reg_read(dev, UNICAM_STA);
		status_info[2] = unicam_reg_read(dev, UNICAM_CTRL);
		status_info[3] = unicam_reg_read(dev, UNICAM_ISTA);
		status_info[4] = unicam_reg_read(dev, UNICAM_IBWP);
		status_info[5] = unicam_reg_read(dev, UNICAM_ANA);
		status_info[6] = dev->streaming ? 1 : 0;
		status_info[7] = dev->sensor_powered ? 1 : 0;
		
		if (copy_to_user((void __user *)arg, status_info,
						 sizeof(status_info)))
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
 * PHẦN 12: I2C ADAPTER DETECTION - ENHANCED FOR BCM2711
 * ============================================================================ */
static struct i2c_adapter *unicam_find_i2c_adapter(struct platform_device *pdev)
{
	struct device_node *i2c_node;
	struct i2c_adapter *adapter;
	int i;
	
	/* Method 1: Get from device tree (preferred) */
	i2c_node = of_parse_phandle(pdev->dev.of_node, "i2c-bus", 0);
	if (i2c_node) {
		dev_info(&pdev->dev, "Found I2C bus in device tree\n");
		adapter = of_find_i2c_adapter_by_node(i2c_node);
		of_node_put(i2c_node);
		if (adapter) {
			dev_info(&pdev->dev, "✓ Using DT I2C adapter: %s\n", adapter->name);
			return adapter;
		}
	}
	
	/* Method 2: Try common camera I2C buses on Pi 4 */
	dev_info(&pdev->dev, "Searching for camera I2C adapter...\n");
	
	int camera_buses[] = {10, 0, 22, 11};
	
	for (i = 0; i < ARRAY_SIZE(camera_buses); i++) {
		adapter = i2c_get_adapter(camera_buses[i]);
		if (adapter) {
			dev_info(&pdev->dev, "✓ Found I2C adapter %d: %s\n",
					 camera_buses[i], adapter->name);
			
			/* Check if this is the CSI/DSI mux */
			if (strstr(adapter->name, "csi") || 
			    strstr(adapter->name, "dsi") ||
			    strstr(adapter->name, "mux")) {
				dev_info(&pdev->dev, "✓✓ This is the camera I2C bus!\n");
				return adapter;
			}
			
			/* Otherwise keep as fallback */
			if (camera_buses[i] == 10) {
				return adapter;  /* i2c-10 is standard for CAM1 */
			}
			
			i2c_put_adapter(adapter);
		}
	}
	
	/* Method 3: Scan for any available adapter */
	dev_warn(&pdev->dev, "⚠ Camera I2C not found, scanning all...\n");
	for (i = 0; i < 32; i++) {
		adapter = i2c_get_adapter(i);
		if (adapter) {
			dev_info(&pdev->dev, "Found I2C-%d: %s\n", i, adapter->name);
			if (i == 10 || i == 0) {
				return adapter;  /* Likely camera bus */
			}
			i2c_put_adapter(adapter);
		}
	}
	
	return NULL;
}

/* ============================================================================
 * PHẦN 13: PLATFORM DRIVER PROBE - FIXED INITIALIZATION SEQUENCE
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
	
	dev_info(&pdev->dev, "\n");
	dev_info(&pdev->dev, "╔════════════════════════════════════════════╗\n");
	dev_info(&pdev->dev, "║  Manual Unicam Driver - Zero Frame Fix     ║\n");
	dev_info(&pdev->dev, "╚════════════════════════════════════════════╝\n");
	dev_info(&pdev->dev, "\n");
	
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
	atomic_set(&dev->frame_ready, 0);
	
	/* ===== PHASE 1: HARDWARE RESOURCES ===== */
	dev_info(&pdev->dev, "PHASE 1: Acquiring hardware resources\n");
	
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
	
	dev_info(&pdev->dev, "✓ IRQ number: %d\n", dev->irq);
	
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
		dev_info(&pdev->dev, "✓ Sensor clock available: %lu Hz\n",
				 clk_get_rate(dev->sensor_clk));
	} else {
		dev->sensor_clk = NULL;
		dev_warn(&pdev->dev, "⚠ Sensor clock not found (may need DT overlay)\n");
	}
	
	/* ===== PHASE 2: GPIO AND REGULATORS ===== */
	dev_info(&pdev->dev, "\nPHASE 2: Acquiring GPIO and power controls\n");
	
	/* Get reset GPIO (CRITICAL for sensor power-on) */
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
	} else {
		dev_warn(&pdev->dev, "⚠ No reset GPIO (check DT overlay)\n");
	}
	
	/* Get optional power-down GPIO */
	dev->pwdn_gpio = devm_gpiod_get_optional(&pdev->dev, "pwdn",
											  GPIOD_OUT_LOW);
	if (IS_ERR(dev->pwdn_gpio)) {
		dev->pwdn_gpio = NULL;
	}
	
	/* Get regulators (may be optional if always-on) */
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

	/* ===== PHASE 3: DMA MEMORY ===== */
	dev_info(&pdev->dev, "\nPHASE 3: Allocating DMA memory\n");
	
	/* Set DMA mask */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set DMA mask\n");
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
	
	dev_info(&pdev->dev, "✓ DMA buffer allocated\n");
	dev_info(&pdev->dev, "  Size:    %zu bytes (%d MB)\n",
			 dev->buf_size, (int)(dev->buf_size / (1024*1024)));
	dev_info(&pdev->dev, "  Virtual: %p\n", dev->virt_addr);
	dev_info(&pdev->dev, "  Bus:     0x%08llx\n", (u64)dev->bus_addr);
	dev_info(&pdev->dev, "  Frames:  %d max\n", MAX_FRAMES);
	
	/* ===== PHASE 4: I2C AND SENSOR ===== */
	dev_info(&pdev->dev, "\nPHASE 4: Connecting to sensor via I2C\n");
	
	/* Find I2C adapter */
	adapter = unicam_find_i2c_adapter(pdev);
	
	if (!adapter) {
		dev_err(&pdev->dev, "✗ Failed to find I2C adapter\n");
		dev_err(&pdev->dev, "✗ Check Device Tree overlay is loaded!\n");
		dev_err(&pdev->dev, "✗ Expected: dtoverlay=imx219 in config.txt\n");
		dev->sensor_client = NULL;
		goto skip_sensor;
	}
	
	dev_info(&pdev->dev, "✓ Using I2C adapter: %s\n", adapter->name);
	
	/* Create I2C client */
	dev->sensor_client = i2c_new_client_device(adapter, &board_info);
	i2c_put_adapter(adapter);
	
	if (IS_ERR(dev->sensor_client)) {
		dev_err(&pdev->dev, "✗ Failed to create I2C client: %ld\n",
				PTR_ERR(dev->sensor_client));
		dev->sensor_client = NULL;
		goto skip_sensor;
	}
	
	dev_info(&pdev->dev, "✓ I2C client created (addr: 0x%02x)\n",
			 dev->sensor_client->addr);

	/* Power on sensor */
	dev_info(&pdev->dev, "\nAttempting sensor power-on sequence...\n");
	ret = imx219_power_on(dev);
	if (ret) {
		dev_err(&pdev->dev, "✗ Sensor power-on failed: %d\n", ret);
		dev_err(&pdev->dev, "✗ Check:\n");
		dev_err(&pdev->dev, "✗   - Camera cable connected correctly\n");
		dev_err(&pdev->dev, "✗   - Blue tab facing HDMI ports\n");
		dev_err(&pdev->dev, "✗   - GPIO expander configured in DT\n");
		i2c_unregister_device(dev->sensor_client);
		dev->sensor_client = NULL;
		goto skip_sensor;
	}
	
	/* Verify sensor communication */
	dev_info(&pdev->dev, "Verifying sensor communication...\n");
	ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODEL_ID_H, &model_id_h);
	if (ret == 0) {
		ret = imx219_read_reg(dev->sensor_client, IMX219_REG_MODEL_ID_L, &model_id_l);
	}
	
	if (ret == 0) {
		u16 model_id = (model_id_h << 8) | model_id_l;
		if (model_id == IMX219_MODEL_ID) {
			dev_info(&pdev->dev, "\n");
			dev_info(&pdev->dev, "✓✓✓ IMX219 SENSOR DETECTED! ✓✓✓\n");
			dev_info(&pdev->dev, "✓✓✓ Model ID: 0x%04x ✓✓✓\n", model_id);
			dev_info(&pdev->dev, "\n");
		} else {
			dev_warn(&pdev->dev, "⚠ Unexpected model ID: 0x%04x\n", model_id);
		}
	} else {
		dev_warn(&pdev->dev, "⚠ Cannot read sensor registers\n");
		dev_warn(&pdev->dev, "⚠ I2C communication may be unstable\n");
		dev_warn(&pdev->dev, "⚠ Check I2C bus muxing (should be i2c-10 or i2c-0)\n");
	}

skip_sensor:
	/* ===== PHASE 5: IRQ SETUP ===== */
	dev_info(&pdev->dev, "\nPHASE 5: Setting up interrupt handler\n");
	
	/* Request IRQ */
	ret = devm_request_irq(&pdev->dev, dev->irq, unicam_isr,
						   IRQF_SHARED, DRIVER_NAME, dev);
	if (ret) {
		dev_err(&pdev->dev, "✗ Failed to request IRQ %d: %d\n",
				dev->irq, ret);
		goto err_i2c;
	}
	
	dev_info(&pdev->dev, "✓ IRQ %d registered with IRQF_SHARED\n", dev->irq);
	
	/* ===== PHASE 6: CHARACTER DEVICE ===== */
	dev_info(&pdev->dev, "\nPHASE 6: Creating character device\n");
	
	ret = alloc_chrdev_region(&dev->devt, 0, 1, DEVICE_NAME);
	if (ret) {
		dev_err(&pdev->dev, "✗ Failed to allocate char device region\n");
		goto err_i2c;
	}
	
	cdev_init(&dev->cdev, &unicam_fops);
	dev->cdev.owner = THIS_MODULE;
	
	ret = cdev_add(&dev->cdev, dev->devt, 1);
	if (ret) {
		dev_err(&pdev->dev, "✗ Failed to add char device\n");
		goto err_chrdev;
	}
	
	/* Create device class */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
	dev->class = class_create(DRIVER_NAME);
#else
	dev->class = class_create(THIS_MODULE, DRIVER_NAME);
#endif
	
	if (IS_ERR(dev->class)) {
		ret = PTR_ERR(dev->class);
		goto err_cdev;
	}
	
	/* Create device node */
	dev->char_device = device_create(dev->class, &pdev->dev, dev->devt,
									 NULL, DEVICE_NAME);
	if (IS_ERR(dev->char_device)) {
		ret = PTR_ERR(dev->char_device);
		goto err_class;
	}
	
	dev_info(&pdev->dev, "✓ Device node: /dev/%s\n", DEVICE_NAME);
	
	dev->initialized = true;
	
	/* ===== FINAL STATUS ===== */
	dev_info(&pdev->dev, "\n");
	dev_info(&pdev->dev, "╔════════════════════════════════════════════╗\n");
	dev_info(&pdev->dev, "║  Driver Initialization Complete!           ║\n");
	dev_info(&pdev->dev, "╠════════════════════════════════════════════╣\n");
	dev_info(&pdev->dev, "║  Device:  /dev/%s                      ║\n", DEVICE_NAME);
	dev_info(&pdev->dev, "║  Buffer:  %d MB                         ║\n",
			 (int)(DMA_BUFFER_SIZE / (1024*1024)));
	dev_info(&pdev->dev, "║  Sensor:  %s ║\n",
			 dev->sensor_client ? "IMX219 Ready         " : "Not detected         ");
	dev_info(&pdev->dev, "║  I2C:     %s ║\n",
			 dev->sensor_client ? "Connected            " : "Not available        ");
	dev_info(&pdev->dev, "║  Status:  Ready for streaming              ║\n");
	dev_info(&pdev->dev, "╚════════════════════════════════════════════╝\n");
	dev_info(&pdev->dev, "\n");
	
	if (!dev->sensor_client) {
		dev_warn(&pdev->dev, "⚠⚠⚠ WARNING ⚠⚠⚠\n");
		dev_warn(&pdev->dev, "Sensor not detected - streaming will fail\n");
		dev_warn(&pdev->dev, "Troubleshooting:\n");
		dev_warn(&pdev->dev, "  1. Check camera cable connection\n");
		dev_warn(&pdev->dev, "  2. Verify dtoverlay=imx219 in /boot/config.txt\n");
		dev_warn(&pdev->dev, "  3. Ensure start_x=0 (not using legacy stack)\n");
		dev_warn(&pdev->dev, "  4. Run: i2cdetect -y 10 (should see 0x10)\n");
		dev_warn(&pdev->dev, "\n");
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
	
	dev_info(&pdev->dev, "✓ Driver removed successfully\n");
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
		.of_match_table= unicam_of_match,
	},
};

module_platform_driver(unicam_driver);

MODULE_AUTHOR("Embedded Vision Systems");
MODULE_DESCRIPTION("Manual DMA Driver for BCM2711 Unicam and IMX219");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.2");
