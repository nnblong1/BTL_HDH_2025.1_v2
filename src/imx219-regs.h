/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sony IMX219 CMOS Image Sensor Register Definitions
 * For use with Manual DMA Unicam Driver
 */

#ifndef _IMX219_REGS_H_
#define _IMX219_REGS_H_

#include <linux/types.h>

/* I2C Device Address */
#define IMX219_I2C_ADDR         0x10

/* Basic Control Registers */
#define IMX219_REG_MODE_SELECT          0x0100
#define IMX219_MODE_STANDBY             0x00
#define IMX219_MODE_STREAMING           0x01

#define IMX219_REG_SW_RESET             0x0103
#define IMX219_SW_RESET                 0x01

/* Model ID Registers */
#define IMX219_REG_MODEL_ID_H           0x0000
#define IMX219_REG_MODEL_ID_L           0x0001
#define IMX219_MODEL_ID                 0x0219

/* CSI-2 Configuration */
#define IMX219_REG_CSI_LANE_MODE        0x0114
#define IMX219_CSI_2_LANE_MODE          0x01
#define IMX219_CSI_4_LANE_MODE          0x03

#define IMX219_REG_DPHY_CTRL            0x0128
#define IMX219_REG_EXCK_FREQ_H          0x012a
#define IMX219_REG_EXCK_FREQ_L          0x012b

/* Special CSI Access Registers */
#define IMX219_REG_CSI_ACCESS_0         0x30eb

/* Gain Control */
#define IMX219_REG_ANALOG_GAIN          0x0157
#define IMX219_ANALOG_GAIN_MIN          0x00
#define IMX219_ANALOG_GAIN_MAX          0xe0
#define IMX219_ANALOG_GAIN_DEFAULT      0x00

#define IMX219_REG_DIGITAL_GAIN_H       0x0158
#define IMX219_REG_DIGITAL_GAIN_L       0x0159

/* Exposure Control */
#define IMX219_REG_EXPOSURE_H           0x015a
#define IMX219_REG_EXPOSURE_L           0x015b

/* Frame Timing */
#define IMX219_REG_FRM_LENGTH_A_H       0x0160
#define IMX219_REG_FRM_LENGTH_A_L       0x0161
#define IMX219_REG_LINE_LENGTH_A_H      0x0162
#define IMX219_REG_LINE_LENGTH_A_L      0x0163

/* Cropping (Region of Interest) */
#define IMX219_REG_X_ADD_STA_A_H        0x0164
#define IMX219_REG_X_ADD_STA_A_L        0x0165
#define IMX219_REG_X_ADD_END_A_H        0x0166
#define IMX219_REG_X_ADD_END_A_L        0x0167
#define IMX219_REG_Y_ADD_STA_A_H        0x0168
#define IMX219_REG_Y_ADD_STA_A_L        0x0169
#define IMX219_REG_Y_ADD_END_A_H        0x016a
#define IMX219_REG_Y_ADD_END_A_L        0x016b

/* Output Size */
#define IMX219_REG_X_OUTPUT_SIZE_H      0x016c
#define IMX219_REG_X_OUTPUT_SIZE_L      0x016d
#define IMX219_REG_Y_OUTPUT_SIZE_H      0x016e
#define IMX219_REG_Y_OUTPUT_SIZE_L      0x016f

/* Binning Mode */
#define IMX219_REG_BINNING_MODE_H_A     0x0174
#define IMX219_REG_BINNING_MODE_V_A     0x0175
#define IMX219_BINNING_NONE             0x00
#define IMX219_BINNING_2X2_DIGITAL      0x01
#define IMX219_BINNING_2X2_ANALOG       0x03  /* Special high-speed mode */

/* Scaling */
#define IMX219_REG_SCALE_MODE           0x0170
#define IMX219_REG_SCALE_M_H            0x0171
#define IMX219_REG_SCALE_M_L            0x0172

/* Test Pattern */
#define IMX219_REG_TEST_PATTERN         0x0600
#define IMX219_TEST_PATTERN_DISABLE     0x00
#define IMX219_TEST_PATTERN_COLOR_BARS  0x01
#define IMX219_TEST_PATTERN_SOLID_COLOR 0x02
#define IMX219_TEST_PATTERN_GREY_COLOR  0x03
#define IMX219_TEST_PATTERN_PN9         0x04

/* Data Format */
#define IMX219_REG_CSI_DATA_FORMAT_A_H  0x018c
#define IMX219_REG_CSI_DATA_FORMAT_A_L  0x018d
#define IMX219_REG_CSI_DATA_FORMAT_B_H  0x018e
#define IMX219_REG_CSI_DATA_FORMAT_B_L  0x018f
#define IMX219_DATA_FORMAT_RAW8         0x0808
#define IMX219_DATA_FORMAT_RAW10        0x0a0a

/* Clock Configuration */
#define IMX219_REG_VTPXCK_DIV           0x0301
#define IMX219_REG_VTSYCK_DIV           0x0303
#define IMX219_REG_PREPLLCK_VT_DIV      0x0304
#define IMX219_REG_PREPLLCK_OP_DIV      0x0305
#define IMX219_REG_PLL_VT_MPY_H         0x0306
#define IMX219_REG_PLL_VT_MPY_L         0x0307
#define IMX219_REG_OPPXCK_DIV           0x0309
#define IMX219_REG_OPSYCK_DIV           0x030b
#define IMX219_REG_PLL_OP_MPY_H         0x030c
#define IMX219_REG_PLL_OP_MPY_L         0x030d

/* Internal Clock Settings */
#define IMX219_REG_CLK_SETTING_0        0x300a
#define IMX219_REG_CLK_SETTING_1        0x300b

/* Sensor Information */
#define IMX219_NATIVE_WIDTH             3280
#define IMX219_NATIVE_HEIGHT            2464
#define IMX219_PIXEL_ARRAY_LEFT         8
#define IMX219_PIXEL_ARRAY_TOP          8
#define IMX219_PIXEL_ARRAY_WIDTH        3280
#define IMX219_PIXEL_ARRAY_HEIGHT       2464

/* Mode 7 Configuration Structure (640x480 @ 200fps) */
struct imx219_reg {
	u16 addr;
	u8 val;
};

/* Mode 7: 640x480, RAW10, 2-lane, 2x2 Analog Binning, ~200fps */
static const struct imx219_reg imx219_mode7_640x480[] = {
	/* Stop streaming */
	{IMX219_REG_MODE_SELECT, IMX219_MODE_STANDBY},
	
	/* CSI-2 Access Configuration */
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x30eb, 0x05},
	{0x30eb, 0x09},
	
	/* CSI-2 Lane Configuration */
	{IMX219_REG_CSI_LANE_MODE, IMX219_CSI_2_LANE_MODE},
	
	/* D-PHY Control */
	{IMX219_REG_DPHY_CTRL, 0x00},
	
	/* External Clock Frequency (24 MHz) */
	{IMX219_REG_EXCK_FREQ_H, 0x18},
	{IMX219_REG_EXCK_FREQ_L, 0x00},
	
	/* Analog Gain (Default) */
	{IMX219_REG_ANALOG_GAIN, IMX219_ANALOG_GAIN_DEFAULT},
	
	/* Frame Length (determines FPS) */
	{IMX219_REG_FRM_LENGTH_A_H, 0x01},
	{IMX219_REG_FRM_LENGTH_A_L, 0x11},  /* 0x0111 = 273 lines */
	
	/* Line Length */
	{IMX219_REG_LINE_LENGTH_A_H, 0x0d},
	{IMX219_REG_LINE_LENGTH_A_L, 0xe8},  /* 0x0de8 = 3560 pixels */
	
	/* Cropping Configuration */
	{IMX219_REG_X_ADD_STA_A_H, 0x03},
	{IMX219_REG_X_ADD_STA_A_L, 0xe8},    /* X start = 1000 */
	{IMX219_REG_X_ADD_END_A_H, 0x08},
	{IMX219_REG_X_ADD_END_A_L, 0xe7},    /* X end = 2279 */
	{IMX219_REG_Y_ADD_STA_A_H, 0x02},
	{IMX219_REG_Y_ADD_STA_A_L, 0xf0},    /* Y start = 752 */
	{IMX219_REG_Y_ADD_END_A_H, 0x06},
	{IMX219_REG_Y_ADD_END_A_L, 0xaf},    /* Y end = 1711 */
	
	/* Output Size (640x480) */
	{IMX219_REG_X_OUTPUT_SIZE_H, 0x02},
	{IMX219_REG_X_OUTPUT_SIZE_L, 0x80},  /* 640 */
	{IMX219_REG_Y_OUTPUT_SIZE_H, 0x01},
	{IMX219_REG_Y_OUTPUT_SIZE_L, 0xe0},  /* 480 */
	
	/* 2x2 Analog Binning (CRITICAL for high FPS) */
	{IMX219_REG_BINNING_MODE_H_A, IMX219_BINNING_2X2_ANALOG},
	{IMX219_REG_BINNING_MODE_V_A, IMX219_BINNING_2X2_ANALOG},
	
	/* CSI Data Format (RAW10) */
	{IMX219_REG_CSI_DATA_FORMAT_A_H, 0x0a},
	{IMX219_REG_CSI_DATA_FORMAT_A_L, 0x0a},
	{IMX219_REG_CSI_DATA_FORMAT_B_H, 0x0a},
	{IMX219_REG_CSI_DATA_FORMAT_B_L, 0x0a},
	
	/* PLL Configuration for Video Timing */
	{IMX219_REG_VTPXCK_DIV, 0x05},
	{IMX219_REG_VTSYCK_DIV, 0x01},
	{IMX219_REG_PREPLLCK_VT_DIV, 0x03},
	{IMX219_REG_PREPLLCK_OP_DIV, 0x03},
	{IMX219_REG_PLL_VT_MPY_H, 0x00},
	{IMX219_REG_PLL_VT_MPY_L, 0x39},     /* PLL multiplier = 57 */
	{IMX219_REG_OPPXCK_DIV, 0x0a},
	{IMX219_REG_OPSYCK_DIV, 0x01},
	{IMX219_REG_PLL_OP_MPY_H, 0x00},
	{IMX219_REG_PLL_OP_MPY_L, 0x39},
	
	/* Start streaming */
	{IMX219_REG_MODE_SELECT, IMX219_MODE_STREAMING},
};

#define IMX219_MODE7_REG_COUNT ARRAY_SIZE(imx219_mode7_640x480)

#endif /* _IMX219_REGS_H_ */