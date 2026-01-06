/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * BCM2835/BCM2711 Unicam Register Definitions
 * Manual DMA Driver for Raspberry Pi 4B
 */

#ifndef _BCM2835_UNICAM_REGS_H_
#define _BCM2835_UNICAM_REGS_H_

#include <linux/bits.h>

/* 
 * BCM2711 Unicam Base Addresses
 * Legacy address 0x7e801000 maps to physical 0xfe801000
 */

/* Register Offsets (add to base address after ioremap) */
#define UNICAM_CTRL             0x000  /* Control Register */
#define UNICAM_STA              0x004  /* Status Register */
#define UNICAM_ANA              0x008  /* Analog Control */
#define UNICAM_PRI              0x00c  /* Priority Control */
#define UNICAM_CLK              0x010  /* Clock Control */
#define UNICAM_CLT              0x014  /* Clock Timing */
#define UNICAM_DAT0             0x018  /* Data Lane 0 Timing */
#define UNICAM_DAT1             0x01c  /* Data Lane 1 Timing */
#define UNICAM_DAT2             0x020  /* Data Lane 2 Timing */
#define UNICAM_DAT3             0x024  /* Data Lane 3 Timing */
#define UNICAM_DLT              0x028  /* Data Lane Timing */
#define UNICAM_CMP0             0x02c  /* Compare 0 */
#define UNICAM_CMP1             0x030  /* Compare 1 */
#define UNICAM_CAP0             0x034  /* Capture 0 */
#define UNICAM_CAP1             0x038  /* Capture 1 */

/* Image/Embedded Data Registers */
#define UNICAM_ICTL             0x100  /* Image Control */
#define UNICAM_ISTA             0x104  /* Image Status */
#define UNICAM_IDI0             0x108  /* Image Data ID 0 */
#define UNICAM_IPIPE            0x10c  /* Image Pipe Control */
#define UNICAM_IBSA0            0x110  /* Image Buffer Start Address 0 */
#define UNICAM_IBEA0            0x114  /* Image Buffer End Address 0 */
#define UNICAM_IBLS             0x118  /* Image Buffer Line Stride */
#define UNICAM_IBWP             0x11c  /* Image Buffer Write Pointer */
#define UNICAM_IHWIN            0x120  /* Image Horizontal Window */
#define UNICAM_IVWIN            0x124  /* Image Vertical Window */
#define UNICAM_IHSTA            0x128  /* Image Horizontal Start */
#define UNICAM_IVSTA            0x12c  /* Image Vertical Start */
#define UNICAM_ICC              0x130  /* Image Compression Control */
#define UNICAM_ICS              0x134  /* Image Compression Status */
#define UNICAM_IDC              0x138  /* Image Decompression Control */

/* Data Lane Registers */
#define UNICAM_DCS              0x200  /* Data Control/Status */
#define UNICAM_DBSA0            0x210  /* Data Buffer Start Address 0 */
#define UNICAM_DBEA0            0x214  /* Data Buffer End Address 0 */
#define UNICAM_DBWP             0x218  /* Data Buffer Write Pointer */
#define UNICAM_DBCTL            0x300  /* Data Buffer Control */
#define UNICAM_MISC             0x400  /* Miscellaneous Control */

/* UNICAM_CTRL Register Bits */
#define UNICAM_CPE              BIT(0)   /* Core Peripheral Enable */
#define UNICAM_HPE              BIT(1)   /* Header Peripheral Enable */
#define UNICAM_SOE              BIT(2)   /* Start Of Frame Enable */
#define UNICAM_DCM              BIT(3)   /* DMA Control Mode */
#define UNICAM_SLS              BIT(4)   /* Strobe Length Select */
#define UNICAM_PFT_MASK         0x000000e0
#define UNICAM_PFT_SHIFT        5
#define UNICAM_OET_MASK         0x00700000
#define UNICAM_OET_SHIFT        20

/* UNICAM_STA Register Bits */
#define UNICAM_SYN              BIT(0)   /* Synchronized */
#define UNICAM_CS               BIT(1)   /* Clock Stable */
#define UNICAM_SBE              BIT(2)   /* Start of Block Error */
#define UNICAM_PBE              BIT(3)   /* Packet Boundary Error */
#define UNICAM_HOE              BIT(4)   /* Header Offset Error */
#define UNICAM_PLE              BIT(5)   /* Packet Length Error */
#define UNICAM_SSC              BIT(6)   /* Short Synchronization Code */
#define UNICAM_CRCE             BIT(7)   /* CRC Error */
#define UNICAM_OES              BIT(8)   /* Output Enable Status */
#define UNICAM_IFO              BIT(9)   /* Image FIFO Overflow */
#define UNICAM_OFO              BIT(10)  /* Output FIFO Overflow */
#define UNICAM_BFO              BIT(11)  /* Block FIFO Overflow */
#define UNICAM_DL               BIT(12)  /* Data Lane */
#define UNICAM_PS               BIT(13)  /* Peripheral Status */
#define UNICAM_IS               BIT(14)  /* Interrupt Status */
#define UNICAM_PI0              BIT(15)  /* Priority Interrupt 0 */
#define UNICAM_PI1              BIT(16)  /* Priority Interrupt 1 */

/* UNICAM_ANA Register Bits */
#define UNICAM_APD              BIT(0)   /* Analog Power Down */
#define UNICAM_BPD              BIT(1)   /* Bandgap Power Down */
#define UNICAM_AR               BIT(2)   /* Analog Reset */
#define UNICAM_DDL              BIT(3)   /* Double Data Lane */
#define UNICAM_PTATADJ_MASK     0x000000f0
#define UNICAM_PTATADJ_SHIFT    4

/* UNICAM_PRI Register - Priority Control */
#define UNICAM_PE               BIT(0)   /* Priority Enable */
#define UNICAM_PT_MASK          0x000000fe
#define UNICAM_PT_SHIFT         1
#define UNICAM_NP_MASK          0x00007f00
#define UNICAM_NP_SHIFT         8
#define UNICAM_PP_MASK          0x007f8000
#define UNICAM_PP_SHIFT         15
#define UNICAM_BS_MASK          0x00800000
#define UNICAM_BS_SHIFT         23
#define UNICAM_BL_MASK          0xff000000
#define UNICAM_BL_SHIFT         24

/* UNICAM_CLK Register - Clock Control */
#define UNICAM_CLE              BIT(0)   /* Clock Lane Enable */
#define UNICAM_CLPD             BIT(1)   /* Clock Lane Power Down */
#define UNICAM_CLLPE            BIT(2)   /* Clock Lane Low Power Enable */
#define UNICAM_CLTRE            BIT(3)   /* Clock Lane Termination Enable */
#define UNICAM_CLHSE            BIT(4)   /* Clock Lane High Speed Enable */
#define UNICAM_CLSTE            BIT(29)  /* Clock Lane Stop State Enable */

/* UNICAM_ICTL Register - Image Control & Interrupt Enable */
#define UNICAM_FSIE             BIT(0)   /* Frame Start Interrupt Enable */
#define UNICAM_FEIE             BIT(1)   /* Frame End Interrupt Enable */
#define UNICAM_IBOB             BIT(2)   /* Image Buffer Overrun Block */
#define UNICAM_FCM              BIT(3)   /* Frame Capture Mode */
#define UNICAM_TFC              BIT(4)   /* Trigger Frame Capture */
#define UNICAM_LIP_MASK         0x000007e0
#define UNICAM_LIP_SHIFT        5
#define UNICAM_LCIE_MASK        0x001ff800
#define UNICAM_LCIE_SHIFT       11

/* UNICAM_ISTA Register - Image Status & Interrupt Flags */
#define UNICAM_FSI              BIT(0)   /* Frame Start Interrupt */
#define UNICAM_FEI              BIT(1)   /* Frame End Interrupt */
#define UNICAM_LCI              BIT(2)   /* Line Capture Interrupt */

/* UNICAM_IDI0 Register - Image Data Identifier */
#define UNICAM_ID0_MASK         0x000000ff /* Data Type */
#define UNICAM_ID1_MASK         0x0000ff00 /* Virtual Channel + Data Type */
#define UNICAM_ID2_MASK         0x00ff0000
#define UNICAM_ID3_MASK         0xff000000

/* CSI-2 Data Types (for IDI0 register) */
#define UNICAM_DT_YUV420_8      0x18
#define UNICAM_DT_YUV420_10     0x19
#define UNICAM_DT_YUV422_8      0x1e
#define UNICAM_DT_YUV422_10     0x1f
#define UNICAM_DT_RGB444        0x20
#define UNICAM_DT_RGB555        0x21
#define UNICAM_DT_RGB565        0x22
#define UNICAM_DT_RGB666        0x23
#define UNICAM_DT_RGB888        0x24
#define UNICAM_DT_RAW6          0x28
#define UNICAM_DT_RAW7          0x29
#define UNICAM_DT_RAW8          0x2a
#define UNICAM_DT_RAW10         0x2b
#define UNICAM_DT_RAW12         0x2c
#define UNICAM_DT_RAW14         0x2d

/* UNICAM_IPIPE Register - Image Pipe Control */
#define UNICAM_PUM_MASK         0x00000007 /* Packing/Unpacking Mode */
#define UNICAM_PUM_SHIFT        0
#define UNICAM_PUM_NONE         0
#define UNICAM_PUM_UNPACK       1
#define UNICAM_PUM_PACK         2
#define UNICAM_DEM_MASK         0x00000018 /* Demux Mode */
#define UNICAM_DEM_SHIFT        3
#define UNICAM_DEBL_MASK        0x000000e0 /* Debayer Line */
#define UNICAM_DEBL_SHIFT       5
#define UNICAM_ICM_MASK         0x00000300 /* Image Conversion Mode */
#define UNICAM_ICM_SHIFT        8
#define UNICAM_IDM_MASK         0x00000c00 /* Image Decimation Mode */
#define UNICAM_IDM_SHIFT        10

/* UNICAM_MISC Register */
#define UNICAM_FL0              BIT(6)   /* Frame Length 0 */
#define UNICAM_FL1              BIT(9)   /* Frame Length 1 */

/* Helper Macros */
#define UNICAM_REG_READ(base, reg)       readl((base) + (reg))
#define UNICAM_REG_WRITE(base, reg, val) writel((val), (base) + (reg))

#endif /* _BCM2835_UNICAM_REGS_H_ */