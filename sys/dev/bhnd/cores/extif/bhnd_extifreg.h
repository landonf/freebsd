/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright 2005, Broadcom Corporation      
 * All Rights Reserved.
 *
 * This file is derived from the sbextif.h header distributed with the
 * Asus WL-500g firmware source code release.
 *       
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY      
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM      
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS      
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $FreeBSD$
 */
 
/*
 * Hardware-specific External Interface I/O core definitions
 * for the BCM47xx family of SiliconBackplane-based chips.
 *
 * The External Interface core supports a total of three external chip selects
 * supporting external interfaces. One of the external chip selects is
 * used for Flash, one is used for PCMCIA, and the other may be
 * programmed to support either a synchronous interface or an
 * asynchronous interface. The asynchronous interface can be used to
 * support external devices such as UARTs and the BCM2019 Bluetooth
 * baseband processor.
 * The external interface core also contains 2 on-chip 16550 UARTs, clock
 * frequency control, a watchdog interrupt timer, and a GPIO interface.
 */

#ifndef _BHND_CORES_EXTIF_BHND_EXTIFREG_H_
#define	_BHND_CORES_EXTIF_BHND_EXTIFREG_H_

/* external interface address space */
#define	EXTIF_PCMCIA_MEMBASE(x)	(x)
#define	EXTIF_PCMCIA_IOBASE(x)	((x) + 0x100000)
#define	EXTIF_PCMCIA_CFGBASE(x)	((x) + 0x200000)
#define	EXTIF_CFGIF_BASE(x)	((x) + 0x800000)
#define	EXTIF_FLASH_BASE(x)	((x) + 0xc00000)

#define	EXTIF_CORECONTROL		0x0
#define	EXTIF_EXTSTATUS		0x4

/* pcmcia control registers */
#define	EXTIF_PCMCIA_CONFIG	0x10
#define	EXTIF_PCMCIA_MEMWAIT	0x14
#define	EXTIF_PCMCIA_ATTRWAIT	0x18
#define	EXTIF_PCMCIA_IOWAIT	0x1C

/* programmable interface control registers */
#define	EXTIF_PROG_CONFIG	0x20
#define	EXTIF_PROG_WAITCOUNT	0x24

/* flash control registers */
#define	EXTIF_FLASH_CONFIG	0x28
#define	EXTIF_FLASH_WAITCOUNT	0x2C

#define	EXTIF_WATCHDOG		0x40

/* clock control */
#define	EXTIF_CLOCKCONTROL_N	0x44
#define	EXTIF_CLOCKCONTROL_SB	0x48
#define	EXTIF_CLOCKCONTROL_PCI	0x4C
#define	EXTIF_CLOCKCONTROL_MII	0x50

/* clock control (chipc-compatible naming) */
#define	EXTIF_CLKC_N			EXTIF_CLOCKCONTROL_N
#define	EXTIF_CLKC_SB			EXTIF_CLOCKCONTROL_SB
#define	EXTIF_CLKC_PCI			EXTIF_CLOCKCONTROL_PCI
#define	EXTIF_CLKC_M2			EXTIF_CLOCKCONTROL_MII

/* gpio
 * 
 * Five instances of output and output enable registers
 * are present to allow driver software for multiple cores to control
 * gpio outputs without needing to share a single register pair.
 */
#define	EXTIF_GPIOIN		0x60

#define	EXTIF_GPIO_OUT		0x64
#define	EXTIF_GPIO_OUT_EN	0x68
#define	EXTIF_GPIO_OUT_PAIRS	5

#define	EXTIF_EJTAGOUTEN	0x90
#define	EXTIF_GPIOINTPOLARITY	0x94
#define	EXTIF_GPIOINTMASK	0x98

#define	EXTIF_UART_DATA		0x300
#define	EXTIF_UART_TIMER	0x310
#define	EXTIF_UART_FCR		0x320
#define	EXTIF_UART_LCR		0x330
#define	EXTIF_UART_MCR		0x340
#define	EXTIF_UART_LSR		0x360
#define	EXTIF_UART_MSR		0x370
#define	EXTIF_UART_SCRATCH	0x380

/* corecontrol */
#define	EXTIF_CC_UE		(1 << 0)		/* uart enable */

/* extstatus */
#define	EXTIF_ES_EM		(1 << 0)		/* endian mode (ro) */
#define	EXTIF_ES_EI		(1 << 1)		/* external interrupt pin (ro) */
#define	EXTIF_ES_GI		(1 << 2)		/* gpio interrupt pin (ro) */

/* gpio bit mask */
#define	EXTIF_GPIO_BIT0	(1 << 0)
#define	EXTIF_GPIO_BIT1	(1 << 1)
#define	EXTIF_GPIO_BIT2	(1 << 2)
#define	EXTIF_GPIO_BIT3	(1 << 3)
#define	EXTIF_GPIO_BIT4	(1 << 4)
#define	EXTIF_GPIO_BIT5	(1 << 5)
#define	EXTIF_GPIO_BIT6	(1 << 6)
#define	EXTIF_GPIO_BIT7	(1 << 7)

/* pcmcia/prog/flash_config */
#define	EXTIF_CF_EN		(1 << 0)	/* enable */
#define	EXTIF_CF_EM_MASK	0xe		/* mode */
#define	EXTIF_CF_EM_SHIFT	1
#define	EXTIF_CF_EM_FLASH	0x0		/* flash/asynchronous mode */
#define	EXTIF_CF_EM_SYNC	0x2		/* synchronous mode */
#define	EXTIF_CF_EM_PCMCIA	0x4		/* pcmcia mode */
#define	EXTIF_CF_DS		(1 << 4)	/* destsize:  0=8bit, 1=16bit */
#define	EXTIF_CF_BS		(1 << 5)	/* byteswap */
#define	EXTIF_CF_CD_MASK	0xc0		/* clock divider */
#define	EXTIF_CF_CD_SHIFT	6
#define	EXTIF_CF_CD_DIV2	0x0		/* backplane/2 */
#define	EXTIF_CF_CD_DIV3	0x40		/* backplane/3 */
#define	EXTIF_CF_CD_DIV4	0x80		/* backplane/4 */
#define	EXTIF_CF_CE		(1 << 8)	/* clock enable */
#define	EXTIF_CF_SB		(1 << 9)	/* size/bytestrobe (synch only) */

/* pcmcia_memwait */
#define	EXTIF_PM_W0_MASK	0x3f		/* waitcount0 */
#define	EXTIF_PM_W1_MASK	0x1f00		/* waitcount1 */
#define	EXTIF_PM_W1_SHIFT	8
#define	EXTIF_PM_W2_MASK	0x1f0000	/* waitcount2 */
#define	EXTIF_PM_W2_SHIFT	16
#define	EXTIF_PM_W3_MASK	0x1f000000	/* waitcount3 */
#define	EXTIF_PM_W3_SHIFT	24

/* pcmcia_attrwait */
#define	EXTIF_PA_W0_MASK	0x3f		/* waitcount0 */
#define	EXTIF_PA_W1_MASK	0x1f00		/* waitcount1 */
#define	EXTIF_PA_W1_SHIFT	8
#define	EXTIF_PA_W2_MASK	0x1f0000	/* waitcount2 */
#define	EXTIF_PA_W2_SHIFT	16
#define	EXTIF_PA_W3_MASK	0x1f000000	/* waitcount3 */
#define	EXTIF_PA_W3_SHIFT	24

/* pcmcia_iowait */
#define	EXTIF_PI_W0_MASK	0x3f		/* waitcount0 */
#define	EXTIF_PI_W1_MASK	0x1f00		/* waitcount1 */
#define	EXTIF_PI_W1_SHIFT	8
#define	EXTIF_PI_W2_MASK	0x1f0000	/* waitcount2 */
#define	EXTIF_PI_W2_SHIFT	16
#define	EXTIF_PI_W3_MASK	0x1f000000	/* waitcount3 */
#define	EXTIF_PI_W3_SHIFT	24

/* prog_waitcount */
#define	EXTIF_PW_W0_MASK	0x0000001f	/* waitcount0 */
#define	EXTIF_PW_W1_MASK	0x00001f00	/* waitcount1 */
#define	EXTIF_PW_W1_SHIFT	8
#define	EXTIF_PW_W2_MASK	0x001f0000	/* waitcount2 */
#define	EXTIF_PW_W2_SHIFT	16
#define	EXTIF_PW_W3_MASK	0x1f000000	/* waitcount3 */
#define	EXTIF_PW_W3_SHIFT	24

#define	EXTIF_PW_W0		0x0000000c
#define	EXTIF_PW_W1		0x00000a00
#define	EXTIF_PW_W2		0x00020000
#define	EXTIF_PW_W3		0x01000000

/* flash_waitcount */
#define	EXTIF_FW_W0_MASK	0x1f		/* waitcount0 */
#define	EXTIF_FW_W1_MASK	0x1f00		/* waitcount1 */
#define	EXTIF_FW_W1_SHIFT	8
#define	EXTIF_FW_W2_MASK	0x1f0000	/* waitcount2 */
#define	EXTIF_FW_W2_SHIFT	16
#define	EXTIF_FW_W3_MASK	0x1f000000	/* waitcount3 */
#define	EXTIF_FW_W3_SHIFT	24

/* watchdog */
#define	EXTIF_WATCHDOG_CLOCK	48000000	/* Hz */

/* Clock control default values */
#define	EXTIF_CC_DEF_N		0x0009		/* Default values for bcm4710 */
#define	EXTIF_CC_DEF_100	0x04020011
#define	EXTIF_CC_DEF_33		0x11030011
#define	EXTIF_CC_DEF_25		0x11050011

#endif /* _BHND_CORES_EXTIF_BHND_EXTIFREG_H_ */
