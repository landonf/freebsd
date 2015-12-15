/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2010 Broadcom Corporation
 * All rights reserved.
 *
 * This file is derived from the hndsoc.h, pci_core.h, and pcie_core.h headers
 * distributed with Broadcom's initial brcm80211 Linux driver release, as
 * contributed to the Linux staging repository.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * $FreeBSD$
 */

#ifndef _BHND_CORES_PCIBREG_H_
#define _BHND_CORES_PCIBREG_H_

/*
 * PCI DMA Constants
 */

#define	BHND_PCI_DMA32_TRANSLATION	0x40000000	/* Client Mode sb2pcitranslation2 (1 GB) */
#define	BHND_PCI_DMA32_SZ		0x40000000	/* Client Mode sb2pcitranslation2 size in bytes */

/*
 * PCI Core Registers
 */

#define	BHND_PCI_CTL			0x000	/**< PCI core control*/
#define	BHND_PCI_ARB_CTL		0x010	/**< PCI arbiter control */
#define	BHND_PCI_CLKRUN_CTL		0x014	/**< PCI clckrun control (>= rev11) */
#define	BHND_PCI_INTR_STATUS		0x020	/**< Interrupt status */
#define	BHND_PCI_INTR_MASK		0x024	/**< Interrupt mask */
#define	BHND_PCI_SBTOPCI_MBOX		0x028	/**< Sonics to PCI mailbox */
#define	BHND_PCI_BCAST_ADDR		0x050	/**< Sonics broadcast address (pci) */
#define	BHND_PCI_BCAST_DATA		0x054	/**< Sonics broadcast data (pci) */
#define	BHND_PCI_GPIO_IN		0x060	/**< GPIO input (>= rev2) */
#define	BHND_PCI_GPIO_OUT		0x064	/**< GPIO output (>= rev2) */
#define	BHND_PCI_GPIO_EN		0x068	/**< GPIO output enable (>= rev2) */
#define	BHND_PCI_GPIO_CTL		0x06C	/**< GPIO control (>= rev2) */
#define	BHND_PCI_SBTOPCI0		0x100	/**< Sonics to PCI translation 0 */
#define	BHND_PCI_SBTOPCI1		0x104	/**< Sonics to PCI translation 1 */
#define	BHND_PCI_SBTOPCI2		0x108	/**< Sonics to PCI translation 2 */
#define	BHND_PCI_FUNC0_CFG		0x400	/**< PCI function 0 cfg space (>= rev8) */
#define	BHND_PCI_FUNC1_CFG		0x500	/**< PCI function 1 cfg space (>= rev8) */
#define	BHND_PCI_FUNC2_CFG		0x600	/**< PCI function 2 cfg space (>= rev8) */
#define	BHND_PCI_FUNC3_CFG		0x700	/**< PCI function 3 cfg space (>= rev8) */
#define	BHND_PCI_SPROM_SHADOW		0x800	/**< PCI SPROM shadow */

/* BHND_PCI_CTL */
#define	BHND_PCI_CTL_RST_OE		0x01	/* When set, drives PCI_RESET out to pin */
#define	BHND_PCI_CTL_RST		0x02	/* Value driven out to pin */
#define	BHND_PCI_CTL_CLK_OE		0x04	/* When set, drives clock as gated by PCI_CLK out to pin */
#define	BHND_PCI_CTL_CLK		0x08	/* Gate for clock driven out to pin */

/* BHND_PCI_ARB_CTL */
#define	BHND_PCI_ARB_INT		0x01	/* When set, use an internal arbiter */
#define	BHND_PCI_ARB_EXT		0x02	/* When set, use an external arbiter */

/* BHND_PCI_ARB_CTL - ParkID (>= rev8) */
#define	BHND_PCI_ARB_PARKID_MASK	0x1c	/* Selects which agent is parked on an idle bus */
#define	BHND_PCI_ARB_PARKID_SHIFT	2
#define	BHND_PCI_ARB_PARKID_EXT0	0	/* External master 0 */
#define	BHND_PCI_ARB_PARKID_EXT1	1	/* External master 1 */
#define	BHND_PCI_ARB_PARKID_EXT2	2	/* External master 2 */
#define	BHND_PCI_ARB_PARKID_EXT3	3	/* External master 3 (rev >= 11) */
#define	BHND_PCI_ARB_PARKID_INT_r10	3	/* Internal master (rev < 11) */
#define	BHND_PCI_ARB_PARKID_INT_r11	4	/* Internal master (rev >= 11) */
#define	BHND_PCI_ARB_PARKID_LAST_r10	4	/* Last active master (rev < 11) */
#define	BHND_PCI_ARB_PARKID_LAST_r11	5	/* Last active master (rev >= 11) */

/* BHND_PCI_CLKRUN_CTL */
#define	BHND_PCI_CLKRUN_DSBL		0x8000	/* Bit 15 forceClkrun */

/* BHND_PCI_INTR_STATUS / BHND_PCI_INTR_MASK */
#define	BHND_PCI_INTR_A			0x01	/* PCI INTA# is asserted */
#define	BHND_PCI_INTR_B			0x02	/* PCI INTB# is asserted */
#define	BHND_PCI_INTR_SERR		0x04	/* PCI SERR# has been asserted (write one to clear) */
#define	BHND_PCI_INTR_PERR		0x08	/* PCI PERR# has been asserted (write one to clear) */

/* BHND_PCI_SBTOPCI_MBOX
 * (General) PCI/SB mailbox interrupts, two bits per pci function */
#define	BHND_PCI_SBTOPCI_MBOX_F0_0	0x100	/* function 0, int 0 */
#define	BHND_PCI_SBTOPCI_MBOX_F0_1	0x200	/* function 0, int 1 */
#define	BHND_PCI_SBTOPCI_MBOX_F1_0	0x400	/* function 1, int 0 */
#define	BHND_PCI_SBTOPCI_MBOX_F1_1	0x800	/* function 1, int 1 */
#define	BHND_PCI_SBTOPCI_MBOX_F2_0	0x1000	/* function 2, int 0 */
#define	BHND_PCI_SBTOPCI_MBOX_F2_1	0x2000	/* function 2, int 1 */
#define	BHND_PCI_SBTOPCI_MBOX_F3_0	0x4000	/* function 3, int 0 */
#define	BHND_PCI_SBTOPCI_MBOX_F3_1	0x8000	/* function 3, int 1 */

/* BHND_PCI_BCAST_ADDR */
#define	BHNC_PCI_BCAST_ADDR_MASK	0xFF	/* Broadcast register address */

/* Sonics to PCI translation types */
#define BHND_PCI_SBTOPCI0_MASK	0xfc000000
#define BHND_PCI_SBTOPCI1_MASK	0xfc000000
#define BHND_PCI_SBTOPCI2_MASK	0xc0000000

/* Access type bits (0:1) */
#define	BHND_PCI_SBTOPCI_MEM		0
#define	BHND_PCI_SBTOPCI_IO		1
#define	BHND_PCI_SBTOPCI_CFG0		2
#define	BHND_PCI_SBTOPCI_CFG1		3

#define	BHND_PCI_SBTOPCI_PREF		0x4	/* prefetch enable */
#define	BHND_PCI_SBTOPCI_BURST		0x8	/* burst enable */

#define	BHND_PCI_SBTOPCI_RC_MASK	0x30	/* read command (>= rev11) */
#define	BHND_PCI_SBTOPCI_RC_READ	0x00	/* memory read */
#define	BHND_PCI_SBTOPCI_RC_READLINE	0x10	/* memory read line */
#define	BHND_PCI_SBTOPCI_RC_READMULTI	0x20	/* memory read multiple */

/* PCI core index in SROM shadow area */
#define	BHND_PCI_SRSH_PI_OFFSET		0	/* first word */
#define	BHND_PCI_SRSH_PI_MASK		0xf000	/* bit 15:12 */
#define	BHND_PCI_SRSH_PI_SHIFT		12	/* bit 15:12 */

#endif /* _BHND_CORES_PCIBREG_H_ */
