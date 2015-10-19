/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2010 Broadcom Corporation
 * 
 * This file is derived from the pcicfg.h header distributed with
 * Broadcom's initial brcm80211 Linux driver release, as
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

#ifndef	_BHND_BHND_PCIREG_H_
#define	_BHND_BHND_PCIREG_H_

/*
 * PCI/PCIE Bridge Configuration Registers.
 * The BHND PCI/PCIE bridge cores support both host and device modes.
 * 
 * = MAJOR CORE REVISIONS =
 * 
 * There have been three major revisions to the BAR0/BAR1 memory mappings used
 * in BHND PCI/PCIE bridge cores:
 * 
 * == PCI_V0: PCI (cid=0x804, revision ??? TBD) ==
 * 8KB BAR0:
 * 	[0x0000+0x0800]		mapped backplane address space.
 * 	[0x0800+0x0800]		SPROM shadow
 * 	[0x1000+0x0800]		external SPROM
 * 	[0x1800+0x0800]		pci core registers
 * 
 * == PCI_V1: PCI (cid=0x804, revision <= 12) ==
 * 8KB BAR0:
 * 	[0x0000+0x1000]		mapped backplane address space.
 * 	[0x1000+0x0800]		SPROM shadow
 * 	[0x1800+0x0800]		pci core registers
 *
 * == PCI_V2: PCIE (cid=0x820) and PCI (cid=0x804, revision >= 13) 
 * 16KB BAR0:
 * 	[0x0000+0x1000]		mapped backplane address space (window 0).
 * 	[0x1000+0x1000]		mapped backplane address space (window 1).
 * 	[0x2000+0x1000]		pci/pcie core registers
 * 	[0x3000+0x1000]		chipcommon core registers
 * 
 * 
 * 
 * = MINOR CORE REVISIONS =
 * 
 * == PCI/PCIE Cores Revision >= 3 ==
 * - Mapped GPIO CSRs into the PCI config space. Refer to
 *   BHND_PCI_GPIO_*.
 * 
 * == PCI/PCIE Cores Revision >= 14 ==
 * - Mapped the clock CSR into the PCI config space. Refer to
 *   BHND_PCI_CLK_CTL_ST
 * 
 * = Hardware Bugs =
 * == BAR1 ==
 * 
 * The BHND PCI(e) cores hypothetically support an additional memory mapping
 * of the backplane address space via BAR1, but this appears to be subject
 * to a hardware bug in which BAR1 is initially configured with a 4 byte
 * length.
 * 
 * A work-around for this bug may be possible by writing to the PCI core's
 * BAR1 config register (0x4e0), but this requires further research -- I've
 * found three sources for information on the BAR1 PCI core configuration that
 * may be relevant:
 * 	- The QLogix NetXTreme 10GB PCIe NIC seems to use the same PCIE
 * 	  core IP block as is used in other BHND devices. The bxe(4) driver
 * 	  contains example initialization code and register constants
 * 	  that may apply (e.g. GRC_BAR2_CONFIG/PCI_CONFIG_2_BAR2_SIZE).
 * 	- The publicly available Broadcom BCM440X data sheet (440X-PG02-R)
 * 	  appears to (partially) document a Broadcom PCI(e) core that has a
 * 	  seemingly compatible programming model.
 * 	- The Android bcmdhd driver sources include a possible work-around
 *	  implementation (writing to 0x4e0) in dhd_pcie.c
 */

/* PCI/PCIE Config Registers */
#define	BHND_PCI_BAR0_WIN0		0x80	/* backplane address space accessed by BAR0 */
#define	BHND_PCI_BAR1_WIN0		0x84	/* backplane address space accessed by BAR1. */
#define	BHND_PCI_SPROM_CONTROL		0x88	/* sprom property control */
#define	BHND_PCI_BAR1_CONTROL		0x8c	/* BAR1 region prefetch/burst control */
#define	BHND_PCI_INT_STATUS		0x90	/* PCI and other cores interrupts */
#define	BHND_PCI_INT_MASK		0x94	/* mask of PCI and other cores interrupts */
#define	BHND_PCI_TO_SB_MB		0x98	/* signal backplane interrupts */
#define	BHND_PCI_BACKPLANE_ADDR		0xa0	/* address an arbitrary location on the system backplane */
#define	BHND_PCI_BACKPLANE_DATA		0xa4	/* data at the location specified by above address */
#define	BHND_PCI_CLK_CTL_ST		0xa8	/* pci config space clock control/status (>=rev14) */
#define	BHND_PCI_BAR0_WIN1		0xac	/* backplane address space accessed by second 4KB of BAR0 (pcie, pci >=rev13) */
#define	BHND_PCI_GPIO_IN		0xb0	/* pci config space gpio input (>=rev3) */
#define	BHND_PCI_GPIO_OUT		0xb4	/* pci config space gpio output (>=rev3) */
#define	BHND_PCI_GPIO_OUTEN		0xb8	/* pci config space gpio output enable (>=rev3) */

/* PCI BAR Address Map */
#define	BHND_PCI_V0_BAR0_SIZE		0x2000	/* 8KB BAR0 */
#define	BHND_PCI_V0_BAR0_WIN0_OFFSET	0x0	/* bar0 + 0x0 accesses configurable 4K region of backplane address space */
#define BHND_PCI_V0_BAR0_WIN0_SIZE	0x1000	/* bar0 window size */
#define	BHND_PCI_V0_BAR0_SHADOW_OFFSET	0x0800	/* bar0 + 2K accesses sprom shadow (in pci core) */
#define	BHND_PCI_V0_BAR0_SPROM_OFFSET	0x1000	/* bar0 + 4K accesses external sprom */
#define	BHND_PCI_V0_BAR0_PCIREGS_OFFSET	0x1800	/* bar0 + 6K accesses pci core registers */

#define	BHND_PCI_V1_BAR0_SIZE		BHND_PCI_V0_BAR0_SIZE
#define	BHND_PCI_V1_BAR0_WIN0_OFFSET	BHND_PCI_V0_BAR0_WIN0_OFFSET
#define BHND_PCI_V1_BAR0_WIN0_SIZE	BHND_PCI_V0_BAR0_WIN0_SIZE
#define	BHND_PCI_V1_BAR0_SPROM_OFFSET	0x1000	/* bar0 + 4K accesses sprom shadow (in pci core) */
#define	BHND_PCI_V1_BAR0_PCIREGS_OFFSET	0x1800	/* bar0 + 6K accesses pci core registers */

#define	BHND_PCI_V2_BAR0_SIZE		0x4000	/* 16KB BAR0 */
#define	BHND_PCI_V2_BAR0_WIN0_OFFSET	BHND_PCI_V0_BAR0_WIN0_OFFSET
#define BHND_PCI_V2_BAR0_WIN0_SIZE	BHND_PCI_V0_BAR0_WIN0_SIZE
#define	BHND_PCI_V2_BAR0_WIN1_OFFSET	0x1000	/* bar0 + 4KB accesses an additional 4K window */
#define BHND_PCI_V2_BAR0_WIN1_SIZE	0x1000	/* bar0 "window 1" size */
#define	BHND_PCI_V2_PCIREGS_OFFSET	0x2000	/* bar0 + 8K accesses pci/pcie core registers */
#define	BHND_PCI_V2_CCREGS_OFFSET	0x3000	/* bar0 + 12K accesses chipc core registers */

/* PCI_INT_STATUS */
#define	BHND_PCI_SBIM_STATUS_SERR	0x4	/* backplane SBErr interrupt status */

/* PCI_INT_MASK */
#define	BHND_PCI_SBIM_SHIFT		8	/* backplane core interrupt mask bits offset */
#define	BHND_PCI_SBIM_MASK		0xff00	/* backplane core interrupt mask */
#define	BHND_PCI_SBIM_MASK_SERR	0x4	/* backplane SBErr interrupt mask */

/* PCI_SPROM_CONTROL */
#define BHND_PCI_SPROM_SZ_MSK		0x02	/* SPROM Size Mask */
#define BHND_PCI_SPROM_LOCKED		0x08	/* SPROM Locked */
#define	BHND_PCI_SPROM_BLANK		0x04	/* indicating a blank SPROM */
#define BHND_PCI_SPROM_WRITEEN		0x10	/* SPROM write enable */
#define BHND_PCI_SPROM_BOOTROM_WE	0x20	/* external bootrom write enable */
#define BHND_PCI_SPROM_BACKPLANE_EN	0x40	/* Enable indirect backplane access */
#define BHND_PCI_SPROM_OTPIN_USE	0x80	/* device OTP In use */

/* Bits in PCI command and status regs */
#define BHND_PCI_CMD_IO		0x00000001	/* I/O enable */
#define BHND_PCI_CMD_MEMORY	0x00000002	/* Memory enable */
#define BHND_PCI_CMD_MASTER	0x00000004	/* Master enable */
#define BHND_PCI_CMD_SPECIAL	0x00000008	/* Special cycles enable */
#define BHND_PCI_CMD_INVALIDATE	0x00000010	/* Invalidate? */
#define BHND_PCI_CMD_VGA_PAL	0x00000040	/* VGA Palate */
#define BHND_PCI_STAT_TA	0x08000000	/* target abort status */

#endif /* _BHND_BHND_PCIREG_H_ */
