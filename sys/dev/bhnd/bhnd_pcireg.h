/*
 * Broadcom HND PCI Configuration Register Constants.
 * 
 * This file was derived from the pcicfg.h header provided with
 * Broadcom's initial brcm80211 Linux driver release, as
 * contributed to the Linux staging repository. 
 * 
 * Copyright (c) 2010 Broadcom Corporation
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
 */

#ifndef	_BHND_BHND_PCIREG_H_
#define	_BHND_BHND_PCIREG_H_

/*
 * Broadcom HND PCI configuration registers.
 */

#define	BHND_PCI_BAR0_WIN		0x80	/* backplane address space accessed by BAR0 */
#define	BHND_PCI_BAR1_WIN		0x84	/* backplane address space accessed by BAR1 */
#define	BHND_PCI_SPROM_CONTROL		0x88	/* sprom property control */
#define	BHND_PCI_BAR1_CONTROL		0x8c	/* BAR1 region burst control */
#define	BHND_PCI_INT_STATUS		0x90	/* PCI and other cores interrupts */
#define	BHND_PCI_INT_MASK		0x94	/* mask of PCI and other cores interrupts */
#define BHND_PCI_TO_SB_MB		0x98	/* signal backplane interrupts */
#define BHND_PCI_BACKPLANE_ADDR		0xa0	/* address an arbitrary location on the system backplane */
#define BHND_PCI_BACKPLANE_DATA		0xa4	/* data at the location specified by above address */
#define	BHND_PCI_CLK_CTL_ST		0xa8	/* pci config space clock control/status (>=rev14) */
#define	BHND_PCI_BAR0_WIN2		0xac	/* backplane address space accessed by second 4KB of BAR0 (if AXI) */
#define	BHND_PCI_GPIO_IN		0xb0	/* pci config space gpio input (>=rev3) */
#define	BHND_PCI_GPIO_OUT		0xb4	/* pci config space gpio output (>=rev3) */
#define	BHND_PCI_GPIO_OUTEN		0xb8	/* pci config space gpio output enable (>=rev3) */

#define	BHND_PCI_BAR0_SHADOW_OFFSET	(2 * 1024)	/* bar0 + 2K accesses sprom shadow (in pci core) */
#define	BHND_PCI_BAR0_SPROM_OFFSET	(4 * 1024)	/* bar0 + 4K accesses external sprom */
#define	BHND_PCI_BAR0_PCIREGS_OFFSET	(6 * 1024)	/* bar0 + 6K accesses pci core registers */
#define	BHND_PCI_BAR0_PCISBR_OFFSET	(4 * 1024)	/* pci core SB registers are at the end of the
							 * 8KB window, so their address is the "regular"
							 * address plus 4K
							 */

#define BHND_PCI_BAR0_WINSZ		(4 * 1024)	/* bar0 window size (corerev < 13) */

/* On pci corerev >= 13 and all pcie, the bar0 is now 16KB and it maps: */
#define	BHND_PCI_16KB0_PCIREGS_OFFSET	(8 * 1024)	/* bar0 + 8K accesses pci/pcie core registers */
#define	BHND_PCI_16KB0_CCREGS_OFFSET	(12 * 1024)	/* bar0 + 12K accesses chipc core registers */
#define BHND_PCI_16KBB0_WINSZ		(16 * 1024)	/* bar0 window size */

/* On AXI chips we have a second configurable window within bar0 */
#define	BHND_PCI_16KB0_WIN2_OFFSET	(4 * 1024)	/* bar0 + 4K is "Window 2" */
#define	BHND_PCI_16KB0_WIN2SZ		(4 * 1024)	/* bar0 "Window 2" size */

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
