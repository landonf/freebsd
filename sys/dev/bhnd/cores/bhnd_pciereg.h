/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2010 Broadcom Corporation
 * All rights reserved.
 *
 * This file is derived from the pci_core.h and pcie_core.h headers distributed
 * with Broadcom's initial brcm80211 Linux driver release, as
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

#ifndef _BHND_CORES_PCIEREG_H_
#define	_BHND_CORES_PCIEREG_H_

#include "bhnd_pcireg.h"

/*
 * PCIe-Gen1 Core Registers
 */

#define	BHND_PCIE_CTL		BHND_PCI_CTL		/**< PCI core control*/
#define	BHND_PCIE_BIST_STATUS	0x00C			/**< BIST status */
#define	BHND_PCIE_GPIO_SEL	0x010			/**< GPIO select */
#define	BHND_PCIE_GPIO_OUT_EN	0x014			/**< GPIO output enable */
#define	BHND_PCIE_INTR_STATUS	BHND_PCI_INTR_STATUS	/**< Interrupt status */
#define	BHND_PCIE_INTR_MASK	BHND_PCI_INTR_MASK	/**< Interrupt mask */
#define	BHND_PCIE_SBTOPCI_MBOX	BHND_PCI_SBTOPCI_MBOX	/**< Sonics to PCI mailbox */
#define	BHND_PCIE_SBTOPCI0	BHND_PCI_SBTOPCI0	/**< Sonics to PCI translation 0 */
#define	BHND_PCIE_SBTOPCI1	BHND_PCI_SBTOPCI1	/**< Sonics to PCI translation 1 */
#define	BHND_PCIE_SBTOPCI2	BHND_PCI_SBTOPCI2	/**< Sonics to PCI translation 2 */

/* indirect pci config space access */
#define	BHND_PCIE_CFG_ADDR	0x120			/**< pcie config space address */
#define	BHND_PCIE_CFG_DATA	0x124			/**< pcie config space data */

/* indirect mdio access to serdes */
#define	BHND_PCIE_MDIO_CTL	0x128			/**< mdio control */
#define	BHND_PCIE_MDIO_DATA	0x12C			/**< mdio data */

/* indirect protocol phy/dllp/tlp register access */
#define	BHND_PCIE_IND_ADDR	0x130			/**< internal protocol register address */
#define	BHND_PCIE_IND_DATA	0x134			/**< internal protocol register data */

#define	BHND_PCIE_CLKREQEN_CTL	0x138			/**< clkreq rdma control */
#define	BHND_PCIE_FUNC0_CFG	BHND_PCI_FUNC0_CFG	/**< PCI function 0 cfg space */
#define	BHND_PCIE_FUNC1_CFG	BHND_PCI_FUNC1_CFG	/**< PCI function 1 cfg space */
#define	BHND_PCIE_FUNC2_CFG	BHND_PCI_FUNC2_CFG	/**< PCI function 2 cfg space */
#define	BHND_PCIE_FUNC3_CFG	BHND_PCI_FUNC3_CFG	/**< PCI function 3 cfg space */
#define	BHND_PCIE_SPROM_SHADOW	BHND_PCI_SPROM_SHADOW	/**< PCI SPROM shadow */

/* BHND_PCIE_CTL */
#define	BHND_PCIE_CTL_RST_OE	BHND_PCI_CTL_RST_OE	/* When set, drives PCI_RESET out to pin */
#define	BHND_PCIE_CTL_RST	BHND_PCI_CTL_RST_OE	/* Value driven out to pin */

/* BHND_PCI_INTR_STATUS / BHND_PCI_INTR_MASK */
#define	BHND_PCIE_INTR_A	BHND_PCI_INTR_A		/* PCIE INTA message is received */
#define	BHND_PCIE_INTR_B	BHND_PCI_INTR_B		/* PCIE INTB message is received */
#define	BHND_PCIE_INTR_FATAL	0x04			/* PCIE INTFATAL message is received */
#define	BHND_PCIE_INTR_NFATAL	0x08			/* PCIE INTNONFATAL message is received */
#define	BHND_PCIE_INTR_CORR	0x10			/* PCIE INTCORR message is received */
#define	BHND_PCIE_INTR_PME	0x20			/* PCIE INTPME message is received */

/* SB to PCIE translation masks */
#define	BHND_PCIE_SBTOPCI0_MASK	BHND_PCI_SBTOPCI0_MASK
#define	BHND_PCIE_SBTOPCI0_MASK	BHND_PCI_SBTOPCI1_MASK
#define	BHND_PCIE_SBTOPCI0_MASK	BHND_PCI_SBTOPCI2_MASK

/* Access type bits (0:1) */
#define	BHND_PCIE_SBTOPCI_MEM	BHND_PCI_SBTOPCI_MEM
#define	BHND_PCIE_SBTOPCI_IO	BHND_PCI_SBTOPCI_IO
#define	BHND_PCIE_SBTOPCI_CFG0	BHND_PCI_SBTOPCI_CFG0
#define	BHND_PCIE_SBTOPCI_CFG1	BHND_PCI_SBTOPCI_CFG1

#define	BHND_PCIE_SBTOPCI_PREF	BHND_PCI_SBTOPCI_PREF	/* prefetch enable */
#define	BHND_PCIE_SBTOPCI_BURST	BHND_PCI_SBTOPCI_BURST	/* burst enable */

/* BHND_PCIE_CFG_ADDR / BHND_PCIE_CFG_DATA */
#define	BHND_PCIE_CFG_ADDR_FUNC_MASK	0x7000
#define	BHND_PCIE_CFG_ADDR_FUNC_SHIFT	12
#define	BHND_PCIE_CFG_ADDR_REG_MASK	0x0FFF
#define	BHND_PCIE_CFG_ADDR_REG_SHIFT	0

#define	BHND_PCIE_CFG_OFFSET(f, r)	\
	((((f) & BHND_PCIE_CFG_ADDR_FUNC_MASK) << BHND_PCIE_CFG_ADDR_FUNC_SHIFT) | \
	(((r) & BHND_PCIE_CFG_ADDR_FUNC_SHIFT) << BHND_PCIE_CFG_ADDR_REG_SHIFT))

/* PCIE protocol regs Indirect Address */
#define	BHND_PCIE_IND_ADDR_PROT_MASK	0x300
#define	BHND_PCIE_IND_ADDR_PROT_SHIFT	8
#define	BHND_PCIE_IND_ADDR_PL_TLP	0
#define	BHND_PCIE_IND_ADDR_PL_DLLP	1
#define	BHND_PCIE_IND_ADDR_PL_PLP	2

/* PCIE protocol PHY diagnostic registers */
#define	BHND_PCIE_IND_PLP_MODEREG		0x200	/* Mode */
#define	BHND_PCIE_IND_PLP_STATUSREG		0x204	/* Status */
#define	BHND_PCIE_IND_PLP_LTSSMCTRLREG		0x208	/* LTSSM control */
#define	BHND_PCIE_IND_PLP_LTLINKNUMREG		0x20c	/* Link Training Link number */
#define	BHND_PCIE_IND_PLP_LTLANENUMREG		0x210	/* Link Training Lane number */
#define	BHND_PCIE_IND_PLP_LTNFTSREG		0x214	/* Link Training N_FTS */
#define	BHND_PCIE_IND_PLP_ATTNREG		0x218	/* Attention */
#define	BHND_PCIE_IND_PLP_ATTNMASKREG		0x21C	/* Attention Mask */
#define	BHND_PCIE_IND_PLP_RXERRCTR		0x220	/* Rx Error */
#define	BHND_PCIE_IND_PLP_RXFRMERRCTR		0x224	/* Rx Framing Error */
#define	BHND_PCIE_IND_PLP_RXERRTHRESHREG	0x228	/* Rx Error threshold */
#define	BHND_PCIE_IND_PLP_TESTCTRLREG		0x22C	/* Test Control reg */
#define	BHND_PCIE_IND_PLP_SERDESCTRLOVRDREG	0x230	/* SERDES Control Override */
#define	BHND_PCIE_IND_PLP_TIMINGOVRDREG		0x234	/* Timing param override */
#define	BHND_PCIE_IND_PLP_RXTXSMDIAGREG		0x238	/* RXTX State Machine Diag */
#define	BHND_PCIE_IND_PLP_LTSSMDIAGREG		0x23C	/* LTSSM State Machine Diag */

/* PCIE protocol DLLP diagnostic registers */
#define	BHND_PCIE_IND_DLLP_LCREG		0x100	/* Link Control */
#define	BHND_PCIE_IND_DLLP_LSREG		0x104	/* Link Status */
#define	BHND_PCIE_IND_DLLP_LAREG		0x108	/* Link Attention */
#define	BHND_PCIE_IND_DLLP_LAMASKREG		0x10C	/* Link Attention Mask */
#define	BHND_PCIE_IND_DLLP_NEXTTXSEQNUMREG	0x110	/* Next Tx Seq Num */
#define	BHND_PCIE_IND_DLLP_ACKEDTXSEQNUMREG	0x114	/* Acked Tx Seq Num */
#define	BHND_PCIE_IND_DLLP_PURGEDTXSEQNUMREG	0x118	/* Purged Tx Seq Num */
#define	BHND_PCIE_IND_DLLP_RXSEQNUMREG		0x11C	/* Rx Sequence Number */
#define	BHND_PCIE_IND_DLLP_LRREG		0x120	/* Link Replay */
#define	BHND_PCIE_IND_DLLP_LACKTOREG		0x124	/* Link Ack Timeout */
#define	BHND_PCIE_IND_DLLP_PMTHRESHREG		0x128	/* Power Management Threshold */
#define	BHND_PCIE_IND_DLLP_RTRYWPREG		0x12C	/* Retry buffer write ptr */
#define	BHND_PCIE_IND_DLLP_RTRYRPREG		0x130	/* Retry buffer Read ptr */
#define	BHND_PCIE_IND_DLLP_RTRYPPREG		0x134	/* Retry buffer Purged ptr */
#define	BHND_PCIE_IND_DLLP_RTRRWREG		0x138	/* Retry buffer Read/Write */
#define	BHND_PCIE_IND_DLLP_ECTHRESHREG		0x13C	/* Error Count Threshold */
#define	BHND_PCIE_IND_DLLP_TLPERRCTRREG		0x140	/* TLP Error Counter */
#define	BHND_PCIE_IND_DLLP_ERRCTRREG		0x144	/* Error Counter */
#define	BHND_PCIE_IND_DLLP_NAKRXCTRREG		0x148	/* NAK Received Counter */
#define	BHND_PCIE_IND_DLLP_TESTREG		0x14C	/* Test */
#define	BHND_PCIE_IND_DLLP_PKTBIST		0x150	/* Packet BIST */
#define	BHND_PCIE_IND_DLLP_PCIE11		0x154	/* DLLP PCIE 1.1 reg */

#define	BHND_PCIE_IND_DLLP_LSREG_LINKUP		(1 << 16)

/* PCIE protocol TLP diagnostic registers */
#define	BHND_PCIE_IND_TLP_CONFIGREG		0x000	/* Configuration */
#define	BHND_PCIE_IND_TLP_WORKAROUNDSREG	0x004	/* TLP Workarounds */
#define	BHND_PCIE_IND_TLP_WRDMAUPPER		0x010	/* Write DMA Upper Address */
#define	BHND_PCIE_IND_TLP_WRDMALOWER		0x014	/* Write DMA Lower Address */
#define	BHND_PCIE_IND_TLP_WRDMAREQ_LBEREG	0x018	/* Write DMA Len/ByteEn Req */
#define	BHND_PCIE_IND_TLP_RDDMAUPPER		0x01C	/* Read DMA Upper Address */
#define	BHND_PCIE_IND_TLP_RDDMALOWER		0x020	/* Read DMA Lower Address */
#define	BHND_PCIE_IND_TLP_RDDMALENREG		0x024	/* Read DMA Len Req */
#define	BHND_PCIE_IND_TLP_MSIDMAUPPER		0x028	/* MSI DMA Upper Address */
#define	BHND_PCIE_IND_TLP_MSIDMALOWER		0x02C	/* MSI DMA Lower Address */
#define	BHND_PCIE_IND_TLP_MSIDMALENREG		0x030	/* MSI DMA Len Req */
#define	BHND_PCIE_IND_TLP_SLVREQLENREG		0x034	/* Slave Request Len */
#define	BHND_PCIE_IND_TLP_FCINPUTSREQ		0x038	/* Flow Control Inputs */
#define	BHND_PCIE_IND_TLP_TXSMGRSREQ		0x03C	/* Tx StateMachine and Gated Req */
#define	BHND_PCIE_IND_TLP_ADRACKCNTARBLEN	0x040	/* Address Ack XferCnt and ARB Len */
#define	BHND_PCIE_IND_TLP_DMACPLHDR0		0x044	/* DMA Completion Hdr 0 */
#define	BHND_PCIE_IND_TLP_DMACPLHDR1		0x048	/* DMA Completion Hdr 1 */
#define	BHND_PCIE_IND_TLP_DMACPLHDR2		0x04C	/* DMA Completion Hdr 2 */
#define	BHND_PCIE_IND_TLP_DMACPLMISC0		0x050	/* DMA Completion Misc0 */
#define	BHND_PCIE_IND_TLP_DMACPLMISC1		0x054	/* DMA Completion Misc1 */
#define	BHND_PCIE_IND_TLP_DMACPLMISC2		0x058	/* DMA Completion Misc2 */
#define	BHND_PCIE_IND_TLP_SPTCTRLLEN		0x05C	/* Split Controller Req len */
#define	BHND_PCIE_IND_TLP_SPTCTRLMSIC0		0x060	/* Split Controller Misc 0 */
#define	BHND_PCIE_IND_TLP_SPTCTRLMSIC1		0x064	/* Split Controller Misc 1 */
#define	BHND_PCIE_IND_TLP_BUSDEVFUNC		0x068	/* Bus/Device/Func */
#define	BHND_PCIE_IND_TLP_RESETCTR		0x06C	/* Reset Counter */
#define	BHND_PCIE_IND_TLP_RTRYBUF		0x070	/* Retry Buffer value */
#define	BHND_PCIE_IND_TLP_TGTDEBUG1		0x074	/* Target Debug Reg1 */
#define	BHND_PCIE_IND_TLP_TGTDEBUG2		0x078	/* Target Debug Reg2 */
#define	BHND_PCIE_IND_TLP_TGTDEBUG3		0x07C	/* Target Debug Reg3 */
#define	BHND_PCIE_IND_TLP_TGTDEBUG4		0x080	/* Target Debug Reg4 */

/* MDIO control */
#define	BHND_PCIE_MDIOCTL_DIVISOR_MASK		0x7f	/* clock to be used on MDIO */
#define	BHND_PCIE_MDIOCTL_DIVISOR_VAL		0x2
#define	BHND_PCIE_MDIOCTL_PREAM_EN		0x80	/* Enable preamble sequnce */
#define	BHND_PCIE_MDIOCTL_ACCESS_DONE		0x100	/* Tranaction complete */

/* MDIO Data */
#define	BHND_PCIE_MDIODATA_MASK			0x0000ffff	/* data 2 bytes */
#define	BHND_PCIE_MDIODATA_TA			0x00020000	/* Turnaround */
#define	BHND_PCIE_MDIODATA_REGADDR_SHIFT_OLD	18		/* Regaddr shift (rev < 10) */
#define	BHND_PCIE_MDIODATA_REGADDR_MASK_OLD	0x003c0000	/* Regaddr Mask (rev < 10) */
#define	BHND_PCIE_MDIODATA_DEVADDR_SHIFT_OLD	22		/* Physmedia devaddr shift (rev < 10) */
#define	BHND_PCIE_MDIODATA_DEVADDR_MASK_OLD	0x0fc00000	/* Physmedia devaddr Mask (rev < 10) */
#define	BHND_PCIE_MDIODATA_REGADDR_SHIFT	18		/* Regaddr shift */
#define	BHND_PCIE_MDIODATA_REGADDR_MASK		0x007c0000	/* Regaddr Mask */
#define	BHND_PCIE_MDIODATA_DEVADDR_SHIFT	23		/* Physmedia devaddr shift */
#define	BHND_PCIE_MDIODATA_DEVADDR_MASK		0x0f800000	/* Physmedia devaddr Mask */
#define	BHND_PCIE_MDIODATA_WRITE		0x10000000	/* write Transaction */
#define	BHND_PCIE_MDIODATA_READ			0x20000000	/* Read Transaction */
#define	BHND_PCIE_MDIODATA_START		0x40000000	/* start of Transaction */

#define	BHND_PCIE_MDIODATA_DEV_ADDR		0x0	/* dev address for serdes */
#define	BHND_PCIE_MDIODATA_BLK_ADDR		0x1F	/* blk address for serdes */

/* 
 * MDIO devices (SERDES modules)
 *  unlike old pcie cores (rev < 10), rev10 pcie serde organizes registers into
 * a few blocks.
 *  two layers mapping (blockidx, register offset) is required
 */
#define	BHND_PCIE_MDIO_DEV_IEEE0		0x000
#define	BHND_PCIE_MDIO_DEV_IEEE1		0x001
#define	BHND_PCIE_MDIO_DEV_BLK0			0x800
#define	BHND_PCIE_MDIO_DEV_BLK1			0x801
#define	BHND_PCIE_MDIO_DEV_BLK2			0x802
#define	BHND_PCIE_MDIO_DEV_BLK3			0x803
#define	BHND_PCIE_MDIO_DEV_BLK4			0x804
#define	BHND_PCIE_MDIO_DEV_TXPLL		0x808	/* TXPLL register block idx */
#define	BHND_PCIE_MDIO_DEV_TXCTRL0		0x820
#define	BHND_PCIE_MDIO_DEV_SERDESID		0x831
#define	BHND_PCIE_MDIO_DEV_RXCTRL0		0x840

/* serdes regs (rev < 10) */
#define	BHND_PCIE_MDIODATA_DEV_PLL       	0x1d	/* SERDES PLL Dev */
#define	BHND_PCIE_MDIODATA_DEV_TX        	0x1e	/* SERDES TX Dev */
#define	BHND_PCIE_MDIODATA_DEV_RX        	0x1f	/* SERDES RX Dev */
/* SERDES RX registers */
#define	BHND_PCIE_SERDES_RX_CTRL		1	/* Rx cntrl */
#define	BHND_PCIE_SERDES_RX_TIMER1		2	/* Rx Timer1 */
#define	BHND_PCIE_SERDES_RX_CDR			6	/* CDR */
#define	BHND_PCIE_SERDES_RX_CDRBW		7	/* CDR BW */

/* SERDES RX control register */
#define	BHND_PCIE_SERDES_RX_CTRL_FORCE		0x80	/* rxpolarity_force */
#define	BHND_PCIE_SERDES_RX_CTRL_POLARITY	0x40	/* rxpolarity_value */

/* SERDES PLL registers */
#define	BHND_PCIE_SERDES_PLL_CTRL		1	/* PLL control reg */
#define	BHND_PCIE_PLL_CTRL_FREQDET_EN		0x4000	/* bit 14 is FREQDET on */

/* Power management threshold */
#define	BHND_PCIE_L0THRESHOLDTIME_MASK		0xFF00	/* bits 0 - 7 */
#define	BHND_PCIE_L1THRESHOLDTIME_MASK		0xFF00	/* bits 8 - 15 */
#define	BHND_PCIE_L1THRESHOLDTIME_SHIFT		8	/* PCIE_L1THRESHOLDTIME_SHIFT */
#define	BHND_PCIE_L1THRESHOLD_WARVAL		0x72	/* WAR value */
#define	BHND_PCIE_ASPMTIMER_EXTEND		0x01000000	/* > rev7: enable extend ASPM timer */

/* SPROM offsets */
#define	BHND_PCIE_SRSH_ASPM_OFFSET		4	/* word 4 */
#define	BHND_PCIE_SRSH_ASPM_ENB			0x18	/* bit 3, 4 */
#define	BHND_PCIE_SRSH_ASPM_L1_ENB		0x10	/* bit 4 */
#define	BHND_PCIE_SRSH_ASPM_L0s_ENB		0x8	/* bit 3 */
#define	BHND_PCIE_SRSH_PCIE_MISC_CONFIG		5	/* word 5 */
#define	BHND_PCIE_SRSH_L23READY_EXIT_NOPERST	0x8000	/* bit 15 */
#define	BHND_PCIE_SRSH_CLKREQ_OFFSET_REV5	20	/* word 20 for srom rev <= 5 */
#define	BHND_PCIE_SRSH_CLKREQ_OFFSET_REV8	52	/* word 52 for srom rev 8 */
#define	BHND_PCIE_SRSH_CLKREQ_ENB		0x0800	/* bit 11 */
#define	BHND_PCIE_SRSH_BD_OFFSET		6	/* word 6 */
#define	BHND_PCIE_SRSH_AUTOINIT_OFFSET		18	/* auto initialization enable */

/* Linkcontrol reg offset in PCIE Cap */
#define	BHND_PCIE_CAP_LINKCTRL_OFFSET		16	/* linkctrl offset in pcie cap */
#define	BHND_PCIE_CAP_LCREG_ASPML0s		0x01	/* ASPM L0s in linkctrl */
#define	BHND_PCIE_CAP_LCREG_ASPML1		0x02	/* ASPM L1 in linkctrl */
#define	BHND_PCIE_CLKREQ_ENAB			0x100	/* CLKREQ Enab in linkctrl */

#define	BHND_PCIE_ASPM_ENAB			3	/* ASPM L0s & L1 in linkctrl */
#define	BHND_PCIE_ASPM_L1_ENAB			2	/* ASPM L0s & L1 in linkctrl */
#define	BHND_PCIE_ASPM_L0s_ENAB			1	/* ASPM L0s & L1 in linkctrl */
#define	BHND_PCIE_ASPM_DISAB			0	/* ASPM L0s & L1 in linkctrl */

/* Status reg PCIE_PLP_STATUSREG */
#define	BHND_PCIE_PLP_POLARITYINV_STAT	0x10

#endif				/* _BHND_CORES_PCIEREG_H_ */
