/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2018 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2015 Broadcom Corporation. All Rights Reserved.
 * All rights reserved.
 *
 * This file was derived from the sbhnddma.h header distributed with the Asus
 * RT-N18 firmware source code release.
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
 * $Id: sbhnddma.h 419467 2013-08-21 09:19:48Z $
 *
 * $FreeBSD$
 */

#ifndef _BHND_DMA_BHND_DMA64REG_H_
#define _BHND_DMA_BHND_DMA64REG_H_

#if 0
#define	BHND_D64_ADDREXT_MASK	0xC000000000000000ULL		/**< mask of extended address bits */
#define	BHND_D64_ADDREXT_SHIFT	62
#endif

#define	BHND_D64_ADDRWIDTH_BASE	62			/**< address width of 64-bit DMA channel without addrext support */
#define	BHND_D64_ADDRWIDTH_EXT	BHND_DMA_ADDR_64BIT	/**< address width of 64-bit DMA channel with addrext support */

#define	BHND_D64_CHAN_SIZE	32			/**< size of single 64-bit DMA channel register block */
#define	BHND_D64_CHAN_PAIR_SIZE	(BHND_D64_CHAN_SIZE*2)	/**< size of a 64-bit DMA channel pair register block */

#define	BHND_D64_TX_OFFSET	0			/**< TX offset within 64-bit channel pair */
#define	BHND_D64_RX_OFFSET	BHND_D64_CHAN_SIZE	/**< RX offset within 64-bit channel pair */

#define	BHND_D64_DESC_SIZE	sizeof(dma64dd_t)	/**< 64-bit DMA descriptor size */

/**
 * Register offset to the @p _num 64-bit channel pair.
 */
#define	BHND_D64_CHAN_PAIR_OFFSET(_num)	\
	(BHND_D64_CHAN_PAIR_SIZE * (_num))

/**
 * Register offset to the 64-bit channel with @p _dir and @p _num.
 */
#define	BHND_D64_CHAN_OFFSET(_dir, _num) (				\
	(_dir) == BHND_DMA_TX ?						\
	     (BHND_D64_CHAN_PAIR_OFFSET(_num) + BHND_D64_TX_OFFSET) :	\
	     (BHND_D64_CHAN_PAIR_OFFSET(_num) + BHND_D64_RX_OFFSET)	\
)

/* DMA per-channel (TX or RX) register offsets */
#define	BHND_D64_CTRL			0x00		/**< channel control (enable, et al) */
#define	BHND_D64_PTR			0x04		/**< last descriptor posted to chip */
#define	BHND_D64_ADDRLOW		0x08		/**< descriptor ring base address, bits 0:31 (8K aligned) */
#define	BHND_D64_ADDRHIGH		0x08		/**< descriptor ring base address, bits 32:61 (8K aligned) */
#define	BHND_D64_STATUS0		0x0C		/**< channel status */
#define	BHND_D64_STATUS1		0x10		/**< additional channel status */

/* BHND_D64_CTRL (common) */
#define	BHND_D64_CTRL_EN		0x00000001	/* transmit/receive enable */
#define	BHND_D64_CTRL_PD		0x00000800	/* parity check disable */
#define	BHND_D64_CTRL_AE_MASK		0x00030000	/* address extension bits */
#define	BHND_D64_CTRL_AE_SHIFT		16
#define	BHND_D64_CTRL_BL_MASK		0x001C0000	/* burstlen bits */
#define	BHND_D64_CTRL_BL_SHIFT		18
#define	BHND_D64_CTRL_PC_MASK		0x00E00000	/* prefetch control */
#define	BHND_D64_CTRL_PC_SHIFT		21
#define	BHND_D64_CTRL_PT_MASK		0x03000000	/* prefetch threshold */
#define	BHND_D64_CTRL_PT_SHIFT		24

/* BHND_D64_CTRL (transmit) */
#define	BHND_D64_XC_XE			BHND_D64_CTRL_XE	/* transmit enable */
#define	BHND_D64_XC_SE			0x00000002		/* transmit suspend request */
#define	BHND_D64_XC_LE			0x00000004		/* loopback enable */
#define	BHND_D64_XC_FL			0x00000010		/* flush request */
#define	BHND_D64_XC_MR_MASK		0x000000C0		/* multiple outstanding reads */
#define	BHND_D64_XC_MR_SHIFT		6
#define	BHND_D64_XC_PD			BHND_D64_CTRL_PD
#define	BHND_D64_XC_AE_MASK		BHND_D64_CTRL_AE_MASK
#define	BHND_D64_XC_AE_SHIFT		BHND_D64_CTRL_AE_SHIFT
#define	BHND_D64_XC_BL_MASK		BHND_D64_CTRL_BL_MASK
#define	BHND_D64_XC_BL_SHIFT		BHND_D64_CTRL_BL_SHIFT
#define	BHND_D64_XC_PC_MASK		BHND_D64_CTRL_PC_MASK
#define	BHND_D64_XC_PC_SHIFT		BHND_D64_CTRL_PC_SHIFT
#define	BHND_D64_XC_PT_MASK		BHND_D64_CTRL_PT_MASK
#define	BHND_D64_XC_PT_SHIFT		BHND_D64_CTRL_PT_SHIFT

/* BHND_D64_CTRL (receive) */
#define	BHND_D64_RC_RE			0x00000001		/* receive enable */
#define	BHND_D64_RC_RO_MASK		0x000000fe		/* receive frame offset */
#define	BHND_D64_RC_RO_SHIFT		1
#define	BHND_D64_RC_FM			0x00000100		/* direct fifo receive (pio) mode */
#define	BHND_D64_RC_SH			0x00000200		/* separate rx header descriptor enable */
#define	BHND_D64_RC_OC			0x00000400		/* overflow continue */
#define	BHND_D64_RC_PD			BHND_D64_CTRL_PD
#define	BHND_D64_RC_GE			0x00004000		/* Glom enable */
#define	BHND_D64_RC_AE_MASK		BHND_D64_CTRL_AE_MASK
#define	BHND_D64_RC_AE_SHIFT		BHND_D64_CTRL_AE_SHIFT
#define	BHND_D64_RC_BL_MASK		BHND_D64_CTRL_BL_MASK
#define	BHND_D64_RC_BL_SHIFT		BHND_D64_CTRL_BL_SHIFT
#define	BHND_D64_RC_PC_MASK		BHND_D64_CTRL_PC_MASK
#define	BHND_D64_RC_PC_SHIFT		BHND_D64_CTRL_PC_SHIFT
#define	BHND_D64_RC_PT_MASK		BHND_D64_CTRL_PT_MASK
#define	BHND_D64_RC_PT_SHIFT		BHND_D64_CTRL_PT_SHIFT


/* BHND_D64_PTR (common) */
#define	BHND_D64_PTR_LD_MASK		0x00001fff	/* last valid descriptor */


/* BHND_D64_STATUS0 (common) */
#define	BHND_D64_STATUS0_CD_MIN_MASK	0x00001fff	/* current descriptor pointer (should be probed at runtime) */
#define	BHND_D64_STATUS0_CD_SHIFT	0
#define	BHND_D64_STATUS0_ST_MASK	0xf0000000	/* transmit/receive state */
#define	BHND_D64_STATUS0_ST_SHIFT	28
#define	BHND_D64_STATUS0_ST_DISABLED	0x0		/* disabled */
#define	BHND_D64_STATUS0_ST_ACTIVE	0x1		/* active */
#define	BHND_D64_STATUS0_ST_IDLE	0x2		/* idle wait */
#define	BHND_D64_STATUS0_ST_STOPPED	0x3		/* stopped */
#define	BHND_D64_STATUS0_ST_SUSP	0x4		/* suspend pending */

/* BHND_D64_STATUS0 (transmit) */
#define	BHND_D64_XS0_CD_MIN_MASK	BHND_D64_STATUS0_CD_MIN_MASK	/* current descriptor pointer */
#define	BHND_D64_XS0_CD_SHIFT		BHND_D64_STATUS0_CD_SHIFT
#define	BHND_D64_XS0_XS_MASK		BHND_D64_STATUS0_ST_MASK	/* transmit state */
#define	BHND_D64_XS0_XS_SHIFT		BHND_D64_STATUS0_ST_SHIFT
#define	BHND_D64_XS0_XS_DISABLED	BHND_D64_STATUS0_ST_DISABLED
#define	BHND_D64_XS0_XS_ACTIVE		BHND_D64_STATUS0_ST_ACTIVE
#define	BHND_D64_XS0_XS_IDLE		BHND_D64_STATUS0_ST_IDLE
#define	BHND_D64_XS0_XS_STOPPED		BHND_D64_STATUS0_ST_STOPPED
#define	BHND_D64_XS0_XS_SUSP		BHND_D64_STATUS0_ST_SUSP


/* BHND_D64_STATUS0 (receive) */
#define	BHND_D64_RS0_CD_MIN_MASK	BHND_D64_STATUS0_CD_MIN_MASK	/* current descriptor pointer */
#define	BHND_D64_RS0_CD_SHIFT		BHND_D64_STATUS0_CD_SHIFT
#define	BHND_D64_RS0_RS_MASK		BHND_D64_STATUS0_ST_MASK	/* receive state */
#define	BHND_D64_RS0_RS_SHIFT		BHND_D64_STATUS0_ST_SHIFT
#define	BHND_D64_RS0_RS_DISABLED	BHND_D64_STATUS0_ST_DISABLED
#define	BHND_D64_RS0_RS_ACTIVE		BHND_D64_STATUS0_ST_ACTIVE
#define	BHND_D64_RS0_RS_IDLE		BHND_D64_STATUS0_ST_IDLE
#define	BHND_D64_RS0_RS_STOPPED		BHND_D64_STATUS0_ST_SHIFT
#define	BHND_D64_RS0_RS_SUSP		BHND_D64_STATUS0_ST_SUSP


/* BHND_D64_STATUS1 (common) */
#define	BHND_D64_STATUS1_AD_MIN_MASK	0x00001fff	/* active descriptor pointer (should be probed at runtime) */
#define	BHND_D64_STATUS1_AD_SHIFT	0
#define	BHND_D64_STATUS1_ERR_MASK	0xf0000000	/* transmit/receive errors */
#define	BHND_D64_STATUS1_ERR_SHIFT	28
#define	BHND_D64_STATUS1_ERR_NOERR	0x0		/* no error */
#define	BHND_D64_STATUS1_ERR_DPE	0x1		/* descriptor protocol error */
#define	BHND_D64_STATUS1_ERR_DFOU	0x2		/* data fifo overflow/underrun */
#define	BHND_D64_STATUS1_ERR_DTE	0x3		/* data transfer error */
#define	BHND_D64_STATUS1_ERR_DESRE	0x4		/* descriptor read error */
#define	BHND_D64_STATUS1_ERR_COREE	0x5		/* core error */

/* BHND_D64_STATUS1 (transmit) */
#define	BHND_D64_XS1_AD_MIN_MASK	BHND_D64_STATUS1_AD_MIN_MASK
#define	BHND_D64_XS1_AD_SHIFT		BHND_D64_STATUS1_AD_SHIFT
#define	BHND_D64_XS1_XE_MASK		BHND_D64_STATUS1_ERR_MASK 	/* transmit errors */
#define	BHND_D64_XS1_XE_SHIFT		BHND_D64_STATUS1_ERR_SHIFT
#define	BHND_D64_XS1_XE_NOERR		BHND_D64_STATUS1_ERR_NOERR
#define	BHND_D64_XS1_XE_DPE		BHND_D64_STATUS1_ERR_DPE
#define	BHND_D64_XS1_XE_DFU		BHND_D64_STATUS1_ERR_DFOU	/* data fifo underrun */
#define	BHND_D64_XS1_XE_DTE		BHND_D64_STATUS1_ERR_DTE
#define	BHND_D64_XS1_XE_DESRE		BHND_D64_STATUS1_ERR_DESRE
#define	BHND_D64_XS1_XE_COREE		BHND_D64_STATUS1_ERR_COREE

/* BHND_D64_STATUS1 (receive) */
#define	BHND_D64_RS1_AD_MASK		BHND_D64_STATUS1_AD_MIN_MASK	/* active descriptor */
#define	BHND_D64_RS1_AD_SHIFT		BHND_D64_STATUS1_AD_SHIFT
#define	BHND_D64_RS1_RE_MASK		BHND_D64_STATUS1_ERR_MASK	/* receive errors */
#define	BHND_D64_RS1_RE_SHIFT		BHND_D64_STATUS1_ERR_SHIFT
#define	BHND_D64_RS1_RE_NOERR		BHND_D64_STATUS1_ERR_NOERR	/* no error */
#define	BHND_D64_RS1_RE_DPE		BHND_D64_STATUS1_ERR_DPE	/* descriptor protocol error */
#define	BHND_D64_RS1_RE_DFU		BHND_D64_STATUS1_ERR_DFOU	/* data fifo overflow */
#define	BHND_D64_RS1_RE_DTE		BHND_D64_STATUS1_ERR_DTE	/* data transfer error */
#define	BHND_D64_RS1_RE_DESRE		BHND_D64_STATUS1_ERR_DESRE	/* descriptor read error */
#define	BHND_D64_RS1_RE_COREE		BHND_D64_STATUS1_ERR_COREE	/* core error */

/* fifoaddr */
#define	BHND_DIAG64_FA_OFF_MASK		0xffff		/* offset */
#define	BHND_DIAG64_FA_SEL_MASK		0xf0000		/* select */
#define	BHND_DIAG64_FA_SEL_SHIFT	16
#define	BHND_DIAG64_FA_SEL_XDD		0x0		/* transmit dma data */
#define	BHND_DIAG64_FA_SEL_XDP		0x1		/* transmit dma pointers */
#define	BHND_DIAG64_FA_SEL_RDD		0x4		/* receive dma data */
#define	BHND_DIAG64_FA_SEL_RDP		0x5		/* receive dma pointers */
#define	BHND_DIAG64_FA_SEL_XFD		0x8		/* transmit fifo data */
#define	BHND_DIAG64_FA_SEL_XFP		0x9		/* transmit fifo pointers */
#define	BHND_DIAG64_FA_SEL_RFD		0xc		/* receive fifo data */
#define	BHND_DIAG64_FA_SEL_RFP		0xd		/* receive fifo pointers */
#define	BHND_DIAG64_FA_SEL_RSD		0xe		/* receive frame status data */
#define	BHND_DIAG64_FA_SEL_RSP		0xf		/* receive frame status pointers */

/* descriptor control flags 1 */
#define	BHND_D64_DCTRL1_COREFLAGS	0x0ff00000		/* core specific flags */
#define	BHND_D64_DCTRL1_EOT		0x10000000	/* end of descriptor table */
#define	BHND_D64_DCTRL1_IOC		0x20000000	/* interrupt on completion */
#define	BHND_D64_DCTRL1_EOF		0x40000000	/* end of frame */
#define	BHND_D64_DCTRL1_SOF		0x80000000	/* start of frame */

/* descriptor control flags 2 */
#define	BHND_D64_DCTRL2_BC_MASK		0x00007fff	/* buffer byte count. real data len must <= 16KB */
#define	BHND_D64_DCTRL2_AE		0x00030000	/* address extension bits */
#define	BHND_D64_DCTRL2_AE_SHIFT	16
#define	BHND_D64_DCTRL2_PARITY		0x00040000	/* parity bit */
#define	BHND_D64_DCTRL2_CORE_MASK	0x0ff00000	/* core specific flags */

#define	BHND_D64_RX_FRM_STS_LEN		0x0000ffff	/* frame length mask */
#define	BHND_D64_RX_FRM_STS_OVFL	0x00800000	/* RxOverFlow */
#define	BHND_D64_RX_FRM_STS_DSCRCNT	0x0f000000	/* no. of descriptors used - 1, d11corerev >= 22 */
#define	BHND_D64_RX_FRM_STS_DATATYPE	0xf0000000	/* core-dependent data type */

/**********************************
 * XXX LEGACY DECLARATIONS FOLLOW *
 **********************************/

/* dma registers per channel(xmt or rcv) */
typedef volatile struct {
	uint32_t	control;		/* enable, et al */
	uint32_t	ptr;			/* last descriptor posted to chip */
	uint32_t	addrlow;		/* descriptor ring base address low 32-bits (8K aligned) */
	uint32_t	addrhigh;		/* descriptor ring base address bits 63:32 (8K aligned) */
	uint32_t	status0;		/* current descriptor, xmt state */
	uint32_t	status1;		/* active descriptor, xmt error */
} dma64regs_t;

typedef volatile struct {
	dma64regs_t	tx;		/* dma64 tx channel */
	dma64regs_t	rx;		/* dma64 rx channel */
} dma64regp_t;

typedef volatile struct {		/* diag access */
	uint32_t	fifoaddr;		/* diag address */
	uint32_t	fifodatalow;		/* low 32bits of data */
	uint32_t	fifodatahigh;		/* high 32bits of data */
	uint32_t	pad;			/* reserved */
} dma64diag_t;

/*
 * DMA Descriptor
 * Descriptors are only read by the hardware, never written back.
 */
typedef volatile struct {
	uint32_t	ctrl1;		/* misc control bits */
	uint32_t	ctrl2;		/* buffer count and address extension */
	uint32_t	addrlow;	/* memory address of the date buffer, bits 31:0 */
	uint32_t	addrhigh;	/* memory address of the date buffer, bits 63:32 */
} dma64dd_t;

/*
 * Each descriptor ring must be 8kB aligned, and fit within a contiguous 8kB physical addresss.
 */
#define	D64RINGALIGN_BITS	13
#define	D64MAXRINGSZ		(1 << D64RINGALIGN_BITS)
#define	D64RINGBOUNDARY		(1 << D64RINGALIGN_BITS)

#define	D64MAXDD	(D64MAXRINGSZ / sizeof (dma64dd_t))

/* for cores with large descriptor ring support, descriptor ring size can be up to 4096 */
#define	D64MAXDD_LARGE		((1 << 16) / sizeof (dma64dd_t))

/* for cores with large descriptor ring support (4k descriptors), descriptor ring cannot cross
 * 64K boundary
 */
#define	D64RINGBOUNDARY_LARGE	(1 << 16)

/*
 * Default DMA Burstlen values for USBRev >= 12 and SDIORev >= 11.
 * When this field contains the value N, the burst length is 2**(N + 4) bytes.
 */
#define	D64_DEF_USBBURSTLEN	 2
#define	D64_DEF_SDIOBURSTLEN	1


#ifndef D64_USBBURSTLEN
#define	D64_USBBURSTLEN	DMA_BL_64
#endif
#ifndef D64_SDIOBURSTLEN
#define	D64_SDIOBURSTLEN	DMA_BL_32
#endif

/* transmit channel control */
#define	D64_XC_XE		0x00000001	/* transmit enable */
#define	D64_XC_SE		0x00000002	/* transmit suspend request */
#define	D64_XC_LE		0x00000004	/* loopback enable */
#define	D64_XC_FL		0x00000010	/* flush request */
#define	D64_XC_MR_MASK		0x000000C0	/* Multiple outstanding reads */
#define	D64_XC_MR_SHIFT		6
#define	D64_XC_PD		0x00000800	/* parity check disable */
#define	D64_XC_AE		0x00030000	/* address extension bits */
#define	D64_XC_AE_SHIFT		16
#define	D64_XC_BL_MASK		0x001C0000	/* BurstLen bits */
#define	D64_XC_BL_SHIFT		18
#define	D64_XC_PC_MASK		0x00E00000		/* Prefetch control */
#define	D64_XC_PC_SHIFT		21
#define	D64_XC_PT_MASK		0x03000000		/* Prefetch threshold */
#define	D64_XC_PT_SHIFT		24

/* transmit descriptor table pointer */
#define	D64_XP_LD_MASK		0x00001fff	/* last valid descriptor */

/* transmit channel status */
#define	D64_XS0_CD_MASK		(di->status0_cd_mask)	/* current descriptor pointer */
#define	D64_XS0_XS_MASK		0xf0000000	 	/* transmit state */
#define	D64_XS0_XS_SHIFT		28
#define	D64_XS0_XS_DISABLED	0x00000000	/* disabled */
#define	D64_XS0_XS_ACTIVE	0x10000000	/* active */
#define	D64_XS0_XS_IDLE		0x20000000	/* idle wait */
#define	D64_XS0_XS_STOPPED	0x30000000	/* stopped */
#define	D64_XS0_XS_SUSP		0x40000000	/* suspend pending */

#define	D64_XS1_AD_MASK		(di->status1_ad_mask)	/* active descriptor */
#define	D64_XS1_XE_MASK		0xf0000000	 	/* transmit errors */
#define	D64_XS1_XE_SHIFT	28
#define	D64_XS1_XE_NOERR	0x00000000	/* no error */
#define	D64_XS1_XE_DPE		0x10000000	/* descriptor protocol error */
#define	D64_XS1_XE_DFU		0x20000000	/* data fifo underrun */
#define	D64_XS1_XE_DTE		0x30000000	/* data transfer error */
#define	D64_XS1_XE_DESRE	0x40000000	/* descriptor read error */
#define	D64_XS1_XE_COREE	0x50000000	/* core error */

/* receive channel control */
#define	D64_RC_RE		0x00000001	/* receive enable */
#define	D64_RC_RO_MASK		0x000000fe	/* receive frame offset */
#define	D64_RC_RO_SHIFT		1
#define	D64_RC_FM		0x00000100	/* direct fifo receive (pio) mode */
#define	D64_RC_SH		0x00000200	/* separate rx header descriptor enable */
#define	D64_RC_OC		0x00000400	/* overflow continue */
#define	D64_RC_PD		0x00000800	/* parity check disable */
#define	D64_RC_GE		0x00004000	/* Glom enable */
#define	D64_RC_AE		0x00030000	/* address extension bits */
#define	D64_RC_AE_SHIFT		16
#define	D64_RC_BL_MASK		0x001C0000	/* BurstLen bits */
#define	D64_RC_BL_SHIFT		18
#define	D64_RC_PC_MASK		0x00E00000	/* Prefetch control */
#define	D64_RC_PC_SHIFT		21
#define	D64_RC_PT_MASK		0x03000000	/* Prefetch threshold */
#define	D64_RC_PT_SHIFT		24

/* flags for dma controller */
#define	DMA_CTRL_PEN		(1 << 0)	/* partity enable */
#define	DMA_CTRL_ROC		(1 << 1)	/* rx overflow continue */
#define	DMA_CTRL_RXMULTI	(1 << 2)	/* allow rx scatter to multiple descriptors */
#define	DMA_CTRL_UNFRAMED	(1 << 3)	/* Unframed Rx/Tx data */
#define	DMA_CTRL_USB_BOUNDRY4KB_WAR (1 << 4)
#define	DMA_CTRL_DMA_AVOIDANCE_WAR (1 << 5)	/* DMA avoidance WAR for 4331 */
#define	DMA_CTRL_RXSINGLE	(1 << 6)	/* always single buffer */

/* receive descriptor table pointer */
#define	D64_RP_LD_MASK		0x00001fff	/* last valid descriptor */

/* receive channel status */
#define	D64_RS0_CD_MASK		(di->status0_cd_mask)	/* current descriptor pointer */
#define	D64_RS0_RS_MASK		0xf0000000	 	/* receive state */
#define	D64_RS0_RS_SHIFT		28
#define	D64_RS0_RS_DISABLED	0x00000000	/* disabled */
#define	D64_RS0_RS_ACTIVE	0x10000000	/* active */
#define	D64_RS0_RS_IDLE		0x20000000	/* idle wait */
#define	D64_RS0_RS_STOPPED	0x30000000	/* stopped */
#define	D64_RS0_RS_SUSP		0x40000000	/* suspend pending */

#define	D64_RS1_AD_MASK		0x0001ffff	/* active descriptor */
#define	D64_RS1_RE_MASK		0xf0000000	 	/* receive errors */
#define	D64_RS1_RE_SHIFT		28
#define	D64_RS1_RE_NOERR	0x00000000	/* no error */
#define	D64_RS1_RE_DPO		0x10000000	/* descriptor protocol error */
#define	D64_RS1_RE_DFU		0x20000000	/* data fifo overflow */
#define	D64_RS1_RE_DTE		0x30000000	/* data transfer error */
#define	D64_RS1_RE_DESRE	0x40000000	/* descriptor read error */
#define	D64_RS1_RE_COREE	0x50000000	/* core error */

/* fifoaddr */
#define	D64_FA_OFF_MASK		0xffff		/* offset */
#define	D64_FA_SEL_MASK		0xf0000		/* select */
#define	D64_FA_SEL_SHIFT	16
#define	D64_FA_SEL_XDD		0x00000		/* transmit dma data */
#define	D64_FA_SEL_XDP		0x10000		/* transmit dma pointers */
#define	D64_FA_SEL_RDD		0x40000		/* receive dma data */
#define	D64_FA_SEL_RDP		0x50000		/* receive dma pointers */
#define	D64_FA_SEL_XFD		0x80000		/* transmit fifo data */
#define	D64_FA_SEL_XFP		0x90000		/* transmit fifo pointers */
#define	D64_FA_SEL_RFD		0xc0000		/* receive fifo data */
#define	D64_FA_SEL_RFP		0xd0000		/* receive fifo pointers */
#define	D64_FA_SEL_RSD		0xe0000		/* receive frame status data */
#define	D64_FA_SEL_RSP		0xf0000		/* receive frame status pointers */

/* descriptor control flags 1 */
#define	D64_CTRL_COREFLAGS	0x0ff00000	/* core specific flags */
#define	D64_CTRL1_EOT		((uint32_t)1 << 28)	/* end of descriptor table */
#define	D64_CTRL1_IOC		((uint32_t)1 << 29)	/* interrupt on completion */
#define	D64_CTRL1_EOF		((uint32_t)1 << 30)	/* end of frame */
#define	D64_CTRL1_SOF		((uint32_t)1 << 31)	/* start of frame */

/* descriptor control flags 2 */
#define	D64_CTRL2_BC_MASK	0x00007fff	/* buffer byte count. real data len must <= 16KB */
#define	D64_CTRL2_AE		0x00030000	/* address extension bits */
#define	D64_CTRL2_AE_SHIFT	16
#define	D64_CTRL2_PARITY	0x00040000	/* parity bit */

/* control flags in the range [27:20] are core-specific and not defined here */
#define	D64_CTRL_CORE_MASK	0x0ff00000

#define	D64_RX_FRM_STS_LEN	0x0000ffff	/* frame length mask */
#define	D64_RX_FRM_STS_OVFL	0x00800000	/* RxOverFlow */
#define	D64_RX_FRM_STS_DSCRCNT	0x0f000000	/* no. of descriptors used - 1, d11corerev >= 22 */
#define	D64_RX_FRM_STS_DATATYPE	0xf0000000	/* core-dependent data type */

/* receive frame status */
typedef volatile struct {
	uint16_t len;
	uint16_t flags;
} dma_rxh_t;

#endif /* _BHND_DMA_BHND_DMA64REG_H_ */
