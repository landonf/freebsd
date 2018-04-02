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

#ifndef _BHND_DMA_BHND_DMA32REG_H_
#define _BHND_DMA_BHND_DMA32REG_H_

#define	BHND_D32_ADDREXT_MASK	0xC0000000		/**< mask of extended address bits */
#define	BHND_D32_ADDREXT_SHIFT	30

#define	BHND_D32_ADDRWIDTH_BASE	BHND_DMA_ADDR_30BIT	/**< usable host address width of 32-bit DMA channel without addrext support */
#define	BHND_D32_ADDRWIDTH_EXT	BHND_D32_ADDRWIDTH	/**< usable host address width of 32-bit DMA channel with addrext support */
#define	BHND_D32_ADDRWIDTH	BHND_DMA_ADDR_32BIT	/**< device address width of 32-bit DMA channel */

#define	BHND_D32_CHAN_SIZE	16			/**< size of single 32-bit DMA channel register block */
#define	BHND_D32_CHAN_PAIR_SIZE	(BHND_D32_CHAN_SIZE*2)	/**< size of a 32-bit DMA channel pair's register block */

#define	BHND_D32_TX_OFFSET	0			/**< TX offset within 32-bit channel pair */
#define	BHND_D32_RX_OFFSET	BHND_D32_CHAN_SIZE	/**< RX offset within 32-bit channel pair */

#define	BHND_D32_DESC_SIZE	sizeof(dma32dd_t)	/**< 32-bit DMA descriptor size */

#define	BHND_D32_DIAG_SIZE	16			/**< 32-bit DMA diagnostic register block size */

/**
 * Register offset to the @p _num 32-bit channel pair.
 */
#define	BHND_D32_CHAN_PAIR_OFFSET(_num)	\
	(BHND_D32_CHAN_PAIR_SIZE * (_num))

/**
 * Register offset to the 32-bit channel with @p _dir and @p _num.
 */
#define	BHND_D32_CHAN_OFFSET(_dir, _num) (				\
	(_dir) == BHND_DMA_TX ?						\
	     (BHND_D32_CHAN_PAIR_OFFSET(_num) + BHND_D32_TX_OFFSET) :	\
	     (BHND_D32_CHAN_PAIR_OFFSET(_num) + BHND_D32_RX_OFFSET)	\
)

/* DMA per-channel (TX or RX) register offsets */
#define	BHND_D32_CTRL			0x00			/* channel control (enable, et al) */
#define	BHND_D32_ADDR			0x04			/* descriptor ring base address, bits 0:31 (4K aligned) */
#define	BHND_D32_PTR			0x08			/* last descriptor posted to chip */
#define	BHND_D32_STATUS			0x0C			/* channel status */
#define	BHND_D32_STATUS0		BHND_D32_STATUS		/* for compatibility with D64 */
#define	BHND_D32_STATUS1		BHND_D32_STATUS		/* for compatibility with D64 */

/* BHND_D32_CTRL (common) */
#define	BHND_D32_CTRL_EN		0x00000001		/* transmit/receive enable */
#define	BHND_D32_CTRL_PD		0x00000800		/* parity check disable */
#define	BHND_D32_CTRL_AE_MASK		0x00030000		/* address extension bits */
#define	BHND_D32_CTRL_AE_SHIFT		16
#define	BHND_D32_CTRL_BL_MASK		0x001C0000		/* burstlen bits */
#define	BHND_D32_CTRL_BL_SHIFT		18
#define	BHND_D32_CTRL_PC_MASK		0x00E00000		/* prefetch control */
#define	BHND_D32_CTRL_PC_SHIFT		21
#define	BHND_D32_CTRL_PT_MASK		0x03000000		/* prefetch threshold */
#define	BHND_D32_CTRL_PT_SHIFT		24

/* BHND_D32_CTRL (transmit) */
#define	BHND_D32_XC_XE			BHND_D32_CTRL_EN	/* transmit enable */
#define	BHND_D32_XC_SE			0x00000002		/* transmit suspend request */
#define	BHND_D32_XC_LE			0x00000004		/* loopback enable */
#define	BHND_D32_XC_FL			0x00000010		/* flush request */
#define	BHND_D32_XC_MR_MASK		0x000000C0		/* multiple outstanding reads */
#define	BHND_D32_XC_MR_SHIFT		6
#define	BHND_D32_XC_PD			BHND_D32_CTRL_PD
#define	BHND_D32_XC_AE_MASK		BHND_D32_CTRL_AE_MASK
#define	BHND_D32_XC_AE_SHIFT		BHND_D32_CTRL_AE_SHIFT
#define	BHND_D32_XC_BL_MASK		BHND_D32_CTRL_BL_MASK
#define	BHND_D32_XC_BL_SHIFT		BHND_D32_CTRL_BL_SHIFT
#define	BHND_D32_XC_PC_MASK		BHND_D32_CTRL_PC_MASK
#define	BHND_D32_XC_PC_SHIFT		BHND_D32_CTRL_PC_SHIFT
#define	BHND_D32_XC_PT_MASK		BHND_D32_CTRL_PT_MASK
#define	BHND_D32_XC_PT_SHIFT		BHND_D32_CTRL_PT_SHIFT

/* BHND_D32_CTRL (receive) */
#define	BHND_D32_RC_RE			BHND_D32_CTRL_EN	/* receive enable */
#define	BHND_D32_RC_RO_MASK		0x000000fe		/* receive frame offset */
#define	BHND_D32_RC_RO_SHIFT		1
#define	BHND_D32_RC_FM			0x00000100		/* direct fifo receive (pio) mode */
#define	BHND_D32_RC_SH			0x00000200		/* separate rx header descriptor enable */
#define	BHND_D32_RC_OC			0x00000400		/* overflow continue */
#define	BHND_D32_RC_PD			BHND_D32_CTRL_PD
#define	BHND_D32_RC_AE_MASK		BHND_D32_CTRL_AE_MASK
#define	BHND_D32_RC_AE_SHIFT		BHND_D32_CTRL_AE_SHIFT
#define	BHND_D32_RC_BL_MASK		BHND_D32_CTRL_BL_MASK
#define	BHND_D32_RC_BL_SHIFT		BHND_D32_CTRL_BL_SHIFT
#define	BHND_D32_RC_PC_MASK		BHND_D32_CTRL_PC_MASK
#define	BHND_D32_RC_PC_SHIFT		BHND_D32_CTRL_PC_SHIFT
#define	BHND_D32_RC_PT_MASK		BHND_D32_CTRL_PT_MASK
#define	BHND_D32_RC_PT_SHIFT		BHND_D32_CTRL_PT_SHIFT


/* BHND_D32_PTR (common) */
#define	BHND_D32_PTR_LD_MASK		0xfff		/* last valid descriptor */

/* BHND_D32_STATUS (common) */
#define	BHND_D32_STATUS_CD_MASK		0x00000fff	/* current descriptor pointer */
#define	BHND_D32_STATUS_CD_SHIFT	0
#define	BHND_D32_STATUS_ST_MASK		0x0000f000	/* transmit/receive state */
#define	BHND_D32_STATUS_ST_SHIFT	12
#define	BHND_D32_STATUS_ST_DISABLED	0x0		/* disabled */
#define	BHND_D32_STATUS_ST_ACTIVE	0x1		/* active */
#define	BHND_D32_STATUS_ST_IDLE		0x2		/* idle wait */
#define	BHND_D32_STATUS_ERR_MASK	0x000f0000	/* transmit/receive errors */
#define	BHND_D32_STATUS_ERR_SHIFT	16
#define	BHND_D32_STATUS_ERR_NOERR	0x0		/* no error */
#define	BHND_D32_STATUS_ERR_DPE		0x1		/* descriptor protocol error */
#define	BHND_D32_STATUS_ERR_DFOU	0x2		/* data fifo overflow/underrun */
#define	BHND_D32_STATUS_ERR_BEBRW	0x3		/* bus error on buffer read/write */
#define	BHND_D32_STATUS_ERR_BEDA	0x4		/* bus error on descriptor access */
#define	BHND_D32_STATUS_AD_MASK		0xfff00000	/* active descriptor */
#define	BHND_D32_STATUS_AD_SHIFT	20

/* BHND_D32_STATUS (common, D64 compatibility) */
#define	BHND_D32_STATUS0_CD_MASK	BHND_D32_STATUS_CD_MASK
#define	BHND_D32_STATUS0_CD_SHIFT	BHND_D32_STATUS_CD_SHIFT
#define	BHND_D32_STATUS1_AD_MASK	BHND_D32_STATUS_AD_MASK
#define	BHND_D32_STATUS1_AD_SHIFT	BHND_D32_STATUS_AD_SHIFT

/* Transmit BHND_D32_STATUS fields */
#define	BHND_D32_XS_CD_MASK	BHND_D32_STATUS_CD_MASK
#define	BHND_D32_XS_CD_SHIFT	BHND_D32_STATUS_CD_SHIFT
#define	BHND_D32_XS_XS_MASK	BHND_D32_STATUS_ST_MASK		/* transmit state */
#define	BHND_D32_XS_XS_SHIFT	BHND_D32_STATUS_ST_SHIFT
#define	BHND_D32_XS_XS_DISABLED	BHND_D32_STATUS_ST_DISABLED
#define	BHND_D32_XS_XS_ACTIVE	BHND_D32_STATUS_ST_ACTIVE
#define	BHND_D32_XS_XS_IDLE	BHND_D32_STATUS_ST_IDLE
#define	BHND_D32_XS_XS_STOPPED	0x3				/* stopped */
#define	BHND_D32_XS_XS_SUSP	0x4				/* suspend pending */
#define	BHND_D32_XS_XE_MASK	BHND_D32_STATUS_ERR_MASK	/* transmit errors */
#define	BHND_D32_XS_XE_SHIFT	BHND_D32_STATUS_ERR_SHIFT
#define	BHND_D32_XS_XE_NOERR	BHND_D32_STATUS_ERR_NOERR
#define	BHND_D32_XS_XE_DPE	BHND_D32_STATUS_ERR_DPE
#define	BHND_D32_XS_XE_DFU	BHND_D32_STATUS_ERR_DFOU	/* data fifo underrun */
#define	BHND_D32_XS_XE_BEBR	BHND_D32_STATUS_ERR_BEBRW	/* bus error on buffer read */
#define	BHND_D32_XS_XE_BEDA	BHND_D32_STATUS_ERR_BEDA
#define	BHND_D32_XS_AD_MASK	BHND_D32_STATUS_AD_MASK		/* active descriptor */
#define	BHND_D32_XS_AD_SHIFT	BHND_D32_STATUS_AD_SHIFT

/* Receive BHND_D32_STATUS fields */
#define	BHND_D32_RS_CD_MASK	BHND_D32_STATUS_CD_MASK
#define	BHND_D32_RS_CD_SHIFT	BHND_D32_STATUS_CD_SHIFT
#define	BHND_D32_RS_RS_MASK	BHND_D32_STATUS_ST_MASK		/* receive state */
#define	BHND_D32_RS_RS_SHIFT	BHND_D32_STATUS_ST_SHIFT
#define	BHND_D32_RS_RS_DISABLED	BHND_D32_STATUS_ST_DISABLED
#define	BHND_D32_RS_RS_ACTIVE	BHND_D32_STATUS_ST_ACTIVE
#define	BHND_D32_RS_RS_IDLE	BHND_D32_STATUS_ST_IDLE
#define	BHND_D32_RS_RS_STOPPED	0x3				/* reserved */
#define	BHND_D32_RS_RE_MASK	BHND_D32_STATUS_ERR_MASK	/* receive errors */
#define	BHND_D32_RS_RE_SHIFT	BHND_D32_STATUS_ERR_SHIFT
#define	BHND_D32_RS_RE_NOERR	BHND_D32_STATUS_ERR_NOERR
#define	BHND_D32_RS_RE_DPE	BHND_D32_STATUS_ERR_DPE
#define	BHND_D32_RS_RE_DFO	BHND_D32_STATUS_ERR_DFOU	/* data fifo overflow */
#define	BHND_D32_RS_RE_BEBW	BHND_D32_STATUS_ERR_BEBRW	/* bus error on buffer write */
#define	BHND_D32_RS_RE_BEDA	BHND_D32_STATUS_ERR_BEDA
#define	BHND_D32_RS_AD_MASK	BHND_D32_STATUS_AD_MASK
#define	BHND_D32_RS_AD_SHIFT	BHND_D32_STATUS_AD_SHIFT

/* DMA diagnostic register offsets */
#define	BHND_DIAG32_FIFO_ADDR		0x00		/* diag address */
#define	BHND_DIAG32_FIFO_DATA_LOW	0x04		/* low 32-bits of data */
#define	BHND_DIAG32_FIFO_DATA_HIGH	0x08		/* low 32-bits of data */

/* BHND_DIAG32_FIFO_ADDR fields */
#define	BHND_DIAG32_FA_OFF_MASK		0xffff		/* offset */
#define	BHND_DIAG32_FA_SEL_MASK		0xf0000		/* select */
#define	BHND_DIAG32_FA_SEL_SHIFT	16
#define	BHND_DIAG32_FA_SEL_XDD		0x00000		/* transmit dma data */
#define	BHND_DIAG32_FA_SEL_XDP		0x10000		/* transmit dma pointers */
#define	BHND_DIAG32_FA_SEL_RDD		0x40000		/* receive dma data */
#define	BHND_DIAG32_FA_SEL_RDP		0x50000		/* receive dma pointers */
#define	BHND_DIAG32_FA_SEL_XFD		0x80000		/* transmit fifo data */
#define	BHND_DIAG32_FA_SEL_XFP		0x90000		/* transmit fifo pointers */
#define	BHND_DIAG32_FA_SEL_RFD		0xc0000		/* receive fifo data */
#define	BHND_DIAG32_FA_SEL_RFP		0xd0000		/* receive fifo pointers */
#define	BHND_DIAG32_FA_SEL_RSD		0xe0000		/* receive frame status data */
#define	BHND_DIAG32_FA_SEL_RSP		0xf0000		/* receive frame status pointers */

/* DMA descriptor control flags */
#define	BHND_D32_DCTRL_BC_MASK		0x00001fff		/* buffer byte count, real data len must <= 4KB */
#define	BHND_D32_DCTRL_AE		((uint32_t)3 << 16)	/* address extension bits */
#define	BHND_D32_DCTRL_AE_SHIFT		16
#define	BHND_D32_DCTRL_PARITY		((uint32_t)3 << 18)	/* parity bit */
#define	BHND_D32_DCTRL_EOT		((uint32_t)1 << 28)	/* end of descriptor table */
#define	BHND_D32_DCTRL_IOC		((uint32_t)1 << 29)	/* interrupt on completion */
#define	BHND_D32_DCTRL_EOF		((uint32_t)1 << 30)	/* end of frame */
#define	BHND_D32_DCTRL_SOF		((uint32_t)1 << 31)	/* start of frame */
#define	BHND_D32_DCTRL_CORE_MASK	0x0ff00000		/* core-specific control flags */
#define	BHND_D32_DCTRL_CORE_SHIFT	20

/**********************************
 * XXX LEGACY DECLARATIONS FOLLOW *
 **********************************/

/* dma registers per channel(xmt or rcv) */
typedef volatile struct {
	uint32_t	control;		/* enable, et al */
	uint32_t	addr;			/* descriptor ring base address (4K aligned) */
	uint32_t	ptr;			/* last descriptor posted to chip */
	uint32_t	status;			/* current active descriptor, et al */
} dma32regs_t;

typedef volatile struct {
	dma32regs_t	xmt;		/* dma tx channel */
	dma32regs_t	rcv;		/* dma rx channel */
} dma32regp_t;

typedef volatile struct {	/* diag access */
	uint32_t	fifoaddr;		/* diag address */
	uint32_t	fifodatalow;		/* low 32bits of data */
	uint32_t	fifodatahigh;		/* high 32bits of data */
	uint32_t	pad;			/* reserved */
} dma32diag_t;

/*
 * DMA Descriptor
 * Descriptors are only read by the hardware, never written back.
 */
typedef volatile struct {
	uint32_t	ctrl;		/* misc control bits & bufcount */
	uint32_t	addr;		/* data buffer address */
} dma32dd_t;

/*
 * Each descriptor ring must be 4096byte aligned, and fit within a single 4096byte page.
 */
#define	D32RINGALIGN_BITS	12
#define	D32MAXRINGSZ		(1 << D32RINGALIGN_BITS)
#define	D32RINGALIGN		(1 << D32RINGALIGN_BITS)

#define	D32MAXDD	(D32MAXRINGSZ / sizeof (dma32dd_t))

/* transmit channel control */
#define	XC_XE		((uint32_t)1 << 0)	/* transmit enable */
#define	XC_SE		((uint32_t)1 << 1)	/* transmit suspend request */
#define	XC_LE		((uint32_t)1 << 2)	/* loopback enable */
#define	XC_FL		((uint32_t)1 << 4)	/* flush request */
#define	XC_MR_MASK	0x000000C0		/* Multiple outstanding reads */
#define	XC_MR_SHIFT	6
#define	XC_PD		((uint32_t)1 << 11)	/* parity check disable */
#define	XC_AE		((uint32_t)3 << 16)	/* address extension bits */
#define	XC_AE_SHIFT	16
#define	XC_BL_MASK	0x001C0000		/* BurstLen bits */
#define	XC_BL_SHIFT	18
#define	XC_PC_MASK	0x00E00000		/* Prefetch control */
#define	XC_PC_SHIFT	21
#define	XC_PT_MASK	0x03000000		/* Prefetch threshold */
#define	XC_PT_SHIFT	24

/* transmit descriptor table pointer */
#define	XP_LD_MASK	0xfff			/* last valid descriptor */

/* transmit channel status */
#define	XS_CD_MASK	0x0fff			/* current descriptor pointer */
#define	XS_XS_MASK	0xf000			/* transmit state */
#define	XS_XS_SHIFT	12
#define	XS_XS_DISABLED	0x0000			/* disabled */
#define	XS_XS_ACTIVE	0x1000			/* active */
#define	XS_XS_IDLE	0x2000			/* idle wait */
#define	XS_XS_STOPPED	0x3000			/* stopped */
#define	XS_XS_SUSP	0x4000			/* suspend pending */
#define	XS_XE_MASK	0xf0000			/* transmit errors */
#define	XS_XE_SHIFT	16
#define	XS_XE_NOERR	0x00000			/* no error */
#define	XS_XE_DPE	0x10000			/* descriptor protocol error */
#define	XS_XE_DFU	0x20000			/* data fifo underrun */
#define	XS_XE_BEBR	0x30000			/* bus error on buffer read */
#define	XS_XE_BEDA	0x40000			/* bus error on descriptor access */
#define	XS_AD_MASK	0xfff00000		/* active descriptor */
#define	XS_AD_SHIFT	20

/* receive channel control */
#define	RC_RE		((uint32_t)1 << 0)	/* receive enable */
#define	RC_RO_MASK	0xfe			/* receive frame offset */
#define	RC_RO_SHIFT	1
#define	RC_FM		((uint32_t)1 << 8)	/* direct fifo receive (pio) mode */
#define	RC_SH		((uint32_t)1 << 9)	/* separate rx header descriptor enable */
#define	RC_OC		((uint32_t)1 << 10)	/* overflow continue */
#define	RC_PD		((uint32_t)1 << 11)	/* parity check disable */
#define	RC_AE		((uint32_t)3 << 16)	/* address extension bits */
#define	RC_AE_SHIFT	16
#define	RC_BL_MASK	0x001C0000		/* BurstLen bits */
#define	RC_BL_SHIFT	18
#define	RC_PC_MASK	0x00E00000		/* Prefetch control */
#define	RC_PC_SHIFT	21
#define	RC_PT_MASK	0x03000000		/* Prefetch threshold */
#define	RC_PT_SHIFT	24

/* receive descriptor table pointer */
#define	RP_LD_MASK	0xfff			/* last valid descriptor */

/* receive channel status */
#define	RS_CD_MASK	0x0fff			/* current descriptor pointer */
#define	RS_RS_MASK	0xf000			/* receive state */
#define	RS_RS_SHIFT	12
#define	RS_RS_DISABLED	0x0000			/* disabled */
#define	RS_RS_ACTIVE	0x1000			/* active */
#define	RS_RS_IDLE	0x2000			/* idle wait */
#define	RS_RS_STOPPED	0x3000			/* reserved */
#define	RS_RE_MASK	0xf0000			/* receive errors */
#define	RS_RE_SHIFT	16
#define	RS_RE_NOERR	0x00000			/* no error */
#define	RS_RE_DPE	0x10000			/* descriptor protocol error */
#define	RS_RE_DFO	0x20000			/* data fifo overflow */
#define	RS_RE_BEBW	0x30000			/* bus error on buffer write */
#define	RS_RE_BEDA	0x40000			/* bus error on descriptor access */
#define	RS_AD_MASK	0xfff00000		/* active descriptor */
#define	RS_AD_SHIFT	20

/* fifoaddr */
#define	FA_OFF_MASK	0xffff			/* offset */
#define	FA_SEL_MASK	0xf0000			/* select */
#define	FA_SEL_SHIFT	16
#define	FA_SEL_XDD	0x00000			/* transmit dma data */
#define	FA_SEL_XDP	0x10000			/* transmit dma pointers */
#define	FA_SEL_RDD	0x40000			/* receive dma data */
#define	FA_SEL_RDP	0x50000			/* receive dma pointers */
#define	FA_SEL_XFD	0x80000			/* transmit fifo data */
#define	FA_SEL_XFP	0x90000			/* transmit fifo pointers */
#define	FA_SEL_RFD	0xc0000			/* receive fifo data */
#define	FA_SEL_RFP	0xd0000			/* receive fifo pointers */
#define	FA_SEL_RSD	0xe0000			/* receive frame status data */
#define	FA_SEL_RSP	0xf0000			/* receive frame status pointers */

/* descriptor control flags */
#define	CTRL_BC_MASK	0x00001fff		/* buffer byte count, real data len must <= 4KB */
#define	CTRL_AE		((uint32_t)3 << 16)	/* address extension bits */
#define	CTRL_AE_SHIFT	16
#define	CTRL_PARITY	((uint32_t)3 << 18)	/* parity bit */
#define	CTRL_EOT	((uint32_t)1 << 28)	/* end of descriptor table */
#define	CTRL_IOC	((uint32_t)1 << 29)	/* interrupt on completion */
#define	CTRL_EOF	((uint32_t)1 << 30)	/* end of frame */
#define	CTRL_SOF	((uint32_t)1 << 31)	/* start of frame */

/* control flags in the range [27:20] are core-specific and not defined here */
#define	CTRL_CORE_MASK	0x0ff00000

#endif /* _BHND_DMA_BHND_DMA32REG_H_ */
