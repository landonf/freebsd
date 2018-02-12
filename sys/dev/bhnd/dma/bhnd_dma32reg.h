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
#define XC_MR_MASK	0x000000C0		/* Multiple outstanding reads */
#define XC_MR_SHIFT	6
#define	XC_PD		((uint32_t)1 << 11)	/* parity check disable */
#define	XC_AE		((uint32_t)3 << 16)	/* address extension bits */
#define	XC_AE_SHIFT	16
#define XC_BL_MASK	0x001C0000		/* BurstLen bits */
#define XC_BL_SHIFT	18
#define XC_PC_MASK	0x00E00000		/* Prefetch control */
#define XC_PC_SHIFT	21
#define XC_PT_MASK	0x03000000		/* Prefetch threshold */
#define XC_PT_SHIFT	24

/* Multiple outstanding reads */
#define DMA_MR_1	0
#define DMA_MR_2	1
/* 2, 3: reserved */

/* DMA Burst Length in bytes */
#define DMA_BL_16	0
#define DMA_BL_32	1
#define DMA_BL_64	2
#define DMA_BL_128	3
#define DMA_BL_256	4
#define DMA_BL_512	5
#define DMA_BL_1024	6

/* Prefetch control */
#define DMA_PC_0	0
#define DMA_PC_4	1
#define DMA_PC_8	2
#define DMA_PC_16	3
/* others: reserved */

/* Prefetch threshold */
#define DMA_PT_1	0
#define DMA_PT_2	1
#define DMA_PT_4	2
#define DMA_PT_8	3

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
#define RC_BL_MASK	0x001C0000		/* BurstLen bits */
#define RC_BL_SHIFT	18
#define RC_PC_MASK	0x00E00000		/* Prefetch control */
#define RC_PC_SHIFT	21
#define RC_PT_MASK	0x03000000		/* Prefetch threshold */
#define RC_PT_SHIFT	24

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
