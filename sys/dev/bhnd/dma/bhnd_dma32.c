/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2018 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2015 Broadcom Corporation. All Rights Reserved.
 * All rights reserved.
 *
 * Portions of this file were derived from the hnddma.c source distributed with
 * the Asus RT-N18 firmware source code release.
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
 * $Id: hnddma.c 497985 2014-08-21 10:43:51Z $
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/endian.h>
#include <sys/libkern.h>
#include <sys/malloc.h>
#undef MALLOC
#undef FREE
#include <sys/sbuf.h>
#include <sys/systm.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h>

#include "bhnd_dmavar.h"

#include "bhnd_dma32var.h"
#include "bhnd_dma32reg.h"

/* XXX required until we adopt bhnd(9) */
#include "siutils.h"


/* Prototypes for 32-bit routines */
static bool dma32_txreset(dma_info_t *di);
static bool dma32_rxreset(dma_info_t *di);
static bool dma32_txsuspendedidle(dma_info_t *di);
static int  dma32_txfast(dma_info_t *di, void *p0, bool commit);
static void *dma32_getnexttxp(dma_info_t *di, txd_range_t range);
static void dma32_txrotate(dma_info_t *di);
static void dma32_txinit(dma_info_t *di);
static bool dma32_txenabled(dma_info_t *di);
static void dma32_txsuspend(dma_info_t *di);
static void dma32_txresume(dma_info_t *di);
static bool dma32_txsuspended(dma_info_t *di);
static void dma32_txflush(dma_info_t *di);
static void dma32_txflush_clear(dma_info_t *di);
static void dma32_txreclaim(dma_info_t *di, txd_range_t range);
static bool dma32_txstopped(dma_info_t *di);
static bool dma32_rxstopped(dma_info_t *di);
static bool dma32_rxenabled(dma_info_t *di);
#if defined(BCMDBG)
static void dma32_dumpring(dma_info_t *di, struct sbuf *s, dma32dd_t *ring, u_int start,
	u_int end, u_int max_num);
static void dma32_dump(dma_info_t *di, struct sbuf *s, bool dumpring);
static void dma32_dumptx(dma_info_t *di, struct sbuf *s, bool dumpring);
static void dma32_dumprx(dma_info_t *di, struct sbuf *s, bool dumpring);
#endif 

const di_fcn_t dma32proc = {
	(di_detach_t)_dma_detach,
	(di_txinit_t)dma32_txinit,
	(di_txreset_t)dma32_txreset,
	(di_txenabled_t)dma32_txenabled,
	(di_txsuspend_t)dma32_txsuspend,
	(di_txresume_t)dma32_txresume,
	(di_txsuspended_t)dma32_txsuspended,
	(di_txsuspendedidle_t)dma32_txsuspendedidle,
	(di_txflush_t)dma32_txflush,
	(di_txflush_clear_t)dma32_txflush_clear,
	(di_txfast_t)dma32_txfast,
	NULL,
	NULL,
	(di_txstopped_t)dma32_txstopped,
	(di_txreclaim_t)dma32_txreclaim,
	(di_getnexttxp_t)dma32_getnexttxp,
	(di_peeknexttxp_t)_dma_peeknexttxp,
	(di_peekntxp_t)_dma_peekntxp,
	(di_txblock_t)_dma_txblock,
	(di_txunblock_t)_dma_txunblock,
	(di_txactive_t)_dma_txactive,
	(di_txrotate_t)dma32_txrotate,

	(di_rxinit_t)_dma_rxinit,
	(di_rxreset_t)dma32_rxreset,
	(di_rxidle_t)dma32_rxidle,
	(di_rxstopped_t)dma32_rxstopped,
	(di_rxenable_t)_dma_rxenable,
	(di_rxenabled_t)dma32_rxenabled,
	(di_rx_t)_dma_rx,
	(di_rxfill_t)_dma_rxfill,
	(di_rxreclaim_t)_dma_rxreclaim,
	(di_getnextrxp_t)_dma_getnextrxp,
	(di_peeknextrxp_t)_dma_peeknextrxp,
	(di_rxparam_get_t)_dma_rx_param_get,

	(di_fifoloopbackenable_t)_dma_fifoloopbackenable,
	(di_getvar_t)_dma_getvar,
	(di_counterreset_t)_dma_counterreset,
	(di_ctrlflags_t)_dma_ctrlflags,

#if defined(BCMDBG)
	(di_dump_t)dma32_dump,
	(di_dumptx_t)dma32_dumptx,
	(di_dumprx_t)dma32_dumprx,
#else
	NULL,
	NULL,
	NULL,
#endif 
	(di_rxactive_t)_dma_rxactive,
	(di_txpending_t)_dma_txpending,
	(di_txcommitted_t)_dma_txcommitted,
	(di_pktpool_set_t)_dma_pktpool_set,
	(di_rxtxerror_t)_dma_rxtx_error,
	(di_burstlen_set_t)_dma_burstlen_set,
	(di_avoidancecnt_t)_dma_avoidancecnt,
	(di_param_set_t)_dma_param_set,
	NULL,
	NULL,
	40
};

/** init the tx or rx descriptor */
/* XXX inline? */
void
dma32_dd_upd(dma_info_t *di, dma32dd_t *ddring, dmaaddr_t pa, u_int outidx, uint32_t *flags,
	uint32_t bufcount)
{
	bhnd_dma_chan *chan = di->chan;

	/* dma32 uses 32-bit control to fit both flags and bufcounter */
	*flags = *flags | (bufcount & CTRL_BC_MASK);

	if ((di->dataoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
		W_SM(&ddring[outidx].addr, BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
		W_SM(&ddring[outidx].ctrl, BUS_SWAP32(*flags));
	} else {
		/* address extension */
		uint32_t ae;
		ASSERT(chan->dma->addrext);
		ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
		PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

		*flags |= (ae << CTRL_AE_SHIFT);
		W_SM(&ddring[outidx].addr, BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
		W_SM(&ddring[outidx].ctrl, BUS_SWAP32(*flags));
	}
}

#if defined(BCMDBG)
static void
dma32_dumpring(dma_info_t *di, struct sbuf *s, dma32dd_t *ring, u_int start, u_int end,
	u_int max_num)
{
	u_int i;

	for (i = start; i != end; i = XXD((i + 1), max_num)) {
		/* in the format of high->low 8 bytes */
		sbuf_printf(s, "ring index %d: %#x %x\n",
			i, R_SM(&ring[i].addr), R_SM(&ring[i].ctrl));
	}
}

static void
dma32_dumptx(dma_info_t *di, struct sbuf *s, bool dumpring)
{
	if (di->ntxd == 0)
		return;

	sbuf_printf(s, "DMA32: txd32 %p txdpa %#x txp %p txin %d txout %d "
	            "txavail %d txnodesc %d\n", di->txd32, PHYSADDRLO(di->txdpa), di->txp, di->txin,
	            di->txout, di->hnddma.txavail, di->hnddma.txnodesc);

	sbuf_printf(s, "xmtcontrol %#x xmtaddr %#x xmtptr %#x xmtstatus %#x\n",
		R_REG(di->osh, &di->d32txregs->control),
		R_REG(di->osh, &di->d32txregs->addr),
		R_REG(di->osh, &di->d32txregs->ptr),
		R_REG(di->osh, &di->d32txregs->status));

	if (dumpring && di->txd32)
		dma32_dumpring(di, s, di->txd32, di->txin, di->txout, di->ntxd);
}

static void
dma32_dumprx(dma_info_t *di, struct sbuf *s, bool dumpring)
{
	if (di->nrxd == 0)
		return;

	sbuf_printf(s, "DMA32: rxd32 %p rxdpa %#x rxp %p rxin %d rxout %d\n",
	            di->rxd32, PHYSADDRLO(di->rxdpa), di->rxp, di->rxin, di->rxout);

	sbuf_printf(s, "rcvcontrol %#x rcvaddr %#x rcvptr %#x rcvstatus %#x\n",
		R_REG(di->osh, &di->d32rxregs->control),
		R_REG(di->osh, &di->d32rxregs->addr),
		R_REG(di->osh, &di->d32rxregs->ptr),
		R_REG(di->osh, &di->d32rxregs->status));
	if (di->rxd32 && dumpring)
		dma32_dumpring(di, s, di->rxd32, di->rxin, di->rxout, di->nrxd);
}

static void
dma32_dump(dma_info_t *di, struct sbuf *s, bool dumpring)
{
	dma32_dumptx(di, s, dumpring);
	dma32_dumprx(di, s, dumpring);
}

#endif /* BCMDBG */


/* 32-bit DMA functions */

static void
dma32_txinit(dma_info_t *di)
{
	uint32_t control = XC_XE;

	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	di->txin = di->txout = di->xs0cd = 0;
	di->hnddma.txavail = di->ntxd - 1;

	/* clear tx descriptor ring */
	BZERO_SM(DISCARD_QUAL(di->txd32, void), (di->ntxd * sizeof(dma32dd_t)));

	/* These bits 20:18 (burstLen) of control register can be written but will take
	 * effect only if these bits are valid. So this will not affect previous versions
	 * of the DMA. They will continue to have those bits set to 0.
	 */
	control |= (di->txburstlen << XC_BL_SHIFT);
	control |= (di->txmultioutstdrd << XC_MR_SHIFT);
	control |= (di->txprefetchctl << XC_PC_SHIFT);
	control |= (di->txprefetchthresh << XC_PT_SHIFT);

	if ((di->hnddma.dmactrlflags & DMA_CTRL_PEN) == 0)
		control |= XC_PD;
	W_REG(di->osh, &di->d32txregs->control, control);
	_dma_ddtable_init(di, di->txdpa);
}

static bool
dma32_txenabled(dma_info_t *di)
{
	uint32_t xc;

	/* If the chip is dead, it is not enabled :-) */
	xc = R_REG(di->osh, &di->d32txregs->control);
	return ((xc != 0xffffffff) && (xc & XC_XE));
}

static void
dma32_txsuspend(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	OR_REG(di->osh, &di->d32txregs->control, XC_SE);
}

static void
dma32_txresume(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	AND_REG(di->osh, &di->d32txregs->control, ~XC_SE);
}

static bool
dma32_txsuspended(dma_info_t *di)
{
	return (di->ntxd == 0) || ((R_REG(di->osh, &di->d32txregs->control) & XC_SE) == XC_SE);
}

static void
dma32_txflush(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	OR_REG(di->osh, &di->d32txregs->control, XC_SE | XC_FL);
}

static void
dma32_txflush_clear(dma_info_t *di)
{
	uint32_t status;

	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	SPINWAIT(((status = (R_REG(di->osh, &di->d32txregs->status) & XS_XS_MASK))
		 != XS_XS_DISABLED) &&
		 (status != XS_XS_IDLE) &&
		 (status != XS_XS_STOPPED),
		 (10000));
	AND_REG(di->osh, &di->d32txregs->control, ~XC_FL);
}

static void
dma32_txreclaim(dma_info_t *di, txd_range_t range)
{
	void *p;

	BHND_DMA_TRACE(di, "dma_txreclaim %s",
	           (range == HNDDMA_RANGE_ALL) ? "all" :
	           ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered"));

	if (di->txin == di->txout)
		return;

	while ((p = dma32_getnexttxp(di, range)))
		PKTFREE(di->osh, p, TRUE);
}

static bool
dma32_txstopped(dma_info_t *di)
{
	return ((R_REG(di->osh, &di->d32txregs->status) & XS_XS_MASK) == XS_XS_STOPPED);
}

static bool
dma32_rxstopped(dma_info_t *di)
{
	return ((R_REG(di->osh, &di->d32rxregs->status) & RS_RS_MASK) == RS_RS_STOPPED);
}

bool
dma32_alloc(dma_info_t *di, u_int direction)
{
	u_int size;
	u_int ddlen;
	void *va;
	u_int alloced;
	uint16_t align;
	uint16_t align_bits;

	ddlen = sizeof(dma32dd_t);

	size = (direction == DMA_TX) ? (di->ntxd * ddlen) : (di->nrxd * ddlen);

	alloced = 0;
	align_bits = di->dmadesc_align;
	align = (1 << align_bits);

	if (direction == DMA_TX) {
		if ((va = dma_ringalloc(di->osh, D32RINGALIGN, size, &align_bits, &alloced,
			&di->txdpaorig, &di->tx_dmah)) == NULL) {
			BHND_DMA_ERROR(di, "DMA_ALLOC_CONSISTENT(ntxd) failed");
			return FALSE;
		}

		PHYSADDRHISET(di->txdpa, 0);
		ASSERT(PHYSADDRHI(di->txdpaorig) == 0);
		di->txd32 = (dma32dd_t *)roundup((uintptr_t)va, align);
		di->txdalign = (u_int)((int8_t *)(uintptr_t)di->txd32 - (int8_t *)va);

		PHYSADDRLOSET(di->txdpa, PHYSADDRLO(di->txdpaorig) + di->txdalign);
		/* Make sure that alignment didn't overflow */
		ASSERT(PHYSADDRLO(di->txdpa) >= PHYSADDRLO(di->txdpaorig));

		di->txdalloc = alloced;
		ASSERT(ISALIGNED(di->txd32, align));
	} else {
		if ((va = dma_ringalloc(di->osh, D32RINGALIGN, size, &align_bits, &alloced,
			&di->rxdpaorig, &di->rx_dmah)) == NULL) {
			BHND_DMA_ERROR(di, "DMA_ALLOC_CONSISTENT(nrxd) failed");
			return FALSE;
		}

		PHYSADDRHISET(di->rxdpa, 0);
		ASSERT(PHYSADDRHI(di->rxdpaorig) == 0);
		di->rxd32 = (dma32dd_t *)roundup((uintptr_t)va, align);
		di->rxdalign = (u_int)((int8_t *)(uintptr_t)di->rxd32 - (int8_t *)va);

		PHYSADDRLOSET(di->rxdpa, PHYSADDRLO(di->rxdpaorig) + di->rxdalign);
		/* Make sure that alignment didn't overflow */
		ASSERT(PHYSADDRLO(di->rxdpa) >= PHYSADDRLO(di->rxdpaorig));
		di->rxdalloc = alloced;
		ASSERT(ISALIGNED(di->rxd32, align));
	}

	return TRUE;
}

static bool
dma32_txreset(dma_info_t *di)
{
	uint32_t status;

	if (di->ntxd == 0)
		return TRUE;

	/* suspend tx DMA first */
	W_REG(di->osh, &di->d32txregs->control, XC_SE);
	SPINWAIT(((status = (R_REG(di->osh, &di->d32txregs->status) & XS_XS_MASK))
		 != XS_XS_DISABLED) &&
		 (status != XS_XS_IDLE) &&
		 (status != XS_XS_STOPPED),
		 (10000));

	W_REG(di->osh, &di->d32txregs->control, 0);
	SPINWAIT(((status = (R_REG(di->osh,
	         &di->d32txregs->status) & XS_XS_MASK)) != XS_XS_DISABLED),
	         10000);

	/* We should be disabled at this point */
	if (status != XS_XS_DISABLED) {
		BHND_DMA_ERROR(di, "status %#x != XS_XS_DISABLED at "
		    "txreset", status);
		ASSERT(status == XS_XS_DISABLED);
		OSL_DELAY(300);
	}

	return (status == XS_XS_DISABLED);
}

bool
dma32_rxidle(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->nrxd == 0)
		return TRUE;

	return ((R_REG(di->osh, &di->d32rxregs->status) & RS_CD_MASK) ==
	        R_REG(di->osh, &di->d32rxregs->ptr));
}

static bool
dma32_rxreset(dma_info_t *di)
{
	uint32_t status;

	if (di->nrxd == 0)
		return TRUE;

	W_REG(di->osh, &di->d32rxregs->control, 0);
	SPINWAIT(((status = (R_REG(di->osh,
	         &di->d32rxregs->status) & RS_RS_MASK)) != RS_RS_DISABLED),
	         10000);

	return (status == RS_RS_DISABLED);
}

static bool
dma32_rxenabled(dma_info_t *di)
{
	uint32_t rc;

	rc = R_REG(di->osh, &di->d32rxregs->control);
	return ((rc != 0xffffffff) && (rc & RC_RE));
}

static bool
dma32_txsuspendedidle(dma_info_t *di)
{
	if (di->ntxd == 0)
		return TRUE;

	if (!(R_REG(di->osh, &di->d32txregs->control) & XC_SE))
		return 0;

	if ((R_REG(di->osh, &di->d32txregs->status) & XS_XS_MASK) != XS_XS_IDLE)
		return 0;

	OSL_DELAY(2);
	return ((R_REG(di->osh, &di->d32txregs->status) & XS_XS_MASK) == XS_XS_IDLE);
}

/**
 * !! tx entry routine
 * supports full 32bit dma engine buffer addressing so
 * dma buffers can cross 4 Kbyte page boundaries.
 *
 * WARNING: call must check the return value for error.
 *   the error(toss frames) could be fatal and cause many subsequent hard to debug problems
 */
static int
dma32_txfast(dma_info_t *di, void *p0, bool commit)
{
	void *p, *next;
	u_char *data;
	u_int len;
	uint16_t txout;
	uint32_t flags = 0;
	dmaaddr_t pa;

	BHND_DMA_TRACE_ENTRY(di);

	txout = di->txout;

	/*
	 * Walk the chain of packet buffers
	 * allocating and initializing transmit descriptor entries.
	 */
	for (p = p0; p; p = next) {
		u_int nsegs, j;
		hnddma_seg_map_t *map;

		data = PKTDATA(di->osh, p);
		len = PKTLEN(di->osh, p);
#ifdef BCM_DMAPAD
		len += PKTDMAPAD(di->osh, p);
#endif
		next = PKTNEXT(di->osh, p);

		/* return nonzero if out of tx descriptors */
		if (NEXTTXD(txout) == di->txin)
			goto outoftxd;

		if (len == 0)
			continue;

		if (DMASGLIST_ENAB)
			bzero(&di->txp_dmah[txout], sizeof(hnddma_seg_map_t));

		/* get physical address of buffer start */
#ifdef BCM_SECURE_DMA
		pa = SECURE_DMA_MAP(di->osh, data, len, DMA_TX, p, &di->txp_dmah[txout],
			&di->sec_cma_info_tx, 0);
#else
		pa = DMA_MAP(di->osh, data, len, DMA_TX, p, &di->txp_dmah[txout]);
#endif
		if (DMASGLIST_ENAB) {
			map = &di->txp_dmah[txout];

			/* See if all the segments can be accounted for */
			if (map->nsegs > (u_int)(di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1))
				goto outoftxd;

			nsegs = map->nsegs;
		} else
			nsegs = 1;

		for (j = 1; j <= nsegs; j++) {
			flags = 0;
			if (p == p0 && j == 1)
				flags |= CTRL_SOF;

			/* With a DMA segment list, Descriptor table is filled
			 * using the segment list instead of looping over
			 * buffers in multi-chain DMA. Therefore, EOF for SGLIST is when
			 * end of segment list is reached.
			 */
			if ((!DMASGLIST_ENAB && next == NULL) ||
			    (DMASGLIST_ENAB && j == nsegs))
				flags |= (CTRL_IOC | CTRL_EOF);
			if (txout == (di->ntxd - 1))
				flags |= CTRL_EOT;

			if (DMASGLIST_ENAB) {
				len = map->segs[j - 1].length;
				pa = map->segs[j - 1].addr;
			}
			ASSERT(PHYSADDRHI(pa) == 0);

			dma32_dd_upd(di, di->txd32, pa, txout, &flags, len);
			ASSERT(di->txp[txout] == NULL);

			txout = NEXTTXD(txout);
		}

		/* See above. No need to loop over individual buffers */
		if (DMASGLIST_ENAB)
			break;
	}

	/* if last txd eof not set, fix it */
	if (!(flags & CTRL_EOF))
		W_SM(&di->txd32[PREVTXD(txout)].ctrl, BUS_SWAP32(flags | CTRL_IOC | CTRL_EOF));

	/* save the packet */
	di->txp[PREVTXD(txout)] = p0;

	/* bump the tx descriptor index */
	di->txout = txout;

	/* kick the chip */
	if (commit)
		W_REG(di->osh, &di->d32txregs->ptr, I2B(txout, dma32dd_t));

	/* tx flow control */
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

	return (0);

outoftxd:
	BHND_DMA_ERROR(di, "out of TX descriptors");
	PKTFREE(di->osh, p0, TRUE);
	di->hnddma.txavail = 0;
	di->hnddma.txnobuf++;
	di->hnddma.txnodesc++;
	return (-1);
}

/**
 * Reclaim next completed txd (txds if using chained buffers) in the range
 * specified and return associated packet.
 * If range is HNDDMA_RANGE_TRANSMITTED, reclaim descriptors that have be
 * transmitted as noted by the hardware "CurrDescr" pointer.
 * If range is HNDDMA_RANGE_TRANSFERED, reclaim descriptors that have be
 * transfered by the DMA as noted by the hardware "ActiveDescr" pointer.
 * If range is HNDDMA_RANGE_ALL, reclaim all txd(s) posted to the ring and
 * return associated packet regardless of the value of hardware pointers.
 */
static void *
dma32_getnexttxp(dma_info_t *di, txd_range_t range)
{
	uint16_t start, end, i;
	uint16_t active_desc;
	void *txp;

	BHND_DMA_TRACE(di, "dma_getnexttxp %s",
	           (range == HNDDMA_RANGE_ALL) ? "all" :
	           ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered"));

	if (di->ntxd == 0)
		return (NULL);

	txp = NULL;

	start = di->txin;
	if (range == HNDDMA_RANGE_ALL)
		end = di->txout;
	else {
		dma32regs_t *dregs = di->d32txregs;

		if (di->txin == di->xs0cd) {
			end = (uint16_t)B2I(R_REG(di->osh, &dregs->status) & XS_CD_MASK, dma32dd_t);
			di->xs0cd = end;
		} else
			end = di->xs0cd;


		if (range == HNDDMA_RANGE_TRANSFERED) {
			active_desc = (uint16_t)((R_REG(di->osh, &dregs->status) & XS_AD_MASK) >>
			                       XS_AD_SHIFT);
			active_desc = (uint16_t)B2I(active_desc, dma32dd_t);
			if (end != active_desc)
				end = PREVTXD(active_desc);
		}
	}

	if ((start == 0) && (end > di->txout))
		goto bogus;

	for (i = start; i != end && !txp; i = NEXTTXD(i)) {
		dmaaddr_t pa;
		hnddma_seg_map_t *map = NULL;
		u_int size, j, nsegs;

		PHYSADDRLOSET(pa, (BUS_SWAP32(R_SM(&di->txd32[i].addr)) - di->dataoffsetlow));
		PHYSADDRHISET(pa, 0);

		if (DMASGLIST_ENAB) {
			map = &di->txp_dmah[i];
			size = map->origsize;
			nsegs = map->nsegs;
		} else {
			size = (BUS_SWAP32(R_SM(&di->txd32[i].ctrl)) & CTRL_BC_MASK);
			nsegs = 1;
		}

		for (j = nsegs; j > 0; j--) {
			W_SM(&di->txd32[i].addr, 0xdeadbeef);

			txp = di->txp[i];
			di->txp[i] = NULL;
			if (j > 1)
				i = NEXTTXD(i);
		}
#ifdef BCM_SECURE_DMA
		SECURE_DMA_UNMAP(di->osh, pa, size, DMA_TX, NULL, NULL, &di->sec_cma_info_tx, 0);
#else
		DMA_UNMAP(di->osh, pa, size, DMA_TX, txp, map);
#endif
	}

	di->txin = i;

	/* tx flow control */
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

	return (txp);

bogus:
	BHND_DMA_TRACE(di, "bogus curr: start %d end %d txout %d",
	          start, end, di->txout);
	return (NULL);
}

void *
dma32_getnextrxp(dma_info_t *di, bool forceall)
{
	uint16_t i, curr;
	void *rxp;
	dmaaddr_t pa;
	/* if forcing, dma engine must be disabled */
	ASSERT(!forceall || !dma32_rxenabled(di));

	i = di->rxin;

	/* return if no packets posted */
	if (i == di->rxout)
		return (NULL);

	if (di->rxin == di->rs0cd) {
		curr = (uint16_t)B2I(R_REG(di->osh, &di->d32rxregs->status) & RS_CD_MASK, dma32dd_t);
		di->rs0cd = curr;
	} else
		curr = di->rs0cd;

	/* ignore curr if forceall */
	if (!forceall && (i == curr))
		return (NULL);

	/* get the packet pointer that corresponds to the rx descriptor */
	rxp = di->rxp[i];
	ASSERT(rxp);
	di->rxp[i] = NULL;

	PHYSADDRLOSET(pa, (BUS_SWAP32(R_SM(&di->rxd32[i].addr)) - di->dataoffsetlow));
	PHYSADDRHISET(pa, 0);

	/* clear this packet from the descriptor ring */
#ifdef BCM_SECURE_DMA
	SECURE_DMA_UNMAP(di->osh, pa, di->rxbufsize, DMA_RX, NULL, NULL, &di->sec_cma_info_rx, 0);
#else
	DMA_UNMAP(di->osh, pa,
	          di->rxbufsize, DMA_RX, rxp, &di->rxp_dmah[i]);
#endif

	W_SM(&di->rxd32[i].addr, 0xdeadbeef);

	di->rxin = NEXTRXD(i);

	return (rxp);
}

/**
 * Rotate all active tx dma ring entries "forward" by (ActiveDescriptor - txin).
 */
static void
dma32_txrotate(dma_info_t *di)
{
	uint16_t ad;
	u_int nactive;
	u_int rot;
	uint16_t old, new;
	uint32_t w;
	uint16_t first, last;

	ASSERT(dma32_txsuspendedidle(di));

	nactive = _dma_txactive(di);
	ad = B2I(((R_REG(di->osh, &di->d32txregs->status) & XS_AD_MASK) >> XS_AD_SHIFT), dma32dd_t);
	rot = TXD(ad - di->txin);

	ASSERT(rot < di->ntxd);

	/* full-ring case is a lot harder - don't worry about this */
	if (rot >= (di->ntxd - nactive)) {
		BHND_DMA_ERROR(di, "dma_txrotate: ring full - punt");
		return;
	}

	first = di->txin;
	last = PREVTXD(di->txout);

	/* move entries starting at last and moving backwards to first */
	for (old = last; old != PREVTXD(first); old = PREVTXD(old)) {
		new = TXD(old + rot);

		/*
		 * Move the tx dma descriptor.
		 * EOT is set only in the last entry in the ring.
		 */
		w = BUS_SWAP32(R_SM(&di->txd32[old].ctrl)) & ~CTRL_EOT;
		if (new == (di->ntxd - 1))
			w |= CTRL_EOT;
		W_SM(&di->txd32[new].ctrl, BUS_SWAP32(w));
		W_SM(&di->txd32[new].addr, R_SM(&di->txd32[old].addr));

		/* zap the old tx dma descriptor address field */
		W_SM(&di->txd32[old].addr, BUS_SWAP32(0xdeadbeef));

		/* move the corresponding txp[] entry */
		ASSERT(di->txp[new] == NULL);
		di->txp[new] = di->txp[old];

		/* Move the segment map as well */
		if (DMASGLIST_ENAB) {
			bcopy(&di->txp_dmah[old], &di->txp_dmah[new], sizeof(hnddma_seg_map_t));
			bzero(&di->txp_dmah[old], sizeof(hnddma_seg_map_t));
		}

		di->txp[old] = NULL;
	}

	/* update txin and txout */
	di->txin = ad;
	di->txout = TXD(di->txout + rot);
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

	/* kick the chip */
	W_REG(di->osh, &di->d32txregs->ptr, I2B(di->txout, dma32dd_t));
}
