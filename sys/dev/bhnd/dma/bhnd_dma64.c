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

#include "bhnd_dmavar.h"
#include "bhnd_dmareg.h"

#include "bhnd_dma64var.h"
#include "bhnd_dma64reg.h"

/* XXX required until we adopt bhnd(9) */
#include "siutils.h"


/* Prototypes for 64-bit routines */
static bool dma64_txreset(dma_info_t *di);
static bool dma64_rxreset(dma_info_t *di);
static bool dma64_txsuspendedidle(dma_info_t *di);
static int  dma64_txfast(dma_info_t *di, void *p0, bool commit);
static int  dma64_txunframed(dma_info_t *di, void *p0, u_int len, bool commit);
static void *dma64_getpos(dma_info_t *di, bool direction);
static void *dma64_getnexttxp(dma_info_t *di, txd_range_t range);
static void dma64_txrotate(dma_info_t *di);

static void dma64_txinit(dma_info_t *di);
static bool dma64_txenabled(dma_info_t *di);
static void dma64_txsuspend(dma_info_t *di);
static void dma64_txresume(dma_info_t *di);
static bool dma64_txsuspended(dma_info_t *di);
static void dma64_txflush(dma_info_t *di);
static void dma64_txflush_clear(dma_info_t *di);
static void dma64_txreclaim(dma_info_t *di, txd_range_t range);
static bool dma64_txstopped(dma_info_t *di);
static bool dma64_rxstopped(dma_info_t *di);
static bool dma64_rxenabled(dma_info_t *di);

#if defined(BCMDBG)
static void dma64_dumpring(dma_info_t *di, struct sbuf *s, dma64dd_t *ring, u_int start,
	u_int end, u_int max_num);
static void dma64_dump(dma_info_t *di, struct sbuf *s, bool dumpring);
static void dma64_dumptx(dma_info_t *di, struct sbuf *s, bool dumpring);
static void dma64_dumprx(dma_info_t *di, struct sbuf *s, bool dumpring);
#endif 



static inline uint32_t parity32(uint32_t data);

const di_fcn_t dma64proc = {
	(di_detach_t)_dma_detach,
	(di_txinit_t)dma64_txinit,
	(di_txreset_t)dma64_txreset,
	(di_txenabled_t)dma64_txenabled,
	(di_txsuspend_t)dma64_txsuspend,
	(di_txresume_t)dma64_txresume,
	(di_txsuspended_t)dma64_txsuspended,
	(di_txsuspendedidle_t)dma64_txsuspendedidle,
	(di_txflush_t)dma64_txflush,
	(di_txflush_clear_t)dma64_txflush_clear,
	(di_txfast_t)dma64_txfast,
	(di_txunframed_t)dma64_txunframed,
	(di_getpos_t)dma64_getpos,
	(di_txstopped_t)dma64_txstopped,
	(di_txreclaim_t)dma64_txreclaim,
	(di_getnexttxp_t)dma64_getnexttxp,
	(di_peeknexttxp_t)_dma_peeknexttxp,
	(di_peekntxp_t)_dma_peekntxp,
	(di_txblock_t)_dma_txblock,
	(di_txunblock_t)_dma_txunblock,
	(di_txactive_t)_dma_txactive,
	(di_txrotate_t)dma64_txrotate,

	(di_rxinit_t)_dma_rxinit,
	(di_rxreset_t)dma64_rxreset,
	(di_rxidle_t)dma64_rxidle,
	(di_rxstopped_t)dma64_rxstopped,
	(di_rxenable_t)_dma_rxenable,
	(di_rxenabled_t)dma64_rxenabled,
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
	(di_dump_t)dma64_dump,
	(di_dumptx_t)dma64_dumptx,
	(di_dumprx_t)dma64_dumprx,
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
	/* XXX only used by DMA64 ? */
	(dma_glom_enable_t)_dma_glom_enable,
	(dma_active_rxbuf_t)_dma_activerxbuf,
	40
};

/** Check for odd number of 1's */
static inline uint32_t parity32(uint32_t data)
{
	data ^= data >> 16;
	data ^= data >> 8;
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;

	return (data & 1);
}

#define DMA64_DD_PARITY(dd)  parity32((dd)->addrlow ^ (dd)->addrhigh ^ (dd)->ctrl1 ^ (dd)->ctrl2)

/* XXX inline? */
void
dma64_dd_upd(dma_info_t *di, dma64dd_t *ddring, dmaaddr_t pa, u_int outidx, uint32_t *flags,
	uint32_t bufcount)
{
	uint32_t ctrl2 = bufcount & D64_CTRL2_BC_MASK;

	/* PCI bus with big(>1G) physical address, use address extension */
#if defined(__mips__) && defined(IL_BIGENDIAN)
	if ((di->dataoffsetlow == SI_SDRAM_SWAPPED) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
#else
	if ((di->dataoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
#endif /* defined(__mips__) && defined(IL_BIGENDIAN) */
		ASSERT((PHYSADDRHI(pa) & PCI64ADDR_HIGH) == 0);

		W_SM(&ddring[outidx].addrlow, BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
		W_SM(&ddring[outidx].addrhigh, BUS_SWAP32(PHYSADDRHI(pa) + di->dataoffsethigh));
		W_SM(&ddring[outidx].ctrl1, BUS_SWAP32(*flags));
		W_SM(&ddring[outidx].ctrl2, BUS_SWAP32(ctrl2));
	} else {
		/* address extension for 32-bit PCI */
		uint32_t ae;
		ASSERT(di->addrext);

		ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
		PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;
		ASSERT(PHYSADDRHI(pa) == 0);

		ctrl2 |= (ae << D64_CTRL2_AE_SHIFT) & D64_CTRL2_AE;
		W_SM(&ddring[outidx].addrlow, BUS_SWAP32(PHYSADDRLO(pa) + di->dataoffsetlow));
		W_SM(&ddring[outidx].addrhigh, BUS_SWAP32(0 + di->dataoffsethigh));
		W_SM(&ddring[outidx].ctrl1, BUS_SWAP32(*flags));
		W_SM(&ddring[outidx].ctrl2, BUS_SWAP32(ctrl2));
	}
	if (di->hnddma.dmactrlflags & DMA_CTRL_PEN) {
		if (DMA64_DD_PARITY(&ddring[outidx])) {
			W_SM(&ddring[outidx].ctrl2, BUS_SWAP32(ctrl2 | D64_CTRL2_PARITY));
		}
	}

#if defined(BCM47XX_CA9) && !defined(__NetBSD__)
#ifndef BCM_SECURE_DMA
	DMA_MAP(di->osh, (void *)(((u_int)(&ddring[outidx])) & ~0x1f), 32, DMA_TX, NULL, NULL);
#else
	SECURE_DMA_DD_MAP(di->osh, (void *)(((u_int)(&ddring[outidx])) & ~0x1f),
		32, DMA_TX, NULL, NULL);
#endif
#endif /* BCM47XX_CA9 && !__NetBSD__ */
}

#if defined(BCMDBG)
static void
dma64_dumpring(dma_info_t *di, struct sbuf *s, dma64dd_t *ring, u_int start, u_int end,
	u_int max_num)
{
	u_int i;

	for (i = start; i != end; i = XXD((i + 1), max_num)) {
		/* in the format of high->low 16 bytes */
		sbuf_printf(s, "ring index %d: 0x%x %x %x %x\n",
			i, R_SM(&ring[i].addrhigh), R_SM(&ring[i].addrlow),
			R_SM(&ring[i].ctrl2), R_SM(&ring[i].ctrl1));
	}
}

static void
dma64_dumptx(dma_info_t *di, struct sbuf *s, bool dumpring)
{
	if (di->ntxd == 0)
		return;

	sbuf_printf(s, "DMA64: txd64 %p txdpa 0x%x txdpahi 0x%x txp %p txin %d txout %d "
	            "txavail %d txnodesc %d\n", di->txd64, PHYSADDRLO(di->txdpa),
	            PHYSADDRHI(di->txdpaorig), di->txp, di->txin, di->txout, di->hnddma.txavail,
	            di->hnddma.txnodesc);

	sbuf_printf(s, "xmtcontrol 0x%x xmtaddrlow 0x%x xmtaddrhigh 0x%x "
		       "xmtptr 0x%x xmtstatus0 0x%x xmtstatus1 0x%x\n",
		       R_REG(di->osh, &di->d64txregs->control),
		       R_REG(di->osh, &di->d64txregs->addrlow),
		       R_REG(di->osh, &di->d64txregs->addrhigh),
		       R_REG(di->osh, &di->d64txregs->ptr),
		       R_REG(di->osh, &di->d64txregs->status0),
		       R_REG(di->osh, &di->d64txregs->status1));

	sbuf_printf(s, "DMA64: DMA avoidance applied %d\n", di->dma_avoidance_cnt);

	if (dumpring && di->txd64) {
		dma64_dumpring(di, s, di->txd64, di->txin, di->txout, di->ntxd);
	}
}

static void
dma64_dumprx(dma_info_t *di, struct sbuf *s, bool dumpring)
{
	if (di->nrxd == 0)
		return;

	sbuf_printf(s, "DMA64: rxd64 %p rxdpa 0x%x rxdpahi 0x%x rxp %p rxin %d rxout %d\n",
	            di->rxd64, PHYSADDRLO(di->rxdpa), PHYSADDRHI(di->rxdpaorig), di->rxp,
	            di->rxin, di->rxout);

	sbuf_printf(s, "rcvcontrol 0x%x rcvaddrlow 0x%x rcvaddrhigh 0x%x rcvptr "
		       "0x%x rcvstatus0 0x%x rcvstatus1 0x%x\n",
		       R_REG(di->osh, &di->d64rxregs->control),
		       R_REG(di->osh, &di->d64rxregs->addrlow),
		       R_REG(di->osh, &di->d64rxregs->addrhigh),
		       R_REG(di->osh, &di->d64rxregs->ptr),
		       R_REG(di->osh, &di->d64rxregs->status0),
		       R_REG(di->osh, &di->d64rxregs->status1));
	if (di->rxd64 && dumpring) {
		dma64_dumpring(di, s, di->rxd64, di->rxin, di->rxout, di->nrxd);
	}
}

static void
dma64_dump(dma_info_t *di, struct sbuf *s, bool dumpring)
{
	dma64_dumptx(di, s, dumpring);
	dma64_dumprx(di, s, dumpring);
}

#endif /* BCMDBG */

/* 64-bit DMA functions */

static void
dma64_txinit(dma_info_t *di)
{
	uint32_t control;

	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	di->txin = di->txout = di->xs0cd = di->xs0cd_snapshot = 0;
	di->hnddma.txavail = di->ntxd - 1;

	/* clear tx descriptor ring */
	BZERO_SM((void *)(uintptr_t)di->txd64, (di->ntxd * sizeof(dma64dd_t)));

	/* These bits 20:18 (burstLen) of control register can be written but will take
	 * effect only if these bits are valid. So this will not affect previous versions
	 * of the DMA. They will continue to have those bits set to 0.
	 */
	control = R_REG(di->osh, &di->d64txregs->control);
	control = (control & ~D64_XC_BL_MASK) | (di->txburstlen << D64_XC_BL_SHIFT);
	control = (control & ~D64_XC_MR_MASK) | (di->txmultioutstdrd << D64_XC_MR_SHIFT);
	control = (control & ~D64_XC_PC_MASK) | (di->txprefetchctl << D64_XC_PC_SHIFT);
	control = (control & ~D64_XC_PT_MASK) | (di->txprefetchthresh << D64_XC_PT_SHIFT);
	W_REG(di->osh, &di->d64txregs->control, control);

	control = D64_XC_XE;
	/* DMA engine with out alignment requirement requires table to be inited
	 * before enabling the engine
	 */
	if (!di->aligndesc_4k)
		_dma_ddtable_init(di, di->txdpa);

	if ((di->hnddma.dmactrlflags & DMA_CTRL_PEN) == 0)
		control |= D64_XC_PD;
	OR_REG(di->osh, &di->d64txregs->control, control);

	/* DMA engine with alignment requirement requires table to be inited
	 * before enabling the engine
	 */
	if (di->aligndesc_4k)
		_dma_ddtable_init(di, di->txdpa);
}

static bool
dma64_txenabled(dma_info_t *di)
{
	uint32_t xc;

	/* If the chip is dead, it is not enabled :-) */
	xc = R_REG(di->osh, &di->d64txregs->control);
	return ((xc != 0xffffffff) && (xc & D64_XC_XE));
}

static void
dma64_txsuspend(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	OR_REG(di->osh, &di->d64txregs->control, D64_XC_SE);
}

static void
dma64_txresume(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	AND_REG(di->osh, &di->d64txregs->control, ~D64_XC_SE);
}

static bool
dma64_txsuspended(dma_info_t *di)
{
	return (di->ntxd == 0) ||
	        ((R_REG(di->osh, &di->d64txregs->control) & D64_XC_SE) == D64_XC_SE);
}

static void
dma64_txflush(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	OR_REG(di->osh, &di->d64txregs->control, D64_XC_SE | D64_XC_FL);
}

static void
dma64_txflush_clear(dma_info_t *di)
{
	uint32_t status;

	BHND_DMA_TRACE_ENTRY(di);

	if (di->ntxd == 0)
		return;

	SPINWAIT(((status = (R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_XS_MASK)) !=
	          D64_XS0_XS_DISABLED) &&
	         (status != D64_XS0_XS_IDLE) &&
	         (status != D64_XS0_XS_STOPPED),
	         10000);
	AND_REG(di->osh, &di->d64txregs->control, ~D64_XC_FL);
}

static void
dma64_txreclaim(dma_info_t *di, txd_range_t range)
{
	void *p;

	BHND_DMA_TRACE(di, "dma_txreclaim %s",
	           (range == HNDDMA_RANGE_ALL) ? "all" :
	           ((range == HNDDMA_RANGE_TRANSMITTED) ? "transmitted" : "transfered"));

	if (di->txin == di->txout)
		return;

	while ((p = dma64_getnexttxp(di, range))) {
		/* For unframed data, we don't have any packets to free */
		if (!(di->hnddma.dmactrlflags & DMA_CTRL_UNFRAMED))
			PKTFREE(di->osh, p, TRUE);
	}
}

static bool
dma64_txstopped(dma_info_t *di)
{
	return ((R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_XS_MASK) == D64_XS0_XS_STOPPED);
}

static bool
dma64_rxstopped(dma_info_t *di)
{
	return ((R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_RS_MASK) == D64_RS0_RS_STOPPED);
}

bool
dma64_alloc(dma_info_t *di, u_int direction)
{
	uint32_t size;
	u_int ddlen;
	void *va;
	u_int alloced = 0;
	uint32_t align;
	uint16_t align_bits;

	ddlen = sizeof(dma64dd_t);

	size = (direction == DMA_TX) ? (di->ntxd * ddlen) : (di->nrxd * ddlen);
	align_bits = di->dmadesc_align;
	align = (1 << align_bits);

	if (direction == DMA_TX) {
		if ((va = dma_ringalloc(di->osh,
			(di->status0_cd_mask == BHND_D64_STATUS0_CD_MIN_MASK) ? D64RINGBOUNDARY : D64RINGBOUNDARY_LARGE,
			size, &align_bits, &alloced,
			&di->txdpaorig, &di->tx_dmah)) == NULL) {
			BHND_DMA_ERROR(di, "DMA_ALLOC_CONSISTENT(ntxd) failed");
			return FALSE;
		}
		align = (1 << align_bits);

		/* adjust the pa by rounding up to the alignment */
		PHYSADDRLOSET(di->txdpa, roundup(PHYSADDRLO(di->txdpaorig), align));
		PHYSADDRHISET(di->txdpa, PHYSADDRHI(di->txdpaorig));

		/* Make sure that alignment didn't overflow */
		ASSERT(PHYSADDRLO(di->txdpa) >= PHYSADDRLO(di->txdpaorig));

		/* find the alignment offset that was used */
		di->txdalign = (u_int)(PHYSADDRLO(di->txdpa) - PHYSADDRLO(di->txdpaorig));

		/* adjust the va by the same offset */
		di->txd64 = (dma64dd_t *)((uintptr_t)va + di->txdalign);

		di->txdalloc = alloced;
		ASSERT(ISALIGNED(PHYSADDRLO(di->txdpa), align));
	} else {
		if ((va = dma_ringalloc(di->osh,
			(di->status0_cd_mask == BHND_D64_STATUS0_CD_MIN_MASK) ? D64RINGBOUNDARY : D64RINGBOUNDARY_LARGE,
			size, &align_bits, &alloced,
			&di->rxdpaorig, &di->rx_dmah)) == NULL) {
			BHND_DMA_ERROR(di, "DMA_ALLOC_CONSISTENT(nrxd) failed");
			return FALSE;
		}
		align = (1 << align_bits);

		/* adjust the pa by rounding up to the alignment */
		PHYSADDRLOSET(di->rxdpa, roundup(PHYSADDRLO(di->rxdpaorig), align));
		PHYSADDRHISET(di->rxdpa, PHYSADDRHI(di->rxdpaorig));

		/* Make sure that alignment didn't overflow */
		ASSERT(PHYSADDRLO(di->rxdpa) >= PHYSADDRLO(di->rxdpaorig));

		/* find the alignment offset that was used */
		di->rxdalign = (u_int)(PHYSADDRLO(di->rxdpa) - PHYSADDRLO(di->rxdpaorig));

		/* adjust the va by the same offset */
		di->rxd64 = (dma64dd_t *)((uintptr_t)va + di->rxdalign);

		di->rxdalloc = alloced;
		ASSERT(ISALIGNED(PHYSADDRLO(di->rxdpa), align));
	}

	return TRUE;
}

static bool
dma64_txreset(dma_info_t *di)
{
	uint32_t status;

	if (di->ntxd == 0)
		return TRUE;

	/* suspend tx DMA first */
	W_REG(di->osh, &di->d64txregs->control, D64_XC_SE);
	SPINWAIT(((status = (R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_XS_MASK)) !=
	          D64_XS0_XS_DISABLED) &&
	         (status != D64_XS0_XS_IDLE) &&
	         (status != D64_XS0_XS_STOPPED),
	         10000);

	W_REG(di->osh, &di->d64txregs->control, 0);
	SPINWAIT(((status = (R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_XS_MASK)) !=
	          D64_XS0_XS_DISABLED),
	         10000);

	/* We should be disabled at this point */
	if (status != D64_XS0_XS_DISABLED) {
		BHND_DMA_ERROR(di, "status %#x != D64_XS0_XS_DISABLED at "
		    "txreset", status);
		OSL_DELAY(300);
	}

	return (status == D64_XS0_XS_DISABLED);
}

bool
dma64_rxidle(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->nrxd == 0)
		return TRUE;

	return ((R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_CD_MASK) ==
		(R_REG(di->osh, &di->d64rxregs->ptr) & D64_RS0_CD_MASK));
}

static bool
dma64_rxreset(dma_info_t *di)
{
	uint32_t status;

	if (di->nrxd == 0)
		return TRUE;

	W_REG(di->osh, &di->d64rxregs->control, 0);
	SPINWAIT(((status = (R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_RS_MASK)) !=
	          D64_RS0_RS_DISABLED), 10000);

	return (status == D64_RS0_RS_DISABLED);
}

static bool
dma64_rxenabled(dma_info_t *di)
{
	uint32_t rc;

	rc = R_REG(di->osh, &di->d64rxregs->control);
	return ((rc != 0xffffffff) && (rc & D64_RC_RE));
}

static bool
dma64_txsuspendedidle(dma_info_t *di)
{

	if (di->ntxd == 0)
		return TRUE;

	if (!(R_REG(di->osh, &di->d64txregs->control) & D64_XC_SE))
		return 0;

	if ((R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_XS_MASK) == D64_XS0_XS_IDLE)
		return 1;

	return 0;
}

/**
 * Useful when sending unframed data.  This allows us to get a progress report from the DMA.
 * We return a pointer to the beginning of the data buffer of the current descriptor.
 * If DMA is idle, we return NULL.
 */
static void*
dma64_getpos(dma_info_t *di, bool direction)
{
	void *va;
	bool idle;
	uint16_t cur_idx;

	if (direction == DMA_TX) {
		cur_idx = B2I(((R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_CD_MASK) -
		               di->ptrbase) & D64_XS0_CD_MASK, dma64dd_t);
		idle = !NTXDACTIVE(di->txin, di->txout);
		va = di->txp[cur_idx];
	} else {
		cur_idx = B2I(((R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_CD_MASK) -
		               di->ptrbase) & D64_RS0_CD_MASK, dma64dd_t);
		idle = !NRXDACTIVE(di->rxin, di->rxout);
		va = di->rxp[cur_idx];
	}

	/* If DMA is IDLE, return NULL */
	if (idle) {
		BHND_DMA_TRACE(di, "DMA idle, return NULL");
		va = NULL;
	}

	return va;
}

/**
 * TX of unframed data
 *
 * Adds a DMA ring descriptor for the data pointed to by "buf".
 * This is for DMA of a buffer of data and is unlike other hnddma TX functions
 * that take a pointer to a "packet"
 * Each call to this is results in a single descriptor being added for "len" bytes of
 * data starting at "buf", it doesn't handle chained buffers.
 */
static int
dma64_txunframed(dma_info_t *di, void *buf, u_int len, bool commit)
{
	uint16_t txout;
	uint32_t flags = 0;
	dmaaddr_t pa; /* phys addr */

	txout = di->txout;

	/* return nonzero if out of tx descriptors */
	if (NEXTTXD(txout) == di->txin)
		goto outoftxd;

	if (len == 0)
		return 0;
#ifdef BCM_SECURE_DMA
	pa = SECURE_DMA_MAP(di->osh, buf, len, DMA_TX, NULL, NULL, &di->sec_cma_info_tx, 0);
#else
	pa = DMA_MAP(di->osh, buf, len, DMA_TX, NULL, &di->txp_dmah[txout]);
#endif

	flags = (D64_CTRL1_SOF | D64_CTRL1_IOC | D64_CTRL1_EOF);

	if (txout == (di->ntxd - 1))
		flags |= D64_CTRL1_EOT;

	dma64_dd_upd(di, di->txd64, pa, txout, &flags, len);
	ASSERT(di->txp[txout] == NULL);

	/* save the buffer pointer - used by dma_getpos */
	di->txp[txout] = buf;

	txout = NEXTTXD(txout);
	/* bump the tx descriptor index */
	di->txout = txout;

	/* kick the chip */
	if (commit) {
		W_REG(di->osh, &di->d64txregs->ptr, di->ptrbase + I2B(txout, dma64dd_t));
	}

	/* tx flow control */
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

	return (0);

outoftxd:
	BHND_DMA_ERROR(di, "out of TX descriptors");
	di->hnddma.txavail = 0;
	di->hnddma.txnobuf++;
	return (-1);
}


/**
 * !! tx entry routine
 * WARNING: call must check the return value for error.
 *   the error(toss frames) could be fatal and cause many subsequent hard to debug problems
 */
static int
dma64_txfast(dma_info_t *di, void *p0, bool commit)
{
	void *p, *next;
	u_char *data;
	u_int len;
	uint16_t txout;
	uint32_t flags = 0;
	dmaaddr_t pa;
	bool war;

	BHND_DMA_TRACE_ENTRY(di);

	txout = di->txout;
	war = (di->hnddma.dmactrlflags & DMA_CTRL_DMA_AVOIDANCE_WAR) ? TRUE : FALSE;

	/*
	 * Walk the chain of packet buffers
	 * allocating and initializing transmit descriptor entries.
	 */
	for (p = p0; p; p = next) {
		u_int nsegs, j, segsadd;
		hnddma_seg_map_t *map = NULL;

		data = PKTDATA(di->osh, p);
		len = PKTLEN(di->osh, p);
#ifdef BCM_DMAPAD
		len += PKTDMAPAD(di->osh, p);
#endif /* BCM_DMAPAD */
		next = PKTNEXT(di->osh, p);

		/* return nonzero if out of tx descriptors */
		if (NEXTTXD(txout) == di->txin)
			goto outoftxd;

		if (len == 0)
			continue;

		/* get physical address of buffer start */
		if (DMASGLIST_ENAB)
			bzero(&di->txp_dmah[txout], sizeof(hnddma_seg_map_t));

#ifdef BCM_SECURE_DMA

		if (DMASGLIST_ENAB) {
			pa = SECURE_DMA_MAP(di->osh, data, len, DMA_TX, p, &di->txp_dmah[txout],
				&di->sec_cma_info_tx, 0);
		}
		else {
			pa = SECURE_DMA_MAP(di->osh, data, len, DMA_TX, NULL, NULL,
				&di->sec_cma_info_tx, 0);
		}
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

		segsadd = 0;
		for (j = 1; j <= nsegs; j++) {
			flags = 0;
			if (p == p0 && j == 1)
				flags |= D64_CTRL1_SOF;

			/* With a DMA segment list, Descriptor table is filled
			 * using the segment list instead of looping over
			 * buffers in multi-chain DMA. Therefore, EOF for SGLIST is when
			 * end of segment list is reached.
			 */
			if ((!DMASGLIST_ENAB && next == NULL) ||
			    (DMASGLIST_ENAB && j == nsegs))
				flags |= (D64_CTRL1_IOC | D64_CTRL1_EOF);
			if (txout == (di->ntxd - 1))
				flags |= D64_CTRL1_EOT;

			if (DMASGLIST_ENAB) {
				len = map->segs[j - 1].length;
				pa = map->segs[j - 1].addr;
				if (len > 128 && war) {
					u_int remain, new_len, align64;
					/* check for 64B aligned of pa */
					align64 = (u_int)(PHYSADDRLO(pa) & 0x3f);
					align64 = (64 - align64) & 0x3f;
					new_len = len - align64;
					remain = new_len % 128;
					if (remain > 0 && remain <= 4) {
						uint32_t buf_addr_lo;
						uint32_t tmp_flags =
							flags & (~(D64_CTRL1_EOF | D64_CTRL1_IOC));
						flags &= ~(D64_CTRL1_SOF | D64_CTRL1_EOT);
						remain += 64;
						dma64_dd_upd(di, di->txd64, pa, txout,
							&tmp_flags, len-remain);
						ASSERT(di->txp[txout] == NULL);
						txout = NEXTTXD(txout);
						/* return nonzero if out of tx descriptors */
						if (txout == di->txin) {
							BHND_DMA_ERROR(di, "dma_txfast: Out-of-DMA"
								" descriptors (txin %d txout %d"
								" nsegs %d)",
								di->txin, di->txout, nsegs);
							goto outoftxd;
						}
						if (txout == (di->ntxd - 1))
							flags |= D64_CTRL1_EOT;
						buf_addr_lo = PHYSADDRLO(pa);
						PHYSADDRLOSET(pa, (PHYSADDRLO(pa) + (len-remain)));
						if (PHYSADDRLO(pa) < buf_addr_lo) {
							PHYSADDRHISET(pa, (PHYSADDRHI(pa) + 1));
						}
						len = remain;
						segsadd++;
						di->dma_avoidance_cnt++;
					}
				}
			}
			dma64_dd_upd(di, di->txd64, pa, txout, &flags, len);
			ASSERT(di->txp[txout] == NULL);

			txout = NEXTTXD(txout);
			/* return nonzero if out of tx descriptors */
			if (txout == di->txin) {
				BHND_DMA_ERROR(di, "dma_txfast: Out-of-DMA descriptors"
					   " (txin %d txout %d nsegs %d)",
					   di->txin, di->txout, nsegs);
				goto outoftxd;
			}
		}
		if (segsadd && DMASGLIST_ENAB)
			map->nsegs += segsadd;

		/* See above. No need to loop over individual buffers */
		if (DMASGLIST_ENAB)
			break;
	}

	/* if last txd eof not set, fix it */
	if (!(flags & D64_CTRL1_EOF))
		W_SM(&di->txd64[PREVTXD(txout)].ctrl1,
		     BUS_SWAP32(flags | D64_CTRL1_IOC | D64_CTRL1_EOF));

	/* save the packet */
	di->txp[PREVTXD(txout)] = p0;

	/* bump the tx descriptor index */
	di->txout = txout;

	/* kick the chip */
	if (commit)
		W_REG(di->osh, &di->d64txregs->ptr, di->ptrbase + I2B(txout, dma64dd_t));

	/* tx flow control */
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

	return (0);

outoftxd:
	BHND_DMA_ERROR(di, "out of TX descriptors");
	PKTFREE(di->osh, p0, TRUE);
	di->hnddma.txavail = 0;
	di->hnddma.txnobuf++;
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
dma64_getnexttxp(dma_info_t *di, txd_range_t range)
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
		dma64regs_t *dregs = di->d64txregs;

		if (di->txin == di->xs0cd) {
			end = (uint16_t)(B2I(((R_REG(di->osh, &dregs->status0) & D64_XS0_CD_MASK) -
			      di->ptrbase) & D64_XS0_CD_MASK, dma64dd_t));
			di->xs0cd = end;
		} else
			end = di->xs0cd;

		if (range == HNDDMA_RANGE_TRANSFERED) {
			active_desc = (uint16_t)(R_REG(di->osh, &dregs->status1) & D64_XS1_AD_MASK);
			active_desc = (active_desc - di->ptrbase) & D64_XS0_CD_MASK;
			active_desc = B2I(active_desc, dma64dd_t);
			if (end != active_desc)
				end = PREVTXD(active_desc);
		}
	}


	if ((start == 0) && (end > di->txout))
		goto bogus;

	for (i = start; i != end && !txp; i = NEXTTXD(i)) {
		hnddma_seg_map_t *map = NULL;
		u_int size, j, nsegs;

#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__)) || \
	defined(BCM_SECURE_DMA)
		dmaaddr_t pa;
		PHYSADDRLOSET(pa, (BUS_SWAP32(R_SM(&di->txd64[i].addrlow)) - di->dataoffsetlow));
		PHYSADDRHISET(pa, (BUS_SWAP32(R_SM(&di->txd64[i].addrhigh)) - di->dataoffsethigh));
#endif

		if (DMASGLIST_ENAB) {
			map = &di->txp_dmah[i];
			size = map->origsize;
			nsegs = map->nsegs;
			if (nsegs > (u_int)NTXDACTIVE(i, end)) {
				di->xs0cd = i;
				break;
			}
		} else {
#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__)) || \
	defined(BCM_SECURE_DMA)
			size = (BUS_SWAP32(R_SM(&di->txd64[i].ctrl2)) & D64_CTRL2_BC_MASK);
#endif
			nsegs = 1;
		}

		for (j = nsegs; j > 0; j--) {
#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
			W_SM(&di->txd64[i].addrlow, 0xdeadbeef);
			W_SM(&di->txd64[i].addrhigh, 0xdeadbeef);
#endif

			txp = di->txp[i];
			di->txp[i] = NULL;
			if (j > 1)
				i = NEXTTXD(i);
		}

#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__))
		DMA_UNMAP(di->osh, pa, size, DMA_TX, txp, map);
#endif

#ifdef BCM_SECURE_DMA
	SECURE_DMA_UNMAP(di->osh, pa, size, DMA_TX, NULL, NULL, &di->sec_cma_info_tx, 0);
#endif
	}

	di->txin = i;

	/* tx flow control */
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;

	return (txp);

bogus:
	BHND_DMA_TRACE(di, "bogus curr: start %d end %d txout %d", start, end,
	    di->txout);
	return (NULL);
}

void *
dma64_getnextrxp(dma_info_t *di, bool forceall)
{
	bhnd_dma_chan	*chan;
	void		*rxp;
	uint32_t	 status;
	uint16_t	 i, curr;
#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__)) || \
	defined(BCM_SECURE_DMA)
	dmaaddr_t	 pa;
#endif

	chan = di->chan;

	/* if forcing, dma engine must be disabled */
	ASSERT(!forceall || !dma64_rxenabled(di));

	i = di->rxin;

	/* return if no packets posted */
	if (i == di->rxout)
		return (NULL);

	if (di->rxin == di->rs0cd) {
		status = BHND_DMA_READ_4(chan, BHND_D64_STATUS0);
		curr = ((status & di->status0_cd_mask) - di->ptrbase) /
		    BHND_D64_DESC_SIZE;
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

#if ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__)) || \
	defined(BCM_SECURE_DMA)
	PHYSADDRLOSET(pa, (BUS_SWAP32(R_SM(&di->rxd64[i].addrlow)) - di->dataoffsetlow));
	PHYSADDRHISET(pa, (BUS_SWAP32(R_SM(&di->rxd64[i].addrhigh)) - di->dataoffsethigh));

	/* clear this packet from the descriptor ring */
#ifdef BCM_SECURE_DMA
	SECURE_DMA_UNMAP(di->osh, pa, di->rxbufsize, DMA_RX, NULL, NULL, &di->sec_cma_info_rx, 0);
#else
	DMA_UNMAP(di->osh, pa,
	          di->rxbufsize, DMA_RX, rxp, &di->rxp_dmah[i]);
#endif

	W_SM(&di->rxd64[i].addrlow, 0xdeadbeef);
	W_SM(&di->rxd64[i].addrhigh, 0xdeadbeef);
#endif /* ((!defined(__mips__) && !defined(BCM47XX_CA9)) || defined(__NetBSD__)) */

	di->rxin = NEXTRXD(i);

	return (rxp);
}

/**
 * Rotate all active tx dma ring entries "forward" by (ActiveDescriptor - txin).
 */
static void
dma64_txrotate(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint32_t	 w;
	uint16_t	 ad, old, new;
	uint16_t	 first, last;
	u_int		 nactive, rot;


	ASSERT(dma64_txsuspendedidle(di));

	chan = di->chan;
	nactive = _dma_txactive(di);

	ad = (((BHND_DMA_READ_4(chan, BHND_D64_STATUS1) & di->status1_ad_mask) -
		di->ptrbase) & di->status1_ad_mask) / BHND_D64_DESC_SIZE;
	rot = TXD(ad - di->txin);

	ASSERT(rot < di->ntxd);

	/* full-ring case is a lot harder - don't worry about this */
	/* XXX: what are we not worrying about? */
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
		w = BUS_SWAP32(R_SM(&di->txd64[old].ctrl1)) & ~D64_CTRL1_EOT;
		if (new == (di->ntxd - 1))
			w |= D64_CTRL1_EOT;
		W_SM(&di->txd64[new].ctrl1, BUS_SWAP32(w));

		w = BUS_SWAP32(R_SM(&di->txd64[old].ctrl2));
		W_SM(&di->txd64[new].ctrl2, BUS_SWAP32(w));

		W_SM(&di->txd64[new].addrlow, R_SM(&di->txd64[old].addrlow));
		W_SM(&di->txd64[new].addrhigh, R_SM(&di->txd64[old].addrhigh));

		/* zap the old tx dma descriptor address field */
		W_SM(&di->txd64[old].addrlow, BUS_SWAP32(0xdeadbeef));
		W_SM(&di->txd64[old].addrhigh, BUS_SWAP32(0xdeadbeef));

		/* move the corresponding txp[] entry */
		ASSERT(di->txp[new] == NULL);
		di->txp[new] = di->txp[old];

		/* Move the map */
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
	W_REG(di->osh, &di->d64txregs->ptr, di->ptrbase + I2B(di->txout, dma64dd_t));
}
