/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2018 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2015 Broadcom Corporation. All Rights Reserved.
 * All rights reserved.
 *
 * This file was derived from the hnddma.c source and hnddma.h header
 * distributed with the Asus RT-N18 firmware source code release.
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
 * $Id: hnddma.h 427480 2013-10-03 19:09:47Z $
 *
 * $FreeBSD$
 */

#ifndef _BHND_DMA_BHND_DMAVAR_H_
#define _BHND_DMA_BHND_DMAVAR_H_

#include <sys/types.h>
#include <sys/sbuf.h>

/* TODO: required for dma32regs_t/dma32dd_t and dma64regs_t/dma64dd_t */
#include "bhnd_dma32reg.h"
#include "bhnd_dma64reg.h"

/* XXX default configuration */
#define	BCMDBG
#define	BCMDMA64OSL
#define	BCMDMA32
#define	BCMDMA64

/* TODO: Required until we pull in bhnd(9) replacements */
#include "siutils.h"

/* debug/trace */
#ifdef BCMDBG
#define	DMA_ERROR(args) if (!(*di->msg_level & 1)); else printf args
#define	DMA_TRACE(args) if (!(*di->msg_level & 2)); else printf args
#elif defined(BCMDBG_ERR)
#define	DMA_ERROR(args) if (!(*di->msg_level & 1)); else printf args
#define DMA_TRACE(args)
#else
#define	DMA_ERROR(args)
#define	DMA_TRACE(args)
#endif /* BCMDBG */

#define	DMA_NONE(args)


/*
 * If BCMDMA32 is defined, hnddma will support both 32-bit and 64-bit DMA engines.
 * Otherwise it will support only 64-bit.
 *
 * DMA32_ENAB indicates whether hnddma is compiled with support for 32-bit DMA engines.
 * DMA64_ENAB indicates whether hnddma is compiled with support for 64-bit DMA engines.
 *
 * DMA64_MODE indicates whether the current DMA engine is running as 64-bit.
 */
#ifdef BCMDMA32
#define	DMA32_ENAB(di)		1
#define	DMA64_ENAB(di)		1
#define	DMA64_MODE(di)		((di)->dma64)
#else /* !BCMDMA32 */
#define	DMA32_ENAB(di)		0
#define	DMA64_ENAB(di)		1
#define	DMA64_MODE(di)		1
#endif /* !BCMDMA32 */

/* DMA Scatter-gather list is supported. Note this is limited to TX direction only */
#ifdef BCMDMASGLISTOSL
#define DMASGLIST_ENAB TRUE
#else
#define DMASGLIST_ENAB FALSE
#endif /* BCMDMASGLISTOSL */

typedef const struct hnddma_pub hnddma_t;

extern u_int dma_msg_level;

/* range param for dma_getnexttxp() and dma_txreclaim */
typedef enum txd_range {
	HNDDMA_RANGE_ALL		= 1,
	HNDDMA_RANGE_TRANSMITTED,
	HNDDMA_RANGE_TRANSFERED
} txd_range_t;

/* dma parameters id */
enum dma_param_id {
	HNDDMA_PID_TX_MULTI_OUTSTD_RD	= 0,
	HNDDMA_PID_TX_PREFETCH_CTL,
	HNDDMA_PID_TX_PREFETCH_THRESH,
	HNDDMA_PID_TX_BURSTLEN,

	HNDDMA_PID_RX_PREFETCH_CTL	= 0x100,
	HNDDMA_PID_RX_PREFETCH_THRESH,
	HNDDMA_PID_RX_BURSTLEN
};

/* dma function type */
typedef void (*di_detach_t)(hnddma_t *dmah);
typedef bool (*di_txreset_t)(hnddma_t *dmah);
typedef bool (*di_rxreset_t)(hnddma_t *dmah);
typedef bool (*di_rxidle_t)(hnddma_t *dmah);
typedef void (*di_txinit_t)(hnddma_t *dmah);
typedef bool (*di_txenabled_t)(hnddma_t *dmah);
typedef void (*di_rxinit_t)(hnddma_t *dmah);
typedef void (*di_txsuspend_t)(hnddma_t *dmah);
typedef void (*di_txresume_t)(hnddma_t *dmah);
typedef bool (*di_txsuspended_t)(hnddma_t *dmah);
typedef bool (*di_txsuspendedidle_t)(hnddma_t *dmah);
typedef void (*di_txflush_t)(hnddma_t *dmah);
typedef void (*di_txflush_clear_t)(hnddma_t *dmah);
typedef int (*di_txfast_t)(hnddma_t *dmah, void *p, bool commit);
typedef int (*di_txunframed_t)(hnddma_t *dmah, void *p, u_int len, bool commit);
typedef void* (*di_getpos_t)(hnddma_t *di, bool direction);
typedef void (*di_fifoloopbackenable_t)(hnddma_t *dmah);
typedef bool  (*di_txstopped_t)(hnddma_t *dmah);
typedef bool  (*di_rxstopped_t)(hnddma_t *dmah);
typedef bool  (*di_rxenable_t)(hnddma_t *dmah);
typedef bool  (*di_rxenabled_t)(hnddma_t *dmah);
typedef void* (*di_rx_t)(hnddma_t *dmah);
typedef bool (*di_rxfill_t)(hnddma_t *dmah);
typedef void (*di_txreclaim_t)(hnddma_t *dmah, txd_range_t range);
typedef void (*di_rxreclaim_t)(hnddma_t *dmah);
typedef	uintptr_t	(*di_getvar_t)(hnddma_t *dmah, const char *name);
typedef void* (*di_getnexttxp_t)(hnddma_t *dmah, txd_range_t range);
typedef void* (*di_getnextrxp_t)(hnddma_t *dmah, bool forceall);
typedef void* (*di_peeknexttxp_t)(hnddma_t *dmah);
typedef void* (*di_peekntxp_t)(hnddma_t *dmah, int *len, void *txps[], txd_range_t range);
typedef void* (*di_peeknextrxp_t)(hnddma_t *dmah);
typedef void (*di_rxparam_get_t)(hnddma_t *dmah, uint16_t *rxoffset, uint16_t *rxbufsize);
typedef void (*di_txblock_t)(hnddma_t *dmah);
typedef void (*di_txunblock_t)(hnddma_t *dmah);
typedef u_int (*di_txactive_t)(hnddma_t *dmah);
typedef void (*di_txrotate_t)(hnddma_t *dmah);
typedef void (*di_counterreset_t)(hnddma_t *dmah);
typedef u_int (*di_ctrlflags_t)(hnddma_t *dmah, u_int mask, u_int flags);
typedef char* (*di_dump_t)(hnddma_t *dmah, struct sbuf *s, bool dumpring);
typedef char* (*di_dumptx_t)(hnddma_t *dmah, struct sbuf *s, bool dumpring);
typedef char* (*di_dumprx_t)(hnddma_t *dmah, struct sbuf *s, bool dumpring);
typedef u_int (*di_rxactive_t)(hnddma_t *dmah);
typedef u_int (*di_txpending_t)(hnddma_t *dmah);
typedef u_int (*di_txcommitted_t)(hnddma_t *dmah);
typedef int (*di_pktpool_set_t)(hnddma_t *dmah, pktpool_t *pool);
typedef bool (*di_rxtxerror_t)(hnddma_t *dmah, bool istx);
typedef void (*di_burstlen_set_t)(hnddma_t *dmah, uint8_t rxburstlen, uint8_t txburstlen);
typedef u_int (*di_avoidancecnt_t)(hnddma_t *dmah);
typedef void (*di_param_set_t)(hnddma_t *dmah, uint16_t paramid, uint16_t paramval);
typedef bool (*dma_glom_enable_t) (hnddma_t *dmah, uint32_t val);
typedef u_int (*dma_active_rxbuf_t) (hnddma_t *dmah);

/* dma opsvec */
typedef struct di_fcn_s {
	di_detach_t		detach;
	di_txinit_t             txinit;
	di_txreset_t		txreset;
	di_txenabled_t          txenabled;
	di_txsuspend_t          txsuspend;
	di_txresume_t           txresume;
	di_txsuspended_t        txsuspended;
	di_txsuspendedidle_t    txsuspendedidle;
	di_txflush_t            txflush;
	di_txflush_clear_t      txflush_clear;
	di_txfast_t             txfast;
	di_txunframed_t         txunframed;
	di_getpos_t             getpos;
	di_txstopped_t		txstopped;
	di_txreclaim_t          txreclaim;
	di_getnexttxp_t         getnexttxp;
	di_peeknexttxp_t        peeknexttxp;
	di_peekntxp_t           peekntxp;
	di_txblock_t            txblock;
	di_txunblock_t          txunblock;
	di_txactive_t           txactive;
	di_txrotate_t           txrotate;

	di_rxinit_t             rxinit;
	di_rxreset_t            rxreset;
	di_rxidle_t             rxidle;
	di_rxstopped_t		rxstopped;
	di_rxenable_t		rxenable;
	di_rxenabled_t		rxenabled;
	di_rx_t                 rx;
	di_rxfill_t             rxfill;
	di_rxreclaim_t          rxreclaim;
	di_getnextrxp_t         getnextrxp;
	di_peeknextrxp_t        peeknextrxp;
	di_rxparam_get_t	rxparam_get;

	di_fifoloopbackenable_t fifoloopbackenable;
	di_getvar_t             d_getvar;
	di_counterreset_t       counterreset;
	di_ctrlflags_t          ctrlflags;
	di_dump_t		dump;
	di_dumptx_t		dumptx;
	di_dumprx_t		dumprx;
	di_rxactive_t		rxactive;
	di_txpending_t		txpending;
	di_txcommitted_t	txcommitted;
	di_pktpool_set_t	pktpool_set;
	di_rxtxerror_t		rxtxerror;
	di_burstlen_set_t	burstlen_set;
	di_avoidancecnt_t	avoidancecnt;
	di_param_set_t		param_set;
	dma_glom_enable_t	glom_enab;
	dma_active_rxbuf_t	dma_activerxbuf;
	u_int			endnum;
} di_fcn_t;


/*
 * Exported data structure (read-only)
 */
/* export structure */
struct hnddma_pub {
	const di_fcn_t	*di_fn;		/* DMA function pointers */
	u_int		txavail;	/* # free tx descriptors */
	u_int		dmactrlflags;	/* dma control flags */

	/* rx error counters */
	u_int		rxgiants;	/* rx giant frames */
	u_int		rxnobuf;	/* rx out of dma descriptors */
	/* tx error counters */
	u_int		txnobuf;	/* tx out of dma descriptors */
	u_int		txnodesc;	/* tx out of dma descriptors running count */
};

#define	MAXNAMEL	8		/* 8 char names */

/** dma engine software state */
typedef struct dma_info {
	struct hnddma_pub hnddma;	/* exported structure, don't use hnddma_t,
					 * which could be const
					 */
	u_int		*msg_level;	/* message level pointer */
	char		name[MAXNAMEL];	/* callers name for diag msgs */

	void		*osh;		/* os handle */
	si_t		*sih;		/* sb handle */

	bool		dma64;		/* this dma engine is operating in 64-bit mode */
	bool		addrext;	/* this dma engine supports DmaExtendedAddrChanges */

	union {
		struct {
			dma32regs_t	*txregs_32;	/* 32-bit dma tx engine registers */
			dma32regs_t	*rxregs_32;	/* 32-bit dma rx engine registers */
			dma32dd_t	*txd_32;	/* pointer to dma32 tx descriptor ring */
			dma32dd_t	*rxd_32;	/* pointer to dma32 rx descriptor ring */
		} d32_u;
		struct {
			dma64regs_t	*txregs_64;	/* 64-bit dma tx engine registers */
			dma64regs_t	*rxregs_64;	/* 64-bit dma rx engine registers */
			dma64dd_t	*txd_64;	/* pointer to dma64 tx descriptor ring */
			dma64dd_t	*rxd_64;	/* pointer to dma64 rx descriptor ring */
		} d64_u;
	} dregs;

	uint16_t		dmadesc_align;	/* alignment requirement for dma descriptors */

	uint16_t		ntxd;		/* # tx descriptors tunable */
	uint16_t		txin;		/* index of next descriptor to reclaim */
	uint16_t		txout;		/* index of next descriptor to post */
	void		**txp;		/* pointer to parallel array of pointers to packets */
	osldma_t 	*tx_dmah;	/* DMA TX descriptor ring handle */
	hnddma_seg_map_t	*txp_dmah;	/* DMA MAP meta-data handle */
	dmaaddr_t	txdpa;		/* Aligned physical address of descriptor ring */
	dmaaddr_t	txdpaorig;	/* Original physical address of descriptor ring */
	uint16_t		txdalign;	/* #bytes added to alloc'd mem to align txd */
	uint32_t		txdalloc;	/* #bytes allocated for the ring */
	uint32_t		xmtptrbase;	/* When using unaligned descriptors, the ptr register
					 * is not just an index, it needs all 13 bits to be
					 * an offset from the addr register.
					 */

	uint16_t		nrxd;		/* # rx descriptors tunable */
	uint16_t		rxin;		/* index of next descriptor to reclaim */
	uint16_t		rxout;		/* index of next descriptor to post */
	void		**rxp;		/* pointer to parallel array of pointers to packets */
	osldma_t 	*rx_dmah;	/* DMA RX descriptor ring handle */
	hnddma_seg_map_t	*rxp_dmah;	/* DMA MAP meta-data handle */
	dmaaddr_t	rxdpa;		/* Aligned physical address of descriptor ring */
	dmaaddr_t	rxdpaorig;	/* Original physical address of descriptor ring */
	uint16_t		rxdalign;	/* #bytes added to alloc'd mem to align rxd */
	uint32_t		rxdalloc;	/* #bytes allocated for the ring */
	uint32_t		rcvptrbase;	/* Base for ptr reg when using unaligned descriptors */

	/* tunables */
	uint16_t		rxbufsize;	/* rx buffer size in bytes,
					 * not including the extra headroom
					 */
	u_int		rxextrahdrroom;	/* extra rx headroom, reverseved to assist upper stack
					 *  e.g. some rx pkt buffers will be bridged to tx side
					 *  without byte copying. The extra headroom needs to be
					 *  large enough to fit txheader needs.
					 *  Some dongle driver may not need it.
					 */
	u_int		nrxpost;	/* # rx buffers to keep posted */
	u_int		rxoffset;	/* rxcontrol offset */
	u_int		ddoffsetlow;	/* add to get dma address of descriptor ring, low 32 bits */
	u_int		ddoffsethigh;	/*   high 32 bits */
	u_int		dataoffsetlow;	/* add to get dma address of data buffer, low 32 bits */
	u_int		dataoffsethigh;	/*   high 32 bits */
	bool		aligndesc_4k;	/* descriptor base need to be aligned or not */
	uint8_t		rxburstlen;	/* burstlen field for rx (for cores supporting burstlen) */
	uint8_t		txburstlen;	/* burstlen field for tx (for cores supporting burstlen) */
	uint8_t		txmultioutstdrd; 	/* tx multiple outstanding reads */
	uint8_t 		txprefetchctl;	/* prefetch control for tx */
	uint8_t 		txprefetchthresh;	/* prefetch threshold for tx */
	uint8_t 		rxprefetchctl;	/* prefetch control for rx */
	uint8_t 		rxprefetchthresh;	/* prefetch threshold for rx */
	pktpool_t	*pktpool;	/* pktpool */
	u_int		dma_avoidance_cnt;

	uint32_t 		d64_xs0_cd_mask; /* tx current descriptor pointer mask */
	uint32_t 		d64_xs1_ad_mask; /* tx active descriptor mask */
	uint32_t 		d64_rs0_cd_mask; /* rx current descriptor pointer mask */
	uint16_t		rs0cd;		/* cached value of rcvstatus0 currdescr */
	uint16_t		xs0cd;		/* cached value of xmtstatus0 currdescr */
	uint16_t		xs0cd_snapshot;	/* snapshot of xmtstatus0 currdescr */
#ifdef BCM_SECURE_DMA
	struct sec_cma_info sec_cma_info_rx;
	struct sec_cma_info sec_cma_info_tx;
#endif
} dma_info_t;

/* XXX */
#define d32txregs	dregs.d32_u.txregs_32
#define d32rxregs	dregs.d32_u.rxregs_32
#define txd32		dregs.d32_u.txd_32
#define rxd32		dregs.d32_u.rxd_32

#define d64txregs	dregs.d64_u.txregs_64
#define d64rxregs	dregs.d64_u.rxregs_64
#define txd64		dregs.d64_u.txd_64
#define rxd64		dregs.d64_u.rxd_64


/* TODO: Replace with our own attach */
extern hnddma_t * dma_attach(osl_t *osh, const char *name, si_t *sih,
	volatile void *dmaregstx, volatile void *dmaregsrx,
	u_int ntxd, u_int nrxd, u_int rxbufsize, int rxextheadroom, u_int nrxpost,
	u_int rxoffset, u_int *msg_level);

#ifdef BCMDMA32

#define dma_detach(di)			((di)->di_fn->detach(di))
#define dma_txreset(di)			((di)->di_fn->txreset(di))
#define dma_rxreset(di)			((di)->di_fn->rxreset(di))
#define dma_rxidle(di)			((di)->di_fn->rxidle(di))
#define dma_txinit(di)                  ((di)->di_fn->txinit(di))
#define dma_txenabled(di)               ((di)->di_fn->txenabled(di))
#define dma_rxinit(di)                  ((di)->di_fn->rxinit(di))
#define dma_txsuspend(di)               ((di)->di_fn->txsuspend(di))
#define dma_txresume(di)                ((di)->di_fn->txresume(di))
#define dma_txsuspended(di)             ((di)->di_fn->txsuspended(di))
#define dma_txsuspendedidle(di)         ((di)->di_fn->txsuspendedidle(di))
#define dma_txflush(di)                 ((di)->di_fn->txflush(di))
#define dma_txflush_clear(di)           ((di)->di_fn->txflush_clear(di))
#define dma_txfast(di, p, commit)	((di)->di_fn->txfast(di, p, commit))
#define dma_fifoloopbackenable(di)      ((di)->di_fn->fifoloopbackenable(di))
#define dma_txstopped(di)               ((di)->di_fn->txstopped(di))
#define dma_rxstopped(di)               ((di)->di_fn->rxstopped(di))
#define dma_rxenable(di)                ((di)->di_fn->rxenable(di))
#define dma_rxenabled(di)               ((di)->di_fn->rxenabled(di))
#define dma_rx(di)                      ((di)->di_fn->rx(di))
#define dma_rxfill(di)                  ((di)->di_fn->rxfill(di))
#define dma_txreclaim(di, range)	((di)->di_fn->txreclaim(di, range))
#define dma_rxreclaim(di)               ((di)->di_fn->rxreclaim(di))
#define dma_getvar(di, name)		((di)->di_fn->d_getvar(di, name))
#define dma_getnexttxp(di, range)	((di)->di_fn->getnexttxp(di, range))
#define dma_getnextrxp(di, forceall)    ((di)->di_fn->getnextrxp(di, forceall))
#define dma_peeknexttxp(di)             ((di)->di_fn->peeknexttxp(di))
#define dma_peekntxp(di, l, t, r)       ((di)->di_fn->peekntxp(di, l, t, r))
#define dma_peeknextrxp(di)             ((di)->di_fn->peeknextrxp(di))
#define dma_rxparam_get(di, off, bufs)	((di)->di_fn->rxparam_get(di, off, bufs))

#define dma_txblock(di)                 ((di)->di_fn->txblock(di))
#define dma_txunblock(di)               ((di)->di_fn->txunblock(di))
#define dma_txactive(di)                ((di)->di_fn->txactive(di))
#define dma_rxactive(di)                ((di)->di_fn->rxactive(di))
#define dma_txrotate(di)                ((di)->di_fn->txrotate(di))
#define dma_counterreset(di)            ((di)->di_fn->counterreset(di))
#define dma_ctrlflags(di, mask, flags)  ((di)->di_fn->ctrlflags((di), (mask), (flags)))
#define dma_txpending(di)		((di)->di_fn->txpending(di))
#define dma_txcommitted(di)		((di)->di_fn->txcommitted(di))
#define dma_pktpool_set(di, pool)	((di)->di_fn->pktpool_set((di), (pool)))
#if defined(BCMDBG)
#define dma_dump(di, buf, dumpring)	((di)->di_fn->dump(di, buf, dumpring))
#define dma_dumptx(di, buf, dumpring)	((di)->di_fn->dumptx(di, buf, dumpring))
#define dma_dumprx(di, buf, dumpring)	((di)->di_fn->dumprx(di, buf, dumpring))
#endif 
#define dma_rxtxerror(di, istx)	((di)->di_fn->rxtxerror(di, istx))
#define dma_burstlen_set(di, rxlen, txlen)	((di)->di_fn->burstlen_set(di, rxlen, txlen))
#define dma_avoidance_cnt(di)		((di)->di_fn->avoidancecnt(di))
#define dma_param_set(di, paramid, paramval)	((di)->di_fn->param_set(di, paramid, paramval))
#define dma_activerxbuf(di)		((di)->di_fn->dma_activerxbuf(di))

#else /* BCMDMA32 */
extern const di_fcn_t dma64proc;

#define dma_detach(di)			(dma64proc.detach(di))
#define dma_txreset(di)			(dma64proc.txreset(di))
#define dma_rxreset(di)			(dma64proc.rxreset(di))
#define dma_rxidle(di)			(dma64proc.rxidle(di))
#define dma_txinit(di)                  (dma64proc.txinit(di))
#define dma_txenabled(di)               (dma64proc.txenabled(di))
#define dma_rxinit(di)                  (dma64proc.rxinit(di))
#define dma_txsuspend(di)               (dma64proc.txsuspend(di))
#define dma_txresume(di)                (dma64proc.txresume(di))
#define dma_txsuspended(di)             (dma64proc.txsuspended(di))
#define dma_txsuspendedidle(di)         (dma64proc.txsuspendedidle(di))
#define dma_txflush(di)                 (dma64proc.txflush(di))
#define dma_txflush_clear(di)           (dma64proc.txflush_clear(di))
#define dma_txfast(di, p, commit)	(dma64proc.txfast(di, p, commit))
#define dma_txunframed(di, p, l, commit)(dma64proc.txunframed(di, p, l, commit))
#define dma_getpos(di, dir)		(dma64proc.getpos(di, dir))
#define dma_fifoloopbackenable(di)      (dma64proc.fifoloopbackenable(di))
#define dma_txstopped(di)               (dma64proc.txstopped(di))
#define dma_rxstopped(di)               (dma64proc.rxstopped(di))
#define dma_rxenable(di)                (dma64proc.rxenable(di))
#define dma_rxenabled(di)               (dma64proc.rxenabled(di))
#define dma_rx(di)                      (dma64proc.rx(di))
#define dma_rxfill(di)                  (dma64proc.rxfill(di))
#define dma_txreclaim(di, range)	(dma64proc.txreclaim(di, range))
#define dma_rxreclaim(di)               (dma64proc.rxreclaim(di))
#define dma_getvar(di, name)		(dma64proc.d_getvar(di, name))
#define dma_getnexttxp(di, range)	(dma64proc.getnexttxp(di, range))
#define dma_getnextrxp(di, forceall)    (dma64proc.getnextrxp(di, forceall))
#define dma_peeknexttxp(di)             (dma64proc.peeknexttxp(di))
#define dma_peekntxp(di, l, t, r)       (dma64proc.peekntxp(di, l, t, r))
#define dma_peeknextrxp(di)             (dma64proc.peeknextrxp(di))
#define dma_rxparam_get(di, off, bufs)	(dma64proc.rxparam_get(di, off, bufs))

#define dma_txblock(di)                 (dma64proc.txblock(di))
#define dma_txunblock(di)               (dma64proc.txunblock(di))
#define dma_txactive(di)                (dma64proc.txactive(di))
#define dma_rxactive(di)                (dma64proc.rxactive(di))
#define dma_txrotate(di)                (dma64proc.txrotate(di))
#define dma_counterreset(di)            (dma64proc.counterreset(di))
#define dma_ctrlflags(di, mask, flags)  (dma64proc.ctrlflags((di), (mask), (flags)))
#define dma_txpending(di)		(dma64proc.txpending(di))
#define dma_txcommitted(di)		(dma64proc.txcommitted(di))
#define dma_pktpool_set(di, pool)	(dma64proc.pktpool_set((di), (pool)))
#if defined(BCMDBG)
#define dma_dump(di, buf, dumpring)	(dma64proc.dump(di, buf, dumpring))
#define dma_dumptx(di, buf, dumpring)	(dma64proc.dumptx(di, buf, dumpring))
#define dma_dumprx(di, buf, dumpring)	(dma64proc.dumprx(di, buf, dumpring))
#endif
#define dma_rxtxerror(di, istx)	(dma64proc.rxtxerror(di, istx))
#define dma_burstlen_set(di, rxlen, txlen)	(dma64proc.burstlen_set(di, rxlen, txlen))
#define dma_avoidance_cnt(di)		(dma64proc.avoidancecnt(di))
#define dma_param_set(di, paramid, paramval)	(dma64proc.param_set(di, paramid, paramval))

#define dma_glom_enable(di, val)	(dma64proc.glom_enab(di, val))
#define dma_activerxbuf(di)	(dma64proc.dma_activerxbuf(di))

#endif /* BCMDMA32 */

/* return addresswidth allowed
 * This needs to be done after SB attach but before dma attach.
 * SB attach provides ability to probe backplane and dma core capabilities
 * This info is needed by DMA_ALLOC_CONSISTENT in dma attach
 */
extern u_int dma_addrwidth(si_t *sih, void *dmaregs);

/* pio helpers */
extern void dma_txpioloopback(osl_t *osh, dma32regs_t *);


/* descriptor bumping macros */
#define	XXD(x, n)	((x) & ((n) - 1))	/* faster than %, but n must be power of 2 */
#define	TXD(x)		XXD((x), di->ntxd)
#define	RXD(x)		XXD((x), di->nrxd)
#define	NEXTTXD(i)	TXD((i) + 1)
#define	PREVTXD(i)	TXD((i) - 1)
#define	NEXTRXD(i)	RXD((i) + 1)
#define	PREVRXD(i)	RXD((i) - 1)

#define	NTXDACTIVE(h, t)	TXD((t) - (h))
#define	NRXDACTIVE(h, t)	RXD((t) - (h))

/* macros to convert between byte offsets and indexes */
#define	B2I(bytes, type)	((uint16_t)((bytes) / sizeof(type)))
#define	I2B(index, type)	((index) * sizeof(type))

/* XXX TODO: to be replaced with bhnd_dma_translation */
#define	PCI32ADDR_HIGH		0xc0000000	/* address[31:30] */
#define	PCI32ADDR_HIGH_SHIFT	30		/* address[31:30] */

#define	PCI64ADDR_HIGH		0x80000000	/* address[63] */
#define	PCI64ADDR_HIGH_SHIFT	31		/* address[63] */

/* Common function implementations */
void _dma_detach(dma_info_t *di);
void *_dma_peeknexttxp(dma_info_t *di);
int _dma_peekntxp(dma_info_t *di, int *len, void *txps[], txd_range_t range);

void _dma_txblock(dma_info_t *di);
void _dma_txunblock(dma_info_t *di);
u_int _dma_txactive(dma_info_t *di);

void _dma_rxinit(dma_info_t *di);
void _dma_rxenable(dma_info_t *di);
void *_dma_rx(dma_info_t *di);
bool _dma_rxfill(dma_info_t *di);
void _dma_rxreclaim(dma_info_t *di);
void *_dma_getnextrxp(dma_info_t *di, bool forceall);
void *_dma_peeknextrxp(dma_info_t *di);
void _dma_rx_param_get(dma_info_t *di, uint16_t *rxoffset, uint16_t *rxbufsize);

void _dma_fifoloopbackenable(dma_info_t *di);
uintptr_t _dma_getvar(dma_info_t *di, const char *name);
void _dma_counterreset(dma_info_t *di);
u_int _dma_ctrlflags(dma_info_t *di, u_int mask, u_int flags);

u_int _dma_rxactive(dma_info_t *di);
u_int _dma_txpending(dma_info_t *di);
u_int _dma_txcommitted(dma_info_t *di);
int _dma_pktpool_set(dma_info_t *di, pktpool_t *pool);
bool _dma_rxtx_error(dma_info_t *di, bool istx);
void _dma_burstlen_set(dma_info_t *di, uint8_t rxburstlen, uint8_t txburstlen);
u_int _dma_avoidancecnt(dma_info_t *di);
void _dma_param_set(dma_info_t *di, uint16_t paramid, uint16_t paramval);

void _dma_ddtable_init(dma_info_t *di, u_int direction, dmaaddr_t pa);

void *dma_ringalloc(osl_t *osh, uint32_t boundary, u_int size, uint16_t *alignbits, u_int* alloced,
	dmaaddr_t *descpa, osldma_t **dmah);


/* XXX only used by DMA64 ? */
bool _dma_glom_enable(dma_info_t *di, uint32_t val);
u_int _dma_activerxbuf(dma_info_t *di);

#endif /* _BHND_DMA_BHND_DMAVAR_H_ */
