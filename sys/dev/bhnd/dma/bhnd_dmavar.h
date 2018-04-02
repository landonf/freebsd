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
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/sbuf.h>

#include <dev/bhnd/bhnd_debug.h>

#include "bhnd_dma.h"

MALLOC_DECLARE(M_BHND_DMA);

typedef const struct hnddma_pub hnddma_t; // XXX legacy definition

/**
 * BHND DMA register layout.
 */
typedef enum bhnd_dma_regfmt {
	BHND_DMA_REGFMT32	= 0,	/**< 32-bit register layout */
	BHND_DMA_REGFMT64	= 1,	/**< 64-bit register layout */
} bhnd_dma_regfmt;

const char	*bhnd_dma_direction_name(bhnd_dma_direction direction);

/**
 * BHND DMA engine.
 */
struct bhnd_dma {
	device_t			 owner;		/**< parent device */
	bhnd_dma_regfmt			 regfmt;	/**< DMA engine register layout */
	uint32_t			 quirks;	/**< DMA engine quirks (see bhnd_dma_quirk) */
	
	bus_space_tag_t			 regs_bst;	/**< DMA register block bus tag */
	bus_space_handle_t		 regs_bsh;	/**< DMA register block bus handle */
	bus_size_t			 regs_size;	/**< DMA register block size */
	uintptr_t			 regs_virt;	/**< XXX register virtual base address */

	bool				 addrext;	/**< true if DmaExtendedAddrChanges supported */
	uint32_t 			 st0_cd_mask;	/**< status0 current descriptor pointer mask */
	uint32_t 			 st1_ad_mask;	/**< status1 active descriptor pointer mask */
	u_int				 max_ndesc;	/**< maximum descriptor count */

	bhnd_dma_chan			*tx_chan;	/**< DMA transmit channels */
	size_t				 num_tx_chan;	/**< transmit channel count */

	bhnd_dma_chan			*rx_chan;	/**< DMA receive channels */
	size_t				 num_rx_chan;	/**< receive channel count */

	struct sx			 chan_lock;	/**< channel state lock */
};

/**
 * BHND DMA channel.
 */
struct bhnd_dma_chan {
	bhnd_dma		*dma;		/**< dma engine reference */
	bus_space_handle_t	 bsh;		/**< per-channel register block bus handle */
	bhnd_dma_direction	 direction;	/**< channel direction */
	size_t			 num;		/**< channel number */

	u_int			 ndesc;		/**< descriptor count */
	bool			 enabled;	/**< true if channel has been enabled */

	hnddma_t		*di;		/**< XXX legacy hnddma instance */
};

/* XXX TODO: Use device_printf() variants? Fix level handling! */
#define	_BHND_DMA_CHAN_PRINTF(_level, _ch, _fmt, ...) do {		\
	printf("%s.dma_%s%zu: " _fmt "\n",				\
	    device_get_nameunit((_ch)->dma->owner),			\
	    bhnd_dma_direction_name((_ch)->direction), (_ch)->num,	\
	    ## __VA_ARGS__);						\
} while (0)

#define	BHND_DMA_CHAN_TRACE(_ch, _fmt, ...)	\
	_BHND_DMA_CHAN_PRINTF(BHND_TRACE_LEVEL, (_ch), _fmt, ## __VA_ARGS__)

#define	BHND_DMA_CHAN_ERROR(_ch, _fmt, ...)	\
	_BHND_DMA_CHAN_PRINTF(BHND_ERROR_LEVEL, (_ch), _fmt, ## __VA_ARGS__)

/* XXX TODO: Use device_printf() variants? */
#define	_BHND_DMA_PRINTF(_level, _di, _fmt, ...) do {			\
	if (*(_di)->msg_level >= _level) {				\
		device_printf((_di)->chan->dma->owner,			\
		    "%s%zu" _fmt "\n",					\
		    bhnd_dma_direction_name((_di)->chan->direction),	\
		    (_di)->chan->num, ## __VA_ARGS__);			\
	}								\
} while (0)

#define	BHND_DMA_TRACE_ENTRY(_di)	\
	BHND_DMA_TRACE(_di, "[enter]")

#define	BHND_DMA_TRACE(_di, _fmt, ...)	\
	_BHND_DMA_PRINTF(BHND_TRACE_LEVEL, (_di), " %s(): " _fmt,	\
	    __FUNCTION__, ## __VA_ARGS__)
	
#define	BHND_DMA_DEBUG(_di, _fmt, ...)	\
	_BHND_DMA_PRINTF(BHND_DEBUG_LEVEL, (_di), ": " _fmt,		\
	    ## __VA_ARGS__)

#define	BHND_DMA_ERROR(_di, _fmt, ...)	\
	_BHND_DMA_PRINTF(BHND_ERROR_LEVEL, (_di), ": " _fmt, ## __VA_ARGS__)

#define	BHND_DMA_WARN(_di, _fmt, ...)	\
	_BHND_DMA_PRINTF(BHND_WARN_LEVEL, (_di), ": " _fmt, ## __VA_ARGS__)

/* XXX TODO: normalize these macros */
#define	_BHND_DMA_PRINTF_NEW(_level, _dma, _fmt, ...) do {		\
	device_printf((_dma)->owner, _fmt "\n", ## __VA_ARGS__);	\
} while (0)

#define	BHND_DMA_TRACE_NEW(_dma, _fmt, ...)	\
	_BHND_DMA_PRINTF_NEW(BHND_TRACE_LEVEL, (_dma), _fmt, ## __VA_ARGS__)

#define	BHND_DMA_ERROR_NEW(_dma, _fmt, ...)	\
	_BHND_DMA_PRINTF_NEW(BHND_ERROR_LEVEL, (_dma), _fmt, ## __VA_ARGS__)

#define	BHND_DMA_WARN_NEW(_dma, _fmt, ...)	\
	_BHND_DMA_PRINTF_NEW(BHND_WARN_LEVEL, (_dma), _fmt, ## __VA_ARGS__)

/**********************************
 * XXX LEGACY DECLARATIONS FOLLOW *
 **********************************/

/* XXX */
#define	BHND_BUILD_DMA32	1	/* build DMA32 support */
#define	BHND_BUILD_DMA64	1	/* build DMA64 support */

#define	BHND_DMA_UNIMPL()	panic("%s unimplemented", __FUNCTION__);

#ifdef BHND_BUILD_DMA32
#define	BHND_DMA32_SUPPORT(di)	(true)
#else /* BHND_BUILD_DMA32 */
#define	BHND_DMA32_SUPPORT(di)	(false)
#endif /* !BHND_BUILD_DMA32 */

#ifdef BHND_BUILD_DMA64
#define	BHND_DMA64_SUPPORT(di)	(true)
#else /* BHND_BUILD_DMA64 */
#define	BHND_DMA64_SUPPORT(di)	(false)
#endif /* !BHND_BUILD_DMA64 */

#define	BHND_DMA64_MODE(di)	((di)->dma64)

/* TODO: required for dma32regs_t/dma32dd_t and dma64regs_t/dma64dd_t */
#include "bhnd_dma32reg.h"
#include "bhnd_dma64reg.h"

/* XXX default configuration */
#define	BCMDBG
#define	BCMDMA64OSL

/* TODO: Required until we pull in bhnd(9) replacements */
#include "siutils.h"

/* DMA Scatter-gather list is supported. Note this is limited to TX direction only */
#ifdef BCMDMASGLISTOSL
#define DMASGLIST_ENAB TRUE
#else
#define DMASGLIST_ENAB FALSE
#endif /* BCMDMASGLISTOSL */

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
	bhnd_dma_chan	*chan;		/**< DMA channel reference */

	u_int		*msg_level;	/* message level pointer */
	char		name[MAXNAMEL];	/* callers name for diag msgs */

	void		*osh;		/* os handle */
	si_t		*sih;		/* sb handle */

	bool		dma64;		/* this dma engine is operating in 64-bit mode */

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
	
	uint32_t		ptrbase;	/* base for ptr reg when using unaligned descriptors.
						 *
						 * When using unaligned descriptors, the ptr register
						 * is not just an index, it needs all 13 bits to be
						 * an offset from the addr register. */

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
extern hnddma_t * dma_attach(bhnd_dma_chan *chan,
	const char *name, volatile void *dmaregstx, volatile void *dmaregsrx,
	u_int ntxd, u_int nrxd, u_int rxbufsize, int rxextheadroom, u_int nrxpost,
	u_int rxoffset, u_int *msg_level);

/* XXX: this is a silly approach to handling indirection */
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

#ifdef BHND_DMA64
extern const di_fcn_t dma64proc;
#endif /* BHND_DMA64 */

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

void _dma_ddtable_init(dma_info_t *di, dmaaddr_t pa);

void *dma_ringalloc(osl_t *osh, uint32_t boundary, u_int size, uint16_t *alignbits, u_int* alloced,
	dmaaddr_t *descpa, osldma_t **dmah);


/* XXX only used by DMA64 ? */
bool _dma_glom_enable(dma_info_t *di, uint32_t val);
u_int _dma_activerxbuf(dma_info_t *di);

#endif /* _BHND_DMA_BHND_DMAVAR_H_ */
