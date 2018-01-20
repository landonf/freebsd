/*
 * Misc utility routines for accessing the SOC Interconnects
 * of Broadcom HNBU chips.
 *
 * Copyright (C) 2015, Broadcom Corporation. All Rights Reserved.
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
 * $Id: siutils.h 499106 2014-08-27 15:15:26Z $
 */

#ifndef	_siutils_h_
#define	_siutils_h_

#define BCME_ERROR			-1	/* Error generic */
#define BCME_OK				0	/* Success */


#define SI_PCI_DMA		0x40000000	/* Client Mode sb2pcitranslation2 (1 GB) */
#define SI_PCI_DMA2		0x80000000	/* Client Mode sb2pcitranslation2 (1 GB) */
#define SI_PCI_DMA_SZ		0x40000000	/* Client Mode sb2pcitranslation2 size in bytes */

#define SI_PCIE_DMA_H32		0x80000000	/* PCIE Client Mode sb2pcitranslation2
						 * (2 ZettaBytes), high 32 bits */

/*
 * Data structure to export all chip specific common variables
 *   public (read-only) portion of siutils handle returned by si_attach()/si_kattach()
 */
struct si_pub {
	u_int	bustype;		/* SI_BUS, PCI_BUS */
	u_int	buscoretype;		/* PCI_CORE_ID, PCIE_CORE_ID, PCMCIA_CORE_ID */

	u_int	chip;			/* chip number */
	u_int	chiprev;		/* chip revision */
};


/* for HIGH_ONLY driver, the si_t must be writable to allow states sync from BMAC to HIGH driver
 * for monolithic driver, it is readonly to prevent accident change
 */
#if defined(WLC_HIGH) && !defined(WLC_LOW)
typedef struct si_pub si_t;
#else
typedef const struct si_pub si_t;
#endif

extern uint32_t si_core_sflags(si_t *sih, uint32_t mask, uint32_t val);
extern u_int si_corerev(si_t *sih);
extern u_int si_coreid(si_t *sih);

extern void *si_osh(si_t *sih);
bool si_backplane64(si_t *sih);


/* Bus types */
#define	SI_BUS			0	/* SOC Interconnect */
#define	PCI_BUS			1	/* PCI target */

#ifdef BCMDMA64OSL
typedef struct {
	uint32_t loaddr;
	uint32_t hiaddr;
} dma64addr_t;

typedef dma64addr_t dmaaddr_t;
#define PHYSADDRHI(_pa) ((_pa).hiaddr)
#define PHYSADDRHISET(_pa, _val) \
	do { \
		(_pa).hiaddr = (_val);		\
	} while (0)
#define PHYSADDRLO(_pa) ((_pa).loaddr)
#define PHYSADDRLOSET(_pa, _val) \
	do { \
		(_pa).loaddr = (_val);		\
	} while (0)

#else
typedef unsigned long dmaaddr_t;
#define PHYSADDRHI(_pa) (0)
#define PHYSADDRHISET(_pa, _val)
#define PHYSADDRLO(_pa) ((_pa))
#define PHYSADDRLOSET(_pa, _val) \
	do { \
		(_pa) = (_val);			\
	} while (0)
#endif /* BCMDMA64OSL */

/* One physical DMA segment */
typedef struct  {
	dmaaddr_t addr;
	uint32_t	  length;
} hnddma_seg_t;

#if defined(MACOSX)
/* In MacOS, the OS API may return large number of segments. Setting this number lower
 * will result in failure of dma map
 */
#define MAX_DMA_SEGS 8
#elif defined(__NetBSD__)
/* In NetBSD we also want more segments because the lower level mbuf mapping api might
 * allocate a large number of segments
 */
#define MAX_DMA_SEGS 32
#elif defined(linux)
#define MAX_DMA_SEGS 8
#else
#define MAX_DMA_SEGS 4
#endif


typedef struct {
	void *oshdmah; /* Opaque handle for OSL to store its information */
	u_int origsize; /* Size of the virtual packet */
	u_int nsegs;
	hnddma_seg_t segs[MAX_DMA_SEGS];
} hnddma_seg_map_t;


/* packet headroom necessary to accommodate the largest header in the system, (i.e TXOFF).
 * By doing, we avoid the need  to allocate an extra buffer for the header when bridging to WL.
 * There is a compile time check in wlc.c which ensure that this value is at least as big
 * as TXOFF. This value is used in dma_rxfill (hnddma.c).
 */

#if defined(BCM_RPC_NOCOPY) || defined(BCM_RCP_TXNOCOPY)
/* add 40 bytes to allow for extra RPC header and info  */
#define BCMEXTRAHDROOM 260
#else /* BCM_RPC_NOCOPY || BCM_RPC_TXNOCOPY */
#ifdef CTFMAP
#define BCMEXTRAHDROOM 208
#else /* CTFMAP */
#define BCMEXTRAHDROOM 204
#endif /* CTFMAP */
#endif /* BCM_RPC_NOCOPY || BCM_RPC_TXNOCOPY */

#ifdef BCMDBG

#ifndef BCMDBG_ERR
#define BCMDBG_ERR
#endif /* BCMDBG_ERR */

#define BCMDBG_ASSERT

#endif /* BCMDBG */

/* osl handle type forward declaration */
typedef struct osl_info osl_t;
typedef struct osl_dmainfo osldma_t;

// FBSD_TODO

#define	ISALIGNED(n, align)	((((uintptr_t)n) & ((align) - 1)) == 0)

#define	MALLOC(_osh, _len)	malloc((_len), M_TEMP, M_NOWAIT)
#define	MFREE(_osh, _ptr, _len)	free((_ptr), M_TEMP)
#define	ASSERT(...)		KASSERT(__VA_ARGS__, ("assertion failed"))
#define	MALLOCED(osh)		(0)

#define	OSL_DELAY(_usec)	DELAY(_usec)

#define	DISCARD_QUAL(_var, _type)	__DEQUALIFY(_type *, _var)

#define	PKTDATA(_osh, _pkt)		(_pkt)
#define	PKTLEN(_osh, _pkt)		(0)
#define	PKTFREE(_osh, _pkt, _send)	do {} while (0)
#define	PKTSETLEN(_osh, _pkt, _len)	do {} while (0)
#define	PKTSETNEXT(_osh, _pkt, _next)	do {} while (0)
#define	PKTNEXT(_osh, _pkt)		(NULL)
#define	PKTGET(_osh, _len, _send)	(NULL)
#define	PKTPULL(_osh, _pkt, _bytes)	(NULL)

#define R_REG(osh, r) (\
	sizeof(*(r)) == sizeof(uint8_t) ? osl_readb((osh), (volatile uint8_t*)(r)) : \
	sizeof(*(r)) == sizeof(uint16_t) ? osl_readw((osh), (volatile uint16_t*)(r)) : \
	osl_readl((osh), (volatile uint32_t*)(r)) \
)
#define W_REG(osh, r, v) do { \
	switch (sizeof(*(r))) { \
	case sizeof(uint8_t):	osl_writeb((osh), (volatile uint8_t*)(r), (uint8_t)(v)); break; \
	case sizeof(uint16_t):	osl_writew((osh), (volatile uint16_t*)(r), (uint16_t)(v)); break; \
	case sizeof(uint32_t):	osl_writel((osh), (volatile uint32_t*)(r), (uint32_t)(v)); break; \
	} \
} while (0)

extern uint8_t osl_readb(osl_t *osh, volatile uint8_t *r);
extern uint16_t osl_readw(osl_t *osh, volatile uint16_t *r);
extern uint32_t osl_readl(osl_t *osh, volatile uint32_t *r);
extern void osl_writeb(osl_t *osh, volatile uint8_t *r, uint8_t v);
extern void osl_writew(osl_t *osh, volatile uint16_t *r, uint16_t v);
extern void osl_writel(osl_t *osh, volatile uint32_t *r, uint32_t v);

/* map/unmap direction */
#define	DMA_TX	1	/* TX direction for DMA */
#define	DMA_RX	2	/* RX direction for DMA */

#define	DMA_MAP(osh, va, size, direction, p, dmah)	((dmaaddr_t){0, 0})
#define	DMA_UNMAP(osh, pa, size, direction, p, dmah)	do {} while (0)

#define	DMA_CONSISTENT_ALIGN	osl_dma_consistent_align()
#define	DMA_ALLOC_CONSISTENT(osh, size, align, tot, pap, dmah) \
	osl_dma_alloc_consistent((osh), (size), (align), (tot), (pap))
#define	DMA_FREE_CONSISTENT(osh, va, size, pa, dmah) \
	osl_dma_free_consistent((osh), (void*)(va), (size), (pa))

extern u_int osl_dma_consistent_align(void);
extern void *osl_dma_alloc_consistent(osl_t *osh, u_int size, uint16_t align, u_int *tot, dmaaddr_t *pap);
// extern void osl_sec_cma_baseaddr_memsize(osl_t *osh, dma_addr_t *cma_baseaddr, uint32_t *cma_memsize);
extern void osl_dma_free_consistent(osl_t *osh, void *va, u_int size, dmaaddr_t pa);

#define	R_SM(r)			*(r)
#define	W_SM(r, v)		(*(r) = (v))
#define	BZERO_SM(r, len)	memset((r), '\0', (len))

#define	BUS_SWAP32(_x)	(_x)

/* --------------------------------------------------------------------------
** Register manipulation macros.
*/

#define	SET_REG(osh, r, mask, val)	W_REG((osh), (r), ((R_REG((osh), r) & ~(mask)) | (val)))

#ifndef AND_REG
#define AND_REG(osh, r, v)		W_REG(osh, (r), R_REG(osh, r) & (v))
#endif   /* !AND_REG */

#ifndef OR_REG
#define OR_REG(osh, r, v)		W_REG(osh, (r), R_REG(osh, r) | (v))
#endif   /* !OR_REG */


/*
 * Spin at most 'us' microseconds while 'exp' is true.
 * Caller should explicitly test 'exp' when this completes
 * and take appropriate error action if 'exp' is still true.
 */
#ifdef MACOSX
#define SPINWAIT_POLL_PERIOD	20
#else
#define SPINWAIT_POLL_PERIOD	10
#endif

#define SPINWAIT(exp, us) { \
	u_int countdown = (us) + (SPINWAIT_POLL_PERIOD - 1); \
	while ((exp) && (countdown >= SPINWAIT_POLL_PERIOD)) {\
		OSL_DELAY(SPINWAIT_POLL_PERIOD); \
		countdown -= SPINWAIT_POLL_PERIOD; \
	} \
}

#ifdef BCMPKTPOOL
#define POOL_ENAB(pool)		((pool) && (pool)->inited)
#else /* BCMPKTPOOL */
#define POOL_ENAB(bus)		0
#endif /* BCMPKTPOOL */

#ifndef PKTPOOL_LEN_MAX
#define PKTPOOL_LEN_MAX		40
#endif /* PKTPOOL_LEN_MAX */
#define PKTPOOL_CB_MAX		3

struct pktpool;
typedef void (*pktpool_cb_t)(struct pktpool *pool, void *arg);
typedef struct {
	pktpool_cb_t cb;
	void *arg;
} pktpool_cbinfo_t;

typedef struct pktpool {
	bool inited;
	uint16_t r;
	uint16_t w;
	uint16_t len;
	uint16_t maxlen;
	uint16_t plen;
	bool istx;
	bool empty;
	uint8_t cbtoggle;
	uint8_t cbcnt;
	uint8_t ecbcnt;
	bool emptycb_disable;
	pktpool_cbinfo_t *availcb_excl;
	pktpool_cbinfo_t cbs[PKTPOOL_CB_MAX];
	pktpool_cbinfo_t ecbs[PKTPOOL_CB_MAX];
	void *q[PKTPOOL_LEN_MAX + 1];

#ifdef BCMDBG_POOL
	uint8_t dbg_cbcnt;
	pktpool_cbinfo_t dbg_cbs[PKTPOOL_CB_MAX];
	uint16_t dbg_qlen;
	pktpool_dbg_t dbg_q[PKTPOOL_LEN_MAX + 1];
#endif
} pktpool_t;

extern void* pktpool_get(pktpool_t *pktp);
extern void pktpool_emptycb_disable(pktpool_t *pktp, bool disable);
extern bool pktpool_emptycb_disabled(pktpool_t *pktp);


#endif	/* _siutils_h_ */
