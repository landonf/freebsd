/*
 * OS Abstraction Layer
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
 * $Id: osl.h 419594 2013-08-21 22:47:07Z $
 */

#ifndef _osl_h_
#define _osl_h_

/* osl handle type forward declaration */
typedef struct osl_info osl_t;
typedef struct osl_dmainfo osldma_t;

// FBSD_TODO
// #error "Unsupported OSL requested"

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

#define	DMA_MAP(osh, va, size, direction, p, dmah)	((dmaaddr_t)0)
#define	DMA_UNMAP(osh, pa, size, direction, p, dmah)	do {} while (0)

#define	DMA_CONSISTENT_ALIGN	osl_dma_consistent_align()
#define	DMA_ALLOC_CONSISTENT(osh, size, align, tot, pap, dmah) \
	osl_dma_alloc_consistent((osh), (size), (align), (tot), (pap))
#define	DMA_FREE_CONSISTENT(osh, va, size, pa, dmah) \
	osl_dma_free_consistent((osh), (void*)(va), (size), (pa))

extern u_int osl_dma_consistent_align(void);
extern void *osl_dma_alloc_consistent(osl_t *osh, u_int size, uint16_t align, u_int *tot, u_long *pap);
// extern void osl_sec_cma_baseaddr_memsize(osl_t *osh, dma_addr_t *cma_baseaddr, uint32_t *cma_memsize);
extern void osl_dma_free_consistent(osl_t *osh, void *va, u_int size, u_long pa);

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

#endif	/* _osl_h_ */
