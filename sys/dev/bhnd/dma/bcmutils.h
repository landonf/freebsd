/*
 * Misc useful os-independent macros and functions.
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
 * $Id: bcmutils.h 422014 2013-09-05 14:46:51Z $
 */

#ifndef	_bcmutils_h_
#define	_bcmutils_h_

#include "osl.h"

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

#endif	/* _bcmutils_h_ */
