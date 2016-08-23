/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 * Copyright (c) 2010 Broadcom Corporation
 * 
 * Portions of this file were derived from the aidmp.h header
 * distributed with Broadcom's initial brcm80211 Linux driver release, as
 * contributed to the Linux staging repository.
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
 * $FreeBSD$
 */

#ifndef	_BCMA_BCMA_PL367REG_H_
#define	_BCMA_BCMA_PL367REG_H_

#include "bcma_dmp.h"

/*
 * Register definitions for the PL-367 DMP OOB Router Core.
 * 
 * Documentation for the PL-367 is not publicly available. The exact
 * interpretation of these register definitions is unconfirmed, and may be
 * incorrect.
 */

/* Out-of-band Router registers */
#define	BCMA_OOB_BUSCONFIG	0x020
#define	BCMA_OOB_STATUSA	0x100
#define	BCMA_OOB_STATUSB	0x104
#define	BCMA_OOB_STATUSC	0x108
#define	BCMA_OOB_STATUSD	0x10c
#define	BCMA_OOB_ENABLEA0	0x200
#define	BCMA_OOB_ENABLEA1	0x204
#define	BCMA_OOB_ENABLEA2	0x208
#define	BCMA_OOB_ENABLEA3	0x20c
#define	BCMA_OOB_ENABLEB0	0x280
#define	BCMA_OOB_ENABLEB1	0x284
#define	BCMA_OOB_ENABLEB2	0x288
#define	BCMA_OOB_ENABLEB3	0x28c
#define	BCMA_OOB_ENABLEC0	0x300
#define	BCMA_OOB_ENABLEC1	0x304
#define	BCMA_OOB_ENABLEC2	0x308
#define	BCMA_OOB_ENABLEC3	0x30c
#define	BCMA_OOB_ENABLED0	0x380
#define	BCMA_OOB_ENABLED1	0x384
#define	BCMA_OOB_ENABLED2	0x388
#define	BCMA_OOB_ENABLED3	0x38c
#define	BCMA_OOB_ITCR		0xf00
#define	BCMA_OOB_ITIPOOBA	0xf10
#define	BCMA_OOB_ITIPOOBB	0xf14
#define	BCMA_OOB_ITIPOOBC	0xf18
#define	BCMA_OOB_ITIPOOBD	0xf1c
#define	BCMA_OOB_ITOPOOBA	0xf30
#define	BCMA_OOB_ITOPOOBB	0xf34
#define	BCMA_OOB_ITOPOOBC	0xf38
#define	BCMA_OOB_ITOPOOBD	0xf3c

#endif /* _BCMA_BCMA_PL367REG_H_ */
