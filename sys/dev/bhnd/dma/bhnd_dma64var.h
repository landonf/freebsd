/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2018 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2015 Broadcom Corporation. All Rights Reserved.
 * All rights reserved.
 *
 * This file was derived from the hnddma.c source distributed with the Asus
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
 * $Id: hnddma.c 497985 2014-08-21 10:43:51Z $
 *
 * $FreeBSD$
 */

#ifndef _BHND_DMA_BHND_DMA64VAR_H_
#define _BHND_DMA_BHND_DMA64VAR_H_

#include "bhnd_dmavar.h"

/* Prototypes for 64-bit routines */
extern const di_fcn_t dma64proc;

bool dma64_alloc(dma_info_t *di, u_int direction);
bool dma64_rxidle(dma_info_t *di);

void dma64_dd_upd(dma_info_t *di, dma64dd_t *ddring, dmaaddr_t pa, u_int outidx, uint32_t *flags,
	uint32_t bufcount);

void *dma64_getnextrxp(dma_info_t *di, bool forceall);


#endif /* _BHND_DMA_BHND_DMA64VAR_H_ */
