/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2018 Landon Fuller <landonf@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _BHND_DMA_BHND_DMA_H_
#define _BHND_DMA_BHND_DMA_H_

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cdefs.h>
#include <sys/malloc.h>

#include <machine/bus.h>

/* forward declarations */
typedef struct bhnd_dma			bhnd_dma;
typedef struct bhnd_dma_chan		bhnd_dma_chan;
typedef struct bhnd_dma_chan_attr	bhnd_dma_chan_attr;

/**
 * DMA quirks.
 */
enum bhnd_dma_quirk {
	/**
	 * DMA engine support for DmaExtendedAddrChanges is broken and should
	 * not be used.
	 */
	BHND_DMA_QUIRK_BROKEN_ADDREXT	= (1<<0),
};

/**
 * DMA channel direction.
 */
typedef enum bhnd_dma_direction {
	BHND_DMA_TX	= 0,	/**< transmit */
	BHND_DMA_RX	= 1,	/**< receive */
} bhnd_dma_direction;


int		 bhnd_dma_new(bhnd_dma **dma, device_t owner,
		     struct resource *regs, bus_size_t offset,
		     u_int num_txc, u_int num_rxc, uint32_t quirks);

void		 bhnd_dma_free(struct bhnd_dma *dma);



int			 bhnd_dma_chan_attr_new(bhnd_dma_chan_attr **attr,
			     bhnd_dma *dma, size_t chan_idx,
			     bhnd_dma_direction direction);
void			 bhnd_dma_chan_attr_free(bhnd_dma_chan_attr *attr);

int			 bhnd_dma_chan_attr_set_name(bhnd_dma_chan_attr *attr,
			     const char *fmt, ...) __printflike(2, 3);
const char		*bhnd_dma_chan_attr_get_name(bhnd_dma_chan_attr *attr);

size_t			 bhnd_dma_chan_attr_get_max_ndesc(
			     bhnd_dma_chan_attr *attr);

int			 bhnd_dma_chan_attr_set_ndesc(bhnd_dma_chan_attr *attr,
			     size_t ndesc);
size_t			 bhnd_dma_chan_attr_get_ndesc(bhnd_dma_chan_attr *attr);

int			 bhnd_dma_chan_attr_set_quirks(bhnd_dma_chan_attr *attr,
			     uint32_t quirks);
int			 bhnd_dma_chan_attr_get_quirks(
			     bhnd_dma_chan_attr *attr);

int			 bhnd_dma_chan_new(bhnd_dma_chan **chan, size_t channel, bhnd_dma_direction direction, const char *name, ...) __printflike(4, 5);




#endif /* _BHND_DMA_BHND_DMA_H_ */
