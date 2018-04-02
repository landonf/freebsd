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

#include <sys/types.h>
#include <sys/endian.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/sx.h>

#include <machine/stdarg.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_debug.h>
#include <dev/bhnd/bhnd_ids.h> /* XXX REMOVE: used for hard-coded DMA translation constants */

#include "bhnd_dmavar.h"
#include "bhnd_dmareg.h"
#include "bhnd_dma32reg.h"
#include "bhnd_dma64reg.h"

static int		 bhnd_dma_chan_init(bhnd_dma *dma, bhnd_dma_chan *chan,
			     bhnd_dma_direction direction, size_t chan_num);
static void		 bhnd_dma_chan_fini(struct bhnd_dma_chan *chan);

MALLOC_DEFINE(M_BHND_DMA, "bhnd_dma", "bhnd DMA engine state");

/**
 * Allocate, initialize, and return a new DMA controller instance, mapping
 * the DMA register block from @p regs at @p offset.
 * 
 * On success, the caller is responsible for releasing the DMA controller
 * instance via bhnd_dma_free().
 * 
 * @param[out]	dma	On success, a pointer to the newly allocated DMA
 *			controller instance.
 * @param	owner	The bhnd(4) device containing the DMA registers mapped
 *			by @p regs.
 * @param	regs	The active/mapped SYS_RES_MEMORY resource to be used
 *			when accessing the DMA controller registers. This
 *			resource must remain active and mapped for the lifetime
 *			of the returned DMA controller instance.
 * @param	offset	The offset of the DMA register block within @p regs.
 * @param	num_txc	The number of TX channels mapped at @p offset.
 * @param	num_rxc	The number of RX channels mapped at @p offset.
 * @param	quirks	DMA controller quirks (see bhnd_dma_quirk).
 * 
 * @retval 0		success
 * @retval ENODEV	if the agent/config space for @p child is unavailable.
 * @retval EINVAL	if @p num_txc and @p num_rxc are both 0.
 * @retval ENXIO	if @p offset exceeds the size of @p regs.
 * @retval ENXIO	if @p num_txc or @p num_rxc mapped at @p offset would
 *			exceed the total size of @p regs.
 * @retval non-zero	if initializing the DMA controller instance otherwise
 *			fails, a regular unix error code will be returned.
 */
int
bhnd_dma_new(bhnd_dma **dma, device_t owner, struct resource *regs,
    bus_size_t offset, size_t num_txc, size_t num_rxc, uint32_t quirks)
{
	bhnd_dma		*d;
	bhnd_dma_regfmt		 regfmt;
	bus_space_tag_t	 	 bst;
	bus_space_handle_t	 bsh;
	bus_size_t		 regs_size, cpair_size;
	bus_size_t		 probe_offset;
	rman_res_t		 max_cpair, r_avail_size, r_total_size;
	size_t			 num_cpair;
	uint32_t		 st0_cd_mask, st1_ad_mask;
	uint16_t		 iost;
	u_int			 max_ndesc;
	bool			 addrext;
	int			 error;

	KASSERT(rman_get_flags(regs) & RF_ACTIVE, ("resource must be active"));
	KASSERT(!(rman_get_flags(regs) & RF_UNMAPPED),
	    ("resource must be mapped"));

	/* Determine the channel pair count; there must be at least one RX or
	 * TX channel */
	if ((num_cpair = MAX(num_txc, num_rxc)) == 0)
		return (EINVAL);

	/* Determine the core's DMA register layout and fetch format-specific
	 * constants */
	if ((error = bhnd_read_iost(owner, &iost))) {
		BHND_ERROR_DEV(owner, "failed to read I/O status flags: %d",
		    error);
		return (error);
	}

	if (iost & BHND_IOST_DMA64) {
		regfmt = BHND_DMA_REGFMT64;
	} else {
		regfmt = BHND_DMA_REGFMT32;
	}

	cpair_size = BHND_DMA_REGF(regfmt, CHAN_PAIR_SIZE);

	/*
	 * Determine the maximum number of DMA channel pairs that can be mapped
	 * at the given resource offset, and validate our DMA channel pair
	 * count.
	 * 
	 * We'll be coercing this size to a bus_size_t, so limit to
	 * BUS_SPACE_MAXSIZE
	 */
	r_total_size = MIN(BUS_SPACE_MAXSIZE, rman_get_size(regs));
	if (offset > r_total_size) {
		BHND_ERROR_DEV(owner, "invalid register offset %#jx for "
		    "resource of length %#jx", (uintmax_t)offset,
		    (uintmax_t)r_total_size);
		return (EINVAL);
	}

	r_avail_size = r_total_size - offset;

	/* Validate the DMA channel pair count */
	max_cpair = MIN(SIZE_MAX, r_avail_size / cpair_size);
	if (num_cpair > max_cpair) {
		BHND_ERROR_DEV(owner, "invalid channel count %zu for resource "
		    "of length %#jx", num_cpair,  (uintmax_t)r_total_size);
		return (EINVAL);
	}

	/* Determine the total size of the DMA register block. This cannot
	 * overflow if num_cpair < max_cpair */
	regs_size = num_cpair * BHND_DMA_REGF(regfmt, CHAN_PAIR_SIZE);
	KASSERT(regs_size > 0, ("0 length register block with %zu channel "
	    "pairs and %#jx channel pair size", num_cpair,
	    (uintmax_t)cpair_size));

	/* Request a subregion mapping our DMA register block */
	bst = rman_get_bustag(regs);
	error = bus_space_subregion(bst, rman_get_bushandle(regs), offset,
	    regs_size, &bsh);
	if (error) {
		BHND_ERROR_DEV(owner, "error requesting DMA register "
		    "subregion: %d", error);
		return (error);
	}

	/* Find a valid channel to be used while probing DMA controller
	 * capabilities */
	KASSERT(num_txc > 0 || num_rxc > 0, ("no channels"));
	if (num_txc > 0) {
		probe_offset = BHND_DMA_CHAN_OFFSET(regfmt, BHND_DMA_TX, 0);
	} else if (num_rxc > 0) {
		probe_offset = BHND_DMA_CHAN_OFFSET(regfmt, BHND_DMA_RX, 0);
	}

	/* Do we have addrext support? */
	addrext = false;
	if (!(quirks & BHND_DMA_QUIRK_BROKEN_ADDREXT)) {
		bus_size_t      ctrl_reg;
		uint32_t        ae, ae_mask;

		ae_mask = BHND_DMA_REGF(regfmt, CTRL_AE_MASK);
		ctrl_reg = probe_offset + BHND_DMA_REGF(regfmt, CTRL);
		bus_space_write_4(bst, bsh, ctrl_reg, ae_mask);
		ae = bus_space_read_4(bst, bsh, ctrl_reg);

		if (ae == ae_mask)
			addrext = true;
	}

	/* DMA64 should always support addrext */
	if (!addrext && regfmt == BHND_DMA_REGFMT64) {
		BHND_ERROR_DEV(owner, "missing DMA64 AE support");
		return (ENXIO);
	}

	/* 
	 * Determine the descriptor address mask and the maximum descriptor
	 * count
	 */
	switch (regfmt) {
	case BHND_DMA_REGFMT64: {
		bus_size_t	addrlow_reg, ptr_reg;
		uint32_t	desc_mask;

		addrlow_reg = probe_offset + BHND_D64_ADDRLOW;
		ptr_reg = probe_offset + BHND_D64_PTR;

		/* Should be 0x1fff before 4360B0, 0xffff start from 4360B0 */
		bus_space_write_4(bst, bsh, addrlow_reg, UINT32_MAX);
		desc_mask = bus_space_read_4(bst, bsh, addrlow_reg);

		if (desc_mask & 0xfff) {
			desc_mask = bus_space_read_4(bst, bsh, ptr_reg);
			desc_mask |= 0xf;
		} else {
			_Static_assert(BHND_D64_STATUS0_CD_MIN_MASK ==
			    BHND_D64_STATUS1_AD_MIN_MASK, "CD/AD masks must be "
			    "identical");

			desc_mask = BHND_D64_STATUS0_CD_MIN_MASK;
		}

		st0_cd_mask = desc_mask;
		st1_ad_mask = desc_mask;
		max_ndesc = (desc_mask + 1) / BHND_D64_DESC_SIZE;

		break;
	}

	case BHND_DMA_REGFMT32:
		st0_cd_mask = BHND_D32_STATUS_CD_MASK;
		st1_ad_mask = BHND_D32_STATUS_AD_MASK;
		max_ndesc = ((BHND_D32_STATUS_CD_MASK >>
		    BHND_D32_STATUS_CD_SHIFT) + 1) / BHND_D32_DESC_SIZE;

		break;
	}

	if (!powerof2(max_ndesc)) {
		/* Round down to nearest power of two; should never happen on
		 * supported hardware */
		BHND_ERROR_DEV(owner, "DMA max_ndesc %u is not a power of 2",
		    max_ndesc);
		max_ndesc = (1 << (fls(max_ndesc) - 1));
	}

	BHND_TRACE_DEV(owner, "DMA: st0_cd_mask=%#08x, st1_ad_mask=%#08x, "
	    " max_ndesc=%u", st0_cd_mask, st1_ad_mask, max_ndesc);

	/* Allocate and populate our DMA controller instance */
	d = malloc(sizeof(*d), M_BHND_DMA, M_ZERO | M_WAITOK);
	d->owner = owner;
	d->regfmt = regfmt;
	d->quirks = quirks;

	d->regs_bst = bst;
	d->regs_bsh = bsh;
	d->regs_size = regs_size;

	// XXX hacked in for legacy code
	d->regs_virt = (uintptr_t)rman_get_virtual(regs);
	d->regs_virt += offset;

	d->addrext = addrext;
	d->st0_cd_mask = st0_cd_mask;
	d->st1_ad_mask = st1_ad_mask;
	d->max_ndesc = max_ndesc;

	d->num_tx_chan = num_txc;
	d->num_rx_chan = num_rxc;

	sx_init(&d->chan_lock, "bhnd_dma_chan");

	if (d->num_tx_chan > 0) {
		d->tx_chan = malloc(sizeof(*d->tx_chan) * d->num_tx_chan,
		    M_BHND_DMA, M_ZERO | M_WAITOK);
	} else {
		d->tx_chan = NULL;
	}

	if (d->num_rx_chan > 0) {
		d->rx_chan = malloc(sizeof(*d->rx_chan) * d->num_rx_chan,
		    M_BHND_DMA, M_ZERO | M_WAITOK);
	} else {
		d->rx_chan = NULL;
	}

	for (size_t i = 0; i < d->num_tx_chan; i++) {
		error = bhnd_dma_chan_init(d, &d->tx_chan[i], BHND_DMA_TX, i);
		if (error)
			goto failed;
	}

	for (size_t i = 0; i < d->num_rx_chan; i++) {
		error = bhnd_dma_chan_init(d, &d->rx_chan[i], BHND_DMA_RX, i);
		if (error)
			goto failed;
	}

	*dma = d;
	return (0);

failed:
	bhnd_dma_free(d);
	return (error);
}

/**
 * Return the name for a given DMA direction.
 * 
 * @param direction	The DMA direction to look up.
 */
const char *
bhnd_dma_direction_name(bhnd_dma_direction direction)
{
	switch (direction) {
	case BHND_DMA_TX:
		return ("tx");
	case BHND_DMA_RX:
		return ("rx");
	}

	return ("unknown");
}

/**
 * Free a DMA controller instance and all associated resources.
 * 
 * @param dma The controller to be deallocated.
 */
void
bhnd_dma_free(struct bhnd_dma *dma)
{
	sx_xlock(&dma->chan_lock);

	for (size_t i = 0; i < dma->num_tx_chan; i++)
		bhnd_dma_chan_fini(&dma->tx_chan[i]);

	free(dma->tx_chan, M_BHND_DMA);

	for (size_t i = 0; i < dma->num_rx_chan; i++)
		bhnd_dma_chan_fini(&dma->rx_chan[i]);

	free(dma->rx_chan, M_BHND_DMA);

	sx_xunlock(&dma->chan_lock);
	sx_destroy(&dma->chan_lock);

	free(dma, M_BHND_DMA);
}

/**
 * Initialize a new DMA channel instance.
 * 
 * @param dma		The DMA engine to which @p chan is assigned.
 * @param chan		Channel instance to be initialized.
 * @param direction	Channel direction.
 * @param chan_num	Channel number.
 */
static int
bhnd_dma_chan_init(bhnd_dma *dma, bhnd_dma_chan *chan,
    bhnd_dma_direction direction, size_t chan_num)
{
	bus_size_t	offset, size;
	int		error;

	chan->dma = dma;
	chan->direction = direction;
	chan->num = chan_num;
	chan->enabled = false;
	chan->ndesc = 0;

	/* Request a subregion mapping our per-channel register block */
	size = BHND_DMA_REGF(dma->regfmt, CHAN_SIZE);
	offset = BHND_DMA_CHAN_OFFSET(dma->regfmt, chan->direction, chan->num);

	error = bus_space_subregion(dma->regs_bst, dma->regs_bsh, offset, size,
	    &chan->bsh);
	if (error) {
		BHND_DMA_ERROR_NEW(dma, "error requesting DMA register"
		    "subregion for DMA channel %s%zu: %d",
		    bhnd_dma_direction_name(chan->direction), chan->num, error);
		return (error);
	}

	return (0);
}

/**
 * Free all resources associated with @p chan. The channel must be disabled.
 * 
 * @param chan	The DMA channel instance to be freed.
 */
static void
bhnd_dma_chan_fini(struct bhnd_dma_chan *chan)
{
	sx_assert(&chan->dma->chan_lock, SA_XLOCKED);

	if (chan->enabled) {
		panic("free of active DMA channel %s%zu",
		    bhnd_dma_direction_name(chan->direction), chan->num);
	}
}

/**
 * Return the DMA engine's channel instance for the given channel @p direction
 * and @p chan_num.
 * 
 * @param dma		The DMA engine from which to request a channel instance.
 * @param direction	The requested channel direction.
 * @param chan_num	The requested channel number.
 * 
 * @retval non-NULL	success
 * @retval NULL		if no such channel exists.
 */
bhnd_dma_chan *
bhnd_dma_get_chan(bhnd_dma *dma, bhnd_dma_direction direction, size_t chan_num)
{
	bhnd_dma_chan	*channels;
	size_t		 num_channels;

	switch (direction) {
	case BHND_DMA_TX:
		channels = dma->tx_chan;
		num_channels = dma->num_tx_chan;
		break;
	case BHND_DMA_RX:
		channels = dma->rx_chan;
		num_channels = dma->num_rx_chan;
		break;
	default:
		BHND_DMA_ERROR_NEW(dma, "unknown channel direction: %d",
		    direction);
		return (NULL);
	}

	if (chan_num >= num_channels) {
		BHND_DMA_ERROR_NEW(dma, "invalid channel number: %s%zu",
		    bhnd_dma_direction_name(direction), chan_num);
		return (NULL);
	}

	KASSERT(channels != NULL, ("missing channel state"));
	return (&channels[chan_num]);
}

int
bhnd_dma_chan_enable(bhnd_dma_chan *chan)
{
	void			*regs;
	bhnd_dma_direction	 dir;
	bus_size_t		 chan_offset;

	static u_int msg_level = BHND_TRACE_LEVEL; // XXX override

	sx_xlock(&chan->dma->chan_lock);

	if (chan->enabled) {
		sx_xunlock(&chan->dma->chan_lock);
		return (EBUSY);
	}

	KASSERT(chan->di == NULL, ("active DMA state"));

	if (chan->ndesc == 0) {
		sx_xunlock(&chan->dma->chan_lock);
		return (ENXIO);
	}

	dir = chan->direction;
	chan_offset = BHND_DMA_CHAN_OFFSET(chan->dma->regfmt, dir, chan->num);
	regs = (void *)(chan->dma->regs_virt + chan_offset);

	chan->di = dma_attach(chan, "<TODO>",
	    ((dir == BHND_DMA_TX) ? regs : NULL),
	    ((dir == BHND_DMA_RX) ? regs : NULL),
	    ((dir == BHND_DMA_TX) ? chan->ndesc : 0),
	    ((dir == BHND_DMA_RX) ? chan->ndesc : 0),
	    ((dir == BHND_DMA_RX) ? /* TODO: rxbufsize */ 0 : 0),
	    ((dir == BHND_DMA_RX) ? /* TODO: rxextheadroom */ 0 : 0),
	    ((dir == BHND_DMA_RX) ? /* TODO: nrxpost */ 0 : 0),
	    ((dir == BHND_DMA_RX) ? /* TODO: rxoffset */ 0 : 0),
	    &msg_level /* TODO: msg_level */
	);

	if (chan->di == NULL) {
		sx_xunlock(&chan->dma->chan_lock);
		return (ENXIO);
	}

	chan->enabled = true;
	sx_xunlock(&chan->dma->chan_lock);
	return (0);
}

void
bhnd_dma_chan_disable(bhnd_dma_chan *chan)
{
	sx_xlock(&chan->dma->chan_lock);

	if (!chan->enabled) {
		sx_xunlock(&chan->dma->chan_lock);
		return;
	}

	KASSERT(chan->di != NULL, ("no active DMA state"));

	dma_detach(chan->di);
	chan->enabled = false;
	chan->di = NULL;

	sx_xunlock(&chan->dma->chan_lock);
}

/**
 * Set the number of DMA descriptors to be used by @p chan.
 * 
 * @param chan	The DMA channel to modify.
 * @param ndesc	The number of DMA descriptors to be allocated for @p chan.
 * 
 * @retval 0		success
 * @retval EBUSY	if @p chan has already been enabled (see
 *			bhnd_dma_chan_enable).
 * @retval EINVAL	if @p ndesc is not a power of two.
 * @retval EINVAL	if @p ndesc exceeds the maximum number of descriptors
 *			supported by @p chan (see bhnd_dma_chan_get_max_ndesc).
 */
int
bhnd_dma_chan_set_ndesc(bhnd_dma_chan *chan, u_int ndesc)
{
	sx_xlock(&chan->dma->chan_lock);

	if (chan->enabled) {
		sx_xunlock(&chan->dma->chan_lock);
		return (EBUSY);
	}

	if (!powerof2(ndesc) || ndesc > chan->dma->max_ndesc) {
		sx_xunlock(&chan->dma->chan_lock);
		return (EINVAL);
	}

	chan->ndesc = ndesc;
	sx_xunlock(&chan->dma->chan_lock);

	return (0);
}

/**
 * Return the number of DMA descriptors used by @p chan.
 * 
 * @param chan	The DMA channel to query.
 */
u_int
bhnd_dma_chan_get_ndesc(bhnd_dma_chan *chan)
{
	u_int ndesc;

	sx_slock(&chan->dma->chan_lock);
	ndesc = chan->ndesc;
	sx_sunlock(&chan->dma->chan_lock);

	return (ndesc);
}

/**
 * Return the maximum number of DMA descriptors supported by @p chan.
 * 
 * @param chan	The DMA channel to query.
 */
u_int
bhnd_dma_chan_get_max_ndesc(bhnd_dma_chan *chan)
{
	u_int ndesc;

	sx_slock(&chan->dma->chan_lock);
	ndesc = chan->dma->max_ndesc;
	sx_sunlock(&chan->dma->chan_lock);

	return (ndesc);
}

/*************************************
 * XXX LEGACY OSL DEFINITIONS FOLLOW *
 *************************************/

/* XXX TODO: Hide behind our DMA callbacks */
#include "bhnd_dma32var.h"
#include "bhnd_dma64var.h"

#include "siutils.h"

uint8_t
osl_readb(osl_t *osh, volatile uint8_t *r)
{
	BHND_DMA_UNIMPL();
}

uint16_t
osl_readw(osl_t *osh, volatile uint16_t *r)
{
	BHND_DMA_UNIMPL();
}

uint32_t
osl_readl(osl_t *osh, volatile uint32_t *r)
{
	BHND_DMA_UNIMPL();
}
void
osl_writeb(osl_t *osh, volatile uint8_t *r, uint8_t v)
{
	BHND_DMA_UNIMPL();
}
void
osl_writew(osl_t *osh, volatile uint16_t *r, uint16_t v)
{
	BHND_DMA_UNIMPL();
}
void
osl_writel(osl_t *osh, volatile uint32_t *r, uint32_t v)
{
	BHND_DMA_UNIMPL();
}

u_int
osl_dma_consistent_align(void)
{
	BHND_DMA_UNIMPL();
}

void *
osl_dma_alloc_consistent(osl_t *osh, u_int size, uint16_t align, u_int *tot,
    dmaaddr_t *pap)
{
	BHND_DMA_UNIMPL();
}

void
osl_dma_free_consistent(osl_t *osh, void *va, u_int size, dmaaddr_t pa)
{
	BHND_DMA_UNIMPL();
}

/**********************************
 * XXX LEGACY DMA DEFINITIONS FOLLOW *
 **********************************/

/*
 * Generic Broadcom Home Networking Division (HND) DMA engine SW interface
 * This supports the following chips: BCM42xx, 44xx, 47xx .
 */

/* Common prototypes */
static bool _dma_descriptor_align(dma_info_t *di);
static bool _dma_alloc(dma_info_t *di, u_int direction);

static uint8_t dma_align_sizetobits(u_int size);

/* Default to the global BHND_LOGGING log level */
static u_int default_msg_level = BHND_LOGGING;

static bool
_dma_alloc(dma_info_t *di, u_int direction)
{
	if (di->dma64) {
		return dma64_alloc(di, direction);
	} else {
		return dma32_alloc(di, direction);
	}
}

/** !! may be called with core in reset */
void
_dma_detach(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	/* shouldn't be here if descriptors are unreclaimed */
	ASSERT(di->txin == di->txout);
	ASSERT(di->rxin == di->rxout);

	/* free dma descriptor rings */
	if (di->dma64) {
		if (di->txd64)
			DMA_FREE_CONSISTENT(di->osh, ((int8_t *)(uintptr_t)di->txd64 - di->txdalign),
			                    di->txdalloc, (di->txdpaorig), di->tx_dmah);
		if (di->rxd64)
			DMA_FREE_CONSISTENT(di->osh, ((int8_t *)(uintptr_t)di->rxd64 - di->rxdalign),
			                    di->rxdalloc, (di->rxdpaorig), di->rx_dmah);
	} else {
		if (di->txd32)
			DMA_FREE_CONSISTENT(di->osh, ((int8_t *)(uintptr_t)di->txd32 - di->txdalign),
			                    di->txdalloc, (di->txdpaorig), di->tx_dmah);
		if (di->rxd32)
			DMA_FREE_CONSISTENT(di->osh, ((int8_t *)(uintptr_t)di->rxd32 - di->rxdalign),
			                    di->rxdalloc, (di->rxdpaorig), di->rx_dmah);
	}

	/* free packet pointer vectors */
	if (di->txp)
		MFREE(di->osh, (void *)di->txp, (di->ntxd * sizeof(void *)));
	if (di->rxp)
		MFREE(di->osh, (void *)di->rxp, (di->nrxd * sizeof(void *)));

	/* free tx packet DMA handles */
	if (di->txp_dmah)
		MFREE(di->osh, (void *)di->txp_dmah, di->ntxd * sizeof(hnddma_seg_map_t));

	/* free rx packet DMA handles */
	if (di->rxp_dmah)
		MFREE(di->osh, (void *)di->rxp_dmah, di->nrxd * sizeof(hnddma_seg_map_t));

	/* free our private info structure */
	MFREE(di->osh, (void *)di, sizeof(dma_info_t));

}

hnddma_t *
dma_attach(bhnd_dma_chan *chan, const char *name, volatile void *dmaregstx,
    volatile void *dmaregsrx, u_int ntxd, u_int nrxd, u_int rxbufsize,
    int rxextheadroom, u_int nrxpost, u_int rxoffset, u_int *msg_level)
{
	dma_info_t	*di;
	uint32_t	 ctrl;
	u_int		 size;

	/* allocate private info structure */
	di = malloc(sizeof(dma_info_t), M_BHND_DMA, M_ZERO | M_WAITOK);

	/* XXX TODO */
	di->osh = NULL;
	di->sih = NULL;

	di->chan = chan;

	di->msg_level = msg_level ? msg_level : &default_msg_level;

	/* make a private copy of our callers name */
	strncpy(di->name, name, MAXNAMEL);
	di->name[MAXNAMEL-1] = '\0';

	di->dma64 = (chan->dma->regfmt == BHND_DMA_REGFMT64);

	/* check arguments */
	ASSERT(powerof2(ntxd));
	ASSERT(powerof2(nrxd));

	if (nrxd == 0)
		ASSERT(dmaregsrx == NULL);
	if (ntxd == 0)
		ASSERT(dmaregstx == NULL);

	/* init dma reg pointer */
	if (di->dma64) {
		if (!BHND_DMA64_SUPPORT(di)) {
			BHND_DMA_ERROR(di, "BHND_DMA64 support disabled");
			goto fail;
		}

		di->d64txregs = (dma64regs_t *)dmaregstx;
		di->d64rxregs = (dma64regs_t *)dmaregsrx;
		di->hnddma.di_fn = (const di_fcn_t *)&dma64proc;
	} else {
		if (!BHND_DMA32_SUPPORT(di)) {
			BHND_DMA_ERROR(di, "BHND_DMA32 support disabled");
			goto fail;
		}

		ASSERT(ntxd <= D32MAXDD);
		ASSERT(nrxd <= D32MAXDD);

		di->d32txregs = (dma32regs_t *)dmaregstx;
		di->d32rxregs = (dma32regs_t *)dmaregsrx;
		di->hnddma.di_fn = (const di_fcn_t *)&dma32proc;
	}

	/* Default flags (which can be changed by the driver calling dma_ctrlflags
	 * before enable): For backwards compatibility both Rx Overflow Continue
	 * and Parity are DISABLED.
	 * supports it.
	 */
	di->hnddma.di_fn->ctrlflags(&di->hnddma, DMA_CTRL_ROC | DMA_CTRL_PEN, 0);

	/* save tunables */
	di->ntxd = (uint16_t)ntxd;
	di->nrxd = (uint16_t)nrxd;

	/* the actual dma size doesn't include the extra headroom */
	di->rxextrahdrroom = (rxextheadroom == -1) ? BCMEXTRAHDROOM : rxextheadroom;
	if (rxbufsize > BCMEXTRAHDROOM)
		di->rxbufsize = (uint16_t)(rxbufsize - di->rxextrahdrroom);
	else
		di->rxbufsize = (uint16_t)rxbufsize;

	di->nrxpost = (uint16_t)nrxpost;
	di->rxoffset = (uint8_t)rxoffset;

	/*
	 * Get the default values (POR) of the burstlen. This can be overridden
	 * by the modules if this has to be different. Otherwise this value will
	 * be used to program the control register after the reset or during
	 * the init.
	 */
	ctrl = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, CTRL));

	switch (chan->direction) {
	case BHND_DMA_RX:
		di->rxburstlen = BHND_DMA_GET_BITS(chan, ctrl, RC_BL);
		di->rxprefetchctl = BHND_DMA_GET_BITS(chan, ctrl, RC_PC);
		di->rxprefetchthresh = BHND_DMA_GET_BITS(chan, ctrl, RC_PT);
		break;

	case BHND_DMA_TX:
		di->txburstlen = BHND_DMA_GET_BITS(chan, ctrl, XC_BL);
		di->txmultioutstdrd = BHND_DMA_GET_BITS(chan, ctrl, XC_MR);
		di->txprefetchctl = BHND_DMA_GET_BITS(chan, ctrl, XC_PC);
		di->txprefetchthresh = BHND_DMA_GET_BITS(chan, ctrl, XC_PT);
		break;
	}

	/* XXX TODO fetch DMA translation */
#if 0
	/*
	 * figure out the DMA physical address offset for dd and data
	 *     PCI/PCIE: they map silicon backplace address to zero based memory, need offset
	 *     Other bus: use zero
	 *     SI_BUS BIGENDIAN kludge: use sdram swapped region for data buffer, not descriptor
	 */
	di->ddoffsetlow = 0;
	di->dataoffsetlow = 0;
	/* for pci bus, add offset */
	if (sih->bustype == PCI_BUS) {
		if ((sih->buscoretype == BHND_COREID_PCIE ||
		     sih->buscoretype == BHND_COREID_PCIE2) &&
		    di->dma64) {
			/* pcie with DMA64 */
			di->ddoffsetlow = 0;
			di->ddoffsethigh = SI_PCIE_DMA_H32;
		} else {
			/* pci(DMA32/DMA64) or pcie with DMA32 */
			if ((sih->chip == BHND_CHIPID_BCM4322) ||
			    (sih->chip == BHND_CHIPID_BCM4342) ||
			    (sih->chip == BHND_CHIPID_BCM43221) ||
			    (sih->chip == BHND_CHIPID_BCM43231) ||
			    (sih->chip == BHND_CHIPID_BCM43111) ||
			    (sih->chip == BHND_CHIPID_BCM43112) ||
			    (sih->chip == BHND_CHIPID_BCM43222))
				di->ddoffsetlow = SI_PCI_DMA2;
			else
				di->ddoffsetlow = SI_PCI_DMA;

			di->ddoffsethigh = 0;
		}
		di->dataoffsetlow =  di->ddoffsetlow;
		di->dataoffsethigh =  di->ddoffsethigh;
	}
#endif

#if defined(__mips__) && defined(IL_BIGENDIAN)
	di->dataoffsetlow = di->dataoffsetlow + SI_SDRAM_SWAPPED;
#endif /* defined(__mips__) && defined(IL_BIGENDIAN) */

	/* does the descriptors need to be aligned and if yes, on 4K/8K or not */
	di->aligndesc_4k = _dma_descriptor_align(di);
	if (di->aligndesc_4k) {
		if (di->dma64) {
			di->dmadesc_align = D64RINGALIGN_BITS;
			if ((ntxd < D64MAXDD / 2) && (nrxd < D64MAXDD / 2)) {
				/* for smaller dd table, HW relax the alignment requirement */
				di->dmadesc_align = D64RINGALIGN_BITS  - 1;
			}
		} else
			di->dmadesc_align = D32RINGALIGN_BITS;
	} else {
		/* The start address of descriptor table should be algined to cache line size,
		 * or other structure may share a cache line with it, which can lead to memory
		 * overlapping due to cache write-back operation. In the case of MIPS 74k, the
		 * cache line size is 32 bytes.
		 */
#ifdef __mips__
		di->dmadesc_align = 5;	/* 32 byte alignment */
#else
		di->dmadesc_align = 4;	/* 16 byte alignment */
#endif
	}

	BHND_DMA_TRACE(di, "DMA descriptor align_needed %d, align %d",
	    di->aligndesc_4k, di->dmadesc_align);

	/* allocate tx packet pointer vector */
	if (ntxd) {
		size = ntxd * sizeof(void *);
		if ((di->txp = MALLOC(osh, size)) == NULL) {
			BHND_DMA_ERROR(di, "out of tx memory");
			goto fail;
		}
		bzero(di->txp, size);
	}

	/* allocate rx packet pointer vector */
	if (nrxd) {
		size = nrxd * sizeof(void *);
		if ((di->rxp = MALLOC(osh, size)) == NULL) {
			BHND_DMA_ERROR(di, "out of rx memory");
			goto fail;
		}
		bzero(di->rxp, size);
	}

	/* allocate transmit descriptor ring, only need ntxd descriptors but it must be aligned */
	if (ntxd) {
		if (!_dma_alloc(di, DMA_TX))
			goto fail;
	}

	/* allocate receive descriptor ring, only need nrxd descriptors but it must be aligned */
	if (nrxd) {
		if (!_dma_alloc(di, DMA_RX))
			goto fail;
	}

	if ((di->ddoffsetlow != 0) && !chan->dma->addrext) {
		if (PHYSADDRLO(di->txdpa) > SI_PCI_DMA_SZ) {
			BHND_DMA_ERROR(di, "txdpa %#x: addrext not supported",
			           (uint32_t)PHYSADDRLO(di->txdpa));
			goto fail;
		}
		if (PHYSADDRLO(di->rxdpa) > SI_PCI_DMA_SZ) {
			BHND_DMA_ERROR(di, "rxdpa %#x: addrext not supported",
			           (uint32_t)PHYSADDRLO(di->rxdpa));
			goto fail;
		}
	}

	BHND_DMA_TRACE(di, "ddoffsetlow %#x ddoffsethigh %#x dataoffsetlow %#x dataoffsethigh "
	           "%#x addrext %d", di->ddoffsetlow, di->ddoffsethigh, di->dataoffsetlow,
	           di->dataoffsethigh, chan->dma->addrext);

	/* allocate DMA mapping vectors */
	if (DMASGLIST_ENAB) {
		if (ntxd) {
			size = ntxd * sizeof(hnddma_seg_map_t);
			if ((di->txp_dmah = (hnddma_seg_map_t *)MALLOC(osh, size)) == NULL)
				goto fail;
			bzero(di->txp_dmah, size);
		}

		if (nrxd) {
			size = nrxd * sizeof(hnddma_seg_map_t);
			if ((di->rxp_dmah = (hnddma_seg_map_t *)MALLOC(osh, size)) == NULL)
				goto fail;
			bzero(di->rxp_dmah, size);
		}
	}

	return ((hnddma_t *)di);

fail:
	_dma_detach(di);
	return (NULL);
}


static bool
_dma_descriptor_align(dma_info_t *di)
{
	bhnd_dma_chan *chan = di->chan;

	if (di->dma64) {
		uint32_t addrl;

		/* Check to see if the descriptors need to be aligned on 4K/8K or not */
		BHND_DMA_WRITE_4(chan, BHND_D64_ADDRLOW, 0xFF0);
		addrl = BHND_DMA_READ_4(chan, BHND_D64_ADDRLOW);
		if (addrl != 0)
			return (false);
		else
			return (true);
	} else {
		return (true);
	}
}

/** initialize descriptor table base address */
void
_dma_ddtable_init(dma_info_t *di, dmaaddr_t pa)
{
	struct bhnd_dma_chan *chan = di->chan;

	if (di->dma64) {
		if (!di->aligndesc_4k) {
			di->ptrbase = PHYSADDRLO(pa);
		} else {
			di->ptrbase = 0;
		}

		if ((di->ddoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
			BHND_DMA_WRITE_4(chan, BHND_D64_ADDRLOW,
			    (PHYSADDRLO(pa) + di->ddoffsetlow));
			BHND_DMA_WRITE_4(chan, BHND_D64_ADDRHIGH,
			    (PHYSADDRHI(pa) + di->ddoffsethigh));
		} else {
			/* DMA64 32bits address extension */
			uint32_t ae, ctrl;

			ASSERT(chan->dma->addrext);
			ASSERT(PHYSADDRHI(pa) == 0);

			/* shift the high bit(s) from pa to ae */
			ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
			PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

			BHND_DMA_WRITE_4(chan, BHND_D64_ADDRLOW,
			    (PHYSADDRLO(pa) + di->ddoffsetlow));
			BHND_DMA_WRITE_4(chan, BHND_D64_ADDRHIGH,
			    (PHYSADDRHI(pa) + di->ddoffsethigh));

			ctrl = BHND_DMA_READ_4(chan, BHND_D64_CTRL);
			ctrl &= ~BHND_D64_CTRL_AE_MASK;
			ctrl |= ae << BHND_D64_CTRL_AE_SHIFT;

			BHND_DMA_WRITE_4(chan, BHND_D64_CTRL, ctrl);
		}

	} else {
		ASSERT(PHYSADDRHI(pa) == 0);

		di->ptrbase = 0;
		if ((di->ddoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
			BHND_DMA_WRITE_4(chan, BHND_D32_ADDR, (PHYSADDRLO(pa) +
			    di->ddoffsetlow));
		} else {
			/* dma32 address extension */
			uint32_t ctrl, ae;
			ASSERT(chan->dma->addrext);

			/* shift the high bit(s) from pa to ae */
			ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
			PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

			ctrl = BHND_DMA_READ_4(chan, BHND_D32_CTRL);
			ctrl &= ~BHND_D32_CTRL_AE_MASK;
			ctrl |= ae << BHND_D32_CTRL_AE_SHIFT;

			BHND_DMA_WRITE_4(chan, BHND_D32_CTRL, ctrl);
		}
	}
}

void
_dma_fifoloopbackenable(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint32_t	 ctrl;

	BHND_DMA_TRACE_ENTRY(di);

	chan = di->chan;
	ctrl = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, CTRL));
	ctrl |= BHND_DMA_REG(chan, XC_LE);

	BHND_DMA_WRITE_4(chan, BHND_DMA_REG(chan, CTRL), ctrl);
}

void
_dma_rxinit(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->nrxd == 0)
		return;

	/* During the reset procedure, the active rxd may not be zero if pktpool is
	 * enabled, we need to reclaim active rxd to avoid rxd being leaked.
	 */
	if ((POOL_ENAB(di->pktpool)) && (NRXDACTIVE(di->rxin, di->rxout))) {
		_dma_rxreclaim(di);
	}

	ASSERT(di->rxin == di->rxout);
	di->rxin = di->rxout = di->rs0cd = 0;

	/* clear rx descriptor ring */
	if (di->dma64) {
		BZERO_SM((void *)(uintptr_t)di->rxd64, (di->nrxd * sizeof(dma64dd_t)));

		/* DMA engine with out alignment requirement requires table to be inited
		 * before enabling the engine
		 */
		if (!di->aligndesc_4k)
			_dma_ddtable_init(di, di->rxdpa);

		_dma_rxenable(di);

		if (di->aligndesc_4k)
			_dma_ddtable_init(di, di->rxdpa);
	} else {
		BZERO_SM((void *)(uintptr_t)di->rxd32, (di->nrxd * sizeof(dma32dd_t)));
		_dma_rxenable(di);
		_dma_ddtable_init(di, di->rxdpa);
	}
}

void
_dma_rxenable(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint32_t	 ctrl;
	u_int		 dmactrlflags;

	BHND_DMA_TRACE_ENTRY(di);

	chan = di->chan;
	dmactrlflags = di->hnddma.dmactrlflags;

	ctrl = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, CTRL));

	/* Clear all receive bits, preserving the address extension bits */
	ctrl &= BHND_DMA_REG(chan, RC_AE_MASK);

	/* Enable receive */
	ctrl |= BHND_DMA_REG(chan, RC_RE);

	/* Disable parity checks? */ 
	if ((dmactrlflags & DMA_CTRL_PEN) == 0)
		ctrl |= BHND_DMA_REG(chan, RC_PD);

	/* Continue on overflow? */
	if (dmactrlflags & DMA_CTRL_ROC)
		ctrl |= BHND_DMA_REG(chan, RC_OC);

	ctrl |= (di->rxprefetchctl << BHND_DMA_REG(chan, RC_PC_SHIFT));
	ctrl |= (di->rxprefetchthresh << BHND_DMA_REG(chan, RC_PT_SHIFT));
	ctrl |= (di->rxoffset << BHND_DMA_REG(chan, RC_RO_SHIFT));

	/* On earlier hardware that does not support specifying the burstlen,
	 * the bits may be written, but reads will continue to return 0 */
	ctrl |= (di->rxburstlen << BHND_DMA_REG(chan, RC_BL_SHIFT));

	BHND_DMA_WRITE_4(chan, BHND_DMA_REG(chan, CTRL), ctrl);
}

void
_dma_rx_param_get(dma_info_t *di, uint16_t *rxoffset, uint16_t *rxbufsize)
{
	/* the normal values fit into 16 bits */
	*rxoffset = (uint16_t)di->rxoffset;
	*rxbufsize = (uint16_t)di->rxbufsize;
}

/**
 * !! rx entry routine
 * returns a pointer to the next frame received, or NULL if there are no more
 *   if DMA_CTRL_RXMULTI is defined, DMA scattering(multiple buffers) is supported
 *      with pkts chain
 *   otherwise, it's treated as giant pkt and will be tossed.
 *   The DMA scattering starts with normal DMA header, followed by first buffer data.
 *   After it reaches the max size of buffer, the data continues in next DMA descriptor
 *   buffer WITHOUT DMA header
 */
void *
_dma_rx(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	void		*p, *head, *tail;
	u_int		 len, pkt_len;
	int		 resid = 0;

	chan = di->chan;

next_frame:
	head = _dma_getnextrxp(di, FALSE);
	if (head == NULL)
		return (NULL);

#if (!defined(__mips__) && !defined(BCM47XX_CA9) && !defined(__NetBSD__))
#if defined(BCM4335) || defined(BCM4345) || defined(BCM4350) || defined(BCM43602)
	uint32_t ctrl;
	ctrl = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, CTRL));
	if ((ctrl & BHND_DMA_REG(chan, RC_GE)) != 0) {
		/* In case of glommed pkt get length from hwheader */
		len = le16toh(*((uint16_t *)(PKTDATA(di->osh, head)) + di->rxoffset/2 + 2)) + 4;

		*(uint16_t *)(PKTDATA(di->osh, head)) = len;
	} else {
		len = le16toh(*(uint16_t *)(PKTDATA(di->osh, head)));
	}
#else
	len = le16toh(*(uint16_t *)(PKTDATA(di->osh, head)));
#endif
#else
	{
	int read_count = 0;
	for (read_count = 200; read_count; read_count--) {
		len = le16toh(*(uint16_t *)PKTDATA(di->osh, head));
		if (len != 0)
			break;
		// XXX TODO: (void) required for gcc build with our temporary no-op DMA_MAP() implementation.
		(void)DMA_MAP(di->osh, PKTDATA(di->osh, head), sizeof(uint16_t), DMA_RX, NULL, NULL);
		OSL_DELAY(1);
	}

	if (!len) {
		BHND_DMA_ERROR(di, "%s: dma_rx: frame length (%d)\n", di->name, len);
		PKTFREE(di->osh, head, FALSE);
		goto next_frame;
	}

	}
#endif /* defined(__mips__) */
	BHND_DMA_TRACE(di, "dma_rx len %d", len);

	/* set actual length */
	pkt_len = MIN((di->rxoffset + len), di->rxbufsize);
	PKTSETLEN(di->osh, head, pkt_len);
	resid = len - (di->rxbufsize - di->rxoffset);

	if (resid <= 0) {
		/* Single frame, all good */
	} else if (di->hnddma.dmactrlflags & DMA_CTRL_RXSINGLE) {
		BHND_DMA_TRACE(di, "dma_rx: corrupted length (%d)", len);
		PKTFREE(di->osh, head, FALSE);
		di->hnddma.rxgiants++;
		goto next_frame;
	} else {
		/* multi-buffer rx */
#ifdef BCMDBG
		/* get rid of compiler warning */
		p = NULL;
#endif /* BCMDBG */
		tail = head;
		while ((resid > 0) && (p = _dma_getnextrxp(di, FALSE))) {
			PKTSETNEXT(di->osh, tail, p);
			pkt_len = MIN(resid, (int)di->rxbufsize);
			PKTSETLEN(di->osh, p, pkt_len);

			tail = p;
			resid -= di->rxbufsize;
		}

#ifdef BCMDBG
		if (resid > 0) {
			uint32_t	status, cdesc;
			uint16_t	cur;
			ASSERT(p == NULL);

			/* Fetch the current descriptor address */
			status = BHND_DMA_READ_4(chan,
			    BHND_DMA_REG(chan, STATUS0));			
			cdesc = (status & chan->dma->st0_cd_mask) - di->ptrbase;
			cdesc &= (chan->dma->st0_cd_mask);

			/* Map to a descriptor index */
			cur = cdesc / BHND_DMA_REG(chan, DESC_SIZE);

			BHND_DMA_ERROR(di, "_dma_rx, rxin %d rxout %d, hw_curr %d",
				di->rxin, di->rxout, cur);
		}
#endif /* BCMDBG */

		if ((di->hnddma.dmactrlflags & DMA_CTRL_RXMULTI) == 0) {
			BHND_DMA_ERROR(di, "dma_rx: bad frame length (%d)", len);
			PKTFREE(di->osh, head, FALSE);
			di->hnddma.rxgiants++;
			goto next_frame;
		}
	}

	return (head);
}

/**
 * post receive buffers
 *  return FALSE is refill failed completely and ring is empty
 *  this will stall the rx dma and user might want to call rxfill again asap
 *  This unlikely happens on memory-rich NIC, but often on memory-constrained dongle
 */
bool
_dma_rxfill(dma_info_t *di)
{
	void *p;
	uint16_t rxin, rxout;
	uint32_t flags = 0;
	u_int n;
	u_int i;
	dmaaddr_t pa;
	u_int extra_offset = 0, extra_pad;
	bool ring_empty;
	u_int alignment_req = (di->hnddma.dmactrlflags & DMA_CTRL_USB_BOUNDRY4KB_WAR) ?
				16 : 1;	/* MUST BE POWER of 2 */

	ring_empty = FALSE;

	/*
	 * Determine how many receive buffers we're lacking
	 * from the full complement, allocate, initialize,
	 * and post them, then update the chip rx lastdscr.
	 */

	rxin = di->rxin;
	rxout = di->rxout;

	n = di->nrxpost - NRXDACTIVE(rxin, rxout);

	if (di->rxbufsize > BCMEXTRAHDROOM)
		extra_offset = di->rxextrahdrroom;

	BHND_DMA_TRACE(di, "dma_rxfill: post %d", n);

	for (i = 0; i < n; i++) {
		/* the di->rxbufsize doesn't include the extra headroom, we need to add it to the
		   size to be allocated
		*/
		if (POOL_ENAB(di->pktpool)) {
			ASSERT(di->pktpool);
			p = pktpool_get(di->pktpool);
#ifdef BCMDBG_POOL
			if (p)
				PKTPOOLSETSTATE(p, POOL_RXFILL);
#endif /* BCMDBG_POOL */
		}
		else {
			p = PKTGET(di->osh, (di->rxbufsize + extra_offset +  alignment_req - 1),
				FALSE);
		}
		if (p == NULL) {
			BHND_DMA_TRACE(di, "dma_rxfill: out of rxbufs");
			if (i == 0) {
				if (di->dma64) {
					if (dma64_rxidle(di)) {
						BHND_DMA_TRACE(di, "rxfill64: ring is empty");
						ring_empty = TRUE;
					}
				} else {
					if (dma32_rxidle(di)) {
						BHND_DMA_TRACE(di, "rxfill32: ring is empty");
						ring_empty = TRUE;
					}
				}
			}
			di->hnddma.rxnobuf++;
			break;
		}
		/* reserve an extra headroom, if applicable */
		if (di->hnddma.dmactrlflags & DMA_CTRL_USB_BOUNDRY4KB_WAR) {
			extra_pad = ((alignment_req - (u_int)(((unsigned long)PKTDATA(di->osh, p) -
				(unsigned long)(u_char *)0))) & (alignment_req - 1));
		} else
			extra_pad = 0;

		if (extra_offset + extra_pad)
			PKTPULL(di->osh, p, extra_offset + extra_pad);

#ifdef CTFMAP
		/* mark as ctf buffer for fast mapping */
		if (CTF_ENAB(kcih)) {
			ASSERT((((uint32_t)PKTDATA(di->osh, p)) & 31) == 0);
			PKTSETCTF(di->osh, p);
		}
#endif /* CTFMAP */

		/* Do a cached write instead of uncached write since DMA_MAP
		 * will flush the cache.
		*/
		*(uint32_t *)(PKTDATA(di->osh, p)) = 0;
#if defined(linux) && (defined(BCM47XX_CA9) || defined(__mips__))
		DMA_MAP(di->osh, PKTDATA(di->osh, p), sizeof(uint16_t), DMA_TX, NULL, NULL);
#endif

		if (DMASGLIST_ENAB)
			bzero(&di->rxp_dmah[rxout], sizeof(hnddma_seg_map_t));
#ifdef BCM_SECURE_DMA
		pa = SECURE_DMA_MAP(di->osh, PKTDATA(di->osh, p), di->rxbufsize, DMA_RX,
			NULL, NULL, &di->sec_cma_info_rx, 0);
#else
		pa = DMA_MAP(di->osh, PKTDATA(di->osh, p),
		              di->rxbufsize, DMA_RX, p,
		              &di->rxp_dmah[rxout]);
#endif
		ASSERT(ISALIGNED(PHYSADDRLO(pa), 4));

		/* save the free packet pointer */
		ASSERT(di->rxp[rxout] == NULL);
		di->rxp[rxout] = p;

		/* reset flags for each descriptor */
		flags = 0;
		if (di->dma64) {
			if (rxout == (di->nrxd - 1))
				flags = D64_CTRL1_EOT;

			dma64_dd_upd(di, di->rxd64, pa, rxout, &flags, di->rxbufsize);
		} else {
			if (rxout == (di->nrxd - 1))
				flags = CTRL_EOT;

			ASSERT(PHYSADDRHI(pa) == 0);
			dma32_dd_upd(di, di->rxd32, pa, rxout, &flags, di->rxbufsize);
		}
		rxout = NEXTRXD(rxout);
	}

	di->rxout = rxout;

	/* update the chip lastdscr pointer */
	if (di->dma64) {
		W_REG(di->osh, &di->d64rxregs->ptr, di->ptrbase + I2B(rxout, dma64dd_t));
	} else {
		W_REG(di->osh, &di->d32rxregs->ptr, I2B(rxout, dma32dd_t));
	}

	return ring_empty;
}

/**
 * Fetch the index of the active DMA descriptor.
 * 
 * @param chan DMA channel to query.
 */
static inline uint16_t
bhnd_dma_desc_active(bhnd_dma_chan *chan, dma_info_t *di)
{
	uint32_t status, addr;

	status = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, STATUS1));
	addr = (status & chan->dma->st1_ad_mask) >>
		    BHND_DMA_REG(chan, STATUS1_AD_SHIFT);
	addr = (addr - di->ptrbase) & chan->dma->st1_ad_mask;

	return (addr / BHND_DMA_REG(chan, DESC_SIZE));
}

/**
 * Fetch the index of the current DMA descriptor.
 * 
 * @param chan DMA channel to query.
 */
static inline uint16_t
bhnd_dma_desc_current(bhnd_dma_chan *chan, dma_info_t *di)
{
	uint32_t status, addr;

	status = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, STATUS0));
	addr = (status & chan->dma->st0_cd_mask) >>
	    BHND_DMA_REG(chan, STATUS0_CD_SHIFT);
	addr = (addr - di->ptrbase) & chan->dma->st0_cd_mask;

	return (addr / BHND_DMA_REG(chan, DESC_SIZE));
}

/**
 * Fetch the index of the last DMA descriptor posted to the chip
 * 
 * @param chan DMA channel to query.
 */
static inline uint16_t
bhnd_dma_desc_last(bhnd_dma_chan *chan, dma_info_t *di)
{
	uint32_t ptr, addr;

	ptr = BHND_DMA_READ_4(chan, BHND_DMA_REG(chan, PTR));
	addr = (ptr & chan->dma->st0_cd_mask) >>
	    BHND_DMA_REG(chan, STATUS0_CD_SHIFT);
	addr = (addr - di->ptrbase) & chan->dma->st0_cd_mask;

	return (addr / BHND_DMA_REG(chan, DESC_SIZE));
}

/** like getnexttxp but no reclaim */
void *
_dma_peeknexttxp(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint16_t	 end, i;

	chan = di->chan;

	if (di->ntxd == 0)
		return (NULL);

	end = bhnd_dma_desc_current(chan, di);
	di->xs0cd = end;

	for (i = di->txin; i != end; i = NEXTTXD(i))
		if (di->txp[i])
			return (di->txp[i]);

	return (NULL);
}

int
_dma_peekntxp(dma_info_t *di, int *len, void *txps[], txd_range_t range)
{
	bhnd_dma_chan	*chan;
	void		*txp = NULL;
	uint16_t	 start, end, i;
	u_int		 act;
	int		 k, len_max;

	BHND_DMA_TRACE_ENTRY(di);

	chan = di->chan;

	ASSERT(len);
	ASSERT(txps);
	ASSERT(di);
	if (di->ntxd == 0) {
		*len = 0;
		return BCME_ERROR;
	}

	len_max = *len;
	*len = 0;

	start = di->txin;

	if (range == HNDDMA_RANGE_ALL)
		end = di->txout;
	else {
		end = bhnd_dma_desc_current(chan, di);
		act = bhnd_dma_desc_active(chan, di);

		di->xs0cd = end;
		if (end != act)
			end = PREVTXD(act);
	}

	if ((start == 0) && (end > di->txout))
		return BCME_ERROR;

	k = 0;
	for (i = start; i != end; i = NEXTTXD(i)) {
		txp = di->txp[i];
		if (txp != NULL) {
			if (k < len_max)
				txps[k++] = txp;
			else
				break;
		}
	}
	*len = k;

	return BCME_OK;
}

/** like getnextrxp but not take off the ring */
void *
_dma_peeknextrxp(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint16_t	 end, i;

	chan = di->chan;

	if (di->nrxd == 0)
		return (NULL);

	end = bhnd_dma_desc_current(chan, di);
	di->rs0cd = end;

	for (i = di->rxin; i != end; i = NEXTRXD(i))
		if (di->rxp[i])
			return (di->rxp[i]);

	return (NULL);
}

void
_dma_rxreclaim(dma_info_t *di)
{
	void *p;
	bool origcb = TRUE;

	BHND_DMA_TRACE_ENTRY(di);

	if (POOL_ENAB(di->pktpool) &&
	    ((origcb = pktpool_emptycb_disabled(di->pktpool)) == FALSE))
		pktpool_emptycb_disable(di->pktpool, TRUE);

	while ((p = _dma_getnextrxp(di, TRUE)))
		PKTFREE(di->osh, p, FALSE);

	if (origcb == FALSE)
		pktpool_emptycb_disable(di->pktpool, FALSE);
}

void *
_dma_getnextrxp(dma_info_t *di, bool forceall)
{
	if (di->nrxd == 0)
		return (NULL);

	if (di->dma64) {
		return dma64_getnextrxp(di, forceall);
	} else {
		return dma32_getnextrxp(di, forceall);
	}
}

void
_dma_txblock(dma_info_t *di)
{
	di->hnddma.txavail = 0;
}

void
_dma_txunblock(dma_info_t *di)
{
	di->hnddma.txavail = di->ntxd - NTXDACTIVE(di->txin, di->txout) - 1;
}

u_int
_dma_txactive(dma_info_t *di)
{
	return NTXDACTIVE(di->txin, di->txout);
}

u_int
_dma_txpending(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint16_t	 curr;

	chan = di->chan;

	curr = bhnd_dma_desc_current(chan, di);
	di->xs0cd = curr;

	return NTXDACTIVE(curr, di->txout);
}

u_int
_dma_txcommitted(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint16_t	 ptr;

	chan = di->chan;

	if (di->txin == di->txout)
		return 0;

	if (di->dma64) {
		ptr = BHND_DMA_READ_4(chan, BHND_D64_PTR) / BHND_D64_DESC_SIZE;
	} else {
		ptr = BHND_DMA_READ_4(chan, BHND_D32_PTR) / BHND_D32_DESC_SIZE;
	}

	return NTXDACTIVE(di->txin, ptr);
}

u_int
_dma_rxactive(dma_info_t *di)
{
	return NRXDACTIVE(di->rxin, di->rxout);
}

u_int
_dma_activerxbuf(dma_info_t *di)
{
	bhnd_dma_chan	*chan;
	uint16_t	 curr, last;

	chan = di->chan;

	curr = bhnd_dma_desc_current(chan, di);
	last = bhnd_dma_desc_last(chan, di);

	return NRXDACTIVE(curr, last);
}


void
_dma_counterreset(dma_info_t *di)
{
	/* reset all software counter */
	di->hnddma.rxgiants = 0;
	di->hnddma.rxnobuf = 0;
	di->hnddma.txnobuf = 0;
}

u_int
_dma_ctrlflags(dma_info_t *di, u_int mask, u_int flags)
{
	u_int dmactrlflags;

	/* XXX: Do we really need this? */
	KASSERT(di != NULL, ("NULL DMA handle"));

	dmactrlflags = di->hnddma.dmactrlflags;
	ASSERT((flags & ~mask) == 0);

	dmactrlflags &= ~mask;
	dmactrlflags |= flags;

	/* If trying to enable parity, check if parity is actually supported */
	if (dmactrlflags & DMA_CTRL_PEN) {
		uint32_t control;

		if (di->dma64) {
			control = R_REG(di->osh, &di->d64txregs->control);
			W_REG(di->osh, &di->d64txregs->control, control | D64_XC_PD);
			if (R_REG(di->osh, &di->d64txregs->control) & D64_XC_PD) {
				/* We *can* disable it so it is supported,
				 * restore control register
				 */
				W_REG(di->osh, &di->d64txregs->control, control);
			} else {
				/* Not supported, don't allow it to be enabled */
				dmactrlflags &= ~DMA_CTRL_PEN;
			}
		} else {
			control = R_REG(di->osh, &di->d32txregs->control);
			W_REG(di->osh, &di->d32txregs->control, control | XC_PD);
			if (R_REG(di->osh, &di->d32txregs->control) & XC_PD) {
				W_REG(di->osh, &di->d32txregs->control, control);
			} else {
				/* Not supported, don't allow it to be enabled */
				dmactrlflags &= ~DMA_CTRL_PEN;
			}
		}
	}

	di->hnddma.dmactrlflags = dmactrlflags;

	return (dmactrlflags);
}

/** get the address of the var in order to change later */
uintptr_t
_dma_getvar(dma_info_t *di, const char *name)
{
	if (!strcmp(name, "&txavail"))
		return ((uintptr_t) &(di->hnddma.txavail));
	else {
		ASSERT(0);
	}
	return (0);
}

u_int
_dma_avoidancecnt(dma_info_t *di)
{
	return (di->dma_avoidance_cnt);
}

void
dma_txpioloopback(osl_t *osh, dma32regs_t *regs)
{
	OR_REG(osh, &regs->control, XC_LE);
}

static
uint8_t dma_align_sizetobits(u_int size)
{
	uint8_t bitpos = 0;
	ASSERT(size);
	ASSERT(!(size & (size-1)));
	while (size >>= 1) {
		bitpos ++;
	}
	return (bitpos);
}

/**
 * This function ensures that the DMA descriptor ring will not get allocated
 * across Page boundary. If the allocation is done across the page boundary
 * at the first time, then it is freed and the allocation is done at
 * descriptor ring size aligned location. This will ensure that the ring will
 * not cross page boundary
 */
void *
dma_ringalloc(osl_t *osh, uint32_t boundary, u_int size, uint16_t *alignbits, u_int* alloced,
	dmaaddr_t *descpa, osldma_t **dmah)
{
	void * va;
	uint32_t desc_strtaddr;
	uint32_t alignbytes = 1 << *alignbits;

	if ((va = DMA_ALLOC_CONSISTENT(osh, size, *alignbits, alloced, descpa, dmah)) == NULL)
		return NULL;

	desc_strtaddr = (uint32_t)roundup((u_int)PHYSADDRLO(*descpa), alignbytes);
	if (((desc_strtaddr + size - 1) & boundary) !=
	    (desc_strtaddr & boundary)) {
		*alignbits = dma_align_sizetobits(size);
		DMA_FREE_CONSISTENT(osh, va,
		                    size, *descpa, *dmah);
		va = DMA_ALLOC_CONSISTENT(osh, size, *alignbits, alloced, descpa, dmah);
	}
	return va;
}

// XXX TODO
#if 0
u_int
dma_addrwidth(si_t *sih, void *dmaregs)
{
	dma32regs_t *dma32regs;
	osl_t *osh;

	osh = si_osh(sih);

	/* Perform 64-bit checks only if we want to advertise 64-bit (> 32bit) capability) */
	/* DMA engine is 64-bit capable */
	if ((si_core_sflags(sih, 0, 0) & BHND_IOST_DMA64) == BHND_IOST_DMA64) {
		/* backplane are 64-bit capable */
		if (si_backplane64(sih))
			/* If bus is System Backplane or PCIE then we can access 64-bits */
			if ((sih->bustype == SI_BUS) ||
			    ((sih->bustype == PCI_BUS) &&
			     ((sih->buscoretype == BHND_COREID_PCIE) ||
			      (sih->buscoretype == BHND_COREID_PCIE2))))
				return (BHND_DMA_ADDR_64BIT);

		/* DMA64 is always 32-bit capable, AE is always TRUE */
		ASSERT(_dma64_addrext(osh, (dma64regs_t *)dmaregs));

		return (BHND_DMA_ADDR_32BIT);
	}

	/* Start checking for 32-bit / 30-bit addressing */
	dma32regs = (dma32regs_t *)dmaregs;

	/* For System Backplane, PCIE bus or addrext feature, 32-bits ok */
	if ((sih->bustype == SI_BUS) ||
	    ((sih->bustype == PCI_BUS) &&
	     ((sih->buscoretype == BHND_COREID_PCIE) ||
	      (sih->buscoretype == BHND_COREID_PCIE2))) ||
	    (_dma32_addrext(osh, dma32regs)))
		return (BHND_DMA_ADDR_32BIT);

	/* Fallthru */
	return (BHND_DMA_ADDR_30BIT);
}
#endif

int
_dma_pktpool_set(dma_info_t *di, pktpool_t *pool)
{
	ASSERT(di);
	ASSERT(di->pktpool == NULL);
	di->pktpool = pool;
	return 0;
}

bool
_dma_rxtx_error(dma_info_t *di, bool istx)
{
	// XXX TODO: seems to be entirely unused
#if 0
	uint32_t status1 = 0;
	uint16_t curr;

	if (di->dma64) {

		if (istx) {

			status1 = R_REG(di->osh, &di->d64txregs->status1);

			if ((status1 & D64_XS1_XE_MASK) != D64_XS1_XE_NOERR)
				return TRUE;
			else if ((si_coreid(di->sih) == BHND_COREID_GMAC && si_corerev(di->sih) >= 4) ||
				(si_coreid(di->sih) == BHND_COREID_D11)) {	//cathy add BHND_COREID_D11
				curr = (uint16_t)(B2I(((R_REG(di->osh, &di->d64txregs->status0) &
					D64_XS0_CD_MASK) - di->ptrbase) &
					D64_XS0_CD_MASK, dma64dd_t));

				if (NTXDACTIVE(di->txin, di->txout) != 0 &&
					curr == di->xs0cd_snapshot) {

					/* suspicious */
					return TRUE;
				}
				di->xs0cd_snapshot = di->xs0cd = curr;

				return FALSE;
			}
			else
				return FALSE;
		}
		else {

			status1 = R_REG(di->osh, &di->d64rxregs->status1);

			if ((status1 & D64_RS1_RE_MASK) != D64_RS1_RE_NOERR)
				return TRUE;
			else
				return FALSE;
		}

	} else {
		return FALSE;
	}
#else
	BHND_DMA_UNIMPL();
#endif
}

void
_dma_burstlen_set(dma_info_t *di, uint8_t rxburstlen, uint8_t txburstlen)
{
	di->rxburstlen = rxburstlen;
	di->txburstlen = txburstlen;
}

void
_dma_param_set(dma_info_t *di, uint16_t paramid, uint16_t paramval)
{
	switch (paramid) {
	case HNDDMA_PID_TX_MULTI_OUTSTD_RD:
		di->txmultioutstdrd = (uint8_t)paramval;
		break;

	case HNDDMA_PID_TX_PREFETCH_CTL:
		di->txprefetchctl = (uint8_t)paramval;
		break;

	case HNDDMA_PID_TX_PREFETCH_THRESH:
		di->txprefetchthresh = (uint8_t)paramval;
		break;

	case HNDDMA_PID_TX_BURSTLEN:
		di->txburstlen = (uint8_t)paramval;
		break;

	case HNDDMA_PID_RX_PREFETCH_CTL:
		di->rxprefetchctl = (uint8_t)paramval;
		break;

	case HNDDMA_PID_RX_PREFETCH_THRESH:
		di->rxprefetchthresh = (uint8_t)paramval;
		break;

	case HNDDMA_PID_RX_BURSTLEN:
		di->rxburstlen = (uint8_t)paramval;
		break;

	default:
		break;
	}
}

bool
_dma_glom_enable(dma_info_t *di, uint32_t val)
{
	dma64regs_t *dregs = di->d64rxregs;
	bool ret = TRUE;
	if (val) {
		OR_REG(di->osh, &dregs->control, D64_RC_GE);
		if (!(R_REG(di->osh, &dregs->control) & D64_RC_GE))
			ret = FALSE;
	} else {
		AND_REG(di->osh, &dregs->control, ~D64_RC_GE);
	}
	return ret;
}
