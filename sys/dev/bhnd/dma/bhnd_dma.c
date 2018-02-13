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
#include <sys/malloc.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/bhnd_ids.h> /* XXX REMOVE: used for hard-coded DMA translation constants */

#include "bhnd_dmavar.h"

/* XXX TODO: Hide behind our DMA callbacks */
#include "bhnd_dma32var.h"
#include "bhnd_dma64var.h"

#include "siutils.h"

/*
 * Generic Broadcom Home Networking Division (HND) DMA engine SW interface
 * This supports the following chips: BCM42xx, 44xx, 47xx .
 */

/* Common prototypes */
static bool _dma_isaddrext(dma_info_t *di);
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
dma_attach(osl_t *osh, const char *name, si_t *sih,
	volatile void *dmaregstx, volatile void *dmaregsrx,
	u_int ntxd, u_int nrxd, u_int rxbufsize, int rxextheadroom, u_int nrxpost, u_int rxoffset,
	u_int *msg_level)
{
	dma_info_t *di;
	u_int size;
	uint32_t mask;

	/* allocate private info structure */
	if ((di = MALLOC(osh, sizeof (dma_info_t))) == NULL) {
		return (NULL);
	}

	bzero(di, sizeof(dma_info_t));

	di->msg_level = msg_level ? msg_level : &default_msg_level;

	/* make a private copy of our callers name */
	strncpy(di->name, name, MAXNAMEL);
	di->name[MAXNAMEL-1] = '\0';

	/* old chips w/o sb is no longer supported */
	ASSERT(sih != NULL);

	di->dma64 = ((si_core_sflags(sih, 0, 0) & BHND_IOST_DMA64) == BHND_IOST_DMA64);

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

	BHND_DMA_TRACE(di, "%s: osh %p flags %#x ntxd %d nrxd %d rxbufsize %d "
	           "rxextheadroom %d nrxpost %d rxoffset %d dmaregstx %p dmaregsrx %p",
	           (di->dma64 ? "DMA64" : "DMA32"),
	           osh, di->hnddma.dmactrlflags, ntxd, nrxd,
	           rxbufsize, rxextheadroom, nrxpost, rxoffset, dmaregstx, dmaregsrx);

	di->osh = osh;
	di->sih = sih;

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

	/* Get the default values (POR) of the burstlen. This can be overridden by the modules
	 * if this has to be different. Otherwise this value will be used to program the control
	 * register after the reset or during the init.
	 */
	if (dmaregsrx) {
		if (di->dma64) {
			/* detect the dma descriptor address mask,
			 * should be 0x1fff before 4360B0, 0xffff start from 4360B0
			 */
			W_REG(di->osh, &di->d64rxregs->addrlow, 0xffffffff);
			mask = R_REG(di->osh, &di->d64rxregs->addrlow);

			if (mask & 0xfff)
				mask = R_REG(di->osh, &di->d64rxregs->ptr) | 0xf;
			else
				mask = 0x1fff;

			BHND_DMA_TRACE(di, "dma_rx_mask: %#08x", mask);
			di->d64_rs0_cd_mask = mask;

			if (mask == 0x1fff)
				ASSERT(nrxd <= D64MAXDD);
			else
				ASSERT(nrxd <= D64MAXDD_LARGE);

			di->rxburstlen = (R_REG(di->osh,
				&di->d64rxregs->control) & D64_RC_BL_MASK) >> D64_RC_BL_SHIFT;
			di->rxprefetchctl = (R_REG(di->osh,
				&di->d64rxregs->control) & D64_RC_PC_MASK) >>
				D64_RC_PC_SHIFT;
			di->rxprefetchthresh = (R_REG(di->osh,
				&di->d64rxregs->control) & D64_RC_PT_MASK) >>
				D64_RC_PT_SHIFT;
		} else {
			di->rxburstlen = (R_REG(di->osh,
				&di->d32rxregs->control) & RC_BL_MASK) >> RC_BL_SHIFT;
			di->rxprefetchctl = (R_REG(di->osh,
				&di->d32rxregs->control) & RC_PC_MASK) >> RC_PC_SHIFT;
			di->rxprefetchthresh = (R_REG(di->osh,
				&di->d32rxregs->control) & RC_PT_MASK) >> RC_PT_SHIFT;
		}
	}
	if (dmaregstx) {
		if (di->dma64) {

			/* detect the dma descriptor address mask,
			 * should be 0x1fff before 4360B0, 0xffff start from 4360B0
			 */
			W_REG(di->osh, &di->d64txregs->addrlow, 0xffffffff);
			mask = R_REG(di->osh, &di->d64txregs->addrlow);

			if (mask & 0xfff)
				mask = R_REG(di->osh, &di->d64txregs->ptr) | 0xf;
			else
				mask = 0x1fff;

			BHND_DMA_TRACE(di, "dma_tx_mask: %#08x", mask);
			di->d64_xs0_cd_mask = mask;
			di->d64_xs1_ad_mask = mask;

			if (mask == 0x1fff)
				ASSERT(ntxd <= D64MAXDD);
			else
				ASSERT(ntxd <= D64MAXDD_LARGE);

			di->txburstlen = (R_REG(di->osh,
				&di->d64txregs->control) & D64_XC_BL_MASK) >> D64_XC_BL_SHIFT;
			di->txmultioutstdrd = (R_REG(di->osh,
				&di->d64txregs->control) & D64_XC_MR_MASK) >> D64_XC_MR_SHIFT;
			di->txprefetchctl = (R_REG(di->osh,
				&di->d64txregs->control) & D64_XC_PC_MASK) >> D64_XC_PC_SHIFT;
			di->txprefetchthresh = (R_REG(di->osh,
				&di->d64txregs->control) & D64_XC_PT_MASK) >> D64_XC_PT_SHIFT;
		} else {
			di->txburstlen = (R_REG(di->osh,
				&di->d32txregs->control) & XC_BL_MASK) >> XC_BL_SHIFT;
			di->txmultioutstdrd = (R_REG(di->osh,
				&di->d32txregs->control) & XC_MR_MASK) >> XC_MR_SHIFT;
			di->txprefetchctl = (R_REG(di->osh,
				&di->d32txregs->control) & XC_PC_MASK) >> XC_PC_SHIFT;
			di->txprefetchthresh = (R_REG(di->osh,
				&di->d32txregs->control) & XC_PT_MASK) >> XC_PT_SHIFT;
		}
	}

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

#if defined(__mips__) && defined(IL_BIGENDIAN)
	di->dataoffsetlow = di->dataoffsetlow + SI_SDRAM_SWAPPED;
#endif /* defined(__mips__) && defined(IL_BIGENDIAN) */
	/* WAR64450 : DMACtl.Addr ext fields are not supported in SDIOD core. */
	if ((si_coreid(sih) == BHND_COREID_SDIOD) && ((si_corerev(sih) > 0) && (si_corerev(sih) <= 2)))
		di->addrext = 0;
	else if ((si_coreid(sih) == BHND_COREID_I2S) &&
	         ((si_corerev(sih) == 0) || (si_corerev(sih) == 1)))
		di->addrext = 0;
	else
		di->addrext = _dma_isaddrext(di);

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

	if ((di->ddoffsetlow != 0) && !di->addrext) {
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
	           di->dataoffsethigh, di->addrext);

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
	if (di->dma64) {
		uint32_t addrl;

		/* Check to see if the descriptors need to be aligned on 4K/8K or not */
		if (di->d64txregs != NULL) {
			W_REG(di->osh, &di->d64txregs->addrlow, 0xff0);
			addrl = R_REG(di->osh, &di->d64txregs->addrlow);
			if (addrl != 0)
				return FALSE;
		} else if (di->d64rxregs != NULL) {
			W_REG(di->osh, &di->d64rxregs->addrlow, 0xff0);
			addrl = R_REG(di->osh, &di->d64rxregs->addrlow);
			if (addrl != 0)
				return FALSE;
		}
	}
	return TRUE;
}


/** return TRUE if this dma engine supports DmaExtendedAddrChanges, otherwise FALSE */
static bool
_dma_isaddrext(dma_info_t *di)
{
	if (di->dma64) {
		/* DMA64 supports full 32- or 64-bit operation. AE is always valid */

		/* not all tx or rx channel are available */
		if (di->d64txregs != NULL) {
			if (!_dma64_addrext(di->osh, di->d64txregs)) {
				BHND_DMA_ERROR(di, "DMA64 tx doesn't have AE set");
				ASSERT(0);
			}
			return TRUE;
		} else if (di->d64rxregs != NULL) {
			if (!_dma64_addrext(di->osh, di->d64rxregs)) {
				BHND_DMA_ERROR(di, "DMA64 rx doesn't have AE set");
				ASSERT(0);
			}
			return TRUE;
		}
		return FALSE;
	} else {
		if (di->d32txregs)
			return (_dma32_addrext(di->osh, di->d32txregs));
		else if (di->d32rxregs)
			return (_dma32_addrext(di->osh, di->d32rxregs));
	}

	return FALSE;
}

/** initialize descriptor table base address */
void
_dma_ddtable_init(dma_info_t *di, u_int direction, dmaaddr_t pa)
{
	if (di->dma64) {
		if (!di->aligndesc_4k) {
			if (direction == DMA_TX)
				di->xmtptrbase = PHYSADDRLO(pa);
			else
				di->rcvptrbase = PHYSADDRLO(pa);
		}

		if ((di->ddoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
			if (direction == DMA_TX) {
				W_REG(di->osh, &di->d64txregs->addrlow, (PHYSADDRLO(pa) +
				                                         di->ddoffsetlow));
				W_REG(di->osh, &di->d64txregs->addrhigh, (PHYSADDRHI(pa) +
				                                          di->ddoffsethigh));
			} else {
				W_REG(di->osh, &di->d64rxregs->addrlow, (PHYSADDRLO(pa) +
				                                         di->ddoffsetlow));
				W_REG(di->osh, &di->d64rxregs->addrhigh, (PHYSADDRHI(pa) +
				                                          di->ddoffsethigh));
			}
		} else {
			/* DMA64 32bits address extension */
			uint32_t ae;
			ASSERT(di->addrext);
			ASSERT(PHYSADDRHI(pa) == 0);

			/* shift the high bit(s) from pa to ae */
			ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
			PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

			if (direction == DMA_TX) {
				W_REG(di->osh, &di->d64txregs->addrlow, (PHYSADDRLO(pa) +
				                                         di->ddoffsetlow));
				W_REG(di->osh, &di->d64txregs->addrhigh, di->ddoffsethigh);
				SET_REG(di->osh, &di->d64txregs->control, D64_XC_AE,
					(ae << D64_XC_AE_SHIFT));
			} else {
				W_REG(di->osh, &di->d64rxregs->addrlow, (PHYSADDRLO(pa) +
				                                         di->ddoffsetlow));
				W_REG(di->osh, &di->d64rxregs->addrhigh, di->ddoffsethigh);
				SET_REG(di->osh, &di->d64rxregs->control, D64_RC_AE,
					(ae << D64_RC_AE_SHIFT));
			}
		}

	} else {
		ASSERT(PHYSADDRHI(pa) == 0);
		if ((di->ddoffsetlow == 0) || !(PHYSADDRLO(pa) & PCI32ADDR_HIGH)) {
			if (direction == DMA_TX)
				W_REG(di->osh, &di->d32txregs->addr, (PHYSADDRLO(pa) +
				                                      di->ddoffsetlow));
			else
				W_REG(di->osh, &di->d32rxregs->addr, (PHYSADDRLO(pa) +
				                                      di->ddoffsetlow));
		} else {
			/* dma32 address extension */
			uint32_t ae;
			ASSERT(di->addrext);

			/* shift the high bit(s) from pa to ae */
			ae = (PHYSADDRLO(pa) & PCI32ADDR_HIGH) >> PCI32ADDR_HIGH_SHIFT;
			PHYSADDRLO(pa) &= ~PCI32ADDR_HIGH;

			if (direction == DMA_TX) {
				W_REG(di->osh, &di->d32txregs->addr, (PHYSADDRLO(pa) +
				                                      di->ddoffsetlow));
				SET_REG(di->osh, &di->d32txregs->control, XC_AE, ae <<XC_AE_SHIFT);
			} else {
				W_REG(di->osh, &di->d32rxregs->addr, (PHYSADDRLO(pa) +
				                                      di->ddoffsetlow));
				SET_REG(di->osh, &di->d32rxregs->control, RC_AE, ae <<RC_AE_SHIFT);
			}
		}
	}
}

void
_dma_fifoloopbackenable(dma_info_t *di)
{
	BHND_DMA_TRACE_ENTRY(di);

	if (di->dma64)
		OR_REG(di->osh, &di->d64txregs->control, D64_XC_LE);
	else
		OR_REG(di->osh, &di->d32txregs->control, XC_LE);
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
			_dma_ddtable_init(di, DMA_RX, di->rxdpa);

		_dma_rxenable(di);

		if (di->aligndesc_4k)
			_dma_ddtable_init(di, DMA_RX, di->rxdpa);
	} else {
		BZERO_SM((void *)(uintptr_t)di->rxd32, (di->nrxd * sizeof(dma32dd_t)));
		_dma_rxenable(di);
		_dma_ddtable_init(di, DMA_RX, di->rxdpa);
	}
}

void
_dma_rxenable(dma_info_t *di)
{
	u_int dmactrlflags = di->hnddma.dmactrlflags;

	BHND_DMA_TRACE_ENTRY(di);

	if (di->dma64) {
		uint32_t control = (R_REG(di->osh, &di->d64rxregs->control) & D64_RC_AE) | D64_RC_RE;

		if ((dmactrlflags & DMA_CTRL_PEN) == 0)
			control |= D64_RC_PD;

		if (dmactrlflags & DMA_CTRL_ROC)
			control |= D64_RC_OC;

		/* These bits 20:18 (burstLen) of control register can be written but will take
		 * effect only if these bits are valid. So this will not affect previous versions
		 * of the DMA. They will continue to have those bits set to 0.
		 */
		control &= ~D64_RC_BL_MASK;
		control |= (di->rxburstlen << D64_RC_BL_SHIFT);

		control &= ~D64_RC_PC_MASK;
		control |= (di->rxprefetchctl << D64_RC_PC_SHIFT);

		control &= ~D64_RC_PT_MASK;
		control |= (di->rxprefetchthresh << D64_RC_PT_SHIFT);

		W_REG(di->osh, &di->d64rxregs->control,
		      ((di->rxoffset << D64_RC_RO_SHIFT) | control));
	} else {
		uint32_t control = (R_REG(di->osh, &di->d32rxregs->control) & RC_AE) | RC_RE;

		if ((dmactrlflags & DMA_CTRL_PEN) == 0)
			control |= RC_PD;

		if (dmactrlflags & DMA_CTRL_ROC)
			control |= RC_OC;

		/* These bits 20:18 (burstLen) of control register can be written but will take
		 * effect only if these bits are valid. So this will not affect previous versions
		 * of the DMA. They will continue to have those bits set to 0.
		 */
		control &= ~RC_BL_MASK;
		control |= (di->rxburstlen << RC_BL_SHIFT);

		control &= ~RC_PC_MASK;
		control |= (di->rxprefetchctl << RC_PC_SHIFT);

		control &= ~RC_PT_MASK;
		control |= (di->rxprefetchthresh << RC_PT_SHIFT);

		W_REG(di->osh, &di->d32rxregs->control,
		      ((di->rxoffset << RC_RO_SHIFT) | control));
	}
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
	void *p, *head, *tail;
	u_int len;
	u_int pkt_len;
	int resid = 0;
#if defined(BCM4335) || defined(BCM4345) || defined(BCM4350) || defined(BCM43602)
	dma64regs_t *dregs = di->d64rxregs;
#endif

next_frame:
	head = _dma_getnextrxp(di, FALSE);
	if (head == NULL)
		return (NULL);

#if (!defined(__mips__) && !defined(BCM47XX_CA9) && !defined(__NetBSD__))
#if defined(BCM4335) || defined(BCM4345) || defined(BCM4350) || defined(BCM43602)
	if ((R_REG(osh, &dregs->control) & D64_RC_GE)) {
		/* In case of glommed pkt get length from hwheader */
		len = ltoh16(*((uint16_t *)(PKTDATA(di->osh, head)) + di->rxoffset/2 + 2)) + 4;

		*(uint16_t *)(PKTDATA(di->osh, head)) = len;
	} else {
		len = ltoh16(*(uint16_t *)(PKTDATA(di->osh, head)));
	}
#else
	len = le16toh(*(uint16_t *)(PKTDATA(di->osh, head)));
#endif
#else
	{
	int read_count = 0;
	for (read_count = 200; read_count; read_count--) {
		len = ltoh16(*(uint16_t *)PKTDATA(di->osh, head));
		if (len != 0)
			break;
		DMA_MAP(di->osh, PKTDATA(di->osh, head), sizeof(uint16_t), DMA_RX, NULL, NULL);
		OSL_DELAY(1);
	}

	if (!len) {
		DMA_ERROR(("%s: dma_rx: frame length (%d)\n", di->name, len));
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
			uint16_t cur;
			ASSERT(p == NULL);
			cur = (di->dma64) ?
				B2I(((R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_CD_MASK) -
				di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t) :
				B2I(R_REG(di->osh, &di->d32rxregs->status) & RS_CD_MASK,
				dma32dd_t);
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
		W_REG(di->osh, &di->d64rxregs->ptr, di->rcvptrbase + I2B(rxout, dma64dd_t));
	} else {
		W_REG(di->osh, &di->d32rxregs->ptr, I2B(rxout, dma32dd_t));
	}

	return ring_empty;
}

/** like getnexttxp but no reclaim */
void *
_dma_peeknexttxp(dma_info_t *di)
{
	uint16_t end, i;

	if (di->ntxd == 0)
		return (NULL);

	if (di->dma64) {
		end = (uint16_t)B2I(((R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_CD_MASK) -
		           di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);
		di->xs0cd = end;
	} else {
		end = (uint16_t)B2I(R_REG(di->osh, &di->d32txregs->status) & XS_CD_MASK, dma32dd_t);
		di->xs0cd = end;
	}

	for (i = di->txin; i != end; i = NEXTTXD(i))
		if (di->txp[i])
			return (di->txp[i]);

	return (NULL);
}

int
_dma_peekntxp(dma_info_t *di, int *len, void *txps[], txd_range_t range)
{
	uint16_t start, end, i;
	u_int act;
	void *txp = NULL;
	int k, len_max;

	BHND_DMA_TRACE_ENTRY(di);

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
		if (di->dma64) {
			end = B2I(((R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_CD_MASK) -
				di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);

			act = (u_int)(R_REG(di->osh, &di->d64txregs->status1) & D64_XS1_AD_MASK);
			act = (act - di->xmtptrbase) & D64_XS0_CD_MASK;
			act = (u_int)B2I(act, dma64dd_t);
		} else {
			end = B2I(R_REG(di->osh, &di->d32txregs->status) & XS_CD_MASK, dma32dd_t);

			act = (u_int)((R_REG(di->osh, &di->d32txregs->status) & XS_AD_MASK) >>
				XS_AD_SHIFT);
			act = (u_int)B2I(act, dma32dd_t);
		}

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
	uint16_t end, i;

	if (di->nrxd == 0)
		return (NULL);

	if (di->dma64) {
		end = (uint16_t)B2I(((R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_CD_MASK) -
			di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
		di->rs0cd = end;
	} else {
		end = (uint16_t)B2I(R_REG(di->osh, &di->d32rxregs->status) & RS_CD_MASK, dma32dd_t);
		di->rs0cd = end;
	}

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
	uint16_t curr;

	if (di->dma64) {
		curr = B2I(((R_REG(di->osh, &di->d64txregs->status0) & D64_XS0_CD_MASK) -
		           di->xmtptrbase) & D64_XS0_CD_MASK, dma64dd_t);
		di->xs0cd = curr;
	} else {
		curr = B2I(R_REG(di->osh, &di->d32txregs->status) & XS_CD_MASK, dma32dd_t);
		di->xs0cd = curr;
	}

	return NTXDACTIVE(curr, di->txout);
}

u_int
_dma_txcommitted(dma_info_t *di)
{
	uint16_t ptr;
	u_int txin = di->txin;

	if (txin == di->txout)
		return 0;

	if (di->dma64) {
		ptr = B2I(R_REG(di->osh, &di->d64txregs->ptr), dma64dd_t);
	} else {
		ptr = B2I(R_REG(di->osh, &di->d32txregs->ptr), dma32dd_t);
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
	uint16_t curr, ptr;
	curr = B2I(((R_REG(di->osh, &di->d64rxregs->status0) & D64_RS0_CD_MASK) -
		di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
	ptr =  B2I(((R_REG(di->osh, &di->d64rxregs->ptr) & D64_RS0_CD_MASK) -
		di->rcvptrbase) & D64_RS0_CD_MASK, dma64dd_t);
	return NRXDACTIVE(curr, ptr);
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
					D64_XS0_CD_MASK) - di->xmtptrbase) &
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
