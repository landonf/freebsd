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

#include "bcmutils.h"

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

#endif	/* _siutils_h_ */
