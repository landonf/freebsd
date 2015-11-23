/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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

#ifndef _BHND_BCMA_BCMAB_PCIVAR_H_
#define _BHND_BCMA_BCMAB_PCIVAR_H_

#include "bcmavar.h"

struct bcmab_pci_softc {
	struct bhndb_softc	bhndb_softc;	/**< bhndb state */
	device_t		dev;		/**< bcmab device */
	device_t		parent_dev;	/**< parent PCI device */
	device_t		bus_dev;	/**< child bcma(4) bus */
	bhndb_addr_t		erom_addr;	/**< EROM's base address */
};

#endif /* _BHND_BCMA_BCMAB_PCIVAR_H_ */