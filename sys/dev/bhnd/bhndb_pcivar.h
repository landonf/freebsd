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

#ifndef _BHND_BHND_PCI_H_
#define _BHND_BHND_PCI_H_

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include "bhnd.h"
#include "bhndbvar.h"

DECLARE_CLASS(bhndb_pci_driver);

/**
 * PCI-BHND bridge per-instance state.
 */
struct bhndb_pci_softc {
	device_t		 dev;		/**< bridge device */
	device_t		 pci_dev;	/**< parent pci device */
	size_t			 num_pci_res;	/**< pci resource count */
	struct resource_spec	*pci_res_spec;	/**< pci resource specs */
	struct resource		**pci_res;	/**< pci resources */
	struct rman		 mem_rman;	/**< bus memory resource manager */
};

#endif /* _BHND_BHND_PCI_H_ */