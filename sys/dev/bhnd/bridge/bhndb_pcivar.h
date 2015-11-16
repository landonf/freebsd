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

#ifndef _BHND_BHNDB_PCIVAR_H_
#define _BHND_BHNDB_PCIVAR_H_

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/bhnd/bhnd.h>

#include "bhndbvar.h"

DECLARE_CLASS(bhndb_pci_driver);

/**
 * bhndb driver instance state. Must be first member of all subclass
 * softc structures.
 */
struct bhndb_pci_softc {
	device_t			 dev;		/**< bridge device */
	const struct bhndb_hwcfg	*hwcfg;		/**< hardware config */

	device_t			 pci_dev;	/**< parent pci device */
	size_t				 res_count;	/**< pci resource count */
	struct resource_spec		*res_spec;	/**< pci resource specs */
	struct resource			**res;		/**< pci resources */

	struct rman			 mem_rman;	/**< bus memory manager */
};

extern const struct bhndb_hwcfg bhnd_pci_v1_common_hwcfg;

int	bhndb_pci_probe(device_t dev);
int	bhndb_pci_attach(device_t dev);
int	bhndb_pci_detach(device_t dev);
int	bhndb_pci_suspend(device_t dev);
int	bhndb_pci_resume(device_t dev);
int	bhndb_pci_read_ivar(device_t dev, device_t child, int index,
	    uintptr_t *result);
int	bhndb_pci_write_ivar(device_t dev, device_t child, int index,
	    uintptr_t value);

#endif /* _BHND_BHND_PCIVAR_H_ */