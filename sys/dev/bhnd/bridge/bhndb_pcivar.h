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

extern const struct bhndb_hwcfg sibab_pci_hwcfg_generic;

/**
 * Hardware configuration table entry.
 * 
 * Defines a set of match criteria that may be used to determine the
 * register map and resource configuration for a bhndb bridge device. 
 */
struct bhndb_hw {
	const char			*name;		/**< configuration name */
	const struct bhnd_core_match	*hw_reqs;	/**< match requirements */
	u_int				 num_hw_reqs;	/**< number of match requirements */
	const struct bhndb_hwcfg	*cfg;		/**< associated hardware configuration */
};

extern const struct bhndb_hw bhndb_pci_hw[];

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