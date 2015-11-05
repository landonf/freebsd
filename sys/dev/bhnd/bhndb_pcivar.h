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

#include "bhnd.h"
#include <sys/types.h>
#include <sys/rman.h>

extern devclass_t bhndb_devclass;

/**
 * BHND PCI bridge per-instance state.
 * 
 * This must be the first member of any subclass' softc.
 */
struct bhndb_pci_softc {
	device_t		 dev;		/**< bridge device */
	device_t		 pci_dev;	/**< PCI parent device */
	struct resource		**pci_res;	/**< PCI device resources */
	size_t			 num_pci_res;	/**< PCI device resource count */
	struct rman		 mem_rman;	/**< bus memory resource manager */
};

int			 bhndb_pci_generic_probe(device_t dev);
int			 bhndb_pci_generic_attach(device_t dev);
int			 bhndb_pci_generic_detach(device_t dev);
int			 bhndb_pci_generic_suspend(device_t dev);
int			 bhndb_pci_generic_resume(device_t dev);

struct resource		*bhndb_pci_generic_alloc_resource(device_t dev,
			     device_t child, int type, int *rid, u_long start,
			     u_long end, u_long count, u_int flags);

int			 bhndb_pci_generic_release_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct resource *res);

int			 bhndb_pci_generic_activate_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct resource *r);

int			 bhndb_pci_generic_deactivate_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct resource *r);

struct bhnd_resource	*bhndb_pci_generic_alloc_bhnd_resource(device_t dev,
			     device_t child, int type, int *rid, u_long start,
			     u_long end, u_long count, u_int flags);

int			 bhndb_pci_generic_release_bhnd_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

int			 bhndb_pci_generic_activate_bhnd_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct bhnd_resource *r);

int			 bhndb_pci_generic_deactivate_bhnd_resource(
			     device_t dev, device_t child, int type, int rid,
			     struct bhnd_resource *r);

/**
 * Register window types.
 */
typedef enum {
	BCMA_PCI_WINTYPE_FIXED,	/**< A fixed window that maps a specific core
				 *   type */
	BCMA_PCI_WINTYPE_DYN	/**< A dynamically configurable window */
} bcma_pci_wintype_t;

/**
 * Defines a register window within a BCMA PCI BAR resource that can be
 * adjusted at runtime to map address regions from the SoC interconnect.
 */
struct bcma_pci_regwin {
	u_int			pci_res;	/**< pci_res[] index of the BAR
						     in which this register
						     window is mapped */

	bcma_pci_wintype_t	win_type;	/** register window type */
	union {
		/** BCMA_PCI_WINTYPE_DYN configuration */
		struct {
			bus_size_t	cfg_offset;	/**< window config offset within the pci
							     config space. */
		};
		
		/** BCMA_PCI_WINTYPE_FIXED configuration */
		struct {
			bhnd_devclass_t	bhnd_class;	/**< mapped cores' class */
			u_int		port_num;	/**< mapped port number */
			u_int		region_num;	/**< mapped region number */
		};
	};

	bus_size_t	win_offset;	/**< offset of the register block
					 *   within the PCI resource at */
	bus_size_t	win_size;	/**< size of the register block */
};

/**
 * Simple declaration of a bhnd PCI/PCIe core-based endpoint bridge driver.
 * 
 * @param dname The device/driver name (e.g. `bcmab_pci`)
 * @param dclass The device class name (e.g. `bcma`)
 */
#define	BHNDB_PCI_DECLARE_DRIVER(dname,dclass) \
static device_method_t __CONCAT(dname,_methods)[] = { \
	/* Device interface */ \
	DEVMETHOD(device_probe,			__CONCAT(dname,_probe)),		\
	DEVMETHOD(device_attach,		__CONCAT(dname,_attach)),		\
	DEVMETHOD(device_detach,		__CONCAT(dname,_detach)),		\
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),			\
	DEVMETHOD(device_suspend,		bhndb_pci_generic_suspend),		\
	DEVMETHOD(device_resume,		bhndb_pci_generic_resume),		\
											\
	/* Bus interface */								\
	DEVMETHOD(bus_alloc_resource,		bhndb_pci_generic_alloc_resource),	\
	DEVMETHOD(bus_release_resource,		bhndb_pci_generic_release_resource),	\
	DEVMETHOD(bus_activate_resource,	bhndb_pci_generic_activate_resource),	\
	DEVMETHOD(bus_deactivate_resource,	bhndb_pci_generic_deactivate_resource),	\
											\
	/* BHND interface */								\
	DEVMETHOD(bhndbus_alloc_resource,	bhndb_pci_generic_alloc_bhnd_resource),	\
	DEVMETHOD(bhndbus_release_resource,	bhndb_pci_generic_release_bhnd_resource),	\
	DEVMETHOD(bhndbus_activate_resource,	bhndb_pci_generic_activate_bhnd_resource),	\
	DEVMETHOD(bhndbus_activate_resource,	bhndb_pci_generic_deactivate_bhnd_resource),	\
											\
	DEVMETHOD_END									\
}; \
\
DEFINE_CLASS_0(dclass, __CONCAT(dname,_driver), __CONCAT(dname,_methods), sizeof(struct __CONCAT(dname,_softc))); \
DRIVER_MODULE(dname, pci, __CONCAT(dname,_driver), __CONCAT(dclass,_devclass), NULL, NULL); \
MODULE_VERSION(dname, 1);		\
MODULE_DEPEND(dname, pci, 1, 1, 1);	\
MODULE_DEPEND(dname, bhnd, 1, 1, 1);

#endif /* _BHND_BHND_PCI_H_ */