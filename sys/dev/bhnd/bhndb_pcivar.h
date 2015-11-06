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

#include <sys/types.h>
#include <sys/rman.h>

#include "bhnd.h"
#include "bhndb_pci_if.h"

extern devclass_t bhndb_devclass;

/**
 * BHND PCI bridge per-instance state.
 * 
 * This must be the first member of any subclass' softc.
 */
struct bhndb_pci_softc {
	device_t		 dev;		/**< bridge device */
	device_t		 pci_dev;	/**< parent pci device */
	size_t			 num_pci_res;	/**< pci resource count */
	struct resource_spec	*pci_res_spec;	/**< pci resource specs */
	struct resource		**pci_res;	/**< pci resources */
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
 * BHNDB PCI window types.
 */
typedef enum {
	BHNDB_PCIWIN_TYPE_CORE,		/**< Fixed mapping of a core register block. */
	BHNDB_PCIWIN_TYPE_SPROM,	/**< Fixed mapping PCI SPROM shadow */
	BHNDB_PCIWIN_TYPE_DYN		/**< A dynamically configurable window */
} bhndb_pciwin_type_t;

/**
 * BHNDB PCI window definition.
 */
struct bhndb_pciwin {
	u_int			pci_res;	/**< pci_res[] index of the BAR in which this register window is mapped */
	bus_size_t		offset;		/**< offset of the register block within the PCI BAR */
	bus_size_t		size;		/**< size of the register block */
	bhndb_pciwin_type_t	win_type;	/** window type */
	union {
		/** BHNDB_PCIWIN_TYPE_CORE configuration */
		struct {
			bhnd_devclass_t	class;	/**< mapped cores' class */
			u_int		port;	/**< mapped port number */
			u_int		region;	/**< mapped region number */
		} core;

		/** BHNDB_PCIWIN_TYPE_SPROM configuration */
		struct {} sprom;

		/** BHNDB_PCIWIN_TYPE_DYN configuration */
		struct {
			bus_size_t	cfg_offset;	/**< window config offset within the pci
							     config space. */
		} dynamic;
	};
};

/**
 * BHNDB PCI device definition.
 * 
 * Used for automatic configuration of Broadcom PCI core resources.
 */
struct bhndb_pci_devcfg {
	const struct bhnd_core_match	*req_cores;		/**< required cores */
	size_t				 num_req_cores;		/**< number of required cores */
	const struct bcma_pci_regwin	*register_windows;	/**< device register windows */
	size_t				 num_register_windows;	/**< number of register windows */
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
	DEVMETHOD(bhndbus_enumerate_children,	__CONCAT(dname,_enumerate_children)),	\
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