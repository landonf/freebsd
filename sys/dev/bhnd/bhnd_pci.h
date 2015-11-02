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

int			 bhnd_pci_attach(device_t dev);
int			 bhnd_pci_detach(device_t dev);
int			 bhnd_pci_suspend(device_t dev);
int			 bhnd_pci_resume(device_t dev);

struct resource		*bhnd_pci_alloc_resource(device_t dev,
			     device_t child, int type, int *rid, u_long start,
			     u_long end, u_long count, u_int flags);

int			 bhnd_pci_set_resource(device_t dev,
			     device_t child, int type, int rid, u_long start,
			     u_long count);

int			 bhnd_pci_get_resource(device_t dev,
			     device_t child, int type, int rid, u_long *startp,
			     u_long *countp);

void			 bhnd_pci_delete_resource(device_t dev,
			     device_t child, int type, int rid);

int			 bhnd_pci_release_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct resource *res);

int			 bhnd_pci_adjust_resource(device_t dev,
			     device_t child, int type, struct resource *res,
			     u_long start, u_long end);


int			 bhnd_pci_activate_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct resource *r);

int			 bhnd_pci_deactivate_resource(device_t dev,
			     device_t child, int type, int rid,
			     struct resource *r);

/**
 * Simple declaration of a bhnd-based PCI device driver.
 * 
 * @param dname The device/driver name (e.g. `bcma_pci`)
 * @param dclass The device class name (e.g. `bcma`)
 */
#define	BHND_PCI_DECLARE_DRIVER(dname,dclass) \
static device_method_t __CONCAT(dname,_methods)[] = { \
	/* Device interface */ \
	DEVMETHOD(device_probe,			__CONCAT(dname,_probe)),	\
	DEVMETHOD(device_attach,		__CONCAT(dname,_attach)),	\
	DEVMETHOD(device_detach,		__CONCAT(dname,_detach)),	\
	DEVMETHOD(device_shutdown,		bus_generic_shutdown),		\
	DEVMETHOD(device_suspend,		bhnd_pci_suspend),		\
	DEVMETHOD(device_resume,		bhnd_pci_resume),		\
										\
	/* Bus interface */							\
	DEVMETHOD(bus_print_child,		bhnd_generic_print_child),	\
	DEVMETHOD(bus_probe_nomatch,		bhnd_generic_probe_nomatch),	\
	DEVMETHOD(bus_read_ivar,		__CONCAT(dname,_read_ivar)),	\
	DEVMETHOD(bus_write_ivar,		__CONCAT(dname,_write_ivar)),	\
	DEVMETHOD(bus_child_deleted,		__CONCAT(dname,_child_deleted)),\
										\
	DEVMETHOD(bus_get_resource_list,	__CONCAT(dname,_get_resource_list)),\
	DEVMETHOD(bus_set_resource,		bhnd_pci_set_resource),	\
	DEVMETHOD(bus_get_resource,		bhnd_pci_get_resource),	\
	DEVMETHOD(bus_delete_resource,		bhnd_pci_delete_resource),	\
	DEVMETHOD(bus_alloc_resource,		bhnd_pci_alloc_resource),	\
	DEVMETHOD(bus_adjust_resource,		bhnd_pci_adjust_resource),	\
	DEVMETHOD(bus_release_resource,		bhnd_pci_release_resource),	\
	DEVMETHOD(bus_activate_resource,	bhnd_pci_activate_resource),	\
	DEVMETHOD(bus_deactivate_resource,	bhnd_pci_deactivate_resource),	\
										\
	/* BHND interface */							\
	DEVMETHOD(bhnd_get_rman,		__CONCAT(dname,_get_rman)),	\
	DEVMETHOD(bhnd_get_port_rid,		__CONCAT(dname,_get_port_rid)),	\
										\
	DEVMETHOD_END								\
}; \
\
static devclass_t __CONCAT(dclass,_devclass); \
\
DEFINE_CLASS_0(dclass, __CONCAT(dname,_driver), __CONCAT(dname,_methods), sizeof(struct __CONCAT(dname,_softc))); \
DRIVER_MODULE(dname, pci, __CONCAT(dname,_driver), __CONCAT(dclass,_devclass), NULL, NULL); \
MODULE_VERSION(dname, 1);		\
MODULE_DEPEND(dname, pci, 1, 1, 1);	\
MODULE_DEPEND(dname, bhnd, 1, 1, 1);

#endif /* _BHND_BHND_PCI_H_ */