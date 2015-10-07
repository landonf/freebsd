# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd_pci.c bhnd_if.h bhndreg.h bhndvar.h #TODO: Temporarily hard-coded main module as `bhnd_pci`
SRCS+=	device_if.h bus_if.h pci_if.h

MFILES=	kern/bus_if.m kern/device_if.m dev/pci/pci_if.m \
		dev/bhnd/bhnd_if.m

.include <bsd.kmod.mk>
