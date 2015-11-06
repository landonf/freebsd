# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bcma

KMOD=	bcma
SRCS=	bcma.c bcmareg.h bcmavar.h bcma_private.h \
	bcmab_pci.c bcmab_pcivar.h \
	bcma_erom.c bcma_dmp.h
SRCS+=	device_if.h bus_if.h bhndbus_if.h bhndb_pci_if.h

MFILES=	kern/bus_if.m kern/device_if.m \
	dev/bhnd/bhndbus_if.m dev/bhnd/bhndb_pci_if.m

.include <bsd.kmod.mk>
