# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bcma

KMOD=	bcma
SRCS=	bcma.c bcma_pci.c bcma.h bcmareg.h bcmavar.h \
	bcma_dmp.h
SRCS+=	device_if.h bus_if.h pci_if.h bhnd_if.h

MFILES=	kern/bus_if.m kern/device_if.m \
		dev/pci/pci_if.m dev/bhnd/bhnd_if.m

.include <bsd.kmod.mk>
