# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bcma

KMOD=	bcma
SRCS=	bcma.c bcma_erom.c bcma_pci.c \
	bcmareg.h bcmavar.h bcma_dmp.h
SRCS+=	device_if.h bus_if.h pci_if.h bhndbus_if.h

MFILES=	kern/bus_if.m kern/device_if.m \
	dev/pci/pci_if.m dev/bhnd/bhndbus_if.m

.include <bsd.kmod.mk>
