# $FreeBSD$

.PATH: ${.CURDIR}/../../../../dev/bhnd/cores/pci

KMOD=	bhnd_pci_hostb
SRCS=	bhnd_pci_hostb.c
SRCS+=	bhnd_nvram_map.h
SRCS+=	device_if.h bus_if.h pci_if.h \
	bhnd_bus_if.h

.include <bsd.kmod.mk>
