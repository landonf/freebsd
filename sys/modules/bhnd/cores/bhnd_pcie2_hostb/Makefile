# $FreeBSD$

.PATH: ${.CURDIR}/../../../../dev/bhnd/cores/pcie2

KMOD=	bhnd_pcie2_hostb
SRCS=	bhnd_pcie2_hostb.c
SRCS+=	device_if.h bus_if.h pci_if.h \
	bhnd_bus_if.h

.include <bsd.kmod.mk>
