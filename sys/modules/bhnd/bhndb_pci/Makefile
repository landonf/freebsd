# $FreeBSD$

.PATH: ${.CURDIR}/../../../dev/bhnd/bridge

KMOD=	bhndb_pci
SRCS=	bhndb_pci.c
SRCS+=	bhnd_if.h bhndb_bus_if.h bhndb_if.h

SRCS+=	device_if.h bus_if.h

.include <bsd.kmod.mk>
