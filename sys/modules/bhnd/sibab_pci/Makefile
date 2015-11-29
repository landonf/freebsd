# $FreeBSD$

.PATH:	${.CURDIR}/../../../dev/bhnd/siba

KMOD=	sibab_pci
SRCS=	sibab_pci.c

SRCS+=	bhnd_if.h bhndb_if.h bhndb_bus_if.h
SRCS+=	device_if.h bus_if.h

.include <bsd.kmod.mk>
