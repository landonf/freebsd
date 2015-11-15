# $FreeBSD$

.PATH:	${.CURDIR}/../../../dev/bhnd/bcma

KMOD=	bcmab_pci
SRCS=	bcmab_pci.c

SRCS+=	bhnd_if.h bhndb_if.h bhndb_bus_if.h
SRCS+=	device_if.h bus_if.h bhnd_if.h

.include <bsd.kmod.mk>
