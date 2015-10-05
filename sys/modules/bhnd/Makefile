# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhndreg.h bhndvar.h
SRCS+=	device_if.h bus_if.h pci_if.h

MFILES=	dev/bhnd/bhnd_if.m

.include <bsd.kmod.mk>
