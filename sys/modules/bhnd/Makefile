# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhnd_devinfo.c bhndreg.h bhndvar.h bhndbus_if.h \
	bhnd_pci.c bhnd_pcireg.h \
	bhnd_devids.h bhnd_core.h
SRCS+=	device_if.h bus_if.h

MFILES=	kern/bus_if.m kern/device_if.m dev/bhnd/bhndbus_if.m

.include <bsd.kmod.mk>
