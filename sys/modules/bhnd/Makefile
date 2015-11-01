# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhndreg.h bhndvar.h bhnd_if.h \
	bhnd_pci.c bhnd_pcireg.h \
	bhnd_device_ids.h bhnd_core.h \
SRCS+=	device_if.h bus_if.h

MFILES=	kern/bus_if.m kern/device_if.m dev/bhnd/bhnd_if.m

.include <bsd.kmod.mk>
