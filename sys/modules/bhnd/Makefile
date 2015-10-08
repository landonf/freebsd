# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhnd_if.h bhndreg.h bhndvar.h
SRCS+=	device_if.h bus_if.h

MFILES=	kern/bus_if.m kern/device_if.m \
		dev/bhnd/bhnd_if.m

.include <bsd.kmod.mk>
