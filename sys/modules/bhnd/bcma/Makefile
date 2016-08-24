# $FreeBSD$

.PATH:	${.CURDIR}/../../../dev/bhnd/bcma

KMOD=	bcma
SRCS=	bcma.c bcma_subr.c bcma_erom.c \
	bcma_pl367.c

SRCS+=	device_if.h bus_if.h bhnd_bus_if.h

.include <bsd.kmod.mk>
