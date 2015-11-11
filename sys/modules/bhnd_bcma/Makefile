# $FreeBSD$

.PATH:	${.CURDIR}/../../dev/bhnd/bcma \
	${.CURDIR}/../../dev/bhnd/cores

KMOD=	bhnd_bcma
SRCS=	bcma.c bcma_subr.c bcmavar.h bcma_private.h \
	bcma_erom.c bcma_dmp.h

SRCS+=	bhnd_chipcreg.h
SRCS+=	device_if.h bus_if.h bhnd_if.h

.include <bsd.kmod.mk>
