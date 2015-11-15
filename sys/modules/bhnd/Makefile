# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhnd_subr.c \
	bhnd_if.c bhnd_if.h

SRCS+=	device_if.h bus_if.h

SUBDIR=	bcma \
	bcmab_pci \
	bhndb \
	bhndb_pci \
	cores

.include <bsd.kmod.mk>
.include <bsd.subdir.mk>
