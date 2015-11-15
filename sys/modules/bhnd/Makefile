# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhnd_subr.c bhndvar.h bhnd_private.h \
	bhnd_if.c bhnd_if.h

SRCS+=	bhndb.c \
	bhndb_bus_if.c bhndb_bus_if.h \
	bhndb_if.c bhndb_if.h

SRCS+=	bhnd_ids.h bhnd_core.h

SRCS+=	device_if.h bus_if.h

SUBDIR=	cores

.include <bsd.kmod.mk>
.include <bsd.subdir.mk>
