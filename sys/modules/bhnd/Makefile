# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhnd_subr.c bhndvar.h bhnd_private.h \
	bhndb_pci.c bhndb_pcivar.h cores/bhnd_pcireg.h \
	bhnd_ids.h bhnd_core.h

SRCS+=	bhnd_if.c bhnd_if.h
SRCS+=	bhndb_pci_if.c bhndb_pci_if.h

SRCS+=	device_if.h bus_if.h

SUBDIR=	cores

.include <bsd.kmod.mk>
.include <bsd.subdir.mk>
