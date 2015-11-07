# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhnd
SRCS=	bhnd.c bhnd_devinfo.c bhndreg.h bhndvar.h bhnd_if.h \
	bhndb_pci.c bhndb_pcivar.h bhnd_pcireg.h bhndb_pci_if.h \
	bhnd_devids.h bhnd_core.h
SRCS+=	device_if.h bus_if.h

SUBDIR=	cores

.include <bsd.kmod.mk>
.include <bsd.subdir.mk>
