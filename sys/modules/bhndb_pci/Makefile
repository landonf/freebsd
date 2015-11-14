# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd

KMOD=	bhndb_pci
SRCS=	bhndb_pci.c bhndb_pcivar.h cores/bhnd_pcireg.h \
	bhnd_if.h bhndb_bus_if.h bhndb_hw_if.h

SRCS+=	device_if.h bus_if.h

.include <bsd.kmod.mk>
