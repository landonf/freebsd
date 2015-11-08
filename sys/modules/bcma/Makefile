# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bcma

KMOD=	bcma
SRCS=	bcma.c bcma_subr.c bcmareg.h bcmavar.h bcma_private.h \
	bcmab_pci.c bcmab_pcivar.h \
	bcma_erom.c bcma_dmp.h
SRCS+=	device_if.h bus_if.h bhnd_if.h bhndb_pci_if.h

.include <bsd.kmod.mk>
