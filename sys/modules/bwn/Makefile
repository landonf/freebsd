# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bwn

KMOD=	if_bwn
SRCS=	if_bwn.c if_bwnreg.h if_bwnvar.h \
	if_bwn_pci.c
SRCS+=	device_if.h bus_if.h pci_if.h \
	bhnd_if.h bhndb_pci_if.h

.include <bsd.kmod.mk>

# XXX Work around clang warning, until maintainer approves fix.
CWARNFLAGS.if_bwn.c=	${NO_WSOMETIMES_UNINITIALIZED}
