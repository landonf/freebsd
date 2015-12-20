# $FreeBSD$

.PATH: ${.CURDIR}/../../../../dev/bhnd/cores

KMOD=	bhnd_pci
SRCS=	bhnd_pci.c \
	bhnd_mdio_pcie.c
SRCS+=	device_if.h bus_if.h bhnd_if.h \
	mdio_if.h

.include <bsd.kmod.mk>
