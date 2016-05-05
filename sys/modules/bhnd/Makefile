# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd
.PATH: ${.CURDIR}/../../dev/bhnd/nvram

KMOD=	bhnd
SRCS=	bhnd.c bhnd_subr.c \
	bhnd_sprom.c nvram_subr.c \
	bhnd_nvram_map.h

SRCS+=	bhnd_bus_if.c bhnd_bus_if.h \
	bhnd_chipc_if.c bhnd_chipc_if.h \
	bhnd_nvram_if.c bhnd_nvram_if.h

SRCS+=	device_if.h bus_if.h

SUBDIR=	bcma \
	bcma_bhndb \
	bhndb \
	bhndb_pci \
	cores \
	siba \
	siba_bhndb

bhnd_nvram_map.h: nvram_map
	${SYSDIR}/dev/bhnd/tools/nvram_map_gen.sh ${.ALLSRC} > ${.TARGET}

CLEANFILES+=	bhnd_nvram_map.h

.include <bsd.kmod.mk>
.include <bsd.subdir.mk>
