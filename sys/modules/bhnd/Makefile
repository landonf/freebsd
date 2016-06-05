# $FreeBSD$

.PATH: ${.CURDIR}/../../dev/bhnd
.PATH: ${.CURDIR}/../../dev/bhnd/nvram
.PATH: ${.CURDIR}/../../dev/bhnd/pmu

KMOD=	bhnd
SRCS=	bhnd.c bhnd_subr.c
SRCS+=	bhnd_bus_if.c bhnd_bus_if.h

SRCS+=	bhnd_chipc_if.c bhnd_chipc_if.h

SRCS+=	bhnd_pmu.c
SRCS+=	bhnd_pmu_if.c bhnd_pmu_if.h

SRCS+=	nvram_subr.c
SRCS+=	bhnd_sprom.c bhnd_sprom_subr.c
SRCS+=	bhnd_nvram_map.h bhnd_nvram_map_data.h
SRCS+=	bhnd_nvram_if.c bhnd_nvram_if.h

SRCS+=	device_if.h bus_if.h

SUBDIR=	bcma \
	bcma_bhndb \
	bhndb \
	bhndb_pci \
	cores \
	siba \
	siba_bhndb

.include <bsd.kmod.mk>
.include <bsd.subdir.mk>
