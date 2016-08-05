# $FreeBSD$

.PATH: ${.CURDIR}/../../../dev/bhnd/nvram

KMOD=	bhnd_nvram
SRCS=	bhnd_sprom.c \
	bhnd_sprom_subr.c \
	nvram_subr.c
SRCS+=	bhnd_nvram_map_data.h
SRCS+=	device_if.h bus_if.h bhnd_bus_if.h \
	bhnd_chipc_if.h bhnd_nvram_if.h

.include <bsd.kmod.mk>
