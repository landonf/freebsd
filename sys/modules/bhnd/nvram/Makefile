# $FreeBSD$

.PATH: ${.CURDIR}/../../../dev/bhnd/nvram

KMOD=	bhnd_nvram
SRCS=	bhnd_nvram_if.c bhnd_nvram_if.h
SRCS+=	bhnd_nvram_map.h

SRCS+=	device_if.h bus_if.h

bhnd_nvram_map.h: nvram_map
	${AWK} -f ${SYSDIR}/dev/bhnd/tools/nvram_map_gen.awk ${.ALLSRC} > ${.TARGET}

CLEANFILES+=	bhnd_nvram_map.h

.include <bsd.kmod.mk>
