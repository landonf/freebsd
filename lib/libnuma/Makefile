# $FreeBSD$

PACKAGE=lib${LIB}

.include <src.opts.mk>

LIB=		numa
SHLIB_MAJOR=	1
SRCS=		libnuma.c
INCS=		numa.h
#MAN=		libnuma.3

#HAS_TESTS=
#SUBDIR.${MK_TESTS}+=	tests

.include <bsd.lib.mk>

CFLAGS+= -Wall
# TODO: remove
CFLAGS+= -Wno-unused-parameter
