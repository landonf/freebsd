# $FreeBSD$

.PATH: ${.CURDIR}/../../../dev/bhnd/gpio

KMOD=	bhnd_gpio
SRCS=	bhnd_gpio.c
SRCS+=	bhnd_bus_if.h

SRCS+=	device_if.h bus_if.h gpio_if.h

.include <bsd.kmod.mk>
