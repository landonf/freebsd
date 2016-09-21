/*-
 * Copyright (c) 2016 Landon J. Fuller <landonf@FreeBSD.org>.
 * Copyright (c) 2007 Bruce M. Simpson.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _IF_BWN_SIBA_COMPAT_H_
#define _IF_BWN_SIBA_COMPAT_H_

#define	BWN_USE_SIBA	0
#include "if_bwn_siba.h"

#include "if_bwnvar.h"

/**
 * Compatiblity shim state.
 */
struct bwn_bhnd_ctx {
	// TODO
};

static inline struct bwn_bhnd_ctx *
bwn_bhnd_get_ctx(device_t dev)
{
	struct bwn_softc *sc = device_get_softc(dev);
	return (sc->sc_bus_ctx);
}

/**
 * Fetch and return an NVRAM variable via bhnd_nvram_getvar_*(), or return
 * the given default value if the NVRAM variable is unavailable.
 */
#define	BWN_BHND_NVRAM_RETURN_VAR(_dev, _type, _name, _default)		\
do {									\
	_type ## _t	value;						\
	int		error;						\
									\
	error = bhnd_nvram_getvar_ ## _type(_dev, _name, &value);	\
	if (error) {							\
		device_printf(_dev,					\
		    "error reading NVRAM variable '%s': %d\n", _name,	\
		    error);						\
		return (_default);					\
	}								\
									\
	return (value);							\
} while(0)

#endif /* _IF_BWN_SIBA_COMPAT_H_ */
