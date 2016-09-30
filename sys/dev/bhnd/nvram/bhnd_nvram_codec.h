/*-
 * Copyright (c) 2015-2016 Landon Fuller <landonf@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 * 
 * $FreeBSD$
 */

#ifndef _BHND_NVRAM_BHND_NVRAM_CODEC_H_
#define _BHND_NVRAM_BHND_NVRAM_CODEC_H_

#include <sys/types.h>

#include "bhnd_nvram.h"
#include "bhnd_nvram_io.h"

/* NVRAM parser class */
typedef struct bhnd_nvram_codec_class bhnd_nvram_codec_class_t;

/* NVRAM parser instance */
struct bhnd_nvram_codec;

/** Declare a bhnd_nvram_codec_class with name @p _n */
#define	BHND_NVRAM_CODEC_DECL(_n) \
	extern 	struct bhnd_nvram_codec_class bhnd_nvram_ ## _n ##_class

BHND_NVRAM_CODEC_DECL(bcm);
BHND_NVRAM_CODEC_DECL(tlv);
BHND_NVRAM_CODEC_DECL(btxt);

int		 bhnd_nvram_codec_probe(bhnd_nvram_codec_class_t *cls,
		     struct bhnd_nvram_io *io);

int		 bhnd_nvram_codec_new(bhnd_nvram_codec_class_t *cls,
		     struct bhnd_nvram_codec **nv, struct bhnd_nvram_io *io);

void		 bhnd_nvram_codec_free(struct bhnd_nvram_codec *nv);

const char	*bhnd_nvram_codec_next(struct bhnd_nvram_codec *nv,
		     void **cookiep);

int		 bhnd_nvram_codec_getvar(struct bhnd_nvram_codec *nvc,
		     void *cookiep, void *buf, size_t *len,
		     bhnd_nvram_type type);

const void	*bhnd_nvram_codec_getvar_ptr(struct bhnd_nvram_codec *nvc,
		     void *cookiep, size_t *len, bhnd_nvram_type *type);

#endif /* _BHND_NVRAM_BHND_NVRAM_CODEC_H_ */
