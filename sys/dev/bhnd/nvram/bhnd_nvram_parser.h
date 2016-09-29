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

#ifndef _BHND_NVRAM_BHND_NVRAM_PARSER_H_
#define _BHND_NVRAM_BHND_NVRAM_PARSER_H_

#include <sys/types.h>

#include "bhnd_nvram.h"
#include "bhnd_nvram_io.h"

/* NVRAM parser class */
typedef struct bhnd_nvram_parser_class bhnd_nvram_parser_class_t;

/* NVRAM parser instance */
struct bhnd_nvram_parser;

/** Declare a bhnd_nvram_parser_class with name @p _n */
#define	BHND_NVRAM_PARSER_DECL(_n) \
	extern 	struct bhnd_nvram_parser_class bhnd_nvram_ ## _n ##_class

BHND_NVRAM_PARSER_DECL(bcm);
BHND_NVRAM_PARSER_DECL(tlv);
BHND_NVRAM_PARSER_DECL(btxt);

int		 bhnd_nvram_parser_probe(bhnd_nvram_parser_class_t *cls,
		     struct bhnd_nvram_io *io);

int		 bhnd_nvram_parser_new(bhnd_nvram_parser_class_t *cls,
		     struct bhnd_nvram_parser **nv, struct bhnd_nvram_io *io);

void		 bhnd_nvram_parser_free(struct bhnd_nvram_parser *nv);

const char	*bhnd_nvram_parser_next(struct bhnd_nvram_parser *nv,
		     bhnd_nvram_type *type, size_t *len, void **cookiep);

#endif /* _BHND_NVRAM_BHND_NVRAM_PARSER_H_ */
