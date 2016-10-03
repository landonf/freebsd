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

#ifndef _BHND_NVRAM_BHND_NVRAM_DATA_H_
#define _BHND_NVRAM_BHND_NVRAM_DATA_H_

#include <sys/types.h>

#include "bhnd_nvram.h"
#include "bhnd_nvram_io.h"

/* NVRAM data class */
typedef struct bhnd_nvram_data_class bhnd_nvram_data_class_t;

/* NVRAM data instance */
struct bhnd_nvram_data;

/** Declare a bhnd_nvram_data_class with name @p _n */
#define	BHND_NVRAM_DATA_CLASS_DECL(_n) \
	extern 	struct bhnd_nvram_data_class bhnd_nvram_ ## _n ## _class

BHND_NVRAM_DATA_CLASS_DECL(bcm);
BHND_NVRAM_DATA_CLASS_DECL(tlv);
BHND_NVRAM_DATA_CLASS_DECL(btxt);
BHND_NVRAM_DATA_CLASS_DECL(sprom);

/** bhnd_nvram_data capabilities */
enum {
	/** Supports effecient lookup of variables by name */
	BHND_NVRAM_DATA_CAP_INDEXED	= (1<<0),

	/** Supports direct access to backing buffer */
	BHND_NVRAM_DATA_CAP_READ_PTR	= (1<<1)
};

int		 bhnd_nvram_data_probe(bhnd_nvram_data_class_t *cls,
		     struct bhnd_nvram_io *io);

int		 bhnd_nvram_data_new(bhnd_nvram_data_class_t *cls,
		     struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io);

void		 bhnd_nvram_data_free(struct bhnd_nvram_data *nv);

uint32_t	 bhnd_nvram_data_getcaps(struct bhnd_nvram_data *nv);

const char	*bhnd_nvram_data_next(struct bhnd_nvram_data *nv,
		     void **cookiep);

void		*bhnd_nvram_data_find(struct bhnd_nvram_data *nv,
		     const char *name);

int		 bhnd_nvram_data_getvar(struct bhnd_nvram_data *nv,
		     void *cookiep, void *buf, size_t *len,
		     bhnd_nvram_type type);

const void	*bhnd_nvram_data_getvar_ptr(struct bhnd_nvram_data *nv,
		     void *cookiep, size_t *len, bhnd_nvram_type *type);

const char	*bhnd_nvram_data_getvar_name(struct bhnd_nvram_data *nv,
		     void *cookiep);

#endif /* _BHND_NVRAM_BHND_NVRAM_DATA_H_ */
