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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/nv.h>

#include "bhnd_nvram_common.h"
#include "bhnd_nvram_data.h"
#include "bhnd_nvram_io.h"

union bhnd_nvram_ident;

struct bhnd_nvram_idx;
struct bhnd_nvram_ops;
struct bhnd_nvram_devpath;

struct bhnd_nvram;

LIST_HEAD(bhnd_nvram_devpaths, bhnd_nvram_devpath);

int	bhnd_nvram_parser_identify(struct bhnd_nvram_io *io,
	    bhnd_nvram_format expected, size_t *size_hint);
int	bhnd_nvram_parser_init(struct bhnd_nvram *sc, device_t owner,
	    struct bhnd_nvram_io *io, bhnd_nvram_format fmt);
void	bhnd_nvram_parser_fini(struct bhnd_nvram *sc);

int	bhnd_nvram_parser_getvar(struct bhnd_nvram *sc, const char *name,
	    void *buf, size_t *len, bhnd_nvram_type type);
int	bhnd_nvram_parser_setvar(struct bhnd_nvram *sc, const char *name,
	    const void *buf, size_t len, bhnd_nvram_type type);

/** bhnd nvram parser instance state */
struct bhnd_nvram {
	device_t			 dev;		/**< parent device, or NULL */
	struct bhnd_nvram_data		*nv;		/**< backing data */

	struct bhnd_nvram_idx		*idx;		/**< key index */

	struct bhnd_nvram_devpaths	 devpaths;	/**< device paths */
	nvlist_t			*pending;	/**< uncommitted writes */
};

#endif /* _BHND_NVRAM_BHND_NVRAM_PARSER_H_ */
