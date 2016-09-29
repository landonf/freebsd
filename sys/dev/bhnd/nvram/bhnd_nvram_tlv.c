/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/ctype.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_parser.h"
#include "bhnd_nvram_parservar.h"

#include "bhnd_nvram_tlvreg.h"

/*
 * CFE TLV parser.
 * 
 * The CFE-defined TLV NVRAM format is used on the WGT634U.
 */

struct bhnd_nvram_tlv {
	struct bhnd_nvram_parser	 nv;	/**< common instance state */
	struct bhnd_nvram_io		*data;	/**< backing buffer */
};

BHND_NVRAM_PARSER_DEFN(tlv)

/** Minimal identification header */
struct bhnd_nvram_tlv_ident {
	uint8_t		tag;
	uint8_t		size;
	uint8_t		flags;
	char		envp;
} __packed;

static int
bhnd_nvram_tlv_probe(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_tlv_ident	ident;
	size_t				nbytes;
	int				error;

	/* Look at the initial header for a valid TLV ENV tag */
	nbytes = sizeof(ident);
	if ((error = bhnd_nvram_io_read(io, 0x0, &ident, &nbytes)))
		return (error);

	if (nbytes < sizeof(ident)) {
		/* This *could* be an empty TLV image, but all we're
		 * testing for here is a single 0x0 byte followed by EOF */
		if (nbytes == 1 && ident.tag == NVRAM_TLV_TYPE_END)
			return (BUS_PROBE_LOW_PRIORITY);

		return (ENXIO);
	}

	/* First entry should be a variable record (which we statically
	 * assert as being defined to use a single byte size field) */
	if (ident.tag != NVRAM_TLV_TYPE_ENV)
		return (ENXIO);

	_Static_assert(NVRAM_TLV_TYPE_ENV & NVRAM_TLV_TF_U8_LEN,
	    "TYPE_ENV is not a U8-sized field");

	/* The entry must be at least 3 characters ('x=\0') in length */
	if (ident.size != 3)
		return (ENXIO);

	/* The first character should be a valid key char (alpha) */
	if (!isalpha(ident.envp))
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

static int
bhnd_nvram_tlv_new(struct bhnd_nvram_parser **nv,
    struct bhnd_nvram_io *io)
{
	// TODO
	return (ENXIO);
}

static void
bhnd_nvram_tlv_free(struct bhnd_nvram_parser *nv)
{
	struct bhnd_nvram_tlv *tlv = (struct bhnd_nvram_tlv *)nv;

	bhnd_nvram_io_free(tlv->data);
	free(tlv, M_BHND_NVRAM);
}

static const char *
bhnd_nvram_tlv_next(struct bhnd_nvram_parser *nv, bhnd_nvram_type *type,
    size_t *len, void **cookiep)
{
	// TODO
	return (NULL);
}
