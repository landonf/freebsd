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

#include "bhnd_nvram_data.h"
#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_tlvreg.h"

/*
 * CFE TLV NVRAM data class.
 * 
 * The CFE-defined TLV NVRAM format is used on the WGT634U.
 */

struct bhnd_nvram_tlv {
	struct bhnd_nvram_data	 nv;	/**< common instance state */
	struct bhnd_nvram_io	*data;	/**< backing buffer */
};

BHND_NVRAM_DATA_CLASS_DEFN(tlv, "WGT634U")

/** Minimal identification header */
struct bhnd_nvram_tlv_ident {
	uint8_t		tag;
	uint8_t		size;
	uint8_t		flags;
	char		envp;
} __packed;

#define	TLV_NVLOG(_fmt, ...)	\
	printf("%s: " _fmt, __FUNCTION__, ##__VA_ARGS__)

static int
bhnd_nvram_tlv_probe(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_tlv_ident	ident;
	size_t				nbytes;
	int				error;

	nbytes = bhnd_nvram_io_getsize(io);

	/* Handle what might be an empty TLV image */
	if (nbytes < sizeof(ident)) {
		/* Fetch just the first tag */
		error = bhnd_nvram_io_read(io, 0x0, &ident, sizeof(ident.tag));
		if (error)
			return (error);

		/* This *could* be an empty TLV image, but all we're
		 * testing for here is a single 0x0 byte followed by EOF */
		if (nbytes == 1 && ident.tag == NVRAM_TLV_TYPE_END)
			return (BUS_PROBE_LOW_PRIORITY);
		
		return (ENXIO);
	}

	/* Otherwise, look at the initial header for a valid TLV ENV tag */
	if ((error = bhnd_nvram_io_read(io, 0x0, &ident, sizeof(ident))))
		return (error);

	/* First entry should be a variable record (which we statically
	 * assert as being defined to use a single byte size field) */
	if (ident.tag != NVRAM_TLV_TYPE_ENV)
		return (ENXIO);

	_Static_assert(NVRAM_TLV_TYPE_ENV & NVRAM_TLV_TF_U8_LEN,
	    "TYPE_ENV is not a U8-sized field");

	/* The entry must be at least 3 characters ('x=\0') in length */
	if (ident.size < 3)
		return (ENXIO);

	/* The first character should be a valid key char (alpha) */
	if (!isalpha(ident.envp))
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

/**
 * Initialize @p tlv with the provided NVRAM TLV data mapped by @p src.
 * 
 * @param tlv A newly allocated data instance.
 */
static int
bhnd_nvram_tlv_init(struct bhnd_nvram_tlv *tlv, struct bhnd_nvram_io *src)
{
	struct bhnd_nvram_io	*io;
	const void		*ptr;
	const uint8_t		*start;
	size_t			 offset, size;
	int			 error;

	KASSERT(tlv->data == NULL, ("tlv data already initialized"));
	
	if ((io = bhnd_nvram_iobuf_copy(src)) == NULL)
		return (ENOMEM);

	tlv->data = io;

	/* Parse the TLV input data, initializing our backing
	 * data representation */
	io = bhnd_nvram_iobuf_copy(src);
	size = bhnd_nvram_io_getsize(io);

	/* Fetch a pointer to the full mapped range */
	error = bhnd_nvram_io_read_ptr(io, 0x0, &ptr, size, NULL);
	if (error) {
		TLV_NVLOG("error reading mapped TLV data: %d\n", error);
		return (error);
	}

	start = ptr;
	offset = 0;
	while (offset < size) {
		size_t	rlen, name_len;
		uint8_t	type;

		/* Fetch type */
		type = *(start + offset);

		/* EOF */
		if (type == NVRAM_TLV_TYPE_END) {
			offset++;
			break;
		}

		if ((offset++) == size) {
			TLV_NVLOG("TLV record header truncated at %#zx\n",
			    offset);
			return (EINVAL);
		}

		/* Determine record length */
		if (type & NVRAM_TLV_TF_U8_LEN) {
			rlen = *(start + offset);
		} else {
			rlen = *(start + offset) << 8;
			if ((offset++) == size) {
				TLV_NVLOG("TLV 16-bit record length truncated "
				    "at %#zx\n", offset);
				return (EINVAL);
			}

			rlen |= *(start + offset);
		}

		/* Advance to record value */
		if ((offset++) >= size) {
			TLV_NVLOG("TLV record value truncated at %#zx\n",
			    offset);
			return (EINVAL);
		}

		/* Verify that the full record value is mapped */
		if (rlen > size || size - rlen < offset) {
			TLV_NVLOG("TLV record length %zu truncated by input "
			    "size of %zu\n", rlen, size);
			return (EINVAL);
		}
	
		/* Remaining processing applies only to env records */
		if (type != NVRAM_TLV_TYPE_ENV) {
			offset += rlen;
			continue;
		}

		/* Skip flag field */
		if (rlen >= 1) {
			offset++;
			rlen--;
		} else {
			TLV_NVLOG("invalid zero-length TLV_ENV record value "
			    "at %#zx\n", offset);
			return (EINVAL);
		}

		/* Parse the key=value string */
		error = bhnd_nvram_parse_env(start + offset, rlen, '=', NULL,
		    &name_len, NULL, NULL);
		if (error) {
			TLV_NVLOG("error parsing TLV_ENV data: %d\n", error);
			return (error);
		}

		/* Insert a '\0' character, replacing the '=' delimiter and
		 * allowing us to vend references directly to the variable
		 * name */
		error = bhnd_nvram_io_write(io, offset + name_len,
		    &(char){'\0'}, 1);
		if (error) {
			TLV_NVLOG("error updating TLV_ENV: %d\n", error);
			return (error);
		}

		/* Advance to next entry */
		offset += rlen;
	}

	// TODO - shrink buffer capacity to `offset`
	return (0);
}

static int
bhnd_nvram_tlv_new(struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io)
{
	
	struct bhnd_nvram_tlv	*tlv;
	int			 error;

	/* Allocate and initialize the TLV data instance */
	tlv = malloc(sizeof(*tlv), M_BHND_NVRAM, M_NOWAIT|M_ZERO);
	if (tlv == NULL)
		return (ENOMEM);

	tlv->nv.cls = &bhnd_nvram_tlv_class;
	tlv->data = NULL;

	/* Parse the BTXT input data and initialize our backing
	 * data representation */
	if ((error = bhnd_nvram_tlv_init(tlv, io)))
		goto failed;

	*nv = &tlv->nv;
	return (0);

failed:
	if (tlv->data != NULL)
		bhnd_nvram_io_free(tlv->data);

	free(tlv, M_BHND_NVRAM);

	return (error);
}

static void
bhnd_nvram_tlv_free(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_tlv *tlv = (struct bhnd_nvram_tlv *)nv;

	bhnd_nvram_io_free(tlv->data);
	free(tlv, M_BHND_NVRAM);
}

static uint32_t
bhnd_nvram_tlv_getcaps(struct bhnd_nvram_data *nv)
{
	return (BHND_NVRAM_DATA_CAP_READ_PTR);
}

static const char *
bhnd_nvram_tlv_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	// TODO
	return (NULL);
}

static void *
bhnd_nvram_tlv_find(struct bhnd_nvram_data *nv, const char *name)
{
	return (bhnd_nvram_data_generic_find(nv, name));
}

static int
bhnd_nvram_tlv_getvar(struct bhnd_nvram_data *nv, void *cookiep, void *buf,
    size_t *len, bhnd_nvram_type type)
{
	// TODO
	return (ENXIO);
}

static const void *
bhnd_nvram_tlv_getvar_ptr(struct bhnd_nvram_data *nv, void *cookiep,
    size_t *len, bhnd_nvram_type *type)
{
	// TODO
	return (NULL);
}

static const char *
bhnd_nvram_tlv_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	// TODO
	return (NULL);
}
