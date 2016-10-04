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
#include <sys/endian.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_data.h"
#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_bcmreg.h"

/*
 * Broadcom NVRAM data class.
 * 
 * The Broadcom NVRAM NUL-delimited ASCII format is used by most
 * Broadcom SoCs.
 */

struct bhnd_nvram_bcm {
	struct bhnd_nvram_data	 nv;	/**< common instance state */
	struct bhnd_nvram_io	*data;	/**< backing buffer */
};

BHND_NVRAM_DATA_CLASS_DEFN(bcm, "Broadcom")

#define	BCM_NVLOG(_fmt, ...)	\
	printf("%s: " _fmt, __FUNCTION__, ##__VA_ARGS__)

static int
bhnd_nvram_bcm_probe(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_header	hdr;
	int				error;

	if ((error = bhnd_nvram_io_read(io, 0x0, &hdr, sizeof(hdr))))
		return (error);

	if (le32toh(hdr.magic) != BCM_NVRAM_MAGIC)
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

/**
 * Initialize @p bcm with the provided NVRAM data mapped by @p src.
 * 
 * @param bcm A newly allocated data instance.
 */
static int
bhnd_nvram_bcm_init(struct bhnd_nvram_bcm *bcm, struct bhnd_nvram_io *src)
{
	struct bhnd_nvram_header	 hdr;
	uint8_t				*p;
	void				*ptr;
	size_t				 io_size;
	uint8_t				 crc, valid;
	int				 error;

	if ((error = bhnd_nvram_io_read(src, 0x0, &hdr, sizeof(hdr))))
		return (error);

	if (le32toh(hdr.magic) != BCM_NVRAM_MAGIC)
		return (ENXIO);

	/* Fetch the actual NVRAM image size */
	io_size = le32toh(hdr.size);

	/* Copy out the NVRAM image */
	bcm->data = bhnd_nvram_iobuf_copy_range(src, 0x0, io_size);
	if (bcm->data == NULL)
		return (ENOMEM);

	/* Fetch a non-const pointer to the NVRAM image buffer */
	error = bhnd_nvram_io_write_ptr(bcm->data, 0x0, &ptr, io_size, NULL);
	if (error)
		return (error);

	p = ptr;

	/* Verify the CRC */
	valid = BCM_NVRAM_GET_BITS(hdr.cfg0, BCM_NVRAM_CFG0_CRC);
	crc = bhnd_nvram_crc8(p + BCM_NVRAM_CRC_SKIP,
	    io_size - BCM_NVRAM_CRC_SKIP, BHND_NVRAM_CRC8_INITIAL);

	if (crc != valid) {
		BCM_NVLOG("warning: NVRAM CRC error (crc=%#hhx, "
		    "expected=%hhx)\n", crc, valid);
	}

	/* Process the buffer */
	// TODO

	return (0);
}

static int
bhnd_nvram_bcm_new(struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_bcm	*bcm;
	int			 error;

	/* Allocate and initialize the BCM data instance */
	bcm = malloc(sizeof(*bcm), M_BHND_NVRAM, M_NOWAIT|M_ZERO);
	if (bcm == NULL)
		return (ENOMEM);

	bcm->nv.cls = &bhnd_nvram_bcm_class;
	bcm->data = NULL;

	/* Parse the BTXT input data and initialize our backing
	 * data representation */
	if ((error = bhnd_nvram_bcm_init(bcm, io)))
		goto failed;

	*nv = &bcm->nv;
	return (0);

failed:
	if (bcm->data != NULL)
		bhnd_nvram_io_free(bcm->data);

	free(bcm, M_BHND_NVRAM);

	return (error);
}

static void
bhnd_nvram_bcm_free(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_bcm *bcm = (struct bhnd_nvram_bcm *)nv;

	bhnd_nvram_io_free(bcm->data);
	free(bcm, M_BHND_NVRAM);
}

static uint32_t
bhnd_nvram_bcm_getcaps(struct bhnd_nvram_data *nv)
{
	return (BHND_NVRAM_DATA_CAP_READ_PTR);
}

static const char *
bhnd_nvram_bcm_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	// TODO
	return (NULL);
}

static void *
bhnd_nvram_bcm_find(struct bhnd_nvram_data *nv, const char *name)
{
	return (bhnd_nvram_data_generic_find(nv, name));
}

static int
bhnd_nvram_bcm_getvar(struct bhnd_nvram_data *nv, void *cookiep, void *buf,
    size_t *len, bhnd_nvram_type type)
{
	const void	*vptr;
	size_t		 vlen;
	bhnd_nvram_type	 vtype;

	/* Fetch pointer */
	vptr = bhnd_nvram_data_getvar_ptr(nv, cookiep, &vlen, &vtype);
	if (vptr == NULL)
		return (EINVAL);

	/* Attempt value type coercion */
	return (bhnd_nvram_coerce_value(buf, len, type, vptr, vlen, vtype,
	    NULL));
}

static const void *
bhnd_nvram_bcm_getvar_ptr(struct bhnd_nvram_data *nv, void *cookiep,
    size_t *len, bhnd_nvram_type *type)
{
	// TODO
	return (NULL);
}

static const char *
bhnd_nvram_bcm_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	// TODO
	return (NULL);
}
