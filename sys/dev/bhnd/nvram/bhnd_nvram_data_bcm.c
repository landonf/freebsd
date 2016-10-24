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
#include <sys/endian.h>

#ifdef _KERNEL

#include <sys/bus.h>
#include <sys/ctype.h>
#include <sys/malloc.h>
#include <sys/systm.h>

#else /* !_KERNEL */

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#endif /* _KERNEL */

#include "bhnd_nvram_impl.h"
#include "bhnd_nvram_private.h"

#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_data_bcmreg.h"
#include "bhnd_nvram_data_bcmvar.h"

/*
 * Broadcom NVRAM data class.
 * 
 * The Broadcom NVRAM NUL-delimited ASCII format is used by most
 * Broadcom SoCs.
 * 
 * The NVRAM data is encoded as a standard header, followed by series of
 * NUL-terminated 'key=value' strings; the end of the stream is denoted
 * by a single extra NUL character.
 */

struct bhnd_nvram_bcm;

static struct bhnd_nvram_bcmdata	*bhnd_nvram_bcm_gethdrvar(
					     struct bhnd_nvram_bcm *bcm,
					     const char *name);
static struct bhnd_nvram_bcmdata	*bhnd_nvram_bcm_to_hdrvar(
					     struct bhnd_nvram_bcm *bcm,
					     void *cookiep);
static size_t				 bhnd_nvram_bcm_hdrvar_index(
					     struct bhnd_nvram_bcm *bcm,
					     struct bhnd_nvram_bcmdata *hdrvar);
/*
 * Set of BCM header values that are required to be mirrored in the
 * NVRAM data itself.
 *
 * If they're not included in the parsed NVRAM data, we need to vend the
 * header-parsed values with their appropriate keys, and add them in any
 * updates to the NVRAM data.
 *
 * If they're modified in NVRAM, we need to sync the changes with the
 * the NVRAM header values.
 */
static const struct bhnd_nvram_bcmdata bhnd_nvram_bcm_hvars[] = {
	{
		.name	= BCM_NVRAM_CFG0_SDRAM_INIT_VAR,
		.type	= BHND_NVRAM_TYPE_UINT16,
		.size	= sizeof(uint16_t)
	},
	{
		.name	= BCM_NVRAM_CFG1_SDRAM_CFG_VAR,
		.type	= BHND_NVRAM_TYPE_UINT16,
		.size	= sizeof(uint16_t)
	},
	{
		.name	= BCM_NVRAM_CFG1_SDRAM_REFRESH_VAR,
		.type	= BHND_NVRAM_TYPE_UINT16,
		.size	= sizeof(uint16_t)
	},
	{
		.name	= BCM_NVRAM_SDRAM_NCDL_VAR,
		.type	= BHND_NVRAM_TYPE_UINT32,
		.size	= sizeof(uint32_t)
	},
};

/** BCM NVRAM data class instance */
struct bhnd_nvram_bcm {
	struct bhnd_nvram_data		 nv;	/**< common instance state */
	struct bhnd_nvram_io		*data;	/**< backing buffer */

	/** BCM header values */
	struct bhnd_nvram_bcmdata	 hvars[nitems(bhnd_nvram_bcm_hvars)];

	size_t				 count;	/**< total variable count */
};

BHND_NVRAM_DATA_CLASS_DEFN(bcm, "Broadcom")

static int
bhnd_nvram_bcm_probe(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_header	hdr;
	int				error;

	if ((error = bhnd_nvram_io_read(io, 0x0, &hdr, sizeof(hdr))))
		return (error);

	if (le32toh(hdr.magic) != BCM_NVRAM_MAGIC)
		return (ENXIO);

	return (BHND_NVRAM_DATA_PROBE_DEFAULT);
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
	size_t				 io_offset, io_size;
	uint8_t				 crc, valid;
	int				 error;

	if ((error = bhnd_nvram_io_read(src, 0x0, &hdr, sizeof(hdr))))
		return (error);

	if (le32toh(hdr.magic) != BCM_NVRAM_MAGIC)
		return (ENXIO);

	/* Fetch the actual NVRAM image size */
	io_size = le32toh(hdr.size);
	if (io_size < sizeof(hdr)) {
		/* The header size must include the header itself */
		BCM_NVLOG("corrupt header size: %zu\n", io_size);
		return (EINVAL);
	}

	if (io_size > bhnd_nvram_io_getsize(src)) {
		BCM_NVLOG("header size %zu exceeds input size %zu\n",
		    io_size, bhnd_nvram_io_getsize(src));
		return (EINVAL);
	}

	/* Allocate a buffer large enough to hold the NVRAM image, and
	 * an extra EOF-signaling NUL (on the chance it's missing from the
	 * source data) */
	if (io_size == SIZE_MAX)
		return (ENOMEM);

	bcm->data = bhnd_nvram_iobuf_empty(io_size, io_size + 1);
	if (bcm->data == NULL)
		return (ENOMEM);

	/* Fetch a pointer into our backing buffer and copy in the
	 * NVRAM image. */
	error = bhnd_nvram_io_write_ptr(bcm->data, 0x0, &ptr, io_size, NULL);
	if (error)
		return (error);

	p = ptr;
	if ((error = bhnd_nvram_io_read(src, 0x0, p, io_size)))
		return (error);

	/* Verify the CRC */
	valid = BCM_NVRAM_GET_BITS(hdr.cfg0, BCM_NVRAM_CFG0_CRC);
	crc = bhnd_nvram_crc8(p + BCM_NVRAM_CRC_SKIP,
	    io_size - BCM_NVRAM_CRC_SKIP, BHND_NVRAM_CRC8_INITIAL);

	if (crc != valid) {
		BCM_NVLOG("warning: NVRAM CRC error (crc=%#hhx, "
		    "expected=%hhx)\n", crc, valid);
	}

	/* Populate header variable definitions */
#define	BCM_READ_HDR_VAR(_name, _dest, _swap) do {		\
	struct bhnd_nvram_bcmdata *data;				\
	data = bhnd_nvram_bcm_gethdrvar(bcm, _name ##_VAR);		\
	BHND_NV_ASSERT(data != NULL,						\
	    ("no such header variable: " __STRING(_name)));		\
									\
									\
	data->value. _dest = _swap(BCM_NVRAM_GET_BITS(			\
	    hdr. _name ## _FIELD, _name));				\
} while(0)

	BCM_READ_HDR_VAR(BCM_NVRAM_CFG0_SDRAM_INIT,	u16, le16toh);
	BCM_READ_HDR_VAR(BCM_NVRAM_CFG1_SDRAM_CFG,	u16, le16toh);
	BCM_READ_HDR_VAR(BCM_NVRAM_CFG1_SDRAM_REFRESH,	u16, le16toh);
	BCM_READ_HDR_VAR(BCM_NVRAM_SDRAM_NCDL,		u32, le32toh);

	_Static_assert(nitems(bcm->hvars) == 4, "missing initialization for"
	    "NVRAM header variable(s)");

#undef BCM_READ_HDR_VAR

	/* Process the buffer */
	bcm->count = 0;
	io_offset = sizeof(hdr);
	while (io_offset < io_size) {
		char		*envp;
		const char	*name, *value;
		size_t		 envp_len;
		size_t		 name_len, value_len;

		/* Parse the key=value string */
		envp = (char *) (p + io_offset);
		envp_len = strnlen(envp, io_size - io_offset);
		error = bhnd_nvram_parse_env(envp, envp_len, '=', &name,
					     &name_len, &value, &value_len);
		if (error) {
			BCM_NVLOG("error parsing envp at offset %#zx: %d\n",
			    io_offset, error);
			return (error);
		}

		/* Insert a '\0' character, replacing the '=' delimiter and
		 * allowing us to vend references directly to the variable
		 * name */
		*(envp + name_len) = '\0';

		/* Record any NVRAM variables that mirror our header variables.
		 * This is a brute-force search -- for the amount of data we're
		 * operating on, it shouldn't be an issue. */
		for (size_t i = 0; i < nitems(bcm->hvars); i++) {
			/* Already matched? */
			if (bcm->hvars[i].mirrored)
				continue;

			/* Name matches? */
			if ((strcmp(name, bcm->hvars[i].name)) != 0)
				continue;

			/* Mark as mirrored */
			bcm->hvars[i].mirrored = true;
		}

		/* Seek past the value's terminating '\0' */
		io_offset += envp_len;
		if (io_offset == io_size) {
			BCM_NVLOG("missing terminating NUL at offset %#zx\n",
			    io_offset);
			return (EINVAL);
		}

		if (*(p + io_offset) != '\0') {
			BCM_NVLOG("invalid terminator '%#hhx' at offset %#zx\n",
			    *(p + io_offset), io_offset);
			return (EINVAL);
		}

		/* Update variable count */
		bcm->count++;

		/* Seek to the next record */
		if (++io_offset == io_size) {
			char ch;
	
			/* Hit EOF without finding a terminating NUL
			 * byte; we need to grow our buffer and append
			 * it */
			io_size++;
			if ((error = bhnd_nvram_io_setsize(bcm->data, io_size)))
				return (error);

			/* Write NUL byte */
			ch = '\0';
			error = bhnd_nvram_io_write(bcm->data, io_size-1, &ch,
			    sizeof(ch));
			if (error)
				return (error);
		}

		/* Check for explicit EOF (encoded as a single empty NUL
		 * terminated string) */
		if (*(p + io_offset) == '\0')
			break;
	}

	/* Add any non-mirrored header variables to our count */
	for (size_t i = 0; i < nitems(bcm->hvars); i++) {
		/* Counted when the mirrored variable was parsed from backing
		 * data */
		if (bcm->hvars[i].mirrored)
			continue;

		bcm->count++;
	}

	return (0);
}

static int
bhnd_nvram_bcm_new(struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_bcm	*bcm;
	int			 error;

	/* Allocate and initialize the BCM data instance */
	bcm = bhnd_nv_calloc(1, sizeof(*bcm));
	if (bcm == NULL)
		return (ENOMEM);

	bcm->nv.cls = &bhnd_nvram_bcm_class;
	bcm->data = NULL;

	/* Populate default BCM mirrored header value set */
	_Static_assert(sizeof(bcm->hvars) == sizeof(bhnd_nvram_bcm_hvars),
	    "hvars declaration must match bhnd_nvram_bcm_hvars template");
	memcpy(bcm->hvars, bhnd_nvram_bcm_hvars, sizeof(bcm->hvars));

	/* Parse the BCM input data and initialize our backing
	 * data representation */
	if ((error = bhnd_nvram_bcm_init(bcm, io)))
		goto failed;

	*nv = &bcm->nv;
	return (0);

failed:
	if (bcm->data != NULL)
		bhnd_nvram_io_free(bcm->data);

	bhnd_nv_free(bcm);

	return (error);
}

static void
bhnd_nvram_bcm_free(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_bcm *bcm = (struct bhnd_nvram_bcm *)nv;

	bhnd_nvram_io_free(bcm->data);
	bhnd_nv_free(bcm);
}

size_t
bhnd_nvram_bcm_count(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_bcm *bcm = (struct bhnd_nvram_bcm *)nv;
	return (bcm->count);
}

static int
bhnd_nvram_bcm_size(struct bhnd_nvram_data *nv, size_t *size)
{
	struct bhnd_nvram_bcm	*bcm;
	int			 error;

	bcm = (struct bhnd_nvram_bcm *)nv;

	/* The serialized form will be at least the length
	 * of our backing buffer representation */
	*size = bhnd_nvram_io_getsize(bcm->data);

	/* Header variables that require mirroring will expand
	 * the total size */
	for (size_t i = 0; i < nitems(bcm->hvars); i++) {
		struct bhnd_nvram_bcmdata	*hvar;
		size_t				 entry_len;
		size_t				 name_len;
		size_t				 value_len;

		hvar = &bcm->hvars[i];
		if (hvar->mirrored)
			continue;

		/* Fetch the name length */
		name_len = strlen(hvar->name);

		/* Calculate the length of the value's CSTR representation */
		error = bhnd_nvram_coerce_value(NULL, &value_len,
		    BHND_NVRAM_TYPE_CSTR, &hvar->value, hvar->size, hvar->type,
		    NULL);
		if (error)
			return (error);

		/* name_len + '=' + value_len */
		entry_len = name_len + 1 + value_len;

		/* Add to total */
		if (SIZE_MAX - entry_len < *size)
			return (ERANGE);

		*size = *size + entry_len;
	}

	return (0);
}

static int
bhnd_nvram_bcm_serialize(struct bhnd_nvram_data *nv, void *buf, size_t *len)
{
	struct bhnd_nvram_bcm		*bcm;
	struct bhnd_nvram_header	 hdr;
	void				*cookiep;
	const char			*name;
	size_t				 nbytes, req_size, limit;
	uint8_t				 crc;
	int				 error;

	bcm = (struct bhnd_nvram_bcm *)nv;

	/* Save the output buffer limit */
	if (buf == NULL)
		limit = 0;
	else
		limit = *len;

	/* Calculate and provide the actual buffer size */
	if ((error = bhnd_nvram_data_size(nv, &req_size)))
		return (error);

	*len = req_size;

	/* Skip serialization if not requested, or report ENOMEM if
	 * buffer is too small */
	if (buf == NULL) {
		return (0);
	} else if (*len > limit) {
		return (ENOMEM);
	}

	/* Reserve space for the NVRAM header */
	nbytes = sizeof(struct bhnd_nvram_header);

	/* Write all variables to the output buffer */
	cookiep = NULL;
	while ((name = bhnd_nvram_data_next(nv, &cookiep))) {
		const void	*inp;
		bhnd_nvram_type	 itype;
		size_t		 ilen, olen;
		uint8_t		*outp;
		size_t		 name_len, val_len;

		if (limit > nbytes) {
			outp = (uint8_t *)buf + nbytes;
			olen = limit - nbytes;
		} else {
			outp = NULL;
			olen = 0;
		}

		/* Determine length of variable name */
		name_len = strlen(name) + 1;

		/* Write the variable name and '=' delimeter */
		if (olen >= name_len) {
			/* Copy name */
			memcpy(outp, name, name_len - 1);

			/* Append '=' */
			*(outp + name_len - 1) = '=';
		}

		/* Adjust byte counts */
		if (SIZE_MAX - name_len < nbytes)
			return (ERANGE);

		nbytes += name_len;

		/* Reposition output */
		if (limit > nbytes) {
			outp = (uint8_t *)buf + nbytes;
			olen = limit - nbytes;
		} else {
			outp = NULL;
			olen = 0;
		}

		/* Fetch a pointer to the variable data */
		inp = bhnd_nvram_data_getvar_ptr(nv, cookiep, &ilen, &itype);
		if (inp == NULL)
			return (ENXIO);

		/* Coerce to NUL-terminated C string, writing to the output
		 * buffer (or just calculating the length if outp is NULL) */
		val_len = olen;
		error = bhnd_nvram_coerce_value(outp, &val_len,
		    BHND_NVRAM_TYPE_CSTR, inp, ilen, itype, NULL);
		if (error && error != ENOMEM)
			return (error);

		/* Adjust byte counts */
		if (SIZE_MAX - val_len < nbytes)
			return (ERANGE);

		nbytes += val_len;
	}

	/* Write terminating NUL */
	if (nbytes < limit)
		*((uint8_t *)buf + nbytes) = '\0';
	nbytes++;

	/* Provide actual size */
	BHND_NV_ASSERT(req_size == nbytes, ("calculated size incorrect"));
	*len = nbytes;
	if (nbytes > limit && buf != NULL)
		return (ENOMEM);

	/* Fetch current NVRAM header */
	if ((error = bhnd_nvram_io_read(bcm->data, 0x0, &hdr, sizeof(hdr))))
		return (error);

	/* Update values covered by CRC and write to output buffer */
	hdr.size = htole32(*len);
	memcpy(buf, &hdr, sizeof(hdr));

	/* Calculate new CRC */
	crc = bhnd_nvram_crc8((uint8_t *)buf + BCM_NVRAM_CRC_SKIP,
	    *len - BCM_NVRAM_CRC_SKIP, BHND_NVRAM_CRC8_INITIAL);

	/* Update header with valid CRC */
	hdr.cfg0 &= ~BCM_NVRAM_CFG0_CRC_MASK;
	hdr.cfg0 |= (crc << BCM_NVRAM_CFG0_CRC_SHIFT);
	memcpy(buf, &hdr, sizeof(hdr));

	return (0);
}

static uint32_t
bhnd_nvram_bcm_caps(struct bhnd_nvram_data *nv)
{
	return (BHND_NVRAM_DATA_CAP_READ_PTR|BHND_NVRAM_DATA_CAP_DEVPATHS);
}

static const char *
bhnd_nvram_bcm_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	struct bhnd_nvram_bcm		*bcm;
	struct bhnd_nvram_bcmdata	*hvar;
	const void			*ptr;
	const char			*envp, *basep;
	size_t				 io_size, io_offset;
	int				 error;

	bcm = (struct bhnd_nvram_bcm *)nv;
	
	io_offset = sizeof(struct bhnd_nvram_header);
	io_size = bhnd_nvram_io_getsize(bcm->data) - io_offset;

	/* Map backing buffer */
	error = bhnd_nvram_io_read_ptr(bcm->data, io_offset, &ptr, io_size,
	    NULL);
	if (error) {
		BCM_NVLOG("error mapping backing buffer: %d\n", error);
		return (NULL);
	}

	basep = ptr;

	/* Handle header variable iteration */
	hvar = bhnd_nvram_bcm_to_hdrvar(bcm, *cookiep);
	if (hvar != NULL) {
		size_t idx;

		/* Advance to next entry, if any */
		idx = bhnd_nvram_bcm_hdrvar_index(bcm, hvar) + 1;

		/* Find the next header-defined variable that isn't
		 * defined in the NVRAM data, start iteration there */
		for (size_t i = idx; i < nitems(bcm->hvars); i++) {
			if (bcm->hvars[i].mirrored)
				continue;

			*cookiep = &bcm->hvars[i];
			return (bcm->hvars[i].name);
		}

		/* No further header-defined variables; iteration
		 * complete */
		return (NULL);
	}

	/* Handle standard NVRAM data iteration */
	if (*cookiep == NULL) {
		/* Start at the first NVRAM data record */
		envp = basep;
	} else {
		/* Seek to next record */
		envp = *cookiep;
		envp += strlen(envp) + 1;	/* key + '\0' */
		envp += strlen(envp) + 1;	/* value + '\0' */
	}

	/* On EOF, try switching to header variables */
	if ((size_t)(envp - basep) == io_size || *envp == '\0') {
		/* Find first valid header variable */
		for (size_t i = 0; i < nitems(bcm->hvars); i++) {
			if (bcm->hvars[i].mirrored)
				continue;
			
			*cookiep = &bcm->hvars[i];
			return (bcm->hvars[i].name);
		}

		/* No header variables */
		return (NULL);
	}

	*cookiep = (void *)(uintptr_t)envp;
	return (envp);
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
	const void			*vptr;
	size_t				 vlen;
	bhnd_nvram_type			 vtype;

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
	struct bhnd_nvram_bcm		*bcm;
	struct bhnd_nvram_bcmdata	*hvar;
	const char			*envp;

	bcm = (struct bhnd_nvram_bcm *)nv;

	/* Handle header variables */
	if ((hvar = bhnd_nvram_bcm_to_hdrvar(bcm, cookiep)) != NULL) {
		BHND_NV_ASSERT(
		    hvar->size % bhnd_nvram_type_width(hvar->type) == 0,
		    ("length is not aligned to type width"));

		*type = hvar->type;
		*len = hvar->size;
		return (&hvar->value);
	}

	/* Cookie points to key\0value\0 -- get the value address */
	BHND_NV_ASSERT(cookiep != NULL, ("NULL cookiep"));

	envp = cookiep;
	envp += strlen(envp) + 1;	/* key + '\0' */
	*len = strlen(envp) + 1;	/* value + '\0' */
	*type = BHND_NVRAM_TYPE_CSTR;

	return (envp);
}

static const char *
bhnd_nvram_bcm_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	struct bhnd_nvram_bcm		*bcm;
	struct bhnd_nvram_bcmdata	*hvar;

	bcm = (struct bhnd_nvram_bcm *)nv;

	
	/* Handle header variables */
	if ((hvar = bhnd_nvram_bcm_to_hdrvar(bcm, cookiep)) != NULL) {
		return (hvar->name);
	}

	/* Cookie points to key\0value\0 */
	return (cookiep);
}

/**
 * Return the internal BCM data reference for a header-defined variable
 * with @p name, or NULL if none exists.
 */
static struct bhnd_nvram_bcmdata *
bhnd_nvram_bcm_gethdrvar(struct bhnd_nvram_bcm *bcm, const char *name)
{
	for (size_t i = 0; i < nitems(bcm->hvars); i++) {
		if (strcmp(bcm->hvars[i].name, name) == 0)
			return (&bcm->hvars[i]);
	}

	/* Not found */
	return (NULL);
}

/**
 * If @p cookiep references a header-defined variable, return the
 * internal BCM data reference. Otherwise, returns NULL.
 */
static struct bhnd_nvram_bcmdata *
bhnd_nvram_bcm_to_hdrvar(struct bhnd_nvram_bcm *bcm, void *cookiep)
{
#ifdef BHND_NVRAM_INVARIANTS                                                                                                                                                                                                                                
	uintptr_t base, ptr;
#endif

	/* If the cookie falls within the hvar array, it's a
	 * header variable cookie */
	if (nitems(bcm->hvars) == 0)
		return (NULL);

	if (cookiep < (void *)&bcm->hvars[0])
		return (NULL);

	if (cookiep > (void *)&bcm->hvars[nitems(bcm->hvars)-1])
		return (NULL);

#ifdef BHND_NVRAM_INVARIANTS
	base = (uintptr_t)bcm->hvars;
	ptr = (uintptr_t)cookiep;

	BHND_NV_ASSERT((ptr - base) % sizeof(bcm->hvars[0]) == 0,
	    ("misaligned hvar pointer %p/%p", cookiep, bcm->hvars));
#endif /* INVARIANTS */

	return ((struct bhnd_nvram_bcmdata *)cookiep);
}

/**
 * Return the index of @p hdrvar within @p bcm's backing hvars array.
 */
static size_t
bhnd_nvram_bcm_hdrvar_index(struct bhnd_nvram_bcm *bcm,
    struct bhnd_nvram_bcmdata *hdrvar)
{
	BHND_NV_ASSERT(bhnd_nvram_bcm_to_hdrvar(bcm, (void *)hdrvar) != NULL,
	    ("%p is not a valid hdrvar reference", hdrvar));

	return (hdrvar - &bcm->hvars[0]);
}
