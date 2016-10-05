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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/ctype.h>
#include <sys/endian.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_bcmreg.h"

#include "bhnd_nvram_data.h"
#include "bhnd_nvram_datavar.h"

/**
 * Broadcom "Board Text" data class.
 *
 * This format is used to provide external NVRAM data for some
 * fullmac WiFi devices, and as an input format when programming
 * NVRAM/SPROM/OTP.
 */

struct bhnd_nvram_btxt {
	struct bhnd_nvram_data	 nv;	/**< common instance state */
	struct bhnd_nvram_io	*data;	/**< memory-backed board text data */
};

BHND_NVRAM_DATA_CLASS_DEFN(btxt, "Broadcom Board Text")

/** Minimal identification header */
union bhnd_nvram_btxt_ident {
	uint32_t	bcm_magic;
	char		btxt[8];
};

static size_t	bhnd_nvram_btxt_io_offset(struct bhnd_nvram_btxt *btxt,
					  void *cookiep);

static int	bhnd_nvram_btxt_entry_len(struct bhnd_nvram_io *io,
		    size_t offset, size_t *line_len, size_t *env_len);
static int	bhnd_nvram_btxt_seek_next(struct bhnd_nvram_io *io,
		    size_t *offset);
static int	bhnd_nvram_btxt_seek_eol(struct bhnd_nvram_io *io,
		    size_t *offset);

#define	BTXT_NVLOG(_fmt, ...)	\
	printf("%s: " _fmt, __FUNCTION__, ##__VA_ARGS__)

static int
bhnd_nvram_btxt_probe(struct bhnd_nvram_io *io)
{
	union bhnd_nvram_btxt_ident	ident;
	char				c;
	int				error;

	/* Look at the initial header for something that looks like 
	 * an ASCII board text file */
	if ((error = bhnd_nvram_io_read(io, 0x0, &ident, sizeof(ident))))
		return (error);

	/* The BCM NVRAM format uses a 'FLSH' little endian magic value, which
	 * shouldn't be interpreted as BTXT */
	if (le32toh(ident.bcm_magic) == BCM_NVRAM_MAGIC)
		return (ENXIO);

	/* Don't match on non-ASCII data */
	for (size_t i = 0; i < nitems(ident.btxt); i++) {
		c = ident.btxt[i];
		if (!isprint(c) && !isspace(c))
			return (ENXIO);
	}

	/* The first character should either be a valid key char (alpha),
	 * whitespace, or the start of a comment ('#') */
	c = ident.btxt[0];
	if (!isspace(c) && !isalpha(c) && c != '#')
		return (ENXIO);

	/* We assert a low priority, given that we've only scanned an
	 * initial few bytes of the file. In practice, BTXT will generally
	 * be referenced explicitly when parsing files of a known format. */
	return (BUS_PROBE_LOW_PRIORITY);
}

/**
 * Initialize @p btxt with the provided board text data mapped by @p src.
 * 
 * @param btxt A newly allocated data instance.
 */
static int
bhnd_nvram_btxt_init(struct bhnd_nvram_btxt *btxt, struct bhnd_nvram_io *src)
{
	struct bhnd_nvram_io	*io;
	const char		*name, *value;
	size_t			 name_len, value_len;
	size_t			 line_len, env_len;
	size_t			 io_offset, io_size;
	int			 error;

	KASSERT(btxt->data == NULL, ("btxt data already allocated"));
	
	if ((io = bhnd_nvram_iobuf_copy(src)) == NULL)
		return (ENOMEM);

	btxt->data = io;

	io_size = bhnd_nvram_io_getsize(io);
	io_offset = 0;
	while (io_offset < io_size) {
		const void	*envp;
		size_t		 nbytes;

		/* Seek to the next key=value entry */
		if ((error = bhnd_nvram_btxt_seek_next(io, &io_offset)))
			return (error);

		/* Determine the entry and line length */
		error = bhnd_nvram_btxt_entry_len(io, io_offset, &line_len,
		    &env_len);
		if (error)
			return (error);
	
		/* EOF? */
		if (env_len == 0) {
			KASSERT(io_offset == io_size, ("zero-length record "
			    "returned from bhnd_nvram_btxt_seek_next()"));
			break;
		}

		/* Parse the key=value string */
		nbytes = env_len;
		error = bhnd_nvram_io_read_ptr(io, io_offset, &envp, nbytes,
		    NULL);
		if (error)
			return (error);

		error = bhnd_nvram_parse_env(envp, env_len, '=', &name,
		    &name_len, &value, &value_len);
		if (error)
			return (error);

		/* Insert a '\0' character, replacing the '=' delimiter and
		 * allowing us to vend references directly to the variable
		 * name */
		error = bhnd_nvram_io_write(io, io_offset+name_len,
		    &(char){'\0'}, 1);
		if (error)
			return (error);

		/* Advance past EOL */
		io_offset += line_len;
	}

	return (0);
}

static int
bhnd_nvram_btxt_new(struct bhnd_nvram_data **nv, struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_btxt	*btxt;
	int			 error;

	/* Allocate and initialize the BTXT data instance */
	btxt = malloc(sizeof(*btxt), M_BHND_NVRAM, M_NOWAIT|M_ZERO);
	if (btxt == NULL)
		return (ENOMEM);

	btxt->nv.cls = &bhnd_nvram_btxt_class;
	btxt->data = NULL;

	/* Parse the BTXT input data and initialize our backing
	 * data representation */
	if ((error = bhnd_nvram_btxt_init(btxt, io)))
		goto failed;

	*nv = &btxt->nv;
	return (0);

failed:
	if (btxt->data != NULL)
		bhnd_nvram_io_free(btxt->data);

	free(btxt, M_BHND_NVRAM);

	return (error);
}

static void
bhnd_nvram_btxt_free(struct bhnd_nvram_data *nv)
{
	struct bhnd_nvram_btxt	*btxt = (struct bhnd_nvram_btxt *)nv;

	bhnd_nvram_io_free(btxt->data);
	free(nv, M_BHND_NVRAM);
}

static int
bhnd_nvram_btxt_size(struct bhnd_nvram_data *nv, size_t *size)
{
	struct bhnd_nvram_btxt *btxt;
	char			ch;
	int			error;

	btxt = (struct bhnd_nvram_btxt *)nv;

	/* The serialized form will be identical in length
	 * to our backing buffer representation, minus any
	 * terminating NUL that might be included when operating on
	 * BTXT data loaded from a C string */
	*size = bhnd_nvram_io_getsize(btxt->data);
	if (*size == 0)
		return (0);

	/* Check for a terminating NUL */
	error = bhnd_nvram_io_read(btxt->data, *size - 1, &ch, sizeof(ch));
	if (error)
		return (error);

	/* Decrease size to account for NUL */
	if (ch == '\0')
		*size = *size - 1;

	return (0);
}

static int
bhnd_nvram_btxt_serialize(struct bhnd_nvram_data *nv, void *buf, size_t *len)
{
	// TODO
	return (ENXIO);
}

static uint32_t
bhnd_nvram_btxt_getcaps(struct bhnd_nvram_data *nv)
{
	return (BHND_NVRAM_DATA_CAP_READ_PTR);
}

static void *
bhnd_nvram_btxt_find(struct bhnd_nvram_data *nv, const char *name)
{
	return (bhnd_nvram_data_generic_find(nv, name));
}

static const char *
bhnd_nvram_btxt_next(struct bhnd_nvram_data *nv, void **cookiep)
{
	struct bhnd_nvram_btxt	*btxt;
	const void		*nptr;
	size_t			 io_offset, io_size;
	int			 error;

	btxt = (struct bhnd_nvram_btxt *)nv;

	io_size = bhnd_nvram_io_getsize(btxt->data);
	io_offset = bhnd_nvram_btxt_io_offset(btxt, *cookiep);

	/* Already at EOF? */
	if (io_offset == io_size)
		return (NULL);

	/* Seek to the next entry (if any) */
	if ((error = bhnd_nvram_btxt_seek_eol(btxt->data, &io_offset))) {
		BTXT_NVLOG("unexpected error in seek_eol(): %d\n", error);
		return (NULL);
	}

	if ((error = bhnd_nvram_btxt_seek_next(btxt->data, &io_offset))) {
		BTXT_NVLOG("unexpected error in seek_next(): %d\n", error);
		return (NULL);
	}

	/* Provide the new cookie for this offset */
	if (io_offset > UINTPTR_MAX) {
		BTXT_NVLOG("io_offset > UINPTR_MAX!\n");
		return (NULL);
	}

	*cookiep = (void *)(uintptr_t)io_offset;

	/* Hit EOF? */
	if (io_offset == io_size)
		return (NULL);

	/* Fetch the name pointer; it must be at least 1 byte long */
	error = bhnd_nvram_io_read_ptr(btxt->data, io_offset, &nptr, 1, NULL);
	if (error) {
		BTXT_NVLOG("unexpected error in read_ptr(): %d\n", error);
		return (NULL);
	}

	/* Return the name pointer */
	return (nptr);
}

static int
bhnd_nvram_btxt_getvar(struct bhnd_nvram_data *nv, void *cookiep, void *buf,
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

const void *
bhnd_nvram_btxt_getvar_ptr(struct bhnd_nvram_data *nv, void *cookiep,
    size_t *len, bhnd_nvram_type *type)
{
	struct bhnd_nvram_btxt	*btxt;
	const void		*eptr;
	const char		*vptr;
	size_t			 io_offset, io_size;
	size_t			 line_len, env_len;
	int			 error;
	
	btxt = (struct bhnd_nvram_btxt *)nv;
	
	io_size = bhnd_nvram_io_getsize(btxt->data);
	io_offset = bhnd_nvram_btxt_io_offset(btxt, cookiep);

	/* At EOF? */
	if (io_offset == io_size)
		return (NULL);

	/* Determine the entry length */
	error = bhnd_nvram_btxt_entry_len(btxt->data, io_offset, &line_len,
	    &env_len);
	if (error) {
		BTXT_NVLOG("unexpected error in entry_len(): %d\n", error);
		return (NULL);
	}

	/* Fetch the entry's value pointer and length */
	error = bhnd_nvram_io_read_ptr(btxt->data, io_offset, &eptr, env_len,
	    NULL);
	if (error) {
		BTXT_NVLOG("unexpected error in read_ptr(): %d\n", error);
		return (NULL);
	}

	error = bhnd_nvram_parse_env(eptr, env_len, '\0', NULL, NULL, &vptr,
	    len);
	if (error) {
		BTXT_NVLOG("unexpected error in parse_env(): %d\n", error);
		return (NULL);
	}

	/* Type is always CSTR */
	*type = BHND_NVRAM_TYPE_CSTR;

	return (vptr);
}

static const char *
bhnd_nvram_btxt_getvar_name(struct bhnd_nvram_data *nv, void *cookiep)
{
	struct bhnd_nvram_btxt	*btxt;
	const void		*ptr;
	size_t			 io_offset, io_size;
	int			 error;
	
	btxt = (struct bhnd_nvram_btxt *)nv;
	
	io_size = bhnd_nvram_io_getsize(btxt->data);
	io_offset = bhnd_nvram_btxt_io_offset(btxt, cookiep);

	/* At EOF? */
	if (io_offset == io_size)
		panic("invalid cookiep: %p", cookiep);

	/* Variable name is found directly at the given offset; trailing
	 * NUL means we can assume that it's at least 1 byte long */
	error = bhnd_nvram_io_read_ptr(btxt->data, io_offset, &ptr, 1, NULL);
	if (error)
		panic("unexpected error in read_ptr(): %d\n", error);

	return (ptr);
}

/* Convert cookie back to an I/O offset */
static size_t
bhnd_nvram_btxt_io_offset(struct bhnd_nvram_btxt *btxt, void *cookiep)
{
	size_t		io_size;
	uintptr_t	cval;

	io_size = bhnd_nvram_io_getsize(btxt->data);
	cval = (uintptr_t)cookiep;

	KASSERT(cval < SIZE_MAX, ("cookie > SIZE_MAX)"));
	KASSERT(cval <= io_size, ("cookie > io_size)"));

	return ((size_t)cval);
}

/* Determine the entry length and env 'key=value' string length of the entry
 * at @p offset */
static int
bhnd_nvram_btxt_entry_len(struct bhnd_nvram_io *io, size_t offset,
    size_t *line_len, size_t *env_len)
{
	const uint8_t	*baseptr, *p;
	const void	*rbuf;
	size_t		 nbytes;
	int		 error;

	/* Fetch read buffer */
	if ((error = bhnd_nvram_io_read_ptr(io, offset, &rbuf, 0, &nbytes)))
		return (error);

	/* Find record termination (EOL, or '#') */
	p = rbuf;
	baseptr = rbuf;
	while (p - baseptr < nbytes) {
		if (*p == '#' || *p == '\n' || *p == '\r')
			break;

		p++;
	}

	/* Got line length, now trim any trailing whitespace to determine
	 * actual env length */
	*line_len = p - baseptr;
	*env_len = *line_len;

	for (size_t i = 0; i < *line_len; i++) {
		char c = baseptr[*line_len - i - 1];
		if (!isspace(c))
			break;

		*env_len -= 1;
	}

	return (0);
}

/* Seek past the next line ending (\r, \r\n, or \n) */
static int
bhnd_nvram_btxt_seek_eol(struct bhnd_nvram_io *io, size_t *offset)
{
	const uint8_t	*baseptr, *p;
	const void	*rbuf;
	size_t		 nbytes;
	int		 error;

	/* Fetch read buffer */
	if ((error = bhnd_nvram_io_read_ptr(io, *offset, &rbuf, 0, &nbytes)))
		return (error);

	baseptr = rbuf;
	p = rbuf;
	while (p - baseptr < nbytes) {
		char c = *p;

		/* Advance to next char. The next position may be EOF, in which
		 * case a read will be invalid */
		p++;

		if (c == '\r') {
			/* CR, check for optional LF */
			if (p - baseptr < nbytes) {
				if (*p == '\n')
					p++;
			}

			break;
		} else if (c == '\n') {
			break;
		}
	}

	/* Hit newline or EOF */
	*offset += (p - baseptr);
	return (0);
}

/* Seek to the next valid non-comment line (or EOF) */
static int
bhnd_nvram_btxt_seek_next(struct bhnd_nvram_io *io, size_t *offset)
{
	const uint8_t	*baseptr, *p;
	const void	*rbuf;
	size_t		 nbytes;
	int		 error;

	/* Fetch read buffer */
	if ((error = bhnd_nvram_io_read_ptr(io, *offset, &rbuf, 0, &nbytes)))
		return (error);

	/* Skip leading whitespace and comments */
	baseptr = rbuf;
	p = rbuf;
	while (p - baseptr < nbytes) {
		char c = *p;

		/* Skip whitespace */
		if (isspace(c)) {
			p++;
			continue;
		}

		/* Skip entire comment line */
		if (c == '#') {
			size_t line_off = *offset + (p - baseptr);
	
			if ((error = bhnd_nvram_btxt_seek_eol(io, &line_off)))
				return (error);

			p = baseptr + (line_off - *offset);
			continue;
		}

		/* Non-whitespace, non-comment */
		break;
	}

	*offset += (p - baseptr);
	return (0);
}
