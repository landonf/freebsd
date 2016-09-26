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
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/bhnd/bhnd.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_io.h"
#include "bhnd_nvram_iovar.h"

/**
 * Buffer-backed NVRAM I/O context.
 */
struct bhnd_nvram_iobuf {
	struct bhnd_nvram_io	 io;	/**< common I/O instance state */
	const void		*buf;	/**< backing buffer (borrowed ref) */
	size_t			 size;	/**< size of @p buf */
};

BHND_NVRAM_IOPS_DEFN(iobuf)

/**
 * Allocate and return a new I/O context backed by a borrowed
 * reference to @p buffer.
 *
 * The caller is responsible for deallocating the returned I/O context via
 * bhnd_nvram_io_free().
 * 
 * @param	buffer	The buffer to be mapped by the returned I/O context.
 * @param	size	The size of @p buffer, in bytes.
 * 
 * @retval	bhnd_nvram_io	success.
 * @retval	NULL		if allocation fails.
 */
struct bhnd_nvram_io *
bhnd_nvram_iobuf_new(const void *buffer, size_t size)
{
	struct bhnd_nvram_iobuf *iobuf;

	iobuf = malloc(sizeof(*iobuf), M_BHND_NVRAM, M_WAITOK);
	iobuf->io.iops = &bhnd_nvram_iobuf_ops;
	iobuf->buf = buffer;
	iobuf->size = size;

	return (&iobuf->io);
}

static void
bhnd_nvram_iobuf_free(struct bhnd_nvram_io *io)
{
	free(io, M_BHND_NVRAM);
}

static bus_size_t
bhnd_nvram_iobuf_get_size(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_iobuf	*iobuf = (struct bhnd_nvram_iobuf *)io;
	return (iobuf->size);
}

static int
bhnd_nvram_iobuf_read_ptr(struct bhnd_nvram_io *io, bus_size_t offset,
    const void **ptr, bus_size_t *nbytes)
{
	struct bhnd_nvram_iobuf	*iobuf;

	iobuf = (struct bhnd_nvram_iobuf *) io;

	/* Verify offset falls within the buffer range */
	if (offset > iobuf->size)
		return (ENXIO);

	/* Valid read, provide a pointer to the buffer */
	*ptr = ((const uint8_t *)iobuf->buf) + offset;
	*nbytes = bhnd_nvram_bsz_min(*nbytes, iobuf->size - offset);

	return (0);
}

static int
bhnd_nvram_iobuf_read(struct bhnd_nvram_io *io, bus_size_t offset, void *buffer,
    bus_size_t *nbytes)
{
	struct bhnd_nvram_iobuf	*iobuf;
	const void		*ptr;
	int			 error;

	iobuf = (struct bhnd_nvram_iobuf *) io;

	/* Try to fetch our direct pointer */
	if ((error = bhnd_nvram_iobuf_read_ptr(io, offset, &ptr, nbytes)))
		return (error);

	/* Valid read; copy out the bytes */
	memcpy(buffer, ptr, *nbytes);
	return (0);
}

