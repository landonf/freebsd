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
	struct bhnd_nvram_io	 io;		/**< common I/O instance state */
	void			*buf;		/**< backing buffer. if inline-allocated, will
						     be a reference to data[]. */
	size_t			 size;		/**< size of @p buf */
	uint8_t			 data[];	/**< inline buffer allocation */
};

BHND_NVRAM_IOPS_DEFN(iobuf)

/**
 * Allocate and return a new iobuf instance with an uninitialized
 * buffer of @p capacity bytes.
 *
 * The caller is responsible for deallocating the returned I/O context via
 * bhnd_nvram_io_free().
 * 
 * @param	size	The size of the backing buffer to be allocated, in
 *			bytes.
 * 
 * @retval	bhnd_nvram_iobuf	success.
 * @retval	NULL			allocation failed.
 */
static struct bhnd_nvram_iobuf *
bhnd_nvram_iobuf_empty(size_t size)
{
	struct bhnd_nvram_iobuf	*iobuf;
	size_t			 iosz;
	bool			 inline_alloc;

	/* Would iosz+size overflow? */
	if (SIZE_MAX - iosz < size) {
		inline_alloc = false;
		iosz = sizeof(*iobuf);
	} else {
		inline_alloc = true;
		iosz = sizeof(*iobuf) + size;
	}

	/* Allocate I/O context */
	if ((iobuf = malloc(iosz, M_BHND_NVRAM, M_NOWAIT)) == NULL)
		return (NULL);

	iobuf->io.iops = &bhnd_nvram_iobuf_ops;
	iobuf->buf = NULL;
	iobuf->size = size;

	/* Either allocate our backing buffer, or initialize the
	 * backing buffer with a reference to our inline allocation. */
	if (inline_alloc)
		iobuf->buf = &iobuf->data;
	else
		iobuf->buf = malloc(iobuf->size, M_BHND_NVRAM, M_NOWAIT);


	if (iobuf->buf == NULL) {
		free(iobuf, M_BHND_NVRAM);
		return (NULL);
	}

	return (iobuf);
}

/**
 * Allocate and return a new I/O context, copying @p size from @p buffer.
 *
 * The caller is responsible for deallocating the returned I/O context via
 * bhnd_nvram_io_free().
 * 
 * @param	buffer	The buffer data be copied by the returned I/O context.
 * @param	size	The size of @p buffer, in bytes.
 * 
 * @retval	bhnd_nvram_io	success.
 * @retval	NULL		allocation failed.
 */
struct bhnd_nvram_io *
bhnd_nvram_iobuf_new(const void *buffer, size_t size)
{
	struct bhnd_nvram_iobuf *iobuf;

	/* Allocate the iobuf instance */
	if ((iobuf = bhnd_nvram_iobuf_empty(size)) == NULL)
		return (NULL);

	/* Copy the input buffer */
	memcpy(iobuf->buf, buffer, iobuf->size);

	return (&iobuf->io);
}

/**
 * Allocate and return a new I/O context providing an in-memory copy
 * of the data mapped by @p src.
 *
 * The caller is responsible for deallocating the returned I/O context via
 * bhnd_nvram_io_free().
 * 
 * @param	src	The I/O context to be copied.
 * 
 * @retval	bhnd_nvram_io	success.
 * @retval	NULL		allocation failed.
 * @retval	NULL		copying @p src failed.
 */
struct bhnd_nvram_io *
bhnd_nvram_iobuf_copy(struct bhnd_nvram_io *src)
{
	struct bhnd_nvram_iobuf	*iobuf;
	size_t			 size;
	int			 error;

	/* Allocate the iobuf instance */
	size = bhnd_nvram_io_get_size(src);
	if ((iobuf = bhnd_nvram_iobuf_empty(size)) == NULL)
		return (NULL);

	/* Copy the input I/O context */
	if ((error = bhnd_nvram_io_read(src, 0x0, iobuf->buf, &size)))
		return (NULL);

	return (&iobuf->io);
}


static void
bhnd_nvram_iobuf_free(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_iobuf	*iobuf = (struct bhnd_nvram_iobuf *)io;

	/* Free the backing buffer if it wasn't allocated inline */
	if (iobuf->buf != &iobuf->data)
		free(iobuf->buf, M_BHND_NVRAM);

	free(iobuf, M_BHND_NVRAM);
}

static size_t
bhnd_nvram_iobuf_get_size(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_iobuf	*iobuf = (struct bhnd_nvram_iobuf *)io;
	return (iobuf->size);
}

static int
bhnd_nvram_iobuf_read_ptr(struct bhnd_nvram_io *io, size_t offset,
    const void **ptr, size_t *nbytes)
{
	struct bhnd_nvram_iobuf	*iobuf;

	iobuf = (struct bhnd_nvram_iobuf *) io;

	/* Verify offset falls within the buffer range */
	if (offset > iobuf->size)
		return (ENXIO);

	/* Valid read, provide a pointer to the buffer */
	*ptr = ((const uint8_t *)iobuf->buf) + offset;
	*nbytes = ummin(*nbytes, iobuf->size - offset);

	return (0);
}

static int
bhnd_nvram_iobuf_read(struct bhnd_nvram_io *io, size_t offset, void *buffer,
    size_t *nbytes)
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

