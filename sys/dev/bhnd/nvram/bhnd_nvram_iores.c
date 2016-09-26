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
 * BHND resource-backed NVRAM I/O context.
 */
struct bhnd_nvram_iores {
	struct bhnd_nvram_io	 io;		/**< common I/O instance state */
	struct bhnd_resource	*res;		/**< backing resource (borrowed ref) */
	bus_size_t		 offset;	/**< offset within res */
	bus_size_t		 size;		/**< size relative to the base offset */
	u_int			 bus_width;	/**< data type byte width to be used
						     when performing bus operations
						     on res. (1, 2, or 4 bytes) */
};

BHND_NVRAM_IOPS_DEFN(iores);

/**
 * Allocate and return a new I/O context backed by a borrowed reference to @p r.
 *
 * The caller is responsible for deallocating the returned I/O context via
 * bhnd_nvram_io_free().
 * 
 * @param	r		The resource to be mapped by the returned I/O
 *				context.
 * @param	offset		Offset 
 * @param	bus_width	The required I/O width (1, 2, or 4 bytes) to be
 *				used when reading from @p r.
 * 
 * @retval	bhnd_nvram_io	success.
 * @retval	NULL		if allocation fails, or an invalid argument
 *				is supplied.
 */
struct bhnd_nvram_io *
bhnd_nvram_iores_new(struct bhnd_resource *r, bus_size_t offset,
    bus_size_t size, u_int bus_width)
{
	struct bhnd_nvram_iores	*iores;
	rman_res_t		 r_start, r_size;

	/* Verify the bus width */
	switch (bus_width) {
	case 1:
	case 2:
	case 4:
		/* valid */
		break;
	default:
		printf("%s: invalid bus width %u\n", __FUNCTION__, bus_width);
		return (NULL);
	}

	/* offset/size must not exceed BUS_SPACE_MAXSIZE */
	if (size > BUS_SPACE_MAXSIZE || offset > BUS_SPACE_MAXSIZE)
		return (NULL);

	/* offset/size fall within the resource's mapped range */
	r_size = rman_get_size(r->res);
	r_start = rman_get_start(r->res);
	if (r_size < offset || r_size < size || r_size - size < offset)
		return (NULL);

	/* offset/size must be bus_width aligned  */
	if ((r_start + offset) % bus_width != 0) {
		printf("%s: base address %#jx+%#jx not aligned to bus "
		    "width %u\n", __FUNCTION__, r_start, offset, bus_width);
		return (NULL);
	}

	if (size % bus_width != 0) {
		printf("%s: size %jx not aligned to bus width %u\n",
		    __FUNCTION__, size, bus_width);
		return (NULL);
	}

	/* Allocate and return the I/O context */
	iores = malloc(sizeof(*iores), M_BHND_NVRAM, M_WAITOK);
	iores->io.iops = &bhnd_nvram_iores_ops;
	iores->res = r;
	iores->offset = offset;
	iores->size = size;
	iores->bus_width = bus_width;

	return (&iores->io);
}

static void
bhnd_nvram_iores_free(struct bhnd_nvram_io *io)
{
	free(io, M_BHND_NVRAM);
}

static bus_size_t
bhnd_nvram_iores_get_size(struct bhnd_nvram_io *io)
{
	struct bhnd_nvram_iores	*iores = (struct bhnd_nvram_iores *)io;
	return (iores->size);
}

static int
bhnd_nvram_iores_read_ptr(struct bhnd_nvram_io *io, bus_size_t offset,
    const void **ptr, bus_size_t *nbytes)
{
	/* unsupported */
	return (ENODEV);
}

static int
bhnd_nvram_iores_read(struct bhnd_nvram_io *io, bus_size_t offset, void *buffer,
    bus_size_t *nbytes)
{
	struct bhnd_nvram_iores	*iores;
	bus_size_t		 r_offset;

	iores = (struct bhnd_nvram_iores *)io;

	/* Verify offset falls within the resource range */
	if (offset > iores->size)
		return (ENXIO);

	/* Check for eof */
	if (offset == iores->size) {
		*nbytes = 0;
		return (0);
	}

	/* Verify offset alignment */
	if (offset % iores->bus_width != 0)
		return (EFAULT);

	/* Limit nbytes to available range and perform alignment */
	if (*nbytes != 0 && *nbytes < iores->bus_width) {
		/* alignment would produce a zero-length read */
		return (EFAULT);
	}

	*nbytes = bhnd_nvram_bsz_min(*nbytes, iores->size - offset);
	*nbytes = *nbytes - (*nbytes % iores->bus_width);

	/* Determine actual resource offset and perform the read */
	r_offset = iores->offset + offset;
	switch (iores->bus_width) {
	case 1:
		bhnd_bus_read_region_stream_1(iores->res, r_offset, buffer,
		    *nbytes);
		break;
	case 2:
		bhnd_bus_read_region_stream_2(iores->res, r_offset, buffer,
		    *nbytes);
		break;
	case 4:
		bhnd_bus_read_region_stream_4(iores->res, r_offset, buffer,
		    *nbytes);
		break;
	default:
		panic("unreachable!");
	}

	return (0);
}

