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

#include "bhnd_nvram_io.h"
#include "bhnd_nvram_iovar.h"

/** 
 * Read up to @p nbytes from @p io at @p offset.
 * 
 * @param io NVRAM I/O context.
 * @param offset The offset within @p io at which to perform the read.
 * @param[out] buffer Output buffer to which up to @p nbytes from @p io will be
 * written.
 * @param[in,out] nbytes On input, the maximum number of bytes to be read; on
 * successful return, will contain the number of bytes actually read. On EOF, a
 * length of 0 will be returned.
 * 
 * The number of bytes read will only be less than requested if
 * @p offset + @p nbytes exceeds the size returned by bhnd_nvram_io_get_size().
 * 
 * @retval 0 success
 * @retval EIO if an input error occured reading @p io.
 * @retval ENXIO if a request for @p offset exceeds the size of @p io.
 * @retval EFAULT if @p io requires I/O request alignment and @p offset is
 * misaligned.
 * @retval EFAULT if @p io requires I/O request alignment and @p nbytes is
 * misaligned and cannot be rounded down to an aligned non-zero value.
 */
int
bhnd_nvram_io_read(struct bhnd_nvram_io *io, size_t offset, void *buffer,
    size_t *nbytes)
{
	return (io->iops->read(io, offset, buffer, nbytes));
}


/**
 * Attempt to fetch a pointer to @p io's internal read buffer, if
 * supported by @p io.
 * 
 * The returned pointer is only gauranteed to be valid until the next I/O
 * operation performed on @p io; concrete implementations of bhnd_nvram_io
 * may provide stronger gaurantees.
 * 
 * @param io NVRAM I/O context.
 * @param offset The offset within @p io for which to return a buffer pointer.
 * @param[out] ptr On success, will be initialized with a pointer to @p io's
 * internal read buffer.
 * @param[in,out] nbytes On input, the minimum number of bytes that must
 * be available at @p offset; on successful return, will contain the actual
 * number of readable bytes.
 * 
 * @retval 0 success
 * @retval EIO if an input error occured reading @p io.
 * @retval ENODEV if @p io does not support direct access to its backing read 
 * buffer.
 * @retval ENXIO if the request exceeds the size of @p io.
 * @retval EFAULT if @p io requires I/O request alignment and @p offset or
 * @p nbytes are misaligned.
 */
int
bhnd_nvram_io_read_ptr(struct bhnd_nvram_io *io, size_t offset,
    const void **ptr, size_t *nbytes)
{
	return (io->iops->read_ptr(io, offset, ptr, nbytes));
}

/** 
 * Write @p nbytes to @p io at @p offset.
 * 
 * @param io NVRAM I/O context.
 * @param offset The offset within @p io at which to perform the write.
 * @param buffer Data to be written to @p io.
 * @param nbytes The number of bytes to be written from @p buffer.
 * 
 * @retval 0 success
 * @retval EIO if an output error occurs writing to @p io.
 * @retval ENODEV if @p io does not support writing.
 * @retval ENXIO if @p io does not support writes beyond the existing
 * end-of-file, and a write at @p offset would exceed the size of the @p io
 * backing data store.
 * @retval EFAULT if @p io requires I/O request alignment and @p offset or
 * @p nbytes are misaligned.
 */
int
bhnd_nvram_io_write(struct bhnd_nvram_io *io, size_t offset, void *buffer,
    size_t nbytes)
{
	return (io->iops->write(io, offset, buffer, nbytes));
}

/**
 * Attempt to fetch a writable pointer to @p io's internal write buffer, if
 * supported by @p io.
 *
 * The returned pointer is only gauranteed to be valid until the next I/O
 * operation performed on @p io; concrete implementations of bhnd_nvram_io
 * may provide stronger gaurantees.
 * 
 * @param io NVRAM I/O context.
 * @param offset The offset within @p io for which to return a buffer pointer.
 * @param[in,out] ptr On success, will be initialized with a pointer to @p io's
 * internal buffer at which up to @p nbytes may be written.
 * @param[in,out] nbytes On input, the minimum number of bytes that must
 * be writable at @p offset; on successful return, will contain the actual
 * number of writable bytes.
 * 
 * @retval 0 success
 * @retval EIO if an output error occurs preparing @p io's write buffer.
 * @retval ENODEV if @p io does not support direct access to its backing write
 * buffer.
 * @retval ENXIO if @p io does not support writes beyond the existing
 * end-of-file, and a write at @p offset of @p nbytes in length would exceed
 * the size of the @p io backing data store.
 * @retval EFAULT if @p io requires I/O request alignment and @p offset or
 * @p nbytes are misaligned.
 */
int
bhnd_nvram_io_write_ptr(struct bhnd_nvram_io *io, size_t offset, void **ptr,
    size_t *nbytes)
{
	return (io->iops->write_ptr(io, offset, ptr, nbytes));
}

/**
 * Return the total number of bytes readable via @p io.
 * 
 * @param io NVRAM I/O context.
 */
size_t
bhnd_nvram_io_get_size(struct bhnd_nvram_io *io)
{
	return (io->iops->get_size(io));
}

/**
 * Free a previously allocated I/O context, releasing all associated
 * resources.
 * 
 * @param io The I/O context to be freed.
 */
void
bhnd_nvram_io_free(struct bhnd_nvram_io *io)
{
	return (io->iops->free(io));
}

