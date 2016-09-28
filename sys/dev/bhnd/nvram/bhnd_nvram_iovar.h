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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_NVRAM_BHND_NVRAM_IOVAR_H_
#define _BHND_NVRAM_BHND_NVRAM_IOVAR_H_

#include <sys/param.h>

#include "bhnd_nvram_io.h"

/** @see bhnd_nvram_io_read() */
typedef int (bhnd_nvram_iop_read)(struct bhnd_nvram_io *io, size_t offset,
    void *buffer, size_t *nbytes);

/** @see bhnd_nvram_io_read_ptr() */
typedef int (bhnd_nvram_iop_read_ptr)(struct bhnd_nvram_io *io, size_t offset,
    const void **ptr, size_t *nbytes);

/** @see bhnd_nvram_io_write() */
typedef int (bhnd_nvram_iop_write)(struct bhnd_nvram_io *io, size_t offset,
    void *buffer, size_t nbytes);

/** @see bhnd_nvram_io_write_ptr() */
typedef int (bhnd_nvram_iop_write_ptr)(struct bhnd_nvram_io *io, size_t offset,
    void **ptr, size_t nbytes);

/** @see bhnd_nvram_io_get_size() */
typedef size_t (bhnd_nvram_iop_get_size)(struct bhnd_nvram_io *io);

/** @see bhnd_nvram_io_free() */
typedef void (bhnd_nvram_iop_free)(struct bhnd_nvram_io *io);

/**
 * NVRAM abstract I/O operations.
 */
struct bhnd_nvram_iops {
	bhnd_nvram_iop_read		*read;		/**< read() implementation */
	bhnd_nvram_iop_read_ptr		*read_ptr;	/**< read_ptr() implementation */
	bhnd_nvram_iop_get_size		*get_size;	/**< get_size() implementation */
	bhnd_nvram_iop_write		*write;		/**< write() implementation */
	bhnd_nvram_iop_write_ptr	*write_ptr;	/**< write_ptr() implementation */
	bhnd_nvram_iop_free		*free;		/**< free() implementation */
};

/**
 * NVRAM abstract I/O context.
 */
struct bhnd_nvram_io {
	const struct bhnd_nvram_iops	*iops;
};

/**
 * Declare a bhnd_nvram_iops class with name @p _n.
 */
#define	BHND_NVRAM_IOPS_DEFN(_n)					\
	static bhnd_nvram_iop_read	bhnd_nvram_ ## _n ## _read;	\
	static bhnd_nvram_iop_read_ptr	bhnd_nvram_ ## _n ## _read_ptr;	\
	static bhnd_nvram_iop_write	bhnd_nvram_ ## _n ## _write;	\
	static bhnd_nvram_iop_write_ptr	bhnd_nvram_ ## _n ## _write_ptr;\
	static bhnd_nvram_iop_get_size	bhnd_nvram_ ## _n ## _get_size;	\
	static bhnd_nvram_iop_free	bhnd_nvram_ ## _n ## _free;	\
									\
	static struct bhnd_nvram_iops	bhnd_nvram_ ## _n ## _ops = {	\
		.read		= bhnd_nvram_ ## _n ## _read,		\
		.read_ptr	= bhnd_nvram_ ## _n ## _read_ptr,	\
		.write		= bhnd_nvram_ ## _n ## _write,		\
		.write_ptr	= bhnd_nvram_ ## _n ## _write_ptr,	\
		.get_size	= bhnd_nvram_ ## _n ## _get_size,	\
		.free		= bhnd_nvram_ ## _n ## _free		\
	};

#endif /* _BHND_NVRAM_BHND_NVRAM_IOVAR_H_ */
