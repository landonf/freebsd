/*-
 * Copyright (c) 2017 Landon Fuller <landonf@FreeBSD.org>
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


#ifndef _MIPS_BROADCOM_BCM_GEOM_CFE_H_
#define _MIPS_BROADCOM_BCM_GEOM_CFE_H_

#include <sys/param.h>
#include <geom/geom.h>

#include <dev/bhnd/nvram/bhnd_nvram_iovar.h>

#include "bcm_diskvar.h"

#define	CFE_CLASS_NAME		"BCM_CFE"

/**
 * GEOM BCM_CFE instance state
 */
struct g_cfe_softc {
};

/**
 * GEOM BCM_CFE device taste context
 */
struct g_cfe_taste_io {
	struct g_consumer	*cp;		/**< GEOM consumer (borrowed reference) */
	struct bcm_disk		*disk;		/**< disk entry (borrowed reference) */
	struct bcm_bootinfo	 bootinfo;	/**< disk bootinfo, if any */
	bool			 have_bootinfo;	/**< if bootinfo is available */
	off_t			 palign;	/**< minimum partition alignment */
	u_char			*buf;		/**< last read sector(s), or NULL */
	off_t			 buf_off;	/**< offset of last read */
	off_t			 buf_len;	/**< length of last read */
};

/**
 * GEOM BCM_CFE partition probe function.
 * 
 * @param io	The I/O context to be used for reading.
 * @param block	The block offset to be probed.
 * 
 * @retval non-NULL	the identified partition
 * @retval NULL		no match
 */
typedef struct bcm_part *(g_cfe_part_probe)(struct g_cfe_taste_io *io,
						off_t block);

/**
 * GEOM BCM_CFE partition probe description.
 */
struct g_cfe_part_probe_info {
	const char		*name;	/**< human-readable description */
	g_cfe_part_probe	*func;	/**< probe function */
};

SET_DECLARE(g_cfe_part_probe_set, struct g_cfe_part_probe_info);

#define	G_CFE_DEFINE_PART_PROBE(_name, _func)				\
	static const struct g_cfe_part_probe_info _func ## _info = {	\
		.name	= (_name),					\
		.func	= (_func),					\
	};								\
	DATA_SET(g_cfe_part_probe_set, _func ## _info)

/**
 * GEOM-backed bhnd_nvram_io implementation, for use when probing an NVRAM
 * partition.
 */
struct g_cfe_nvram_io {
	struct bhnd_nvram_io	 io;		/**< common I/O instance state */
	struct g_cfe_taste_io	*taste;		/**< GEOM I/O state */
	off_t			 offset;	/**< base I/O offset */
	off_t			 size;		/**< number of bytes readable at base I/O offset */
};


#endif /* _MIPS_BROADCOM_BCM_GEOM_CFE_H_ */
