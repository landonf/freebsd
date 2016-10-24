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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_NVRAM_BHND_NVRAM_STOREVAR_H_
#define _BHND_NVRAM_BHND_NVRAM_STOREVAR_H_

#include <sys/types.h>

#include "bhnd_nvram_store.h"

/** Index is only generated if minimum variable count is met */
#define	NVRAM_IDX_VAR_THRESH	15

/** Name prefix of device path aliases */
#define	NVRAM_DEVPATH_STR	"devpath"
#define	NVRAM_DEVPATH_LEN	(sizeof(NVRAM_DEVPATH_STR) - 1)

/**
 * NVRAM devpath record.
 * 
 * Aliases index values to full device paths.
 */
struct bhnd_nvram_devpath {
	u_long	 index;	/** alias index */
	char	*path;	/** aliased path */

	LIST_ENTRY(bhnd_nvram_devpath) dp_link;
};

/**
 * NVRAM store index.
 * 
 * Provides effecient name-based lookup by maintaining an array of cached
 * cookiep values, sorted lexicographically by variable name.
 */
struct bhnd_nvstore_index {
	size_t				 num_cookiep;	/**< cookiep count */
	void				*cookiep[];	/**< cookiep values */
};

LIST_HEAD(bhnd_nvram_devpaths, bhnd_nvram_devpath);

/** bhnd nvram store instance state */
struct bhnd_nvram_store {
	struct mtx			 mtx;
	struct bhnd_nvram_data		*nv;		/**< backing data */
	struct bhnd_nvstore_index	*idx;		/**< index, or NULL */
	struct bhnd_nvram_devpaths	 devpaths;	/**< device paths */
	nvlist_t			*pending;	/**< uncommitted writes */
};

#define	BHND_NVSTORE_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, "BHND NVRAM store lock", NULL, MTX_DEF)
#define	BHND_NVSTORE_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	BHND_NVSTORE_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	BHND_NVSTORE_LOCK_ASSERT(sc, what)	mtx_assert(&(sc)->mtx, what)
#define	BHND_NVSTORE_LOCK_DESTROY(sc)		mtx_destroy(&(sc)->mtx)

#endif /* _BHND_NVRAM_BHND_NVRAM_STOREVAR_H_ */
