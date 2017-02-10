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

#ifndef _BHND_NVRAM_BHND_NVRAM_STORE_H_
#define _BHND_NVRAM_BHND_NVRAM_STORE_H_

#include <sys/param.h>

#include <sys/bus.h>
#include <sys/kobj.h>

#include <sys/queue.h>

#include "bhnd_nvram.h"
#include "bhnd_nvram_data.h"

/**
 * NVRAM sync callback.
 * 
 * @param 
 * @param force	If true, perform the write immediately. If false, the NVRAM
 *		provider may arbitrarily delay or coalesce writes to prevent
 *		unnecessary flash memory wear.
 * @param ctx	Opaque context.
 */
typedef int (bhnd_nvram_store_sync_f)(struct bhnd_nvram_provider *prov, bool force, void *ctx);

/**
 * NVRAM store initialization parameters.
 */
struct bhnd_nvram_store_init_params {
	struct bhnd_nvram_data	*data;		/**< NVRAM data */
	bhnd_nvram_store_sync_f	 sync;		/**< sync callback */
	void			*sync_ctx;	/**< sync context */
};

DECLARE_CLASS(bhnd_nvram_store_provider);

// TODO: OLD

struct bhnd_nvram_store;

/**
 * NVRAM export flags.
 */
enum {
	BHND_NVSTORE_EXPORT_CHILDREN		= (1<<0),	/**< Include all subpaths */
	BHND_NVSTORE_EXPORT_PRESERVE_DEVPATHS	= (1<<1),	/**< Preserve existing device path definitions (default) */
	BHND_NVSTORE_EXPORT_COMPACT_DEVPATHS	= (1<<2),	/**< Re-encode all device paths using compact syntax */
	BHND_NVSTORE_EXPORT_EXPAND_DEVPATHS	= (1<<3),	/**< Re-encode all device paths using non-compact syntax */
	BHND_NVSTORE_EXPORT_ALL_VARS		= (1<<6|1<<7),	/**< Include all variables (default) */
	BHND_NVSTORE_EXPORT_COMMITTED		= (1<<6),	/**< Include all committed changes */
	BHND_NVSTORE_EXPORT_UNCOMMITTED		= (1<<7),	/**< Include all uncommitted changes (not including deletions) */
	BHND_NVSTORE_EXPORT_DELETED		= (1<<8),	/**< Include all uncommitted deletions (as
								     properties of type BHND_NVRAM_TYPE_NULL) */
};


int	bhnd_nvram_store_new(struct bhnd_nvram_store **store,
	    struct bhnd_nvram_data *data);

int	bhnd_nvram_store_export(struct bhnd_nvram_store *store,
	    const char *path, bhnd_nvram_data_class **cls,
	    bhnd_nvram_plist **props, bhnd_nvram_plist **options,
	    uint32_t flags);

int	bhnd_nvram_store_serialize(struct bhnd_nvram_store *store,
	    const char *path, struct bhnd_nvram_io **data,  uint32_t flags);

#endif /* _BHND_NVRAM_BHND_NVRAM_STORE_H_ */
