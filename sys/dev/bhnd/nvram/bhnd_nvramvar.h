/*-
 * Copyright (c) 2014-2016 Landon Fuller <landonf@FreeBSD.org>
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

#ifndef _BHND_NVRAM_BHND_NVRAMVAR_H_
#define _BHND_NVRAM_BHND_NVRAMVAR_H_

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/queue.h>
#include <sys/refcount.h>
#include <sys/sx.h>

#include "bhnd_nvref.h"

typedef struct bhnd_nvram_dev_entry		bhnd_nvram_dev_entry;
typedef struct bhnd_nvram_path_entry		bhnd_nvram_path_entry;
typedef struct bhnd_nvram_prov			bhnd_nvram_prov;

LIST_HEAD(bhnd_nvram_path_entry_list,		bhnd_nvram_path_entry);
LIST_HEAD(bhnd_nvram_plane_list,		bhnd_nvram_plane);
LIST_HEAD(bhnd_nvram_dev_entry_list,		bhnd_nvram_dev_entry);

typedef struct bhnd_nvram_path_entry_list	bhnd_nvram_path_entry_list;
typedef struct bhnd_nvram_plane_list		bhnd_nvram_plane_list;
typedef struct bhnd_nvram_dev_entry_list	bhnd_nvram_dev_entry_list;

/**
 * NVRAM path provider types.
 */
typedef enum {
	BHND_NVRAM_PROVIDER_NONE	= 0,	/**< no data provider */
	BHND_NVRAM_PROVIDER_DEV		= 1,	/**< device provider */
	BHND_NVRAM_PROVIDER_PATH	= 2,	/**< re-exported path provider */
} bhnd_nvram_prov_type;

/**
 * NVRAM device entry.
 */
struct bhnd_nvram_dev_entry {
	device_t				dev;	/**< provider */

	struct bhnd_nvref			dn_refs;
	LIST_ENTRY(bhnd_nvram_dev_entry)	dn_link;
};

/**
 * NVRAM path provider state.
 */
struct bhnd_nvram_prov {
	bhnd_nvram_prov_type		 type;	/**< provider type */

	/** type-specific provider reference */
	union bhnd_nvram_prov_src {
		bhnd_nvram_dev_entry	*dev;	/**< NVRAM device (BHND_NVRAM_PROVIDER_DEV) */
		bhnd_nvram_phandle	*path;	/**< reexported parent path (BHND_NVRAM_PROVIDER_PATH) */
	} src;
};

/**
 * NVRAM path entry.
 */
struct bhnd_nvram_path_entry {
	bhnd_nvram_phandle		*phandle;	/**< path handle */
	const char			*name;		/**< relative name (borrowed from phandle)*/
	bhnd_nvram_prov			 prov;		/**< data source */
	bhnd_nvram_path_entry		*parent;	/**< strong parent reference, or NULL */
	bhnd_nvram_path_entry_list	 children;	/**< weak references to all children */
	struct bhnd_nvref		 np_refs;

	LIST_ENTRY(bhnd_nvram_path_entry) np_child_link;	/**< path child list link */
	LIST_ENTRY(bhnd_nvram_path_entry) np_hash_link;		/**< path hash table list link */
};

/**
 * NVRAM path handle.
 * 
 * Provides a reference-counted handle to a path within an NVRAM plane.
 */
struct bhnd_nvram_phandle {
	char			*pathname;	/**< fully qualified path name */
	struct bhnd_nvram_plane	*plane;		/**< weak plane reference */

	struct bhnd_nvref	 np_refs;
};

/**
 * NVRAM plane.
 * 
 * Manages a common namespace of NVRAM paths and associated NVRAM devices.
 */
struct bhnd_nvram_plane {
	struct bhnd_nvram_plane		*parent;	/**< parent plane, or NULL */

	bhnd_nvram_path_entry_list	 paths[4];	/**< all paths, hashed by fully-qualified name */
	bhnd_nvram_dev_entry_list	 devices;	/**< registered devices */
	bhnd_nvram_plane_list		 children;	/**< weak references to all children */
	struct sx			 lock;		/**< topology lock */

	struct bhnd_nvref		 np_refs;
	LIST_ENTRY(bhnd_nvram_plane)	 np_link;
};

#define	BHND_NVPLANE_LOCK_INIT(sc) \
	sx_init(&(sc)->lock, "BHND NVRAM plane lock")
#define	BHND_NVPLANE_LOCK_RD(sc)		sx_slock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RD(sc)		sx_sunlock(&(sc)->lock)
#define	BHND_NVPLANE_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_RW(sc)		sx_xlock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RW(sc)		sx_xunlock(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_ASSERT(sc, what)	sx_assert(&(sc)->lock, what)
#define	BHND_NVPLANE_LOCK_DESTROY(sc)		sx_destroy(&(sc)->lock)

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
