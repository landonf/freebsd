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

#ifndef _BHND_NVRAM_BHND_NVRAMVAR_H_
#define _BHND_NVRAM_BHND_NVRAMVAR_H_

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sx.h>

#include "bhnd_nvram.h"

LIST_HEAD(bhnd_nvram_plane_list, bhnd_nvram_plane);
typedef struct bhnd_nvram_plane_list bhnd_nvram_plane_list_t;

/**
 * NVRAM plane provider mapping.
 */
typedef struct bhnd_nvram_plane_pmap {
	bhnd_nvram_prov_t	*pm_prov;	/**< provider instance */
	volatile u_int		 pm_reqs;	/**< active provider request count */
	struct mtx		 pm_lock;	/**< request count mutex */
} bhnd_nvram_plane_pmap_t;

/**
 * NVRAM plane.
 * 
 * Provides a hierarchical namespace of child planes associated per-plane NVRAM
 * providers.
 * 
 * Locking Protocol:
 * - Hierarchical locks must be acquired in parent -> child order.
 * - Peer locks must be acquired in children[n], children[n+1] order.
 * - Locks may be released in any order.
 * - After releasing a lock, no locks may be acquired until all previously
 *   acquired locks have also been released.
 */
struct bhnd_nvram_plane {
	bhnd_nvram_plane_t	*parent;	/**< parent, or NULL */
	bhnd_nvram_plane_pmap_t	*pmap;	/**< provider mapping, or NULL */
	char			*name;		/**< plane's relative name */
	bhnd_nvram_plane_list_t	 children;	/**< child planes */
	volatile u_int		 refs;		/**< reference count */
	struct sx		 lock;		/**< plane lock */

	LIST_ENTRY(bhnd_nvram_plane) np_link;
};

#define	BHND_NVPLANE_LOCK_INIT(sc) \
	sx_init(&(sc)->lock, "BHND NVRAM plane lock")
#define	BHND_NVPLANE_LOCK_RO(sc)		sx_slock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RO(sc)		sx_sunlock(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_RW(sc)		sx_xlock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RW(sc)		sx_xunlock(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_ASSERT(sc, what)	sx_assert(&(sc)->lock, what)
#define	BHND_NVPLANE_LOCK_DESTROY(sc)		sx_destroy(&(sc)->lock)

/** maximum representable number of outstanding provider requests */
#define	BHND_NVPLANE_PROV_REQS_MAX	UINT_MAX

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
