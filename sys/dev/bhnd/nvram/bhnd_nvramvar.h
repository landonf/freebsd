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

#include <sys/kobj.h>
#include <sys/stddef.h>

#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sx.h>

#include "bhnd_nvram.h"

/**
 * NVRAM provider.
 */
struct bhnd_nvram_provider {
	KOBJ_FIELDS;

	volatile u_int				refs;		/**< reference count */
	_Alignas(_Alignof(max_align_t)) u_char	softc[];	/**< instance state */
};

/**
 * NVRAM entry.
 */
struct bhnd_nvram_entry {
	bhnd_nvram_plane		*plane;		/**< defining plane */
	bhnd_nvram_provider		*provider;	/**< provider */
	bhnd_nvram_phandle		 phandle;	/**< provider handle */
	volatile u_int			 refs;		/**< reference count */
};


/**
 * NVRAM plane, providing a hierarchical namespace of NVRAM entries and
 * associated entry properties.
 */
struct bhnd_nvram_plane {
	char			*name;		/**< assigned name */
	bhnd_nvram_provider	*provider;	/**< provider, or NULL if none */
	volatile u_int		 refs;		/**< reference count */
	struct sx		 lock;		/**< plane lock */
};

#define	BHND_NVPLANE_LOCK_INIT(sc) \
	sx_init(&(sc)->lock, "BHND NVRAM plane lock")
#define	BHND_NVPLANE_LOCK_RO(sc)		sx_slock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RO(sc)		sx_sunlock(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_RW(sc)		sx_xlock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RW(sc)		sx_xunlock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK(sc)			sx_unlock(&(sc)->lock)
#define	BHND_NVPLANE_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->lock)
#define	BHND_NVPLANE_DOWNGRADE(sc)		sx_downgrade(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_ASSERT(sc, what)	sx_assert(&(sc)->lock, what)
#define	BHND_NVPLANE_LOCK_DESTROY(sc)		sx_destroy(&(sc)->lock)

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
