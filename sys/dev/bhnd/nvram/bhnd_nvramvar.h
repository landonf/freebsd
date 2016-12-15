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

#ifndef _BHND_NVRAM_BHND_NVRAMVAR_H_
#define _BHND_NVRAM_BHND_NVRAMVAR_H_

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/queue.h>
#include <sys/refcount.h>
#include <sys/sx.h>

LIST_HEAD(bhnd_nvram_phandle_list,	bhnd_nvram_phandle);
LIST_HEAD(bhnd_nvram_plane_list,	bhnd_nvram_plane);
LIST_HEAD(bhnd_nvram_devnode_list,	bhnd_nvram_devnode);

typedef struct bhnd_nvram_phandle_list	bhnd_nvram_phandle_list;
typedef struct bhnd_nvram_plane_list	bhnd_nvram_plane_list;
typedef struct bhnd_nvram_devnode_list	bhnd_nvram_devnode_list;

/**
 * Simple weak reference implementation.
 * 
 * Each strong reference also holds a weak reference; when the strong
 * reference count hits zero, any additional attempts to promote a weak
 * reference to a strong reference will be rejected.
 * 
 * When the weak reference count hits zero, the value may be safely
 * deallocated.
 */
struct bhnd_nvref {
	volatile u_int	 srefs;			/**< strong references */
	volatile u_int	 wrefs;			/**< weak references */
	void		*value;			/**< refcounted value */
	void		(*vfree)(void *value);	/**< free() callback */
};

/**
 * NVRAM device entry.
 */
struct bhnd_nvram_devnode {
	device_t		dev;	/**< provider */
	struct bhnd_nvref	refs;	/**< reference count */

	LIST_ENTRY(bhnd_nvram_devnode) dn_link;
};

/**
 * NVRAM path handle.
 * 
 * Provides a reference-counted handle to an open path within an NVRAM plane.
 */
struct bhnd_nvram_phandle {
	struct bhnd_nvref	 refs;		/**< reference count */
	bhnd_nvram_phandle	*parent;	/**< parent path, or NULL */

	char			*path;		/**< fully qualified path */
	const char		*name;		/**< relative name */

	struct bhnd_nvram_plane	*plane;		/**< weak reference to plane */
	bhnd_nvram_phandle_list	 children;	/**< weak references to all children */

	LIST_ENTRY(bhnd_nvram_phandle) np_link;
};

/**
 * NVRAM plane.
 * 
 * Manages a common namespace of NVRAM paths and associated NVRAM devices.
 */
struct bhnd_nvram_plane {
	struct bhnd_nvref		 refs;		/**< reference count */
	struct bhnd_nvram_plane		*parent;	/**< parent plane, or
							     NULL */

	bhnd_nvram_phandle		*root;		/**< root path */
	bhnd_nvram_devnode_list		 devices;	/**< registered devices */
	bhnd_nvram_plane_list		 children;	/**< weak references to
							     all children */
	struct sx			 lock;		/**< topology lock */

	LIST_ENTRY(bhnd_nvram_plane) np_link;
};

#define	BHND_NVPLANE_LOCK_INIT(sc) \
	sx_init(&(sc)->lock, "BHND NVRAM plane lock")
#define	BHND_NVPLANE_LOCK_RD(sc)		sx_slock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RD(sc)		sx_sunlock(&(sc)->lock)
#define	BHND_NVPLANE_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_RW(sc)		sx_slock(&(sc)->lock)
#define	BHND_NVPLANE_UNLOCK_RW(sc)		sx_sunlock(&(sc)->lock)
#define	BHND_NVPLANE_LOCK_ASSERT(sc, what)	sx_assert(&(sc)->lock, what)
#define	BHND_NVPLANE_LOCK_DESTROY(sc)		sx_destroy(&(sc)->lock)

/**
 * Initialize reference count state.
 */
static inline void
bhnd_nvref_init(struct bhnd_nvref *ref, void *value, void (*vfree)(void *))
{
	/* Single strong reference, plus corresponding weak reference */
	refcount_init(&ref->srefs, 1);
	refcount_init(&ref->wrefs, 1);

	ref->value = value;
	ref->vfree = vfree;
}

/**
 * Return true if the reference is dead; it has no active strong references.
 */
static inline bool
bhnd_nvref_is_zombie(struct bhnd_nvref *ref)
{
	if (atomic_load_acq_int(&ref->srefs) > 0)
		return (false);

	return (true);
}

/**
 * Acquire a weak reference from an existing strong reference.
 */
static inline void
bhnd_nvref_retain_weak(struct bhnd_nvref *ref)
{
	BHND_NV_ASSERT(ref->srefs >= 1, ("dead value"));
	BHND_NV_ASSERT(ref->wrefs >= 1, ("overrelease"));

	refcount_acquire(&ref->wrefs);
}

/**
 * Release a weak reference. If this was the last reference (weak or strong),
 * the value will be deallocated.
 * 
 * Returns true if the value was deallocated, false otherwise.
 */
static inline bool
bhnd_nvref_release_weak(struct bhnd_nvref *ref)
{
	BHND_NV_ASSERT(ref->wrefs >= 1, ("overrelease"));
	BHND_NV_ASSERT(ref->srefs >= ref->wrefs, ("lost weak ref"));

	if (!refcount_release(&ref->wrefs))
		return (false);

	if (ref->vfree != NULL)
		ref->vfree(ref->value);

	return (true);
}

/**
 * Acquire a strong reference from an existing strong reference.
 */
static inline void
bhnd_nvref_retain(struct bhnd_nvref *ref)
{
	BHND_NV_ASSERT(ref->srefs >= 1, ("over-release"));

	/* All strong references also hold a weak reference */
	refcount_acquire(&ref->wrefs);
}

/**
 * Release a strong reference. If this was the last reference (weak or strong),
 * the value will be deallocated.
 * 
 * Returns true if the value was deallocated, false otherwise.
 */
static inline bool
bhnd_nvref_release(struct bhnd_nvref *ref)
{
	BHND_NV_ASSERT(ref->srefs >= 1, ("over-release"));

	/* Drop strong reference */
	refcount_release(&ref->srefs);

	/* All strong references also hold a weak reference. If this was
	 * the last weak reference, this will result in deallocation. */
	return (bhnd_nvref_release_weak(ref));
}

/**
 * Promote a weak reference to a strong reference, returning true on success,
 * false on failure.
 */
static inline bool
bhnd_nvref_promote(struct bhnd_nvref *ref)
{
	u_int srefs;

	do {
		/* Fetch current reference count */
		srefs = ref->srefs;

		/* If the strong reference count is already zero, the object is
		 * already dead */
		if (ref->srefs == 0)
			return (false);

	} while (!atomic_cmpset_acq_int(&ref->srefs, srefs, srefs+1));

	return (true);
}

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
