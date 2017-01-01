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

#include <sys/kobj.h>
#include <sys/lock.h>
#include <sys/queue.h>
#include <sys/refcount.h>
#include <sys/sx.h>

#include <machine/atomic.h>

#include "bhnd_nvram.h"

struct bhnd_nvram_entry;
struct bhnd_nvram_provider;
struct bhnd_nvram_consumer;

LIST_HEAD(bhnd_nvram_entry_list, bhnd_nvram_entry);

/**
 * Reference count data structure supporting both strong and weak references.
 * 
 * One implicit weak reference is held by all strong references; when all
 * strong references are released, this weak reference is also released,
 * allowing deallocation to occur.
 *
 * This avoids ordering concerns around strong and weak reference behavior, at
 * the cost of one additional atomic operation upon discarding the last strong
 * reference.
 *
 *	- When the strong reference count hits zero, the referenced value's
 *	  instance state may be deallocated.
 *	- When the weak reference count hits zero, the reference counted data
 *	  structure itself may be deallocated.
 */
struct bhnd_nvref {
	volatile u_int	 strong;	/* strong refcount */
	volatile u_int	 weak;		/* weak refcount */
};

/**
 * NVRAM canonical path string.
 */
struct bhnd_nvpath {
	char			*pathname;	/**< canonical, fully qualified path name */
	const char		*filename;	/**< relative file name within pathname */
	struct bhnd_nvref	 refs;
};

// TODO: locking
#if 0

#define	BHND_NVRAM_LOCK_INIT(sc) \
	sx_init(&(sc)->topo_lock, "BHND NVRAM topology lock")
#define	BHND_NVRAM_LOCK_RD(sc)			sx_slock(&(sc)->topo_lock)
#define	BHND_NVRAM_UNLOCK_RD(sc)		sx_sunlock(&(sc)->topo_lock)
#define	BHND_NVRAM_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->topo_lock)
#define	BHND_NVRAM_LOCK_RW(sc)			sx_xlock(&(sc)->topo_lock)
#define	BHND_NVRAM_UNLOCK_RW(sc)		sx_xunlock(&(sc)->topo_lock)
#define	BHND_NVRAM_LOCK_ASSERT(sc, what)	\
	sx_assert(&(sc)->topo_lock, what)
#define	BHND_NVRAM_LOCK_DESTROY(sc)		sx_destroy(&(sc)->topo_lock)

#else

#define	BHND_NVRAM_LOCK_INIT(sc) \
	sx_init(&(sc)->topo_lock, "BHND NVRAM topology lock")
#define	BHND_NVRAM_LOCK_RD(sc)			sx_slock(&(sc)->topo_lock)
#define	BHND_NVRAM_UNLOCK_RD(sc)		sx_sunlock(&(sc)->topo_lock)
#define	BHND_NVRAM_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->topo_lock)
#define	BHND_NVRAM_LOCK_RW(sc)			sx_xlock(&(sc)->topo_lock)
#define	BHND_NVRAM_UNLOCK_RW(sc)		sx_xunlock(&(sc)->topo_lock)
#define	BHND_NVRAM_LOCK_ASSERT(sc, what)	\
	sx_assert(&(sc)->topo_lock, what)
#define	BHND_NVRAM_LOCK_DESTROY(sc)		sx_destroy(&(sc)->topo_lock)

#endif

/**
 * NVRAM provider.
 */
struct bhnd_nvram_provider {
	device_t			 dev;		/**< device */
	struct bhnd_nvram_entry_list	 entries;	/**< registered path entries */
	struct sx			 prov_lock;

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_provider)	 np_link;
};

#define	BHND_NVPROV_LOCK_INIT(sc) \
	sx_init(&(sc)->prov_lock, "BHND NVRAM provider lock")
#define	BHND_NVPROV_LOCK_RD(sc)			sx_slock(&(sc)->prov_lock)
#define	BHND_NVPROV_UNLOCK_RD(sc)		sx_sunlock(&(sc)->prov_lock)
#define	BHND_NVPROV_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->prov_lock)
#define	BHND_NVPROV_LOCK_RW(sc)			sx_xlock(&(sc)->prov_lock)
#define	BHND_NVPROV_UNLOCK_RW(sc)		sx_xunlock(&(sc)->prov_lock)
#define	BHND_NVPROV_LOCK_ASSERT(sc, what)	\
	sx_assert(&(sc)->prov_lock, what)
#define	BHND_NVPROV_LOCK_DESTROY(sc)		sx_destroy(&(sc)->prov_lock)

/**
 * NVRAM entry.
 */
struct bhnd_nvram_entry {
	struct bhnd_nvram_provider	*prov;		/**< exporting provider */
	struct bhnd_nvpath		*canon;		/**< provider's canonical path string */

	LIST_HEAD(,bhnd_nvram_consumer)	 consumers;	/**< planes consuming this entry */

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_entry)	 ne_link;
};

/**
 * NVRAM consumer.
 */
struct bhnd_nvram_consumer {
	struct bhnd_nvram_plane		*plane;		/**< consuming plane */

	LIST_ENTRY(bhnd_nvram_consumer)	 nc_link;
};

/**
 * NVRAM path adjacency list entry.
 * 
 * An individual NVRAM path may be registered in one canonical plane, and then
 * re-exported in additional planes, or within the same plane at an additional
 * position.
 *
 * The plane-specific graph of binary parent/child path relations is
 * represented as a table of bhnd_nvram_link instances.
 */
struct bhnd_nvram_link {
	struct bhnd_nvpath		*path;		/**< plane-specific path string */
	struct bhnd_nvram_link		*parent;	/**< parent, or NULL */
	struct bhnd_nvram_entry		*entry;		/**< provider entry, or NULL */

	LIST_HEAD(,bhnd_nvram_link)	 children;	/**< all children */

	LIST_ENTRY(bhnd_nvram_link)	 nl_link;
};

/**
 * NVRAM plane.
 * 
 * Manages a tree of NVRAM planes, NVRAM paths, and associated NVRAM devices.
 */
struct bhnd_nvram_plane {
	struct bhnd_nvram_plane		*parent;	/**< parent, or NULL */
	struct bhnd_nvram_link		*root;		/**< root path, or NULL */

	LIST_HEAD(,bhnd_nvram_plane)	 children;	/**< children */

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_plane)	 child_link;
};

// TODO: locking
#define	BHND_NVPLANE_LOCK_RD(p)
#define	BHND_NVPLANE_UNLOCK_RD(p)
#define	BHND_NVPLANE_LOCK_RW(p)
#define	BHND_NVPLANE_UNLOCK_RW(p)
#define	BHND_NVPLANE_LOCK_ASSERT(p, what)

/**
 * Initialize a the reference count structure.
 * 
 * @param ref	A reference count structure.
 */
#define	BHND_NVREF_INIT(ref) do {					\
	/* Implicit initial strong reference */				\
	refcount_init(&(ref)->strong, 1);				\
									\
	/* Single weak reference shared by all strong references */	\
	refcount_init(&(ref)->weak, 1);					\
} while(0)

/**
 * Retain a strong reference to @p value and return @p value.
 * 
 * @param value	The referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_RETAIN(value, field)	\
	(refcount_acquire(&((value)->field.strong)), (value))

/**
 * Release a strong reference to @p value, possibly deallocating @p value.
 * 
 * @param value		The referenced value.
 * @param field		The value's reference count field.
 * @param fini		The value's finalization callback.
 * @param ...		Additional arguments to be passed to @p fini, if any.
 */
#define	BHND_NVREF_RELEASE(value, field, fini, ...) do {		\
	/* Drop strong reference */					\
	if (refcount_release(&(value)->field.strong)) {			\
		/* No remaining strong references; can finalize		\
		 * instance state. Our implicit weak reference will	\
		 * keep the value pointer alive during finalization */	\
		(fini)(value, ##__VA_ARGS__);				\
									\
		/* Discard the the implicit weak reference shared by	\
		 * all strong references. */				\
		BHND_NVREF_RELEASE_WEAK((value), field);		\
	}								\
} while(0)

/**
 * Retain a weak reference to @p value and return @p value.
 * 
 * @param value	The strongly referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_RETAIN_WEAK(value, field)	\
	(refcount_acquire(&((value)->field.weak)), (value))

/**
 * Release a weak reference, possibly deallocating @p value.
 * 
 * @param value		The weakly referenced value.
 * @param field		The value's reference count field.
 */
#define	BHND_NVREF_RELEASE_WEAK(value, field) do {			\
	/* Drop weak reference */					\
	if (refcount_release(&(value)->field.weak))			\
		bhnd_nv_free(value);					\
} while (0)

/**
 * Promote a weak reference to a strong reference, returning the referenced
 * value on success, or NULL if the value is a zombie -- it has no further
 * strong references remaining.
 * 
 * @param value	The weakly referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_PROMOTE_WEAK(value, field)			\
	(bhnd_nvref_promote_weak(&(value)->field) ?	\
	    (value) : (NULL))

static inline bool
bhnd_nvref_promote_weak(struct bhnd_nvref *ref)
{
	u_int old;

	/* Increment the reference count using compare-and-swap to ensure that
	 * we don't resurrect a value with a strong refcount of zero
	 * (a 'zombie') */
	do {
		/* fetch current strong refcount. if zero, value is a zombie,
		 * and cannot be strongly retained  */
		old = ref->strong;
		if (old == 0)
			return (false);

	} while (!atomic_cmpset_acq_int(&ref->strong, old, old+1));

	return (true);
}

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
