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

LIST_HEAD(bhnd_nvram_phandle_list,	bhnd_nvram_phandle);
LIST_HEAD(bhnd_nvram_plane_list,	bhnd_nvram_plane);
LIST_HEAD(bhnd_nvram_prov_list,		bhnd_nvram_prov);

typedef struct bhnd_nvram_phandle_list	bhnd_nvram_phandle_list;
typedef struct bhnd_nvram_plane_list	bhnd_nvram_plane_list;
typedef struct bhnd_nvram_prov_list	bhnd_nvram_prov_list;

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
 * NVRAM provider.
 */
struct bhnd_nvram_prov {
	device_t			 dev;		/**< device */
	bhnd_nvram_phandle_list		 paths;		/**< registered paths */
	struct bhnd_nvram_plane		*plane;		/**< plane (weak reference) */

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_prov)	 providers_link;
};

/**
 * NVRAM path handle.
 * 
 * Provides a reference-counted handle to a path within an NVRAM plane.
 * 
  * Locking / Reference Counting:
 *
 *	- Child paths hold strong references to their parent; parents hold
 *	  weak references to children.
 *	- Child paths must lock their parents; parents never directly lock
 *	  children.
 *	- Weak references to a child must be explicitly promoted to a strong
 *	  reference  before access, which may fail if all strong references to
 *	  the child path have been released. See BHND_NVREF_PROMOTE_WEAK().
 */
struct bhnd_nvram_phandle {
	struct bhnd_nvref		 refs;

	/* immutable state */
	char				*pathname;	/**< fully qualified path name */
	const char			*name;		/**< relative path name within pathname */
	bhnd_nvram_phandle		*parent;	/**< parent, or NULL */

	/* mutable state */
	struct bhnd_nvram_prov		*prov;		/**< provider, or NULL if none (weak reference) */
	bhnd_nvram_phandle_list		 children;	/**< all children */
	struct sx			 p_lock;	/**< mutable state lock */

	/* externally managed state */
	LIST_ENTRY(bhnd_nvram_phandle)	 children_link;	/**< parent children list entry */
	LIST_ENTRY(bhnd_nvram_phandle)	 pathlist_link;	/**< provider path list entry */
};

#define	BHND_NVPATH_LOCK_INIT(sc) \
	sx_init(&(sc)->p_lock, "BHND NVRAM path lock")
#define	BHND_NVPATH_LOCK_RD(sc)			sx_slock(&(sc)->p_lock)
#define	BHND_NVPATH_UNLOCK_RD(sc)		sx_sunlock(&(sc)->p_lock)
#define	BHND_NVPATH_TRY_UPGRADE(sc)		sx_try_upgrade(&(sc)->p_lock)
#define	BHND_NVPATH_LOCK_RW(sc)			sx_xlock(&(sc)->p_lock)
#define	BHND_NVPATH_UNLOCK_RW(sc)		sx_xunlock(&(sc)->p_lock)
#define	BHND_NVPATH_LOCK_ASSERT(sc, what)	sx_assert(&(sc)->p_lock, what)
#define	BHND_NVPATH_LOCK_DESTROY(sc)		sx_destroy(&(sc)->p_lock)

/**
 * NVRAM plane.
 * 
 * Manages a tree of NVRAM planes, NVRAM paths, and associated NVRAM devices.
 *
 * Locking / Reference Counting:
 *
 *	- Child planes hold strong references to parent planes; parents hold
 *	  weak references to children.
 *	- Child planes must lock their parents; parents never directly lock
 *	  children.
 *	- Weak references to a child must be explicitly promoted to a strong
 *	  reference  before access, which may fail if all strong references to
 *	  the child plane have been released. See BHND_NVREF_PROMOTE_WEAK().
 */
struct bhnd_nvram_plane {
	struct bhnd_nvref		 refs;

	/* immutable state */
	struct bhnd_nvram_plane		*parent;	/**< parent plane, or NULL */
	bhnd_nvram_phandle		*root;		/**< root path */

	/* mutable state */
	bhnd_nvram_prov_list		 providers;	/**< registered providers */
	bhnd_nvram_plane_list		 children;	/**< weak references to all children */
	struct sx			 lock;		/**< mutable state lock */

	/* externally managed state */
	LIST_ENTRY(bhnd_nvram_plane)	 children_link;	/**< parent children list entry */
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
 */
#define	BHND_NVREF_RELEASE(value, field, fini) do {			\
	/* Drop strong reference */					\
	if (refcount_release(&(value)->field.strong)) {			\
		/* No remaining strong references; can finalize		\
		 * instance state. Our implicit weak reference will	\
		 * keep the value pointer alive during finalization */	\
		(fini)(value);						\
									\
		/* Discard the the implicit weak reference shared by	\
		 * all strong references. */				\
		BHND_NVREF_RELEASE_WEAK((value), field);		\
	}								\
} while(0)

/**
 * Assert that @p value is has no remaining strong references and instance
 * state can be safely deallocated.
 * 
 * @param value	The referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_ASSERT_CAN_FREE(value, field) do {		\
	BHND_NV_ASSERT(						\
	    atomic_load_acq_int(&(value)->field.strong) == 0,	\
	    ("cannot free live value"));			\
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
