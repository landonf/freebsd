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
#include <sys/queue.h>
#include <sys/refcount.h>

#ifdef _KERNEL
#include <sys/lock.h>
#include <sys/sx.h>
#else /* !_KERNEL */
#include <pthread.h>
#endif /* __KERNEL */

#include <machine/atomic.h>

#include "bhnd_nvram.h"

LIST_HEAD(bhnd_nvram_consumer_list,	bhnd_nvram_consumer);
LIST_HEAD(bhnd_nvram_link_list,		bhnd_nvram_link);

typedef struct bhnd_nvram_consumer bhnd_nvram_consumer;

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
struct bhnd_nvpath_str {
	char			*pathname;	/**< canonical, fully qualified path name */
	size_t			 pathlen;	/**< length of pathname */

	const char		*basename;	/**< relative file name within pathname */
	size_t			 baselen;	/**< length of basename */

	struct bhnd_nvref	 refs;
};

/**
 * NVRAM consumer record.
 */
struct bhnd_nvram_consumer {
	struct bhnd_nvram_plane		*plane;		/**< referencing plane (weak ref) */
	size_t				 uses;		/**< the number of plane references */

	struct bhnd_nvref		 refs;
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
	struct bhnd_nvpath_str		*path;		/**< plane-specific path string */
	struct bhnd_nvram_link		*parent;	/**< plane-specific parent, or NULL */

	struct bhnd_nvram_provider	*prov;		/**< provider, or NULL */
	struct bhnd_nvpath_str		*prov_path;	/**< provider's canonical path, or NULL */

	struct bhnd_nvram_link_list	 children;	/**< all children */

	LIST_ENTRY(bhnd_nvram_link)	 child_link;
	LIST_ENTRY(bhnd_nvram_link)	 hash_link;
};

/**
 * NVRAM plane.
 * 
 * Manages a tree of NVRAM planes, NVRAM paths, and associated NVRAM devices.
 */
struct bhnd_nvram_plane {
	struct bhnd_nvram_plane		*parent;	/**< parent, or NULL */
	struct bhnd_nvram_link		*root;		/**< root ("/") */
	struct bhnd_nvram_link_list	 prov_map[4];	/**< provider -> link(s) map */

	LIST_HEAD(,bhnd_nvram_plane)	 children;	/**< children */

#ifdef _KERNEL
	struct sx			 plane_lock;
#else /*! _KERNEL */
	pthread_rwlock_t		 plane_lock;
#endif /* _KERNEL */

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_plane)	 child_link;
};

#ifdef _KERNEL

#define	BHND_NVPLANE_LOCK_INIT(sc) \
	sx_init(&(sc)->plane_lock, "BHND NVRAM plane lock")
#define	BHND_NVPLANE_LOCK_RD(sc)		sx_slock(&(sc)->plane_lock)
#define	BHND_NVPLANE_UNLOCK_RD(sc)		sx_sunlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_LOCK_RW(sc)		sx_xlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_UNLOCK_RW(sc)		sx_xunlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_LOCK_ASSERT(sc, what)	\
	sx_assert(&(sc)->plane_lock, what)
#define	BHND_NVPLANE_LOCK_DESTROY(sc)		sx_destroy(&(sc)->plane_lock)

#else /* !_KERNEL */

#define	BHND_NVPLANE_LOCK_INIT(sc) do {					\
	int error = pthread_rwlock_init(&(sc)->plane_lock, NULL);	\
	if (error)							\
		BHND_NV_PANIC("pthread_rwlock_init() failed: %d",	\
		    error);						\
} while(0)

#define	BHND_NVPLANE_LOCK_RD(sc)	pthread_rwlock_rdlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_UNLOCK_RD(sc)	pthread_rwlock_unlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_LOCK_RW(sc)	pthread_rwlock_wrlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_UNLOCK_RW(sc)	pthread_rwlock_unlock(&(sc)->plane_lock)
#define	BHND_NVPLANE_LOCK_DESTROY(sc)	\
	pthread_rwlock_destroy(&(sc)->plane_lock)
#define	BHND_NVPLANE_LOCK_ASSERT(sc, what)

#endif /* _KERNEL */


/**
 * NVRAM provider state.
 */
typedef enum {
	BHND_NVRAM_PROV_ACTIVE		= 0,
	BHND_NVRAM_PROV_STOPPING	= 1,
	BHND_NVRAM_PROV_DEAD		= 2,
} bhnd_nvram_prov_state;

/**
 * NVRAM provider.
 */
struct bhnd_nvram_provider {
	device_t			 dev;		/**< device */
	struct bhnd_nvram_consumer_list	 consumers;	/**< all consumers */
	bhnd_nvram_prov_state		 state;		/**< current provider state */
	volatile u_int			 busy;		/**< busy count. may be incremented
							     atomically with read lock held. */

#ifdef _KERNEL
	struct sx			 prov_lock;
#else /*! _KERNEL */
	pthread_mutex_t			 prov_lock;
	pthread_cond_t			 prov_cond;
#endif /* _KERNEL */

	struct bhnd_nvref		 refs;
};

#ifdef _KERNEL

#define	BHND_NVPROV_LOCK_INIT(sc) \
	sx_init(&(sc)->prov_lock, "BHND NVRAM provider lock")
#define	BHND_NVPROV_LOCK_RD(sc)			sx_slock(&(sc)->prov_lock)
#define	BHND_NVPROV_UNLOCK_RD(sc)		sx_sunlock(&(sc)->prov_lock)
#define	BHND_NVPROV_LOCK_RW(sc)			sx_xlock(&(sc)->prov_lock)
#define	BHND_NVPROV_UNLOCK_RW(sc)		sx_xunlock(&(sc)->prov_lock)

#define	BHND_NVPROV_LOCK_WAIT(sc)		\
	sx_sleep(&(sc)->prov_lock, &(sc)->prov_lock, 0, "bhnd_nvprov", 0);
#define	BHND_NVPROV_LOCK_WAKEUP(sc)		wakeup(&(sc)->prov_lock);

#define	BHND_NVPROV_LOCK_ASSERT(sc, what)	\
	sx_assert(&(sc)->prov_lock, what)
#define	BHND_NVPROV_LOCK_DESTROY(sc)		sx_destroy(&(sc)->prov_lock)

#else /* !_KERNEL */

#define	BHND_NVPROV_LOCK_INIT(sc) do {					\
	int error = pthread_mutex_init(&(sc)->prov_lock, NULL);		\
	if (error)							\
		BHND_NV_PANIC("pthread_mutex_init() failed: %d",	\
		    error);						\
									    \
	error = pthread_cond_init(&(sc)->prov_cond, NULL);		\
	if (error)							\
		BHND_NV_PANIC("pthread_cond_init() failed: %d", error);	\
} while(0)

#define	BHND_NVPROV_LOCK_RD(sc)		pthread_mutex_lock(&(sc)->prov_lock)
#define	BHND_NVPROV_UNLOCK_RD(sc)	pthread_mutex_unlock(&(sc)->prov_lock)
#define	BHND_NVPROV_LOCK_RW(sc)		BHND_NVPROV_LOCK_RD(sc)
#define	BHND_NVPROV_UNLOCK_RW(sc)	BHND_NVPROV_UNLOCK_RD(sc)

#define	BHND_NVPROV_LOCK_WAIT(sc)	\
	pthread_cond_wait(&(sc)->prov_cond, &(sc)->prov_lock);
#define	BHND_NVPROV_LOCK_WAKEUP(sc)	\
	pthread_cond_broadcast(&(sc)->prov_cond);

#define	BHND_NVPROV_LOCK_DESTROY(sc)	pthread_mutex_destroy(&(sc)->prov_lock)
#define	BHND_NVPROV_LOCK_ASSERT(sc, what)

#endif /* _KERNEL */

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
		atomic_thread_fence_acq();				\
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

/**
 * Return the current strong reference count of a strongly or weakly held
 * @p value.
 * 
 * The reference count is provided for informational purposes only; there are
 * no external atomicity gaurantees.
 */
#define	BHND_NVREF_REFCOUNT(value, field)	\
	(atomic_load_acq_int(&(value)->field.strong))

/**
 * Return true if the given weak reference is a zombie-- it has no further
 * strong references remaining, and promotion to a strong reference would
 * fail.
 */
#define	BHND_NVREF_IS_ZOMBIE(value, field)	\
	(BHND_NVREF_REFCOUNT(value, field) == 0)

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
