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
#include <sys/stddef.h>
#include <sys/sx.h>
#else /* !_KERNEL */
#include <pthread.h>
#include <stddef.h>
#endif /* __KERNEL */

#include <machine/atomic.h>

#include "bhnd_nvram.h"

LIST_HEAD(bhnd_nvram_consumer_list,	bhnd_nvram_consumer);
LIST_HEAD(bhnd_nvram_entry_list,	bhnd_nvram_entry);
LIST_HEAD(bhnd_nvram_observer_list,	bhnd_nvram_observer);

typedef struct bhnd_nvram_observer bhnd_nvram_observer;
typedef struct bhnd_nvram_consumer bhnd_nvram_consumer;

struct bhnd_nvobj;
struct bhnd_nvobj_class;

/**
 * NVRAM observable events.
 */
typedef enum {
	/** The object has been disconnected from its enclosing topology and
	 *  marked as invalid; all listeners should discard any held references
	 *  where possible */
	BHND_NVRAM_EVENT_DISCONNECT	= 2,
} bhnd_nvram_event;

/**
 * Event dispatch handler.
 * 
 * @param self		The observing object.
 * @param target	The dispatching object.
 * @param event		The dispatched event.
 * @param info		Event-specific information, if any.
 */
typedef void (bhnd_nvobj_observer_fn)(struct bhnd_nvobj *self,
    struct bhnd_nvobj *target, bhnd_nvram_event event, struct bhnd_nvobj *info);

/**
 * Object finalization operation.
 * 
 * @param self		The observing object.
 */
typedef void (bhnd_nvobj_fini_op)(struct bhnd_nvobj *self);

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
 * Reference-counted NVRAM lock
 */
struct bhnd_nvlock {
#ifdef _KERNEL
	struct sx		nv_lock;
#else /*! _KERNEL */
	pthread_mutex_t		nv_lock;
	pthread_cond_t		nv_cond;
#endif /* _KERNEL */

	struct bhnd_nvref	refs;
};

/**
 * Generic NVRAM object.
 * 
 * Provides reference counting (including both strong and weak references)
 * and generic event listener registration.
 */
struct bhnd_nvobj {
	const struct bhnd_nvobj_class		*cls;		/**< class definition */
	struct bhnd_nvref			 refs;		/**< reference count */
	struct bhnd_nvram_observer_list		 observers;	/**< registered observers (weak refs) */
#ifdef _KERNEL
	struct sx				 obs_lock;	/**< observer list lock */
	struct sx				 obs_free_lock;	/**< must be held to remove/free observer entries */
#else /* !_KERNEL */
	pthread_rwlock_t			 obs_lock;
	pthread_mutex_t				 obs_free_lock;
#endif /* _KERNEL */

	_Alignas(_Alignof(max_align_t)) u_char	 ivars[];
};

/**
 * NVRAM object class description.
 */
struct bhnd_nvobj_class {
	size_t			 size;	/**< object size */
	bhnd_nvobj_fini_op	*fini;	/**< finalizer */
};

/**
 * NVRAM observer registration.
 */
struct bhnd_nvram_observer {
	struct bhnd_nvobj		*obj;	/**< observing object (weak ref) */
	bhnd_nvobj_observer_fn		*fn;	/**< event callback */

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_observer)	 link;	/**< observer list link */
};


#ifdef _KERNEL

#define	BHND_NVOBJ_OBSERVERS_XLOCK(obj)		sx_xlock(&(obj)->obs_lock)
#define	BHND_NVOBJ_OBSERVERS_XUNLOCK(obj)	sx_xunlock(&(obj)->obs_lock)
#define	BHND_NVOBJ_OBSERVERS_SLOCK(obj)		sx_slock(&(obj)->obs_lock)
#define	BHND_NVOBJ_OBSERVERS_SUNLOCK(obj)	sx_sunlock(&(obj)->obs_lock)

#define	BHND_NVOBJ_OBSERVERS_ITER_LOCK(obj)	sx_xlock(&(obj)->obs_free_lock)
#define	BHND_NVOBJ_OBSERVERS_TRY_ITER_LOCK(obj)		\
    sx_try_xlock(&(obj)->obs_free_lock)
#define	BHND_NVOBJ_OBSERVERS_ITER_UNLOCK(obj)	sx_xunlock(&(obj)->obs_free_lock)

#define	BHND_NVOBJ_OBSERVERS_LOCK_ASSERT(obj, what)	\
    sx_assert(&(obj)->obs_lock, what)

#else /* !_KERNEL */

#define	BHND_NVOBJ_OBSERVERS_XLOCK(obj)		\
    pthread_rwlock_rwlock(&(obj)->obs_lock)
#define	BHND_NVOBJ_OBSERVERS_XUNLOCK(obj)	\
    pthread_rwlock_unlock(&(obj)->obs_lock)
#define	BHND_NVOBJ_OBSERVERS_SLOCK(obj)		\
    pthread_rwlock_rdlock(&(obj)->obs_lock)
#define	BHND_NVOBJ_OBSERVERS_SUNLOCK(obj)	\
    pthread_rwlock_unlock(&(obj)->obs_lock)

#define	BHND_NVOBJ_OBSERVERS_ITER_LOCK(obj)	\
    pthread_mutex_lock(&(obj)->obs_free_lock)
#define	BHND_NVOBJ_OBSERVERS_TRY_ITER_LOCK(obj)	\
    pthread_mutex_trylock(&(obj)->obs_free_lock)
#define	BHND_NVOBJ_OBSERVERS_ITER_UNLOCK(obj)	\
    pthread_mutex_unlock(&(obj)->obs_free_lock)

#define	BHND_NVOBJ_OBSERVERS_LOCK_ASSERT(obj, what)

#endif /* _KERNEL */

/**
 * NVRAM entry types.
 */
typedef enum {
	BHND_NVRAM_PLANE_ENTRY	= 1,
	BHND_NVRAM_PROV_ENTRY	= 2,
} bhnd_nvram_entry_type;

/**
 * NVRAM canonical path string.
 */
struct bhnd_nvpath {
	char			*pathname;	/**< canonical, fully qualified path name */
	size_t			 pathlen;	/**< length of pathname */

	const char		*basename;	/**< relative file name within pathname */
	size_t			 baselen;	/**< length of basename */

	struct bhnd_nvref	 refs;
};

/**
 * NVRAM entry consumer record.
 */
struct bhnd_nvram_consumer {
	/* immutable state */
	struct bhnd_nvram_entry		*entry;		/**< referencing entry (weak ref) */
	LIST_ENTRY(bhnd_nvram_consumer)	 nc_link;
};

/**
 * NVRAM entry node.
 */
struct bhnd_nvram_entry {
	struct bhnd_nvref		 refs;
	bool				 invalid;	/**< entry invalidated; disconnection is pending */

	bhnd_nvram_entry_type		 type;		/**< entry type */
	struct bhnd_nvpath		*path;		/**< canonical path */

	struct bhnd_nvram_consumer_list	 consumers;	/**< registered consumers */
	struct bhnd_nvram_entry		*target;	/**< entry alias target, or NULL (weak ref) */
	struct bhnd_nvram_entry		*parent;	/**< parent, or NULL (weak ref) */
	struct bhnd_nvram_entry_list	 children;	/**< all children (strong ref) */
	struct bhnd_nvlock		*topo_lock;	/**< enclosing plane/provider's lock */

	union {
		/* BHND_NVRAM_ENTRY_LINK */
		struct bhnd_nvram_plane		*plane;		/**< enclosing plane (weak ref) */

		/* BHND_NVRAM_ENTRY_PROV */
		struct bhnd_nvram_provider	*provider;	/**< provider (weak ref) */
	} d;

	LIST_ENTRY(bhnd_nvram_entry)	 child_link;	/**< parent's weakly held list entry, if any */
};

/**
 * NVRAM plane.
 * 
 * Manages a tree of NVRAM planes, NVRAM paths, and associated NVRAM devices.
 */
struct bhnd_nvram_plane {
	struct bhnd_nvram_plane		*parent;	/**< parent, or NULL */
	struct bhnd_nvram_entry		*root;		/**< root ("/") */
	struct bhnd_nvram_entry_list	 entries;	/**< all entries */

	LIST_HEAD(,bhnd_nvram_plane)	 children;	/**< children */

	struct bhnd_nvlock		*topo_lock;	/**< topology lock */

	struct bhnd_nvref		 refs;
	LIST_ENTRY(bhnd_nvram_plane)	 child_link;
};

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
	bhnd_nvram_prov_state		 state;		/**< current provider state */
	size_t				 busy;		/**< busy count. */
	struct bhnd_nvram_entry_list	 entries;	/**< exported provider entries */

	struct bhnd_nvlock		*prov_lock;
	struct bhnd_nvref		 refs;
};

#ifdef _KERNEL
#define	BHND_NVLOCK_ASSERT(lock, what)	sx_assert(&(lock)->nv_lock, what)
#else /* !_KERNEL */
#define	BHND_NVLOCK_ASSERT(lock, what)
#endif

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
 * Assert that @p value is has at least one strong reference.
 */
#define	BHND_NVREF_ASSERT_ALIVE(value, field)			\
	BHND_NV_ASSERT(!BHND_NVREF_IS_ZOMBIE((value), (field)),	\
	    ("zombie reference"));

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
 * Return true if the given weak reference is a zombie -- it has no further
 * strong references remaining, and promotion to a strong reference would
 * fail.
 */
#define	BHND_NVREF_IS_ZOMBIE(value, field)	\
	(BHND_NVREF_REFCOUNT(value, field) == 0)

#endif /* _BHND_NVRAM_BHND_NVRAMVAR_H_ */
