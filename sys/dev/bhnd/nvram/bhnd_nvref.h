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

#ifndef _BHND_NVRAM_BHND_NVREF_H_
#define _BHND_NVRAM_BHND_NVREF_H_

#include <sys/types.h>

#include <machine/atomic.h>

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
 * When the strong reference count hits zero, the referenced value may be
 * deallocated.
 *
 * When the weak reference count hits zero, the reference count data structure
 * itself may be deallocated.
 */
struct bhnd_nvref {
	volatile u_int	 strong;	/* strong refcount */
	volatile u_int	 weak;		/* weak refcount */
};

/**
 * Initialize a the reference count structure.
 * 
 * @param ref	A reference count structure.
 */
#define	BHND_NVREF_INIT(ref) do {					\
	/* Implicit initial strong reference */				\
	atomic_set_rel_int(&(ref)->strong, 1);				\
									\
	/* Single weak reference shared by all strong references */	\
	atomic_set_rel_int(&(ref)->weak, 1);				\
} while(0)

/**
 * Retain a strong reference to @p value and return @p value.
 * 
 * @param value	The referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_RETAIN(value, field)	\
	(bhnd_nvref_retain(&((value)->field)), (value))

static inline void
bhnd_nvref_retain(struct bhnd_nvref *ref)
{
	BHND_NV_ASSERT(ref->strong > 0, ("over-release"));
	BHND_NV_ASSERT(ref->strong < UINT_MAX, ("overflow"));
	atomic_add_acq_int(&ref->strong, 1);
}

/**
 * Release a strong reference to @p value, possibly deallocating @p value.
 * 
 * @param value		The referenced value.
 * @param field		The value's reference count field.
 * @param dealloc	The value's deallocation callback.
 */
#define	BHND_NVREF_RELEASE(value, field, dealloc) do {			\
	BHND_NV_ASSERT((value)->field.strong > 0, ("over-release"));	\
	BHND_NV_ASSERT((value)->field.weak > 0, ("over-release"));	\
									\
	/* Drop strong reference */					\
	if (atomic_fetchadd_int(&(value)->field.strong, -1) == 0) {	\
		/* We released the last strong reference; discard the	\
		 * the implicit weak reference shared by all strong 	\
		 * references. */					\
		BHND_NVREF_RELEASE_WEAK((value), field, (dealloc));	\
	}								\
} while(0)

/**
 * Assert that @p value is has no remaining weak or strong references
 * and can be safely deallocated.
 * 
 * @param value	The referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_ASSERT_CAN_FREE(value, field) do {		\
	BHND_NV_ASSERT(						\
	    atomic_load_acq_int(&(value)->field.strong) == 0,	\
	    ("cannot free live value"));			\
								\
	BHND_NV_ASSERT(						\
	    atomic_load_acq_int(&(value)->field.weak) == 0,	\
	    ("cannot free zombie value"));			\
} while(0)

/**
 * Retain a weak reference to @p value and return @p value.
 * 
 * @param value	The strongly referenced value.
 * @param field	The value's reference count field.
 */
#define	BHND_NVREF_RETAIN_WEAK(value, field)	\
	(bhnd_nvref_retain_weak(&((value)->field)), (value))

static inline void
bhnd_nvref_retain_weak(struct bhnd_nvref *ref)
{
	BHND_NV_ASSERT(ref->strong > 0, ("over-release"));
	BHND_NV_ASSERT(ref->weak > 0, ("over-release"));
	BHND_NV_ASSERT(ref->weak < UINT_MAX, ("overflow"));
	atomic_add_acq_int(&ref->weak, 1);
}

/**
 * Release a weak reference, possibly deallocating @p value.
 * 
 * @param value		The weakly referenced value.
 * @param field		The value's reference count field.
 * @param dealloc	The value's deallocation callback.
 */
#define	BHND_NVREF_RELEASE_WEAK(value, field, dealloc) do {		\
	BHND_NV_ASSERT((value)->field.weak > 0, ("over-release"));	\
	BHND_NV_ASSERT((value)->field.weak >= (value)->field.strong,	\
	    ("over-release"));						\
									\
	/* Drop weak reference */					\
	if (atomic_fetchadd_int(&(value)->field.weak, -1) == 0) {	\
		/* Value is dead */					\
		(dealloc)(value);					\
	}								\
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
	(bhnd_nvref_promote_weak(&((value)->field).strong) ?	\
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

#endif /* _BHND_NVRAM_BHND_NVREF_H_ */
