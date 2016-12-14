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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/refcount.h>

#include "bhnd_nvram_private.h"

#include "bhnd_nvramvar.h"

MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");

static void	bhnd_nvram_plane_free(void *value);

/**
 * Allocate and initialize an empty NVRAM plane.
 * 
 * The caller is responsible for releasing the returned NVRAM plane
 * via bhnd_nvram_plane_release().
 * 
 * @param	parent	The parent NVRAM plane, or NULL.
 * 
 * @retval NULL if allocation failed.
 */
struct bhnd_nvram_plane *
bhnd_nvram_plane_new(struct bhnd_nvram_plane *parent)
{
	struct bhnd_nvram_plane	*plane;

	plane = bhnd_nv_calloc(1, sizeof(*plane));
	if (plane == NULL)
		return (NULL);

	/* Initialize plane refcount */
	bhnd_nvref_init(&plane->refs, plane, bhnd_nvram_plane_free);

	/* Initialize empty list of (weakly referenced) children */
	LIST_INIT(&plane->children);

	/* Initialize our state lock */
	BHND_NVPLANE_LOCK_INIT(plane);

	/* Register with parent, if any */
	plane->parent = NULL;
	if (parent != NULL) {
		/* Retain strong parent reference */
		plane->parent = bhnd_nvram_plane_retain(parent);

		/* Retain weak reference on our parent's behalf */
		bhnd_nvref_retain_weak(&plane->refs);

		/* Add to parent's child list */
		BHND_NVPLANE_LOCK_RW(parent);
		LIST_INSERT_HEAD(&parent->children, plane, np_link);
		BHND_NVPLANE_UNLOCK_RW(parent);
	}
			 
	return (plane);
}

/**
 * Release a reference to @p plane.
 *
 * If this is the last reference, all associated resources will be freed.
 * 
 * @param	plane	The NVRAM plane to be released.
 */
void
bhnd_nvram_plane_release(struct bhnd_nvram_plane *plane)
{
	struct bhnd_nvram_plane *parent;

	/* Release our caller's strong reference. If that results in
	 * deallocation, nothing left to do */
	if (bhnd_nvref_release(&plane->refs))
		return;

	/* Are weak references keeping us alive in zombie form? */
	if (!bhnd_nvref_is_zombie(&plane->refs))
		return;

	/*
	 * We're a zombie now. Mark our parent (if any) as unavailable, and
	 * then remove ourselves from the parent.
	 */
	BHND_NVPLANE_LOCK_RW(plane);
	parent = plane->parent;
	plane->parent = NULL;
	BHND_NVPLANE_UNLOCK_RW(plane);

	/* If no parent, nothing else to do */
	if (parent == NULL)
		return;

	/* Drop parent's entry for this child */
	BHND_NVPLANE_LOCK_RW(parent);
	LIST_REMOVE(plane, np_link);
	BHND_NVPLANE_UNLOCK_RW(parent);

	/* Drop parent's weak reference for this child -- may trigger
	 * immediate deallocation of this child. */
	bhnd_nvref_release_weak(&plane->refs);

	/* Drop our now-invalidated strong reference to our parent */
	bhnd_nvram_plane_release(parent);
}

/* bhnd_nvram_plane deallocation callback */
static void
bhnd_nvram_plane_free(void *value)
{
	struct bhnd_nvram_plane *plane = value;

	BHND_NV_ASSERT(LIST_EMPTY(&plane->children),
	    ("free() with active children"));

	/* Drop our own strong parent reference */
	if (plane->parent != NULL)
		bhnd_nvram_plane_release(plane->parent);

	/* Clean up remaining instance state */
	BHND_NVPLANE_LOCK_DESTROY(plane);
	bhnd_nv_free(plane);
}

/**
 * Retain a reference and return @p plane to the caller.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_plane_release().
 * 
 * @param	plane	The NVRAM plane to be retained.
 */
struct bhnd_nvram_plane *
bhnd_nvram_plane_retain(struct bhnd_nvram_plane *plane)
{
	bhnd_nvref_retain(&plane->refs);
	return (plane);
}

/**
 * Register a new NVRAM provider with @p plane.
 * 
 * TODO
 */
int
bhnd_nvram_plane_register(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov)
{
	// TODO
	return (ENXIO);
}

/**
 * Deregister a NVRAM provider and all associated paths.
 * 
 * TODO
 */
int
bhnd_nvram_plane_deregister(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov)
{
	// TODO
	return (ENXIO);
}

/**
 * Register a new NVRAM path.
 * 
 * TODO
 */
int
bhnd_nvram_plane_register_path(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, const char *path)
{
	// TODO
	return (ENXIO);
}

/**
 * Deregister an NVRAM path.
 * 
 * TODO
 */
int
bhnd_nvram_plane_deregister_path(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, const char *path)
{
	// TODO
	return (ENXIO);
}

/**
 * Open and return a caller-owned reference to the NVRAM plane's root path.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing it via bhnd_nvram_plane_release_path().
 * 
 * @param	plane	The NVRAM plane to query.
 * 
 * @returns	A caller-owned reference to the NVRAM plane's root path.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_root(struct bhnd_nvram_plane *plane)
{
	return (bhnd_nvram_plane_open_path(plane, "/"));
}

/**
 * Recursively search @p plane and its parents, opening and returning a path
 * handle for the first path matching @p path, or NULL if not found.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing the reference via bhnd_nvram_plane_release_path().
 * 
 * @param	plane		The NVRAM plane at which to start the search.
 * @param	path		The path to search for.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_find_path(struct bhnd_nvram_plane *plane, const char *path)
{
	struct bhnd_nvram_plane	*p;
	bhnd_nvram_phandle	*phandle;

	/* Walk the plane hierarchy until we hit a match */
	for (p = plane; p != NULL; p = p->parent) {
		phandle = bhnd_nvram_plane_open_path(p, path);
		if (phandle != NULL)
			return (phandle);
	}

	/* Not found */
	return (NULL);
}

/**
 * Open and return a caller-owned reference to the given fully qualified
 * @p path.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing it via bhnd_nvram_plane_release_path().
 * 
 * @param	plane	The NVRAM plane containing @p path.
 * @param	path	The fully qualified path to be opened.
 *
 * @retval non-NULL	if @p path is found in @p plane.
 * @retval NULL		if @p path is not found in @p plane.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_path(struct bhnd_nvram_plane *plane, const char *path)
{
	// TODO
	return (NULL);
}

/**
 * Open and return a caller-owned reference to the parent of @p phandle,
 * if any.
 * @p path.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing it via bhnd_nvram_plane_release_path().
 *
 * @param	phandle	The path whose parent is to be opened.
 *
 * @retval non-NULL	a caller-owned reference to the parent of @p phandle.
 * @retval NULL		if @p phandle is the root of the path hierarchy.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_parent(bhnd_nvram_phandle *phandle)
{
	if (phandle->parent == NULL)
		return (NULL);

	return (bhnd_nvram_plane_retain_path(phandle->parent));
}

/**
 * Retain a new reference to an open path handle, returning @p phandle
 * to the caller.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_plane_release_path().
 * 
 * @param	phandle	The path handle to be retained.
 * 
 * @return Returns the @p phandle argument for convenience.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_retain_path(bhnd_nvram_phandle *phandle)
{
	bhnd_nvref_retain(&phandle->refs);
	return (phandle);
}

/**
 * Release a path handle.
 *
 * @param	phandle	The path handle to be released.
 */
void
bhnd_nvram_plane_release_path(bhnd_nvram_phandle *phandle)
{
	struct bhnd_nvram_phandle *parent;

	/* Release our caller's strong reference. If that results in
	 * deallocation, nothing left to do */
	if (bhnd_nvref_release(&phandle->refs))
		return;

	/* Are weak references keeping us alive in zombie form? */
	if (!bhnd_nvref_is_zombie(&phandle->refs))
		return;

	/*
	 * We're a zombie now. Mark our parent path (if any) as unavailable,
	 * and then remove ourselves from the parent.
	 */
	BHND_NVPLANE_LOCK_RW(phandle->plane);
	parent = phandle->parent;
	phandle->parent = NULL;
	BHND_NVPLANE_UNLOCK_RW(phandle->plane);

	/* If no parent path, nothing else to do */
	if (parent == NULL)
		return;

	/* Drop parent path's entry for this child */
	BHND_NVPLANE_LOCK_RW(parent->plane);
	LIST_REMOVE(phandle, np_link);
	BHND_NVPLANE_UNLOCK_RW(parent->plane);

	/* Drop parent's weak reference for this child -- may trigger
	 * immediate deallocation of this child. */
	bhnd_nvref_release_weak(&phandle->refs);

	/* Drop our now-invalidated strong reference to our parent */
	bhnd_nvram_plane_release_path(parent);
}

/**
 * Recursively search @p phandle and its parents for the given property,
 * opening and returning a path handle for the first path containing a property
 * matching @p propname, or NULL if not found.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing the reference via bhnd_nvram_plane_release_path().
 * 
 * @param	phandle		The path at which to start the search.
 * @param	propname	The property to search for.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_findprop_path(bhnd_nvram_phandle *phandle,
    const char *propname)
{
	// TODO
	return (NULL);
}

int
bhnd_nvram_plane_setprop(bhnd_nvram_phandle *phandle, const char *propname,
    const void *buf, size_t len, bhnd_nvram_type type)
{
	// TODO
	return (ENXIO);
}

/**
 * Read an NVRAM property value defined on @p phandle.
 *
 * @param		phandle		The path handle defining @p propname.
 * @param		propname	The property name.
 * @param[out]		buf		On success, the property value will be
 *					written to this buffer. This argment may
 *					be NULL if the value is not desired.
 * @param[in,out]	len		The maximum capacity of @p buf. On
 *					success, will be set to the actual size
 *					of the requested property.
 * @param		type		The data type to be written to @p buf.
 *
 * @retval 0		success
 * @retval ENOENT	If @p propname is not found in @p phandle.
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 * @retval ENODEV	If the underlying provider for @p phandle is
 *			unavailable.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_plane_getprop(bhnd_nvram_phandle *phandle, const char *propname,
    void *buf, size_t *len, bhnd_nvram_type type)
{
	// TODO
	return (ENXIO);
}

/**
 * Allocate a new buffer and read an NVRAM property value defined on
 * @p phandle to @p buf.
 * 
 * On success, the caller is responsible for freeing the returned buffer
 * via bhnd_nvram_plane_getprop_free().
 *
 * @param		phandle		The path handle defining @p propname.
 * @param		propname	The property name.
 * @param[out]		buf		On success, will be set to a newly
 *					allocated buffer containing the
 *					requested property value.
 * @param[out]		len		On success, will be set to the size
 *					of @p buf.
 * @param		type		The data type to be written to @p buf.
 * @param		flags		Malloc flags (M_*).
 *
 * @retval 0		success
 * @retval ENOENT	If @p propname is not found in @p phandle.
 * @retval ENOMEM	If allocation fails.
 * @retval ENODEV	If the underlying provider for @p phandle is
 *			unavailable.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_plane_getprop_alloc(bhnd_nvram_phandle *phandle,
    const char *propname, void **buf, size_t *len, bhnd_nvram_type type,
    int flags)
{
	size_t	olen;
	int	error;

	/* Loop on allocation; in rare circumstances, the property may
	 * be set to a larger value between calls to getprop() */
	*buf = NULL;
	do {
		/* Determine required size */
		error = bhnd_nvram_plane_getprop(phandle, propname, NULL, len,
		    type);
		if (error)
			return (error);

		/* Free previously allocated buffer, if any */
		if (*buf != NULL)
			bhnd_nv_free(buf);

		*buf = malloc(*len, M_BHND_NVRAM, flags);
		if (buf == NULL)
			return (ENOMEM);

		/* Write to output buffer */
		olen = *len;
		error = bhnd_nvram_plane_getprop(phandle, propname, buf, len,
		    type);
	} while (error == ENOMEM && olen < *len);


	/* Return getprop() result */
	return (error);
}

/**
 * Free a property value previously allocated by
 * bhnd_nvram_plane_getprop_alloc().
 */
void
bhnd_nvram_plane_getprop_free(void *buf)
{
	bhnd_nv_free(buf);
}

/**
 * Return a copy of all properties in the given path.
 * 
 * The caller is responsible for releasing the result via
 * @p bhnd_nvram_plist_release().
 * 
 * @param	phandle	The path handle for which all propertied should be
 *			returned.
 *
 * @retval non-NULL	all properties defined in @p phandle.
 * @retval NULL		if allocation fails.
 */
struct bhnd_nvram_plist *
bhnd_nvram_plane_getprops_copy(bhnd_nvram_phandle *phandle)
{
	// TODO
	return (NULL);
}
