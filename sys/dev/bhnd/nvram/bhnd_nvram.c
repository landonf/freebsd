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

static void				 bhnd_nvram_plane_free(void *value);

static bhnd_nvram_phandle		*bhnd_nvram_plane_new_path(
					     const char *pathname,
					     struct bhnd_nvram_plane *plane,
					     bhnd_nvram_phandle *parent);
static void				 bhnd_nvram_plane_free_path(
					     void *value);

static struct bhnd_nvram_devnode	*bhnd_nvram_plane_find_device(
					     struct bhnd_nvram_plane *plane,
					     device_t dev);

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

	plane->parent = parent;
	LIST_INIT(&plane->children);
	LIST_INIT(&plane->devices);

	/* Initialize plane refcount */
	bhnd_nvref_init(&plane->refs, plane, bhnd_nvram_plane_free);

	/* Initialize our state lock */
	BHND_NVPLANE_LOCK_INIT(plane);

	/* Register with parent, if any */
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
	 * We're a zombie. Remove any active weak reference held by our parent
	 * path. If that's the last weak reference, we'll be fully deallocated.
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
	BHND_NV_ASSERT(plane->parent == NULL,
	    ("free() with active parent"));

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
 * Search for @p device in @p plane, returning its entry if found.
 */
static struct bhnd_nvram_devnode *
bhnd_nvram_plane_find_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_devnode *entry;

	BHND_NVPLANE_LOCK_ASSERT(plane, SX_LOCKED);

	LIST_FOREACH(entry, &plane->devices, dn_link) {
		if (entry->dev == dev) {
			if (bhnd_nvref_is_zombie(&entry->refs))
				return (NULL);

			return (entry);
		}
	}

	/* Not found */
	return (NULL);
}

/**
 * Register a new NVRAM device with @p plane.
 * 
 * @param	plane	The NVRAM plane with which @p dev will be registered.
 * @param	dev	The NVRAM device to register.
 * 
 * @retval 0		success.
 * @retval EEXIST	if @p dev is already registered with @p plane.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_register_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_devnode *entry;

	BHND_NVPLANE_LOCK_RW(plane);

	/* Already registered? */
	if (bhnd_nvram_plane_find_device(plane, dev) != NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EEXIST);
	}

	/* Allocate new entry */
	entry = bhnd_nv_malloc(sizeof(*entry));
	if (entry == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (ENOMEM);
	}

	entry->dev = dev;
	bhnd_nvref_init(&entry->refs, &entry, NULL);

	/* Insert in device list */
	LIST_INSERT_HEAD(&plane->devices, entry, dn_link);

	BHND_NVPLANE_UNLOCK_RW(plane);
	return (0);
}

/**
 * Deregister an NVRAM device and all associated paths.
 * 
 * If @p dev is not currently registered with @p plane, the request will
 * be ignored.
 * 
 * All paths previously registered to @p dev will be deregistered.
 * 
 * @param	plane	The NVRAM plane from which @p device will be removed.
 * @param	dev	The NVRAM device to deregister from @p plane.
 * 
 * @retval 0	success.
 */
int
bhnd_nvram_plane_deregister_device(struct bhnd_nvram_plane *plane,
    device_t dev)
{
	struct bhnd_nvram_devnode *entry;

	BHND_NVPLANE_LOCK_RW(plane);

	entry = bhnd_nvram_plane_find_device(plane, dev);
	if (entry == NULL) {
		/* Ignore request to deregister unrecognized device */
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (0);
	}

	/* Remove from device list */
	LIST_REMOVE(entry, dn_link);
	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Free entry */
	bhnd_nv_free(entry);

	return (0);
}

/**
 * Register a new NVRAM path entry.
 * 
 * @param	plane		The NVRAM plane with which @p pathname will be
 *				registered.
 * @param	dev		The NVRAM device vending the path.
 * @param	pathname	The fully qualified path to be registered.
 * 
 * @retval 0		success.
 * @retval EINVAL	if @p pathname is not a fully-qualified path.
 * @retval EINVAL	if @p dev was not previously registered via
 *			bhnd_nvram_plane_register_device().
 * @retval EEXIST	if @p pathname is already registered in @p plane.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_register_path(struct bhnd_nvram_plane *plane, device_t dev,
    const char *pathname)
{
	const char	*name;
	size_t		 pathlen;
	size_t		 nlen, plen;

	/* Path must be fully qualified */
	if (!bhnd_nvram_is_qualified_path(pathname)) {
		BHND_NV_LOG("invalid path: %s\n", pathname);
		return (EINVAL);
	}

	pathlen = strlen(pathname);

	/* Acquire topology lock. */
	BHND_NVPLANE_LOCK_RW(plane);

	/* The NVRAM device must be registered. */
	if (bhnd_nvram_plane_find_device(plane, dev) == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EINVAL);
	}

	/* The path must not already be registered */
	// TODO

	/* Populate any missing parent parent path entries */
	name = NULL;
	plen = bhnd_nvram_path_dirlen(pathname, pathlen);
	while ((name = bhnd_nvram_path_next(pathname, plen, name, &nlen))) {
		printf("get path = %.*s\n", (int)nlen, name);
	}

	/* Register path entry */
	// TODO

	BHND_NVPLANE_UNLOCK_RW(plane);
	return (0);
}

/**
 * Deregister an NVRAM path entry.
 * 
 * @param	plane		The NVRAM plane with which @p pathname will be
 *				registered.
 * @param	dev		The current NVRAM device registered for
 *				@p pathname.
 * @param	pathname	The fully qualified path to be deregistered.
 * 
 * @retval 0		success.
 * @retval EINVAL	if @p dev was not previously registered via
 *			bhnd_nvram_plane_register_device().
 * @retval ENOENT	if @p pathname is not registered in @p plane, or was
 *			not registered by @p dev.
 */
int
bhnd_nvram_plane_deregister_path(struct bhnd_nvram_plane *plane, device_t dev,
    const char *pathname)
{
	// TODO
	return (ENXIO);
}

/**
 * Allocate and initialize an NVRAM path handle.
 * 
 * The caller is responsible for releasing the returned NVRAM plane
 * via bhnd_nvram_plane_release_path().
 * 
 * @param	pathname	The fully qualified path name.
 * @param	plane		The path's NVRAM plane.
 * @param	parent		The path's NVRAM parent.
 * 
 * @retval NULL if allocation failed.
 */
static bhnd_nvram_phandle *
bhnd_nvram_plane_new_path(const char *pathname, struct bhnd_nvram_plane *plane,
     bhnd_nvram_phandle *parent)
{
	static bhnd_nvram_phandle *phandle;

	BHND_NV_ASSERT(plane == parent->plane, ("parent in foreign plane"));
	BHND_NV_ASSERT(bhnd_nvram_is_qualified_path(pathname),
	    ("path is not fully qualified"));

	/* Allocate our path handle instance */
	phandle = bhnd_nv_malloc(sizeof(*phandle));
	if (phandle == NULL)
		return (NULL);

	bhnd_nvref_init(&phandle->refs, phandle, bhnd_nvram_plane_free_path);

	/* Copy path string */
	phandle->path = bhnd_nv_malloc(strlen(pathname) + 1);
	if (phandle->path == NULL)
		goto failed;

failed:
	if (phandle->path != NULL)
		bhnd_nv_free(phandle->path);

	bhnd_nv_free(phandle);
	return (NULL);
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
	 * We're a zombie. Remove any active weak reference held by our parent
	 * path. If that's the last weak reference, we'll be fully deallocated.
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


/* bhnd_nvram_phandle deallocation callback */
static void
bhnd_nvram_plane_free_path(void *value)
{
	struct bhnd_nvram_phandle *phandle = value;

	BHND_NV_ASSERT(LIST_EMPTY(&phandle->children),
	    ("free() with active children"));

	BHND_NV_ASSERT(phandle->parent == NULL,
	    ("free() with valid parent"));

	/* Drop our weak plane reference */
	bhnd_nvref_release_weak(&phandle->plane->refs);

	/* Clean up remaining instance state */
	bhnd_nv_free(phandle->path);
	bhnd_nv_free(phandle);
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
 * @param	pathname	The path to search for.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_find_path(struct bhnd_nvram_plane *plane, const char *pathname)
{
	struct bhnd_nvram_plane	*p;
	bhnd_nvram_phandle	*phandle;

	/* Walk the plane hierarchy until we hit a match */
	for (p = plane; p != NULL; p = p->parent) {
		phandle = bhnd_nvram_plane_open_path(p, pathname);
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
 * @param	plane		The NVRAM plane containing @p path.
 * @param	pathname	The fully qualified path to be opened.
 *
 * @retval non-NULL	if @p path is found in @p plane.
 * @retval NULL		if @p path is not found in @p plane.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_path(struct bhnd_nvram_plane *plane, const char *pathname)
{
	// TODO
	return (NULL);
}

/**
 * Open and return a caller-owned reference to the parent of @p phandle,
 * if any.
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
bhnd_nvram_plane_parent_path(bhnd_nvram_phandle *phandle)
{
	if (phandle->parent == NULL)
		return (NULL);

	return (bhnd_nvram_plane_retain_path(phandle->parent));
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

/**
 * Insert or update an NVRAM property value in @p phandle.
 * 
 * @param		phandle		The path handle to be updated.
 * @param		propname	The property name.
 * @param[out]		buf		The new property value.
 * @param[in,out]	len		The size of @p buf.
 * @param		type		The data type of @p buf.
 *
 * @retval 0		success
 * @retval ENOENT	If @p propname is not a known property name, and the
 *			definition of arbitrary property names is unsupported
 *			by @p phandle.
 * @retval ENODEV	If the underlying device for @p phandle is unavailable.
 * @retval EINVAL	If @p propname is read-only.
 * @retval EINVAL	If @p propname cannot be set to the given value or
 *			value type.
 */
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
 * @retval ENODEV	If the underlying device for @p phandle is unavailable.
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
 * @retval ENODEV	If the underlying device for @p phandle is unavailable.
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
