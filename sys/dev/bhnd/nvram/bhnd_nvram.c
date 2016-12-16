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

static void				 bhnd_nvram_plane_free(
					     struct bhnd_nvram_plane *plane,
					     bool zombie);

static bool				 bhnd_nvram_plane_has_child(
					     struct bhnd_nvram_plane *plane,
					     struct bhnd_nvram_plane *child);
static void				 bhnd_nvram_plane_register_child(
					     struct bhnd_nvram_plane *plane,
					     struct bhnd_nvram_plane *child);
static void				 bhnd_nvram_plane_deregister_child(
					     struct bhnd_nvram_plane *plane,
					     struct bhnd_nvram_plane *child);

static bhnd_nvram_phandle		*bhnd_nvram_plane_new_path(
					     const char *pathname,
					     struct bhnd_nvram_plane *plane,
					     bhnd_nvram_phandle *parent);
static void				 bhnd_nvram_plane_free_path(
					     bhnd_nvram_phandle *phandle,
					     bool zombie);

static struct bhnd_nvram_devnode	*bhnd_nvram_plane_find_device(
					     struct bhnd_nvram_plane *plane,
					     device_t dev);

static void				 bhnd_nvram_devnode_free(
					     struct bhnd_nvram_devnode *devnode,
					     bool zombie);

static void
bhnd_nvram_devnode_free(struct bhnd_nvram_devnode *devnode, bool zombie)
{
	// XXX TODO
}

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

	BHND_NVREF_INIT(&plane->np_refs);

	BHND_NVPLANE_LOCK_INIT(plane);

	/* Register with parent, if any */
	if (parent != NULL) {
		/* Retain strong parent reference */
		plane->parent = bhnd_nvram_plane_retain(parent);

		/* Add to parent's child list */
		bhnd_nvram_plane_register_child(plane->parent, plane);
	}
			 
	return (plane);
}


/**
 * Attempt to deallocate @p plane and all associated resources.
 * 
 * @param	plane	The NVRAM plane to be deallocated.
 * @param	zombie	True if called after @p plane has become a zombie; it
 *			is kept alive solely by weak references.
 */
static void
bhnd_nvram_plane_free(struct bhnd_nvram_plane *plane, bool zombie)
{
	struct bhnd_nvram_plane		*c, *cnext;
	struct bhnd_nvram_devnode	*devn, *dnext;

	/* If we're now a zombie, attempt to deregister from the parent plane
	 * (if any), dropping the parent's weak reference to this value and
	 * potentially triggering full deallocation. */
	if (zombie) {
		if (plane->parent != NULL)
			bhnd_nvram_plane_deregister_child(plane->parent, plane);

		return;
	}

	BHND_NVREF_ASSERT_CAN_FREE(plane, np_refs);

	/* Release all weak child references */
	LIST_FOREACH_SAFE(c, &plane->children, np_link, cnext)
		BHND_NVREF_RELEASE_WEAK(c, np_refs, bhnd_nvram_plane_free);

	/* Release all strong device references */
	LIST_FOREACH_SAFE(devn, &plane->devices, dn_link, dnext)
		BHND_NVREF_RELEASE(devn, dn_refs, bhnd_nvram_devnode_free);

	/* Release all strong path references */
	// TODO

	/* Release strong reference to parent */
	if (plane->parent != NULL)
		bhnd_nvram_plane_release(plane);

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
	BHND_NVREF_RETAIN(plane, np_refs);
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
	BHND_NVREF_RELEASE(plane, np_refs, bhnd_nvram_plane_free);
}

/**
 * Retain a weak reference to @p child and add to @p plane's list of children.
 * 
 * @param	plane	The NVRAM plane with which @p child will be registered.
 * @param	dev	The NVRAM device to register.
 */
static void
bhnd_nvram_plane_register_child(struct bhnd_nvram_plane *plane,
     struct bhnd_nvram_plane *child)
{
	BHND_NVPLANE_LOCK_RW(plane);

	/* Retain weak reference and add to child list */
	BHND_NVREF_RETAIN_WEAK(child, np_refs);
	LIST_INSERT_HEAD(&plane->children, child, np_link);

	BHND_NVPLANE_UNLOCK_RW(plane);
}

/**
 * Return true if @p child is registered with @p plane, false otherwise.
 * 
 * @param	plane	The NVRAM plane to query.
 * @param	child	The NVRAM child plane to search for.
 */
static bool
bhnd_nvram_plane_has_child(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_plane *child)
{
	struct bhnd_nvram_plane *p;

	BHND_NVPLANE_LOCK_ASSERT(plane, SX_LOCKED);

	LIST_FOREACH(p, &plane->children, np_link) {
		if (p == child)
			return (true);
	}

	/* Not found */
	return (false);
}


/**
 * Remove @p child from @p plane's list of children and release the weak
 * reference acquired bhnd_nvram_plane_register_child().
 * 
 * @param	plane	The NVRAM plane with which @p child will be registered.
 * @param	dev	The NVRAM device to register.
 */
static void
bhnd_nvram_plane_deregister_child(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_plane *child)
{
	BHND_NVPLANE_LOCK_RW(plane);

	/* Ignore (potentially duplicate) deregistration requests for unknown
	 * children */
	if (!bhnd_nvram_plane_has_child(plane, child)) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return;
	}

	/* Remove from child list */
	LIST_REMOVE(child, np_link);

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Release weak reference to child */
	BHND_NVREF_RELEASE_WEAK(child, np_refs, bhnd_nvram_plane_free);
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
		if (entry->dev == dev)
			return (entry);
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
	BHND_NVREF_INIT(&entry->dn_refs);

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

	BHND_NVREF_INIT(&phandle->np_refs);

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
	BHND_NVREF_RETAIN(phandle, np_refs);
	return (phandle);
}


/**
 * Attempt to deallocate @p path and all associated resources.
 * 
 * @param	path	The NVRAM path to be deallocated.
 * @param	zombie	True if called after @p path has become a zombie; it
 *			is kept alive solely by weak references.
 */
static void
bhnd_nvram_plane_free_path(struct bhnd_nvram_phandle *phandle, bool zombie)
{
	struct bhnd_nvram_phandle *c, *cnext;

	/* If we're now a zombie, attempt to deregister from the parent path
	 * (if any), dropping the parent's weak reference to this value and
	 * potentially triggering full deallocation. */
	if (zombie) {
		if (phandle->parent != NULL) {
			// TODO
		}

		return;
	}

	BHND_NVREF_ASSERT_CAN_FREE(phandle, np_refs);

	/* Release all weak child references */
	LIST_FOREACH_SAFE(c, &phandle->children, np_link, cnext)
		BHND_NVREF_RELEASE_WEAK(c, np_refs, bhnd_nvram_plane_free_path);

	/* Release strong parent reference */
	if (phandle->parent != NULL)
		bhnd_nvram_plane_release_path(phandle->parent);

	/* Release weak plane reference */
	BHND_NVREF_RELEASE_WEAK(phandle->plane, np_refs, bhnd_nvram_plane_free);

	/* Clean up remaining instance state */
	bhnd_nv_free(phandle->path);
	bhnd_nv_free(phandle);
}

/**
 * Release a path handle.
 *
 * @param	phandle	The path handle to be released.
 */
void
bhnd_nvram_plane_release_path(bhnd_nvram_phandle *phandle)
{
	BHND_NVREF_RELEASE(phandle, np_refs, bhnd_nvram_plane_free_path);
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
