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
#include <sys/hash.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/refcount.h>

#include "bhnd_nvram_private.h"

#include "bhnd_nvramvar.h"

MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");

static void			 bhnd_nvram_plane_fini(
				     struct bhnd_nvram_plane *plane);

static bool			 bhnd_nvram_plane_has_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);
static void			 bhnd_nvram_plane_register_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);
static void			 bhnd_nvram_plane_deregister_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);

static bhnd_nvram_path_entry	*bhnd_nvram_plane_get_path(
				     struct bhnd_nvram_plane *plane,
				     const char *pathname, size_t pathlen);
static bhnd_nvram_path_entry	*bhnd_nvram_plane_add_path(
				     struct bhnd_nvram_plane *plane,
				     const char *pathname, size_t pathlen);
static void			 bhnd_nvram_plane_remove_path(
				     struct bhnd_nvram_plane *plane,
				     const char *pathname, size_t pathlen);

static bhnd_nvram_path_entry	*bhnd_nvram_path_entry_new(const char *pathname,
				     size_t pathlen,
				     struct bhnd_nvram_plane *plane,
				     bhnd_nvram_path_entry *parent);
static void			 bhnd_nvram_path_entry_fini(
				     bhnd_nvram_path_entry *path);

static bhnd_nvram_phandle	*bhnd_nvram_phandle_new(const char *pathname,
				     size_t pathlen,
				     struct bhnd_nvram_plane *plane);
static void			 bhnd_nvram_phandle_fini(
				     bhnd_nvram_phandle *phandle);

static bhnd_nvram_dev_entry	*bhnd_nvram_plane_find_device(
				     struct bhnd_nvram_plane *plane,
				     device_t dev);

static void			 bhnd_nvram_dev_entry_fini(
				     bhnd_nvram_dev_entry *dev);

static void
bhnd_nvram_dev_entry_fini(bhnd_nvram_dev_entry *devnode)
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
	struct bhnd_nvram_plane		*plane;
	bhnd_nvram_path_entry_list	*bucket;
	bhnd_nvram_path_entry		*root;
	uint32_t			 h;

	plane = bhnd_nv_calloc(1, sizeof(*plane));
	if (plane == NULL)
		return (NULL);

	LIST_INIT(&plane->children);
	LIST_INIT(&plane->devices);

	for (size_t i = 0; i < nitems(plane->paths); i++)
		LIST_INIT(&plane->paths[i]);

	BHND_NVREF_INIT(&plane->np_refs);

	BHND_NVPLANE_LOCK_INIT(plane);

	/* Register with parent, if any */
	plane->parent = NULL;
	if (parent != NULL) {
		/* Retain strong parent reference */
		plane->parent = bhnd_nvram_plane_retain(parent);
	
		/* Add to parent's child list */
		bhnd_nvram_plane_register_child(plane->parent, plane);
	}

	/* Add default root path */
	root = bhnd_nvram_path_entry_new("/", strlen("/"), plane, NULL);
	if (root == NULL) {
		BHND_NVREF_RELEASE(plane, np_refs, bhnd_nvram_plane_fini);
		return (NULL);
	}

	/* Insert into path hash table */
	h = hash32_str(root->phandle->pathname, HASHINIT);
	bucket = &plane->paths[h % nitems(plane->paths)];
	LIST_INSERT_HEAD(bucket, root, np_hash_link);

	return (plane);
}


/**
 * Attempt to deallocate all resources held by @p plane.
 * 
 * @param	plane	The NVRAM plane to be deallocated.
 */
static void
bhnd_nvram_plane_fini(struct bhnd_nvram_plane *plane)
{
	struct bhnd_nvram_plane		*c, *cnext;
	bhnd_nvram_dev_entry		*d, *dnext;

	BHND_NVREF_ASSERT_CAN_FREE(plane, np_refs);

	/* Release all weak child references */
	LIST_FOREACH_SAFE(c, &plane->children, np_link, cnext) {
		LIST_REMOVE(c, np_link);
		BHND_NVREF_RELEASE_WEAK(c, np_refs);
	}

	/* Release all strong device references */
	LIST_FOREACH_SAFE(d, &plane->devices, dn_link, dnext) {
		LIST_REMOVE(d, dn_link);
		BHND_NVREF_RELEASE(d, dn_refs, bhnd_nvram_dev_entry_fini);
	}

	/* Release all strong path references */
	for (size_t i = 0; i < nitems(plane->paths); i++) {
		bhnd_nvram_path_entry *p, *pnext;

		LIST_FOREACH_SAFE(p, &plane->paths[i], np_hash_link, pnext) {
			LIST_REMOVE(p, np_hash_link);
			BHND_NVREF_RELEASE(p, np_refs, 
			    bhnd_nvram_path_entry_fini);
		}
	}

	/* Release strong reference to parent */
	if (plane->parent != NULL)
		bhnd_nvram_plane_release(plane->parent);

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
	return (BHND_NVREF_RETAIN(plane, np_refs));
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
	BHND_NVREF_RELEASE(plane, np_refs, bhnd_nvram_plane_fini);
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
	BHND_NVREF_RELEASE_WEAK(child, np_refs);
}

/**
 * Search for @p device in @p plane, returning its entry if found.
 */
static struct bhnd_nvram_dev_entry *
bhnd_nvram_plane_find_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_dev_entry *entry;

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
	struct bhnd_nvram_dev_entry *entry;

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
	struct bhnd_nvram_dev_entry *entry;

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
 * Return the path entry for @p path, or NULL if not found.
 * 
 * @param	plane		The NVRAM plane to query.
 * @param	pathname	The fully qualified name of the NVRAM path
 *				entry to be fetched.
 * @param	pathlen		The length of @p pathname.
 */
static bhnd_nvram_path_entry *
bhnd_nvram_plane_get_path(struct bhnd_nvram_plane *plane, const char *pathname,
    size_t pathlen)
{
	bhnd_nvram_path_entry_list	*bucket;
	bhnd_nvram_path_entry		*path;
	uint32_t			 h;

	BHND_NVPLANE_LOCK_ASSERT(plane, SX_LOCKED);
	BHND_NV_ASSERT(pathname != NULL, ("NULL path"));

	/* We need the length minus any trailing NUL */
	if (pathlen > 0 && pathname[pathlen - 1] == '\0')
		pathlen--;

	/* Search hash table for the path */
	h = hash32_strn(pathname, pathlen, HASHINIT);
	bucket = &plane->paths[h % nitems(plane->paths)];
	LIST_FOREACH(path, bucket, np_hash_link) {
		if (strncmp(pathname, path->phandle->pathname, pathlen) != 0)
			continue;

		if (path->phandle->pathname[pathlen] == '\0')
			return (path);
	}

	/* Not found */
	return (NULL);
}

/**
 * Add an empty path entry for @p pathname, returning a borrowed reference
 * to the new entry.
 * 
 * Any intermediate paths will be created as required.
 *
 * @param	plane		The NVRAM plane.
 * @param	pathname	The fully qualified name of the NVRAM path
 *				entry to be added.
 * @param	pathlen		The length of @p pathname.
 *
 * @retval non-NULL	success
 * @retval NULL		if allocation fails.
 */
static bhnd_nvram_path_entry *
bhnd_nvram_plane_add_path(struct bhnd_nvram_plane *plane, const char *pathname,
    size_t pathlen)
{
	bhnd_nvram_path_entry	*entry, *child, *top;
	const char		*name;
	size_t			 namelen;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	child = NULL;
	top = NULL;

	if (!bhnd_nvram_is_qualified_path(pathname, pathlen)) {
		BHND_NV_PANIC("path not fully qualified: %.*s",
		    BHND_NV_PRINT_WIDTH(pathlen),  pathname);
	}

	if (bhnd_nvram_plane_get_path(plane, pathname, pathlen) != NULL) {
		BHND_NV_PANIC("duplicate path %.*s",
		    BHND_NV_PRINT_WIDTH(pathlen),  pathname);
	}

	/*
	 * Walk path, adding our parent paths as required. 
	 * 
	 * We avoid linking the paths into the NVRAM plane until all paths have
	 * been successfully allocated -- this simplifies cleanup by allowing
	 * us to simply release the most recently allocated path entry
	 */
	name = NULL;
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		bhnd_nvram_path_entry	*parent;
		size_t			 dirlen;

		/* Determine length of leading directory prefix */
		dirlen = (name + namelen) - pathname;

		/* Does parent path already exist? */
		parent = bhnd_nvram_plane_get_path(plane, pathname, dirlen);
		if (parent != NULL)
			continue;

		/* Attempt to allocate parent entry */
		parent = bhnd_nvram_path_entry_new(pathname, dirlen, plane,
		    child);

		/* Drop our previous child entry; it has either been retained
		 * by the newly allocated instance, or we need to clean up
		 * due to allocation failure */
		if (child != NULL) {
			BHND_NVREF_RELEASE(child, np_refs,
			    bhnd_nvram_path_entry_fini);
		}

		/* Did allocation of the new entry fail? */
		if (parent == NULL)
			return (NULL);

		/* Save the top-most added entry */
		if (top == NULL)
			top = parent;

		/* Save the new child entry */
		child = parent;
	}

	/* Iterate over the newly allocated entries, adding their parent's weak
	 * references and inserting into the path hash table */
	for (entry = child; entry != NULL; entry = entry->parent) {
		bhnd_nvram_path_entry_list	*bucket;
		uint32_t			 h;

		/* Add strong reference to the path hash table */
		h = hash32_str(entry->phandle->pathname, HASHINIT);
		bucket = &plane->paths[h % nitems(plane->paths)];

		BHND_NVREF_RETAIN(entry, np_refs);
		LIST_INSERT_HEAD(bucket, entry, np_hash_link);

		/* Add a weak reference to the parent's list of children */
		if (entry->parent != NULL) {
			BHND_NVREF_RETAIN_WEAK(entry, np_refs);
			LIST_INSERT_HEAD(&entry->parent->children, entry,
			    np_child_link);
		}

		/* Stop once we hit the final newly allocated entry */
		if (entry == top)
			break;
	}

	/* Drop our now extraneously entry reference; a strong reference is
	 * now held by our hash table */	
	BHND_NVREF_RELEASE(child, np_refs, bhnd_nvram_path_entry_fini);

	return (child);
}

/**
 * Remove and release the path entry for @p pathname from @p plane.
 * 
 * @param	plane		The NVRAM plane.
 * @param	pathname	The fully qualified name of the NVRAM path
 *				entry to be removed.
 * @param	pathlen		The length of @p pathname.
 */
static void
bhnd_nvram_plane_remove_path(struct bhnd_nvram_plane *plane,
    const char *pathname, size_t pathlen)
{
	// TODO
#if 0
	BHND_NVPLANE_LOCK_RW(plane);

	BHND_NV_ASSERT(path->phandle->plane == plane,
	    ("path in foreign plane"));
	BHND_NV_ASSERT(child->phandle->plane == plane,
	    ("child in foreign plane"));

	/* Ignore (potentially duplicate) deregistration requests for unknown
	 * children */
	if (!bhnd_nvram_plane_has_child_path(plane, path, child)) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return;
	}

	/* Remove from child list */
	LIST_REMOVE(child, np_link);

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Release weak reference to child */
	BHND_NVREF_RELEASE_WEAK(child, np_refs);
#endif
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
	bhnd_nvram_path_entry	*entry;
	size_t			 pathlen;

	/* Path must be fully qualified */
	pathlen = strlen(pathname);
	if (!bhnd_nvram_is_qualified_path(pathname, pathlen)) {
		BHND_NV_LOG("invalid path: %s\n", pathname);
		return (EINVAL);
	}

	/* Acquire topology lock. */
	BHND_NVPLANE_LOCK_RW(plane);

	/* The NVRAM device must be registered. */
	if (bhnd_nvram_plane_find_device(plane, dev) == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EINVAL);
	}

	/* Fetch or add an empty path entry */
	entry = bhnd_nvram_plane_get_path(plane, pathname, pathlen);
	if (entry != NULL) {
		/* A non-empty path must not already be registered */
		if (entry->prov.type != BHND_NVRAM_PROVIDER_NONE) {
			BHND_NVPLANE_UNLOCK_RW(plane);
			return (EEXIST);
		}
	} else {
		entry = bhnd_nvram_plane_add_path(plane, pathname, pathlen);
		if (entry == NULL) {
			BHND_NVPLANE_UNLOCK_RW(plane);
			return (ENOMEM);
		}
	}

	for (size_t i = 0; i < nitems(plane->paths); i++) {
		bhnd_nvram_path_entry *e;
		LIST_FOREACH(e, &plane->paths[i], np_hash_link) {
			printf("PATH %s\n", e->phandle->pathname);
		}
	}

	// TODO: point path entry at the device provider.

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
 * Allocate and initialize an NVRAM path instance.
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
static bhnd_nvram_path_entry *
bhnd_nvram_path_entry_new(const char *pathname, size_t pathlen,
    struct bhnd_nvram_plane *plane, bhnd_nvram_path_entry *parent)
{
	bhnd_nvram_path_entry	*path;

	BHND_NV_ASSERT(bhnd_nvram_is_qualified_path(pathname, pathlen),
	    ("path is not fully qualified: %.*s", BHND_NV_PRINT_WIDTH(pathlen),
	     pathname));

	/* Allocate our path instance */
	path = bhnd_nv_malloc(sizeof(*path));
	if (path == NULL)
		return (NULL);

	path->prov.type = BHND_NVRAM_PROVIDER_NONE;
	BHND_NVREF_INIT(&path->np_refs);
	LIST_INIT(&path->children);

	/* Allocate path handle */
	path->phandle = bhnd_nvram_phandle_new(pathname, pathlen, plane);
	if (path->phandle == NULL) {
		bhnd_nv_free(path);
		return (NULL);
	}

	/* Fetch borrowed pointer to our relative path name */
	path->name = bhnd_nvram_parse_path_basename(path->phandle->pathname,
	    pathlen, NULL);

	BHND_NV_ASSERT(strlen(path->name) > 0,
	    ("empty basename: %s", path->phandle->pathname));

	/* Retain a strong parent reference */
	path->parent = NULL;
	if (parent != NULL)
		path->parent = BHND_NVREF_RETAIN(parent, np_refs);

	return (path);
}

/**
 * Deallocate @p path and all associated resources.
 * 
 * @param	path	The NVRAM path to be deallocated.
 */
static void
bhnd_nvram_path_entry_fini(bhnd_nvram_path_entry *path)
{
	bhnd_nvram_path_entry *c, *cnext;

	/* Release all weak child references */
	LIST_FOREACH_SAFE(c, &path->children, np_child_link, cnext) {
		LIST_REMOVE(c, np_child_link);
		BHND_NVREF_RELEASE_WEAK(c, np_refs);
	}

	/* Release strong parent reference */
	if (path->parent != NULL) {
		BHND_NVREF_RELEASE(path->parent, np_refs,
		    bhnd_nvram_path_entry_fini);
	}

	/* Release strong path handle reference */
	BHND_NVREF_RELEASE(path->phandle, np_refs, bhnd_nvram_phandle_fini);
}

/**
 * Allocate and initialize an NVRAM path handle.
 * 
 * The caller is responsible for releasing the returned NVRAM plane
 * via bhnd_nvram_path_release().
 * 
 * @param	pathname	The fully qualified path name.
 * @param	plane		The path's NVRAM plane.
 * 
 * @retval NULL if allocation fails.
 */
static bhnd_nvram_phandle *
bhnd_nvram_phandle_new(const char *pathname, size_t pathlen,
    struct bhnd_nvram_plane *plane)
{
	bhnd_nvram_phandle	*phandle;

	BHND_NV_ASSERT(bhnd_nvram_is_qualified_path(pathname, pathlen),
	    ("path is not fully qualified: %.*s", BHND_NV_PRINT_WIDTH(pathlen),
	     pathname));

	phandle = bhnd_nv_malloc(sizeof(*phandle));
	if (phandle == NULL)
		return (NULL);

	phandle->pathname = bhnd_nv_strndup(pathname, pathlen);
	if (phandle->pathname == NULL) {
		bhnd_nv_free(phandle);
		return (NULL);
	}

	BHND_NVREF_INIT(&phandle->np_refs);
	return (phandle);
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
bhnd_nvram_path_retain(bhnd_nvram_phandle *phandle)
{
	return (BHND_NVREF_RETAIN(phandle, np_refs));
}

/**
 * Release a path handle.
 *
 * @param	phandle	The path handle to be released.
 */
void
bhnd_nvram_path_release(bhnd_nvram_phandle *phandle)
{
	BHND_NVREF_RELEASE(phandle, np_refs, bhnd_nvram_phandle_fini);
}

/**
 * Deallocate @p phandle and all associated resources.
 * 
 * @param	phandle	The NVRAM path handle to be deallocated.
 */
static void
bhnd_nvram_phandle_fini(bhnd_nvram_phandle *phandle)
{
	BHND_NVREF_RELEASE_WEAK(phandle->plane, np_refs);
	bhnd_nv_free(phandle->pathname);
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
 * Open and return a borrowed reference to the parent of @p phandle, if any.
 *
 * @param	phandle	The path whose parent is to be opened.
 *
 * @retval non-NULL	a borrowed reference to the parent of @p phandle.
 * @retval NULL		if @p phandle is the root of the path hierarchy.
 */
bhnd_nvram_phandle *
bhnd_nvram_path_get_parent(bhnd_nvram_phandle *phandle)
{
	// TODO: resolve to path
	// TODO: how to handle dead plane reference?
	return (NULL);
}

/**
 * Recursively search @p phandle and its parents for the given property,
 * returning a borrowed path handle for the first path containing a property
 * matching @p propname, or NULL if not found.
 * 
 * @param	phandle		The path at which to start the search.
 * @param	propname	The property to search for.
 * 
 * @retval non-NULL	a borrowed reference to a path containing @p propname.
 * @retval NULL		if @p propname is not found.
 */
bhnd_nvram_phandle *
bhnd_nvram_path_find_proppath(bhnd_nvram_phandle *phandle,
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
bhnd_nvram_path_setprop(bhnd_nvram_phandle *phandle, const char *propname,
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
bhnd_nvram_path_getprop(bhnd_nvram_phandle *phandle, const char *propname,
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
bhnd_nvram_path_getprop_alloc(bhnd_nvram_phandle *phandle,
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
		error = bhnd_nvram_path_getprop(phandle, propname, NULL, len,
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
		error = bhnd_nvram_path_getprop(phandle, propname, buf, len,
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
bhnd_nvram_path_getprop_free(void *buf)
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
bhnd_nvram_path_copyprops(bhnd_nvram_phandle *phandle)
{
	// TODO
	return (NULL);
}
