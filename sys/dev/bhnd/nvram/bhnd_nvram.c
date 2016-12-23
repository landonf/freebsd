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

static bool			 bhnd_nvram_plane_is_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);
static void			 bhnd_nvram_plane_remove_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);

static int			 bhnd_nvram_plane_deregister_device_locked(
				     struct bhnd_nvram_plane *plane,
				     device_t dev);
static int			 bhnd_nvram_plane_add_paths_locked(
				     struct bhnd_nvram_plane *plane,
				     device_t dev, char **pathnames,
				     size_t num_pathnames);

static bhnd_nvram_phandle	*bhnd_nvram_plane_get_phandle(
				     struct bhnd_nvram_plane *plane,
				     const char *pathname, size_t pathlen);
static bhnd_nvram_phandle	*bhnd_nvram_plane_add_phandle(
				     struct bhnd_nvram_plane *plane,
				     const char *pathname, size_t pathlen);

static bhnd_nvram_phandle	*bhnd_nvram_phandle_new(const char *pathname,
				     size_t pathlen,
				     struct bhnd_nvram_plane *plane,
				     bhnd_nvram_phandle *parent);
static void			 bhnd_nvram_phandle_fini(
				     bhnd_nvram_phandle *path);

static bhnd_nvram_dev_entry	*bhnd_nvram_plane_find_device(
				     struct bhnd_nvram_plane *plane,
				     device_t dev);

static void			 bhnd_nvram_dev_entry_fini(
				     bhnd_nvram_dev_entry *dev);

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

	LIST_INIT(&plane->children);
	LIST_INIT(&plane->devices);

	BHND_NVREF_INIT(&plane->refs);

	BHND_NVPLANE_LOCK_INIT(plane);

	/* Register with parent, if any */
	if (parent == NULL) {
		plane->parent = NULL;
	} else {
		/* Retain strong parent reference */
		plane->parent = bhnd_nvram_plane_retain(plane->parent);
	
		/* Add weak reference to our parent's child list */
		BHND_NVPLANE_LOCK_RW(parent);

		BHND_NVREF_RETAIN_WEAK(plane, refs);
		LIST_INSERT_HEAD(&parent->children, plane, children_link);

		BHND_NVPLANE_UNLOCK_RW(parent);
	}

	/* Allocate default root path */
	plane->root = bhnd_nvram_phandle_new("/", strlen("/"), plane, NULL);
	if (plane->root == NULL) {
		bhnd_nvram_plane_release(plane);
		return (NULL);
	}

	return (plane);
}

/**
 * Return true if @p child is a direct child of @p plane, false otherwise.
 * 
 * @param	plane	The NVRAM plane to query.
 * @param	child	The NVRAM child plane to search for.
 */
static bool
bhnd_nvram_plane_is_child(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_plane *child)
{
	struct bhnd_nvram_plane *p;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	LIST_FOREACH(p, &plane->children, children_link) {
		if (p == child)
			return (true);
	}

	/* Not found */
	return (false);
}

/**
 * Remove @p child from @p plane.
 * 
 * @param	plane	The NVRAM plane to modify.
 * @param	child	The NVRAM child plane to remove from @p plane.
 */
static void
bhnd_nvram_plane_remove_child(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_plane *child)
{
	BHND_NVPLANE_LOCK_RW(plane);

	if (!bhnd_nvram_plane_is_child(plane, child))
		BHND_NV_PANIC("plane is not a direct child of parent");

	LIST_REMOVE(plane, children_link);
	BHND_NVREF_RELEASE_WEAK(plane, refs);

	BHND_NVPLANE_UNLOCK_RW(plane);
}

/**
 * Finalize @p plane, deallocating all associated resources.
 * 
 * @param	plane	The NVRAM plane to be finalized.
 */
static void
bhnd_nvram_plane_fini(struct bhnd_nvram_plane *plane)
{
	struct bhnd_nvram_plane		*parent;
	bhnd_nvram_dev_entry		*d, *dnext;

	BHND_NVREF_ASSERT_CAN_FREE(plane, refs);

	BHND_NV_ASSERT(LIST_EMPTY(&plane->children),
	     ("active children did not keep us alive"));

	LIST_FOREACH_SAFE(d, &plane->devices, devs_link, dnext) {
		LIST_REMOVE(d, devs_link);
		BHND_NVREF_RELEASE(d, refs, bhnd_nvram_dev_entry_fini);
	}

	bhnd_nvram_path_release(plane->root);
	BHND_NVPLANE_LOCK_DESTROY(plane);

	/*
	 * Remove our parent's reference to this instance.
	 * 
	 * This may deallocate our plane instance entirely, and no
	 * member references to plane should be made after this point.
	 */
	if ((parent = plane->parent) != NULL) {
		/* Drop parent's reference to this instance */
		bhnd_nvram_plane_remove_child(parent, plane);

		/* Drop our parent reference */
		bhnd_nvram_plane_release(parent);
	}
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
	return (BHND_NVREF_RETAIN(plane, refs));
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
	BHND_NVREF_RELEASE(plane, refs, bhnd_nvram_plane_fini);
}

/**
 * Search for @p device in @p plane, returning its entry if found.
 */
static struct bhnd_nvram_dev_entry *
bhnd_nvram_plane_find_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_dev_entry *entry;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	LIST_FOREACH(entry, &plane->devices, devs_link) {
		if (entry->dev == dev)
			return (entry);
	}

	/* Not found */
	return (NULL);
}

/**
 * Register a new NVRAM device with @p plane.
 * 
 * @param	plane		The NVRAM plane with which @p dev will be
 *				registered.
 * @param	dev		The NVRAM device to register.
 * @param	pathnames	The fully qualified path names to be mapped
 *				to @p dev.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EEXIST	if @p dev is already registered with @p plane.
 * @retval EINVAL	if a path in @p pathnames is not a fully-qualified path.
 * @retval EEXIST	if a path in @p pathnames is already registered in
 *			@p plane.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_register_device(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	struct bhnd_nvram_dev_entry	*entry;
	int				 error;

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

	LIST_INIT(&entry->paths);
	BHND_NVREF_INIT(&entry->refs);

	/* Insert in device list (transfering our strong reference) */
	LIST_INSERT_HEAD(&plane->devices, entry, devs_link);

	/* Try to register paths */
	error = bhnd_nvram_plane_add_paths_locked(plane, dev, pathnames,
	    num_pathnames);
	if (error) {
		/* Path registration failed; deregister device */
		bhnd_nvram_plane_deregister_device_locked(plane, dev);

		BHND_NVPLANE_UNLOCK_RW(plane);
		return (error);
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (0);
}

/**
 * Finalize @p dentry, deallocating all associated resources.
 * 
 * @param	dentry	The NVRAM device entry to be finalized.
 */
static void
bhnd_nvram_dev_entry_fini(bhnd_nvram_dev_entry *dentry)
{
	bhnd_nvram_phandle *phandle, *pnext;

	LIST_FOREACH_SAFE(phandle, &dentry->paths, pathlist_link, pnext) {
		bhnd_nvram_path_release(phandle);
	}
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
	int error;

	BHND_NVPLANE_LOCK_RW(plane);
	error = bhnd_nvram_plane_deregister_device_locked(plane, dev);
	BHND_NVPLANE_UNLOCK_RW(plane);

	return (error);
}

static int
bhnd_nvram_plane_deregister_device_locked(struct bhnd_nvram_plane *plane,
    device_t dev)
{
	struct bhnd_nvram_dev_entry *entry;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	entry = bhnd_nvram_plane_find_device(plane, dev);
	if (entry == NULL) {
		/* Ignore request to deregister unrecognized device */
		return (0);
	}

	/* Drop device entry */
	LIST_REMOVE(entry, devs_link);
	BHND_NVREF_RELEASE(entry, refs, bhnd_nvram_dev_entry_fini);

	return (0);
}

/**
 * Retain and return the path handle for @p pathname, or return NULL if not
 * found.
 * 
 * @param	plane		The NVRAM plane to query.
 * @param	pathname	The fully qualified name of the NVRAM path
 *				handle to be fetched.
 * @param	pathlen		The length of @p pathname.
 */
static bhnd_nvram_phandle *
bhnd_nvram_plane_get_phandle(struct bhnd_nvram_plane *plane,
    const char *pathname, size_t pathlen)
{
	bhnd_nvram_phandle	*path;
	const char		*name;
	size_t			 namelen;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	/* First element must be "/" */
	name = NULL;
	name = bhnd_nvram_parse_path_next(pathname, pathlen, name, &namelen);
	if (namelen != 1 || *name != '/')
		return (NULL);

	/* Walk the path tree */
	path = bhnd_nvram_path_retain(plane->root);
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		bhnd_nvram_phandle	*child;
		bool			 found;

		/* cwd reference is a no-op */
		if (namelen == 1 && *name == '.')
			continue;

		/* parent reference */
		if (namelen == 2 && name[0] == '.' && name[1] == '.') {
			/* Cannot resolve a parent reference if we're already
			 * at the root path */
			if (path->parent == NULL) {
				bhnd_nvram_path_release(path);
				return (NULL);
			}

			/* Replace current path with its parent, and then
			 * continue searching */
			child = path;

			path = bhnd_nvram_path_retain(child->parent);
			bhnd_nvram_path_release(child);

			continue;
		}

		/* locate name in the current path */
		found = false;
		LIST_FOREACH(child, &path->children, children_link) {
			child = BHND_NVREF_PROMOTE_WEAK(child, refs);
			if (child == NULL)
				continue;

			if (strncmp(child->name, name, namelen) != 0) {
				bhnd_nvram_path_release(child);
				continue;
			}

			if (child->name[namelen] != '\0') {
				bhnd_nvram_path_release(child);
				continue;
			}

			/* Subpath matches; replace our current path
			 * reference */
			found = true;
			bhnd_nvram_path_release(path);
			path = child;
			break;
		}

		if (!found) {
			bhnd_nvram_path_release(path);
			return (NULL);
		}
	}

	/* Transfer ownership of the path to our caller */
	return (path);
}

/**
 * Add an empty path handle for @p pathname, returning a caller-owned
 * reference to the new entry.
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
static bhnd_nvram_phandle *
bhnd_nvram_plane_add_phandle(struct bhnd_nvram_plane *plane,
    const char *pathname, size_t pathlen)
{
	bhnd_nvram_phandle	*entry, *child, *top;
	const char		*name;
	size_t			 namelen;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	child = NULL;
	top = NULL;

	if (!bhnd_nvram_is_qualified_path(pathname, pathlen)) {
		BHND_NV_PANIC("path not fully qualified: %.*s",
		    BHND_NV_PRINT_WIDTH(pathlen),  pathname);
	}

	entry = bhnd_nvram_plane_get_phandle(plane, pathname, pathlen);
	if (entry != NULL) {
		bhnd_nvram_path_release(entry);

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
		bhnd_nvram_phandle	*parent;
		size_t			 dirlen;

		/* Determine length of leading directory prefix */
		dirlen = (name + namelen) - pathname;

		/* Does parent path already exist? */
		parent = bhnd_nvram_plane_get_phandle(plane, pathname, dirlen);
		if (parent != NULL)
			continue;

		/* Attempt to allocate parent entry */
		parent = bhnd_nvram_phandle_new(pathname, dirlen, plane, child);

		/* Drop our previous child entry; it has either been retained
		 * by the newly allocated instance, or we need to clean up
		 * due to allocation failure */
		if (child != NULL)
			bhnd_nvram_path_release(child);

		/* Did allocation of the new entry fail? */
		if (parent == NULL)
			return (NULL);

		/* Save the top-most added entry */
		if (top == NULL)
			top = parent;

		/* Save the new child entry */
		child = parent;
	}

	/* Iterate over the newly allocated entries, adding weak references
	 * from their parent path */
	for (entry = child; entry != NULL; entry = entry->parent) {
		/* Insert a weak reference in the parent's list of children */
		if (entry->parent != NULL) {
			BHND_NVREF_RETAIN_WEAK(entry, refs);
			LIST_INSERT_HEAD(&entry->parent->children, entry,
			    children_link);
		}

		/* Stop once we hit the final newly allocated entry */
		if (entry == top)
			break;
	}

	/* Transfer ownership of the path to our caller */
	return (child);
}

/**
 * Register new NVRAM path entries for @p dev.
 * 
 * @param	plane		The NVRAM plane with which @p pathname will be
 *				registered.
 * @param	dev		The NVRAM device vending the path.
 * @param	pathnames	The fully qualified path names to be registered.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if a path in @p pathnames is not a fully-qualified path.
 * @retval EINVAL	if @p dev was not previously registered via
 *			bhnd_nvram_plane_register_device().
 * @retval EEXIST	if a path in @p pathnames is already registered in
 *			@p plane.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_add_paths(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	int error;

	BHND_NVPLANE_LOCK_RW(plane);
	error = bhnd_nvram_plane_add_paths_locked(plane, dev, pathnames,
	    num_pathnames);
	BHND_NVPLANE_UNLOCK_RW(plane);

	return (error);
}

static int
bhnd_nvram_plane_add_paths_locked(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	bhnd_nvram_dev_entry	*dentry;
	bhnd_nvram_phandle_list	 phandles;
	bhnd_nvram_phandle	*phandle, *pnext;
	int			 error;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	/* The NVRAM device must be registered. */
	dentry = bhnd_nvram_plane_find_device(plane, dev);
	if (dentry == NULL)
		return (EINVAL);

	LIST_INIT(&phandles);

	/* Fetch path entries for all requested paths. */
	for (size_t i = 0; i < num_pathnames; i++) {
		const char	*pathname;
		size_t		 pathlen;

		pathname = pathnames[i];
		pathlen = strlen(pathname);

		/* Fetch or add an empty path entry */
		phandle = bhnd_nvram_plane_get_phandle(plane, pathname,
		    pathlen);
		if (phandle != NULL) {
			/* If the entry exists, it must be empty */
			if (phandle->prov.type != BHND_NVRAM_PROVIDER_NONE) {
				BHND_NV_LOG("cannot register duplicate path: "
				    "%s\n", pathname);
				error = EEXIST;
				goto failed;
			}

			/* Retain a strong reference to this existing path */
			bhnd_nvram_path_retain(phandle);
		} else {
			/* Add and fetch a strong reference to a new path
			 * handle */
			phandle = bhnd_nvram_plane_add_phandle(plane, pathname,
			    pathlen);
			if (phandle == NULL) {
				error = ENOMEM;
				goto failed;
			}
		}

		/* Add to list of new handles allocated for the device
		 * provider */
		LIST_INSERT_HEAD(&phandles, phandle, pathlist_link);
	}

	/* Register the new path handles with the device entry */
	LIST_FOREACH_SAFE(phandle, &phandles, pathlist_link, pnext) {
		/* Transfer handle ownership to the device entry */
		LIST_REMOVE(phandle, pathlist_link);
		LIST_INSERT_HEAD(&dentry->paths, phandle, pathlist_link);

		/* Link the path handle back to the device entry */
		phandle->prov.type = BHND_NVRAM_PROVIDER_DEV;
		phandle->prov.src.dev = BHND_NVREF_RETAIN_WEAK(dentry, refs);
	}

	return (0);

failed:
	/* Clean up */
	LIST_FOREACH_SAFE(phandle, &phandles, pathlist_link, pnext) {
		bhnd_nvram_path_release(phandle);
	}

	return (error);
}

/**
 * Deregister NVRAM path entries for @p dev.
 * 
 * @param	plane		The NVRAM plane with which @p pathname will be
 *				registered.
 * @param	dev		The current NVRAM device registered for
 *				@p pathname.
 * @param	pathnames	The fully qualified path names to be
 *				deregistered.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if @p dev was not previously registered via
 *			bhnd_nvram_plane_register_device().
 * @retval ENOENT	if @p pathname is not registered in @p plane, or was
 *			not registered by @p dev.
 */
int
bhnd_nvram_plane_remove_paths(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	// TODO
	return (ENXIO);
}

/**
 * Allocate, initialize, and register an empty NVRAM path handle.
 * 
 * The caller is responsible for releasing the returned NVRAM path handle.
 * 
 * @param	pathname	The fully qualified path name.
 * @param	plane		The path's NVRAM plane.
 * @param	parent		The path's NVRAM parent.
 * 
 * @retval NULL if allocation failed.
 */
static bhnd_nvram_phandle *
bhnd_nvram_phandle_new(const char *pathname, size_t pathlen,
    struct bhnd_nvram_plane *plane, bhnd_nvram_phandle *parent)
{
	bhnd_nvram_phandle *phandle;

	BHND_NV_ASSERT(bhnd_nvram_is_qualified_path(pathname, pathlen),
	    ("path is not fully qualified: %.*s", BHND_NV_PRINT_WIDTH(pathlen),
	     pathname));

	/* Allocate path handle */
	phandle = bhnd_nv_malloc(sizeof(*phandle));
	if (phandle == NULL)
		return (NULL);

	phandle->prov.type = BHND_NVRAM_PROVIDER_NONE;

	BHND_NVREF_INIT(&phandle->refs);
	LIST_INIT(&phandle->children);

	/* Copy absolute path */
	phandle->pathname = bhnd_nv_strndup(pathname, pathlen);
	if (phandle->pathname == NULL) {
		bhnd_nv_free(phandle);
		return (NULL);
	}

	/* Save borrowed pointer to relative path name */
	phandle->name = bhnd_nvram_parse_path_basename(phandle->pathname,
	    pathlen, NULL);

	BHND_NV_ASSERT(strlen(phandle->name) > 0,
	    ("invalid path; empty basename: %s", phandle->pathname));

	/* Retain a strong parent reference */
	phandle->parent = NULL;
	if (parent != NULL)
		phandle->parent = BHND_NVREF_RETAIN(parent, refs);

	return (phandle);
}

/**
 * Finalize @p phandle, deallocating all associated resources.
 * 
 * @param	phandle	The NVRAM path to be finalized.
 */
static void
bhnd_nvram_phandle_fini(bhnd_nvram_phandle *phandle)
{
	bhnd_nvram_phandle *c, *cnext;

	/* Release all weak child references */
	LIST_FOREACH_SAFE(c, &phandle->children, children_link, cnext) {
		LIST_REMOVE(c, children_link);
		BHND_NVREF_RELEASE_WEAK(c, refs);
	}

	/* Release strong parent reference */
	if (phandle->parent != NULL)
		bhnd_nvram_path_release(phandle->parent);

	/* Release weak provider reference */
	switch (phandle->prov.type) {
	case BHND_NVRAM_PROVIDER_NONE:
		break;
	case BHND_NVRAM_PROVIDER_DEV:
		BHND_NVREF_RELEASE_WEAK(phandle->prov.src.dev, refs);
		break;
	case BHND_NVRAM_PROVIDER_PATH:
		BHND_NVREF_RELEASE_WEAK(phandle->prov.src.path, refs);
		break;
	}

	bhnd_nv_free(phandle->pathname);
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
	return (BHND_NVREF_RETAIN(phandle, refs));
}

/**
 * Release a path handle.
 *
 * @param	phandle	The path handle to be released.
 */
void
bhnd_nvram_path_release(bhnd_nvram_phandle *phandle)
{
	BHND_NVREF_RELEASE(phandle, refs, bhnd_nvram_phandle_fini);
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
 * Open and return a caller-owned reference to the given @p path.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing it via bhnd_nvram_plane_release_path().
 * 
 * @param	plane		The NVRAM plane containing @p path.
 * @param	pathname	The path to be opened.
 *
 * @retval non-NULL	if @p path is found in @p plane.
 * @retval NULL		if @p path is not found in @p plane.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_path(struct bhnd_nvram_plane *plane, const char *pathname)
{
	bhnd_nvram_phandle *phandle;

	BHND_NVPLANE_LOCK_RD(plane);
	phandle = bhnd_nvram_plane_get_phandle(plane, pathname,
	    strlen(pathname));
	BHND_NVPLANE_UNLOCK_RD(plane);

	return (phandle);
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
	return (phandle->parent);
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
