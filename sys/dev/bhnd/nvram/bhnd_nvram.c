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

#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/kernel.h>

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

static struct bhnd_nvram_prov	*bhnd_nvram_plane_find_device(
				     struct bhnd_nvram_plane *plane,
				     device_t dev);

static int			 bhnd_nvram_plane_remove_provider(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_prov *prov);


static struct bhnd_nvram_prov	*bhnd_nvram_device_prov_new(device_t dev);
static void			 bhnd_nvram_prov_fini(
				     struct bhnd_nvram_prov *prov);

static bhnd_nvram_phandle	*bhnd_nvram_prov_find_path(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_prov *prov,
				     const char *pathname);
static int			 bhnd_nvram_prov_add_paths(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_prov *prov,
				     char **pathnames, size_t num_pathnames);
static int			 bhnd_nvram_prov_remove_paths(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_prov *prov,
				     char **pathnames, size_t num_pathnames);

static int			 bhnd_nvram_phandle_new(
				     bhnd_nvram_phandle **phandle,
				     const char *pathname,
				     size_t pathlen,
				     bhnd_nvram_phandle *parent);
static void			 bhnd_nvram_phandle_fini(
				     bhnd_nvram_phandle *path);

static bhnd_nvram_phandle	*bhnd_nvram_phandle_open(
				     bhnd_nvram_phandle *root,
				     bhnd_nvram_phandle *cwd,
				     const char *pathname, size_t pathlen);

static int			 bhnd_nvram_phandle_mkdir(
				     bhnd_nvram_phandle *root,
				     bhnd_nvram_phandle *cwd,
				     const char *pathname, size_t pathlen,
				     bhnd_nvram_phandle **child);

static bool			 bhnd_nvram_phandle_is_child(
				     bhnd_nvram_phandle *parent,
				     bhnd_nvram_phandle *child);
static bool			 bhnd_nvram_phandle_has_child(
				     bhnd_nvram_phandle *parent,
				     const char *name);

static void			 bhnd_nvram_phandle_set_provider(
				     bhnd_nvram_phandle *phandle,
				     struct bhnd_nvram_prov *provider);
static void			 bhnd_nvram_phandle_clear_provider(
				     bhnd_nvram_phandle *phandle);

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
	int			 error;

	plane = bhnd_nv_calloc(1, sizeof(*plane));
	if (plane == NULL)
		return (NULL);

	LIST_INIT(&plane->children);
	LIST_INIT(&plane->providers);

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

	/* Create initial root path */
	error = bhnd_nvram_phandle_new(&plane->root, "/", sizeof("/"), NULL);
	if (error) {
		if (error != ENOMEM) {
			BHND_NV_LOG("unexpected error creating root path: %d\n",
			    error);
		}

		bhnd_nvram_plane_release(plane);
		return (NULL);
	}

	return (plane);
}


/**
 * Finalize @p plane, deallocating all associated resources.
 * 
 * @param	plane	The NVRAM plane to be finalized.
 */
static void
bhnd_nvram_plane_fini(struct bhnd_nvram_plane *plane)
{
	struct bhnd_nvram_plane	*parent;
	struct bhnd_nvram_prov	*d, *dnext;

	BHND_NVREF_ASSERT_CAN_FREE(plane, refs);

	BHND_NV_ASSERT(LIST_EMPTY(&plane->children),
	     ("active children did not keep us alive"));

	/* Drop our root path reference */
	bhnd_nvram_path_release(plane->root);

	/* Release all providers */
	LIST_FOREACH_SAFE(d, &plane->providers, providers_link, dnext) {
		LIST_REMOVE(d, providers_link);
		BHND_NVREF_RELEASE(d, refs, bhnd_nvram_prov_fini);
	}

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

	BHND_NVPLANE_LOCK_DESTROY(plane);
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

	BHND_NVPLANE_UNLOCK_RW(plane);

	BHND_NVREF_RELEASE_WEAK(plane, refs);
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
bhnd_nvram_plane_add_device(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	struct bhnd_nvram_prov	*prov;
	int			 error;

	BHND_NVPLANE_LOCK_RW(plane);

	/* Device already registered? */
	if (bhnd_nvram_plane_find_device(plane, dev) != NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EEXIST);
	}

	/* Allocate new provider instance */
	prov = bhnd_nvram_device_prov_new(dev);
	if (prov == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (ENOMEM);
	}

	/* Insert in provider list (transfering our strong reference) */
	LIST_INSERT_HEAD(&plane->providers, prov, providers_link);

	/* Try to register paths */
	error = bhnd_nvram_prov_add_paths(plane, prov, pathnames,
	    num_pathnames);
	if (error) {
		/* Path registration failed; remove provider */
		bhnd_nvram_plane_remove_provider(plane, prov);

		BHND_NVPLANE_UNLOCK_RW(plane);
		return (error);
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (0);
}

/**
 * Search for @p dev in @p plane, returning its provider entry if found.
 */
static struct bhnd_nvram_prov *
bhnd_nvram_plane_find_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_prov *prov;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	LIST_FOREACH(prov, &plane->providers, providers_link) {
		if (prov->dev == dev)
			return (prov);
	}

	/* Not found */
	return (NULL);
}

/**
 * Remove an NVRAM device and all associated paths.
 * 
 * If @p dev is not currently registered with @p plane, the request will
 * be ignored.
 * 
 * All paths previously registered to @p dev will be deregistered.
 * 
 * @param	plane	The NVRAM plane from which @p dev will be removed.
 * @param	dev	The NVRAM device to deregister from @p plane.
 * 
 * @retval 0	success.
 */
int
bhnd_nvram_plane_remove_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_prov	*prov;
	int			 error;

	BHND_NVPLANE_LOCK_RW(plane);

	prov = bhnd_nvram_plane_find_device(plane, dev);
	if (prov == NULL) {
		/* Ignore request to deregister unrecognized provider */
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (0);
	}

	error = bhnd_nvram_plane_remove_provider(plane, prov);

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (error);
}

/**
 * Remove an NVRAM provider and all associated paths.
 * 
 * All paths previously registered to @p prov will be deregistered.
 * 
 * @param	plane	The NVRAM plane from which @p dev will be removed.
 * @param	prov	The NVRAM provider to deregister from @p plane.
 * 
 * @retval 0	success.
 */
static int
bhnd_nvram_plane_remove_provider(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov)
{
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	if (bhnd_nvram_plane_find_device(plane, prov->dev) == NULL)
		BHND_NV_PANIC("provider not registered with plane");

	// TODO: handle busy providers

	LIST_REMOVE(prov, providers_link);
	BHND_NVREF_RELEASE(prov, refs, bhnd_nvram_prov_fini);

	return (0);
}

/**
 * Allocate a new NVRAM provider instance for @p dev.
 * 
 * @param	plane		The NVRAM plane with which @p dev will be
 *				registered.
 * @param	dev		The NVRAM provider device
 * 
 * @retval non-NULL	success.
 * @retval NULL		if allocation fails.
 */
static struct bhnd_nvram_prov *
bhnd_nvram_device_prov_new(device_t dev)
{
	struct bhnd_nvram_prov *prov;

	/* Allocate new provider instance */
	prov = bhnd_nv_calloc(1, sizeof(*prov));
	if (prov == NULL)
		return (NULL);

	prov->dev = dev;
	BHND_NVPROV_LOCK_INIT(prov);

	LIST_INIT(&prov->paths);
	BHND_NVREF_INIT(&prov->refs);

	return (prov);
}

/**
 * Finalize @p prov, deallocating all associated resources.
 * 
 * @param	prov	The NVRAM provider entry to be finalized.
 */
static void
bhnd_nvram_prov_fini(struct bhnd_nvram_prov *prov)
{
	bhnd_nvram_phandle *phandle, *pnext;

	// TODO: release dev?

	LIST_FOREACH_SAFE(phandle, &prov->paths, pathlist_link, pnext) {
		bhnd_nvram_path_release(phandle);
	}
}

/**
 * Register new NVRAM path entries for @p dev.
 * 
 * @param	plane		The NVRAM plane with which @p pathname will be
 *				registered.
 * @param	dev		The NVRAM device vending the path(s).
 * @param	pathnames	The fully qualified path names to be registered.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if a path in @p pathnames is not a fully-qualified path.
 * @retval EINVAL	if @p dev was not previously registered via
 *			bhnd_nvram_plane_add_device().
 * @retval EEXIST	if a path in @p pathnames is already registered in
 *			@p plane.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_add_paths(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	struct bhnd_nvram_prov	*prov;
	int			 error;

	BHND_NVPLANE_LOCK_RW(plane);

	prov = bhnd_nvram_plane_find_device(plane, dev);
	if (prov == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EINVAL);
	}

	error = bhnd_nvram_prov_add_paths(plane, prov, pathnames,
	    num_pathnames);

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (error);
}


/**
 * Deregister NVRAM @p pathnames from @p prov.
 * 
 * @param	plane		The NVRAM plane.
 * @param	prov		The NVRAM provider from which @p pathnames
 *				will be deregistered.
 * @param	pathnames	The fully qualified path names to be registered.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if a path in @p pathnames is not a fully-qualified path.
 * @retval ENOENT	if a path in @p pathnames is not registered in @p plane,
 *			or was not registered by @p prov.
 */
static int
bhnd_nvram_prov_remove_paths(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, char **pathnames, size_t num_pathnames)
{
	bhnd_nvram_phandle *phandle;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	/* Verify that all the requested paths are registered by the provider */
	for (size_t i = 0; i < num_pathnames; i++) {
		phandle = bhnd_nvram_prov_find_path(plane, prov, pathnames[i]);
		if (phandle == NULL) {
			BHND_NV_LOG("cannot deregister path; not registered "
			    "to provider: %s", pathnames[i]);
			return (ENOENT);
		}
	}

	/* Drop all path entries */
	for (size_t i = 0; i < num_pathnames; i++) {
		/* Fetch path handle */
		phandle = bhnd_nvram_prov_find_path(plane, prov, pathnames[i]);

		BHND_NV_ASSERT(phandle != NULL, ("path handle went missing"));

		/* Remove from provider's path list */
		LIST_REMOVE(phandle, pathlist_link);

		/* Drop the phandle's reference to the provider */
		BHND_NV_ASSERT(phandle->prov == prov,
		    ("path handle provider mismatch"));
		bhnd_nvram_phandle_clear_provider(phandle);

		/* Drop provider's strong reference to the phandle */
		bhnd_nvram_path_release(phandle);
	}

	return (0);
}

/**
 * Find and return a borrowed reference to the path handle for @p pathname
 * in @p prov, or NULL if @p pathname is not registered to @p prov.
 * 
 * @param	plane		The NVRAM plane.
 * @param	prov		The NVRAM provider to search for @p pathname.
 * @param	pathname	The fully qualified path name to be located.
 */
static bhnd_nvram_phandle *
bhnd_nvram_prov_find_path(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, const char *pathname)
{
	bhnd_nvram_phandle	*phandle;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	LIST_FOREACH(phandle, &prov->paths, pathlist_link ) {
		if (strcmp(phandle->pathname, pathname) == 0)
			return (phandle);
	}

	/* Not found */
	return (NULL);
}

/**
 * Register new NVRAM path entries to be handled by the given @p prov.
 * 
 * @param	plane		The NVRAM plane.
 * @param	prov		The NVRAM provider prov to which @p pathnames
 *				will be assigned.
 * @param	pathnames	The fully qualified path names to be registered.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if a path in @p pathnames is not a fully-qualified path.
 * @retval EEXIST	if a path in @p pathnames is already registered in
 *			@p plane.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvram_prov_add_paths(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, char **pathnames, size_t num_pathnames)
{
	bhnd_nvram_phandle	*phandle;
	bhnd_nvram_phandle	**phandles;
	int			 error;
	bool			 locked;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	locked = false;

	phandles = bhnd_nv_calloc(num_pathnames, sizeof(*phandles));
	if (phandles == NULL)
		return (ENOMEM);

	/* Fetch (or create) path handles for all requested paths. */
	for (size_t i = 0; i < num_pathnames; i++) {
		const char	*pathname;
		size_t		 pathlen;

		pathname = pathnames[i];
		pathlen = strlen(pathname);

		/* Fetch or create the path */
		error = bhnd_nvram_phandle_mkdir(plane->root, NULL, pathname,
		    pathlen, &phandle);
		if (error)
			goto failed;
	
		phandles[i] = phandle;
	}

	/* Lock all paths */
	for (size_t i = 0; i < num_pathnames; i++)
		BHND_NVPATH_LOCK_RW(phandles[i]);

	locked = true;

	/* Paths must not have an existing provider */
	for (size_t i = 0; i < num_pathnames; i++) {
		if (phandles[i]->prov != NULL) {
			error = EEXIST;
			goto failed;
		}
	}

	/* Register the new path handles with the provider */
	for (size_t i = 0; i < num_pathnames; i++) {
		phandle = phandles[i];

		/* Transfer ownership of the path handle to the provider */
		LIST_INSERT_HEAD(&prov->paths, phandle, pathlist_link);
		bhnd_nvram_phandle_set_provider(phandle, prov);

		/* Drop our path lock */
		BHND_NVPATH_UNLOCK_RW(phandle);
	}

	/* Clean up path array */
	bhnd_nv_free(phandles);

	/* Success */
	return (0);

failed:
	/* Clean up our path handle references  */
	for (size_t i = 0; i < num_pathnames; i++) {
		if (phandles[i] == NULL)
			continue;

		if (locked)
			BHND_NVPATH_UNLOCK_RW(phandles[i]);
	
		bhnd_nvram_path_release(phandles[i]);
	}

	bhnd_nv_free(phandles);

	return (error);
}

/**
 * Remove NVRAM path entries for @p dev.
 * 
 * @param	plane		The NVRAM plane with which @p pathname will be
 *				registered.
 * @param	dev		The current NVRAM device registered for
 *				@p pathnames.
 * @param	pathnames	The fully qualified path names to be
 *				deregistered.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if @p dev was not previously registered via
 *			bhnd_nvram_plane_add_device().
 * @retval ENOENT	if a path in @p pathnames is not registered in @p plane,
 *			or was not registered by @p dev.
 */
int
bhnd_nvram_plane_remove_paths(struct bhnd_nvram_plane *plane, device_t dev,
    char **pathnames, size_t num_pathnames)
{
	struct bhnd_nvram_prov	*prov;
	int			 error;

	BHND_NVPLANE_LOCK_RW(plane);

	prov = bhnd_nvram_plane_find_device(plane, dev);
	if (prov == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EINVAL);
	}

	error = bhnd_nvram_prov_remove_paths(plane, prov, pathnames,
	    num_pathnames);

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (error);
}

/**
 * Allocate, initialize, and register an empty NVRAM path handle.
 * 
 * The caller is responsible for releasing the returned NVRAM path handle.
 * 
 * @param[out]	phandle		On success, the newly allocated path handle.
 * @param	name		The new path's relative path name.
 * @param	namelen		The length of @p pathname.
 * @param	parent		The parent path, or NULL
 * 
 * @retval 0		success
 * @retval EINVAL	if @p parent is NULL and @p name is not "/"
 * @retval EINVAL	if @p parent is not NULL and @p name contains
 *			path delimiters.
 * @retval EEXIST	if @p name already exists in @p parent.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvram_phandle_new(bhnd_nvram_phandle **phandle, const char *name,
    size_t namelen, bhnd_nvram_phandle *parent)
{
	bhnd_nvram_phandle	*p;
	size_t			 pathlen;
	int			 error;


	/* Allocate path handle */
	p = bhnd_nv_calloc(1, sizeof(*p));
	if (p == NULL)
		return (ENOMEM);

	p->prov = NULL;
	p->parent = NULL;

	BHND_NVREF_INIT(&p->refs);
	BHND_NVPATH_LOCK_INIT(p);
	LIST_INIT(&p->children);

	/* Construct a fully-qualified path string */
	if (parent != NULL) {
		size_t baselen, pathsz;

		/* We need the length minus any trailing NUL */
		namelen = strnlen(name, namelen);
		if (namelen == 0) {
			return (EINVAL);
		}

		/* Name component must not be "." or ".." */
		if (!bhnd_nvram_is_normalized_path(name, namelen))
			return (EINVAL);

		/* Name must not contain path delimiters */
		if (memchr(name, '/', namelen) != NULL) {
			error = EINVAL;
			goto failed;
		}

		/* Determine total concatenated length */		
		baselen = strlen(parent->pathname);
		pathlen = baselen + 1 + namelen /* path + '/' + name */;
		pathsz = pathlen + 1; /* + \0' */

		BHND_NV_ASSERT(bhnd_nvram_is_qualified_path(parent->pathname,
		    baselen), ("invalid parent path"));

		/* Allocate path buffer */
		if ((p->pathname = bhnd_nv_malloc(pathsz)) == NULL) {
			error = ENOMEM;
			goto failed;
		}

		/* parent + '/' + name + '\0' */
		strcpy(p->pathname, parent->pathname);
		p->pathname[baselen] = '/';
		strncpy(p->pathname+baselen+1, name, namelen);
		p->pathname[pathlen] = '\0';

		BHND_NV_ASSERT(bhnd_nvram_is_qualified_path(p->pathname,
		    pathlen), ("invalid child path"));
	} else {
		/* Root path must be / */
		if (strncmp(name, "/", namelen) != 0) {
			error = EINVAL;
			goto failed;
		}

		pathlen = namelen;
		p->pathname = bhnd_nv_strndup(name, namelen);
		if (p->pathname == NULL) {
			error = ENOMEM;
			goto failed;
		}
	}

	/* Save borrowed pointer to relative path name */
	p->name = bhnd_nvram_parse_path_basename(p->pathname, pathlen, NULL);

	BHND_NV_ASSERT(strlen(p->name) > 0,
	    ("invalid path; empty basename: %s", p->pathname));

	/* Save parent reference and add ourselves to the parent's list of
	 * children */
	if (parent != NULL) {
		BHND_NVPATH_LOCK_RW(parent);

		/* Path must not already exist */
		if (bhnd_nvram_phandle_has_child(p->parent, p->name)) {
			BHND_NVPATH_UNLOCK_RW(p->parent);
			error = EEXIST;
			goto failed;
		}

		/* Insert weak reference into the parent's child list */
		LIST_INSERT_HEAD(&p->parent->children,
		    BHND_NVREF_RETAIN_WEAK(p, refs), children_link);

		/* Save a strong reference to our parent */
		p->parent = bhnd_nvram_path_retain(parent);

		BHND_NVPATH_UNLOCK_RW(parent);
	}

	/* Transfer ownership of the new instance to the caller */
	*phandle = p;
	return (0);

failed:
	BHND_NV_ASSERT(p->parent == NULL, ("stale parent reference"));
	BHND_NV_ASSERT(LIST_EMPTY(&p->children), ("active children"));

	if (p->pathname != NULL)
		bhnd_nv_free(p->pathname);

	BHND_NVPATH_LOCK_DESTROY(p);
	bhnd_nv_free(p);

	return (error);
}

/**
 * Finalize @p phandle, deallocating all associated resources.
 * 
 * @param	phandle	The NVRAM path to be finalized.
 */
static void
bhnd_nvram_phandle_fini(bhnd_nvram_phandle *phandle)
{
	BHND_NV_ASSERT(phandle->prov == NULL, ("active provider"));
	BHND_NV_ASSERT(LIST_EMPTY(&phandle->children), ("active children"));

	BHND_NVPATH_LOCK_DESTROY(phandle);
	bhnd_nv_free(phandle->pathname);

	if (phandle->parent != NULL) {
		/* Remove our parent's weak reference to this path */
		BHND_NVPATH_LOCK_RW(phandle->parent);

		if (!bhnd_nvram_phandle_is_child(phandle->parent, phandle))
			BHND_NV_PANIC("path is not direct child of parent");

		LIST_REMOVE(phandle, children_link);

		BHND_NVPATH_UNLOCK_RW(phandle->parent);

		/* Drop our strong parent reference */
		bhnd_nvram_path_release(phandle->parent);

		/* Drop our parent's weak reference to this child. This may
		 * result in the phandle being immediately deallocated */
		BHND_NVREF_RELEASE_WEAK(phandle, refs);
	}
}


/**
 * Resolve @p pathname to a path handle, returning a caller-owned reference
 * to the path if found, or NULL if not found.
 * 
 * @param	root		The root path from which absolute paths will
 *				be resolved.
 * @param	cwd		The 'current working directory' from which
 *				relative paths will be resolved, or NULL if
 *				only fully qualified paths should be permitted.
 * @param	pathname	The NVRAM path name to be resolved.
 * @param	pathlen		The length of @p pathname.
 */
static bhnd_nvram_phandle *
bhnd_nvram_phandle_open(bhnd_nvram_phandle *root, bhnd_nvram_phandle *cwd,
    const char *pathname, size_t pathlen)
{
	const char	*name;
	size_t		 namelen;

	/* Find and retain the initial search path */
	name = NULL;
	name = bhnd_nvram_parse_path_next(pathname, pathlen, name, &namelen);

	if (namelen == 1 && *name == '/') {
		cwd = bhnd_nvram_path_retain(root);
	} else {
		if (cwd == NULL) {
			/* Cannot resolve a relative path */
			return (NULL);
		}

		/* save reference to cwd */
		bhnd_nvram_path_retain(cwd);

		/* restart path walking at the first element */
		name = NULL;
	}

	/* Walk the path tree */
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		bhnd_nvram_phandle *p, *child;

		/* cwd reference is a no-op */
		if (namelen == 1 && *name == '.')
			continue;

		/* handle parent references ('..') */
		if (namelen == 2 && name[0] == '.' && name[1] == '.') {
			bhnd_nvram_phandle *parent;

			BHND_NVPATH_LOCK_RD(cwd);

			/* Parent references are cyclical if we're already
			 * at the root path */
			if (cwd->parent == NULL) {
				BHND_NVPATH_UNLOCK_RD(cwd);
				continue;
			}

			/* Retain reference to parent path */
			parent = bhnd_nvram_path_retain(cwd->parent);

			/* Replace current path with parent path and continue
			 * searching */
			BHND_NVPATH_UNLOCK_RD(cwd);
			bhnd_nvram_path_release(cwd);
			cwd = parent;

			continue;
		}

		/* locate name in the current path */
		BHND_NVPATH_LOCK_RD(cwd);
		child = NULL;
		LIST_FOREACH(p, &cwd->children, children_link) {
			p = BHND_NVREF_PROMOTE_WEAK(p, refs);
			if (p == NULL)
				continue;

			if (strncmp(p->name, name, namelen) != 0) {
				bhnd_nvram_path_release(p);
				continue;
			}

			if (p->name[namelen] != '\0') {
				bhnd_nvram_path_release(p);
				continue;
			}

			/* Subpath matches */
			child = p;
			break;
		}
		BHND_NVPATH_UNLOCK_RD(cwd);

		/* Not found? */
		if (child == NULL) {
			bhnd_nvram_path_release(cwd);
			return (NULL);
		}

		/* Replace current path with child path and continue
		 * searching */
		BHND_NVPATH_UNLOCK_RD(cwd);
		bhnd_nvram_path_release(cwd);
		cwd = child;
	}

	/* Transfer reference ownership to our caller */
	return (cwd);
}

/**
 * Create or return a path handle for @p pathname under @p root, returning a
 * caller-owned reference to the entry.
 * 
 * Any intermediate paths will be created as required.
 *
 * @param	root		The root path from which absolute paths will
 *				be resolved.
 * @param	cwd		The 'current working directory' from which
 *				relative paths will be resolved, or NULL if
 *				only fully qualified paths should be permitted.
 * @param	pathname	The name of the NVRAM path entry to be added.
 * @param	pathlen		The length of @p pathname.
 * @param[out]	child		On success, the newly added child's path handle.
 *				The caller is responsible for releasing this
 *				handle via bhnd_nvram_path_release().
 *
 * @retval 0		success
 * @retval EINVAL	if @p cwd is NULL and @p pathname is a relative path.
 * @retval EINVAL	if @p pathname is invalid.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvram_phandle_mkdir(bhnd_nvram_phandle *root, bhnd_nvram_phandle *cwd,
    const char *pathname, size_t pathlen, bhnd_nvram_phandle **child)
{
	const char		*basename, *name;
	size_t			 namelen, baselen;
	int			 error;

	/* The basename must be non-empty, and must be a normalized path
	 * component (i.e. not one of ".", "..") */
	basename = bhnd_nvram_parse_path_basename(pathname, pathlen, &baselen);
	if (baselen == 0 || !bhnd_nvram_is_normalized_path(basename, baselen))
		return (EINVAL);

	/* Find and retain the initial search path */
	name = NULL;
	name = bhnd_nvram_parse_path_next(pathname, pathlen, name, &namelen);

	if (namelen == 1 && *name == '/') {
		cwd = bhnd_nvram_path_retain(root);
	} else {
		if (cwd == NULL) {
			/* Cannot resolve a relative path */
			return (EINVAL);
		}

		/* save reference to cwd */
		bhnd_nvram_path_retain(cwd);

		/* restart path walking at the first element */
		name = NULL;
	}

	/* Walk the input path, resolving or creating all path components */
	name = NULL;
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		bhnd_nvram_phandle *next;

		/* Resolve or create the next path component */
		next = bhnd_nvram_phandle_open(root, cwd, name, namelen);
		if (next == NULL) {
			/* Not found; create the new child path */
			error = bhnd_nvram_phandle_new(&next, name, namelen,
			    cwd);
			if (error) {
				bhnd_nvram_path_release(cwd);
				return (error);
			}
		}

		/* Replace cwd with new path */
		bhnd_nvram_path_release(cwd);
		cwd = next;
	}

	/* Transfer ownership of the path to our caller */
	*child = cwd;
	return (0);
}

/**
 * Return true if @p child is a direct child of @p parent, false otherwise.
 * 
 * @param	parent	The NVRAM path to query.
 * @param	child	The NVRAM child plane to search for.
 */
static bool
bhnd_nvram_phandle_is_child(bhnd_nvram_phandle *parent,
    bhnd_nvram_phandle *child)
{
	bhnd_nvram_phandle *p;

	BHND_NVPATH_LOCK_ASSERT(parent, SA_LOCKED);

	LIST_FOREACH(p, &parent->children, children_link) {
		if (p == child)
			return (true);
	}

	/* Not found */
	return (false);
}

/**
 * Return true if a child with @p name is found in @p parent, false otherwise.
 * 
 * @param	parent	The NVRAM path to query.
 * @param	child	The NVRAM child plane to search for.
 */
static bool
bhnd_nvram_phandle_has_child(bhnd_nvram_phandle *parent, const char *name)
{
	bhnd_nvram_phandle *p;

	BHND_NVPATH_LOCK_ASSERT(parent, SA_LOCKED);

	LIST_FOREACH(p, &parent->children, children_link) {
		if (strcmp(p->name, name) == 0)
			return (true);
	}

	/* Not found */
	return (false);
}

/**
 * Set the provider for @p phandle.
 * 
 * @param	phandle	The NVRAM phandle to be modified. The handle must
 *			not have a provider set.
 * @param	prov	The provider prov to be set.
 */
static void
bhnd_nvram_phandle_set_provider(bhnd_nvram_phandle *phandle,
    struct bhnd_nvram_prov *prov)
{
	BHND_NVPATH_LOCK_ASSERT(phandle, SA_XLOCKED);
	BHND_NV_ASSERT(phandle->prov == NULL, ("provider set"));

	/* Save a weak reference to the provider */
	phandle->prov = BHND_NVREF_RETAIN_WEAK(prov, refs);
}

/**
 * Clear the provider for @p phandle.
 * 
 * @param	phandle	The NVRAM phandle to be modified.
 */
static void
bhnd_nvram_phandle_clear_provider(bhnd_nvram_phandle *phandle)
{
	BHND_NVPATH_LOCK_ASSERT(phandle, SA_XLOCKED);

	if (phandle->prov != NULL) {
		BHND_NVREF_RELEASE_WEAK(phandle->prov, refs);
		phandle->prov = NULL;
	}
}

/**
 * Retain a new reference to an open path handle, returning @p phandle
 * to the caller.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_path_release().
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
 * for releasing it via bhnd_nvram_path_release().
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
 * for releasing the reference via bhnd_nvram_path_release().
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
 * for releasing it via bhnd_nvram_path_release().
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
	phandle = bhnd_nvram_phandle_open(plane->root, NULL, pathname,
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
