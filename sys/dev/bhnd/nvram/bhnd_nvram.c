/*-
 * Copyright (c) 2015-2017 Landon Fuller <landonf@FreeBSD.org>
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
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/refcount.h>
#include <sys/systm.h>

#include "bhnd_nvramvar.h"
#include "bhnd_nvram_private.h"

MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "BHND NVRAM data");


static bool			 bhnd_nvplane_validate_name(const char *name);

static bhnd_nvram_plane_t	*bhnd_nvplane_find_child(
				     bhnd_nvram_plane_t *plane,
				     const char *name, size_t namelen);
static bool			 bhnd_nvplane_is_child_detached(
				     bhnd_nvram_plane_t *child,
				     bhnd_nvram_plane_t *parent);

/**
 * Return true if @p name is a valid NVRAM plane name, false otherwise.
 * 
 * @param name	The name to be evaluated.
 */
static bool
bhnd_nvplane_validate_name(const char *name)
{
	/* Name cannot contain path delimiters */
	if (strchr(name, '/') != NULL)
		return (false);

	/* Name cannot be '.' or '..' */
	if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0)
		return (false);

	return (true);
}


/**
 * Allocate a new NVRAM plane instance.
 * 
 * The caller is responsible for releasing the returned plane instance.
 * 
 * @param[out]	plane		On success, the newly allocated NVRAM plane
 *				instance.
 * @param	name		The name of the new plane instance.
 * @param	parent		The parent plane, or NULL.
 * @param	malloc_flags	Flags to be passed to malloc (see malloc(9)).
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval EINVAL	if @p name is not a valid relative name.
 * @retval EEXIST	if @p name already exists in @p parent.
 */
int
bhnd_nvram_plane_new(bhnd_nvram_plane_t **plane, const char *name,
    bhnd_nvram_plane_t *parent, int malloc_flags)
{
	bhnd_nvram_plane_t *p;

	if (!bhnd_nvplane_validate_name(name))
		return (EINVAL);

	/* Acquire parent lock, check for name conflict */
	if (parent != NULL) {
		bhnd_nvram_plane_t *child;

		BHND_NVPLANE_LOCK_RW(parent);
		LIST_FOREACH(child, &parent->children, np_link) {
			/* Skip if name does not match. The name is immutable;
			 * no locking is required */
			if (strcmp(child->name, name) != 0)
				continue;

			/* Ignore children if they're in the process of being
			 * detached from this parent */
			BHND_NVPLANE_LOCK_RO(child);
			if (child->parent == NULL) {
				BHND_NVPLANE_UNLOCK_RO(child);
				continue;
			}

			BHND_NVPLANE_UNLOCK_RO(child);
			BHND_NVPLANE_UNLOCK_RW(parent);
			return (EEXIST);
		}
	}

	/* Allocate new child plane instance */
	p = bhnd_nv_calloc(1, sizeof(*p), malloc_flags);
	if (p == NULL)
		return (ENOMEM);

	p->name = bhnd_nv_strdup(name, malloc_flags);
	if (p->name == NULL) {
		bhnd_nv_free(p);
		return (ENOMEM);
	}

	refcount_init(&p->refs, 1);
	p->pmap = NULL;

	BHND_NVPLANE_LOCK_INIT(p);
	LIST_INIT(&p->children);

	/* Connect to parent (if any), then drop the parent lock */
	if ((p->parent = parent) != NULL) {
		/* Parent owns a strong reference */
		bhnd_nvram_plane_retain(p);
		LIST_INSERT_HEAD(&parent->children, p, np_link);

		/* Drop parent lock */
		BHND_NVPLANE_UNLOCK_RW(parent);
	}

	*plane = p;
	return (0);
}

/**
 * Retain a strong reference to @p plane, returning @p plane to the caller.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_plane_release().
 * 
 * @param	plane	The NVRAM plane to be retained.
 */
bhnd_nvram_plane_t *
bhnd_nvram_plane_retain(bhnd_nvram_plane_t *plane)
{
	refcount_acquire(&plane->refs);
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
bhnd_nvram_plane_release(bhnd_nvram_plane_t *plane)
{
	if (!refcount_release(&plane->refs))
		return;

	/* Plane may not have been explicitly detached */
	bhnd_nvram_plane_detach(plane);

	/* Detach should release all parent, provider, and child references */
	BHND_NV_ASSERT(plane->parent == NULL, ("parent attached"));
	BHND_NV_ASSERT(plane->pmap == NULL, ("provider attached"));
	BHND_NV_ASSERT(LIST_EMPTY(&plane->children), ("children attached"));

	/* Free remaining internal state */
	bhnd_nv_free(plane->name);
	BHND_NVPLANE_LOCK_DESTROY(plane);

}

/**
 * Explicitly detach the plane (and all of its children) from both its parent
 * plane and its NVRAM provider, marking the plane as unavailable for use.
 * 
 * This operation is idempotent.
 * 
 * @param plane	The plane to be detached.
 */
void
bhnd_nvram_plane_detach(bhnd_nvram_plane_t *plane)
{
	bhnd_nvram_plane_t	*parent, *child, *next;
	bhnd_nvram_plane_list_t	 detached;

	LIST_INIT(&detached);

	/* Unmap the plane's backing provider, if any */
	bhnd_nvram_plane_unmap_provider(plane, NULL);

	/* Acquire a write lock on the parent (if any) and the child plane,
	 * and then detach the plane from its parent.
	 * 
	 * We have to do a bit of lock juggling to enforce our tree locking
	 * protocol */
	BHND_NVPLANE_LOCK_RW(plane);
	if ((parent = plane->parent) != NULL) {
		bhnd_nvram_plane_retain(parent);

		/* Acquire our (parent, child) lock in the correct order */
		BHND_NVPLANE_UNLOCK_RW(plane);
		BHND_NVPLANE_LOCK_RW(parent);
		BHND_NVPLANE_LOCK_RW(plane);

		/* Detach may have already been performed on another thread
		 * after we dropped our write lock */
		if (plane->parent != NULL) {
			BHND_NV_ASSERT(plane->parent == parent,
			    ("parent mismatch"));

			LIST_REMOVE(plane, np_link);
			bhnd_nvram_plane_release(plane);
			plane->parent = NULL;
		}
	}

	/* Acquire locks on all children in the required iteration order */
	LIST_FOREACH(child, &plane->children, np_link)
		BHND_NVPLANE_LOCK_RW(child);

	/* Detach all children from the current plane and move to our free
	 * list. */
	LIST_FOREACH_SAFE(child, &plane->children, np_link, next) {
		/* Already detached? */
		if (bhnd_nvplane_is_child_detached(child, parent))
			continue;

		/* Parent holds a strong reference on the child;
		 * we claim ownership of that reference here */
		child->parent = NULL;
		LIST_REMOVE(child, np_link);
		LIST_INSERT_HEAD(&detached, child, np_link);
	}

	/* Drop all currently held locks so that we can acquire per-child locks
	 * below */
	LIST_FOREACH(child, &plane->children, np_link)
		BHND_NVPLANE_UNLOCK_RW(child);

	BHND_NVPLANE_UNLOCK_RW(parent);
	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Detach and release all of our children */
	LIST_FOREACH_SAFE(child, &detached, np_link, next) {
		LIST_REMOVE(child, np_link);
		bhnd_nvram_plane_detach(child);
		bhnd_nvram_plane_release(child);
	}
}

/**
 * Return a borrowed reference to @p plane's relative name.
 * 
 * @param plane	The NVRAM plane to query.
 */
const char *
bhnd_nvram_plane_get_name(bhnd_nvram_plane_t *plane)
{
	/* Immutable, no locking required */
	return (plane->name);
}

/**
 * Return true if @p plane has been detached from its @p parent, false
 * otherwise.
 */
static bool
bhnd_nvplane_is_child_detached(bhnd_nvram_plane_t *child,
    bhnd_nvram_plane_t *parent)
{
	BHND_NVPLANE_LOCK_ASSERT(parent, SA_LOCKED);
	BHND_NVPLANE_LOCK_ASSERT(child, SA_LOCKED);

	if (child->parent == parent)
		return (false);

	BHND_NV_ASSERT(child->parent == NULL, ("dangling parent reference"));
	return (true);
}

/**
 * Return a borrowed reference to the child with @p name in @p plane, if any.
 */
static bhnd_nvram_plane_t *
bhnd_nvplane_find_child(bhnd_nvram_plane_t *plane, const char *name,
    size_t namelen)
{
	bhnd_nvram_plane_t *child;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	LIST_FOREACH(child, &plane->children, np_link) {
		/* Skip detached children */
		if (bhnd_nvplane_is_child_detached(child, plane))
			continue;

		/* Name is immutable; no locking required for
		 * comparison */
		if (strncmp(child->name, name, namelen) != 0)
			continue;

		if (strlen(child->name) != namelen)
			continue;

		/* Matched */
		return (child);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return a caller-owned reference to a child of @p plane, or NULL if
 * @p path is not found in @p plane.
 * 
 * The caller is responsible for releasing the returned reference via
 * bhnd_nvram_plane_release().
 * 
 * @param plane	The NVRAM plane to query.
 * @param path	A fully normalized NVRAM plane path. The @p pathname may be
 *		relative or absolute, but it will always be resolved relative
 *		to @p plane.
 */
bhnd_nvram_plane_t *
bhnd_nvram_plane_get_child(bhnd_nvram_plane_t *plane, const char *pathname)
{
	bhnd_nvram_plane_t	*cwd, *resolved;
	const char		*name;
	size_t			 namelen, pathlen;
	bool			 failed;

	/* Skip leading '/', if any */
	if (*pathname == '/')
		pathname++;

	/* Deterimine path length, rejecting empty paths */
	pathlen = strlen(pathname);
	if (pathlen == 0)
		return (NULL);

	/* Path must be fully normalized */
	if (!bhnd_nvram_is_normalized_path(pathname, pathlen))
		return (NULL);

	/*
	 * Perform path resolution, walking downwards from our initial plane
	 * until we either fully resolve the path, or we're unable to resolve
	 * a path component.
	 *
	 * We acquire locks at each level of the tree (conforming to our
	 * required parent->child lock order protocol).
	 */
	BHND_NVPLANE_LOCK_RO(plane);

	name = NULL;
	cwd = plane;
	failed = false;
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		bhnd_nvram_plane_t *child;

		/* Does this plane have a matching child? */
		child = bhnd_nvplane_find_child(cwd, name, namelen);
		if (child != NULL) {
			BHND_NVPLANE_LOCK_RO(child);
			cwd = child;
		} else {
			/* Child not found; terminate resolution and clean
			 * up */
			failed = true;
			break;
		}
	}

	/* If resolution succeeded, retain a reference to the resolved plane to
	 * ensure it remains valid (i.e. it is not concurrently deallocated
	 * by another thread) after we drop our locks below; We'll transfer
	 * ownership of this strong reference to our caller.
	 * 
	 * Otherwise, our ownership of parent locks ensures that cwd isn't
	 * released by its parent before we've performed our lock cleanup
	 * traversal, and no reference is required after lock cleanup is
	 * complete. */
	resolved = NULL;
	if (!failed)
		resolved = bhnd_nvram_plane_retain(cwd);

	/* Walk upwards from the last plane we resolved, dropping all
	 * previously acquired locks.  */
	while (cwd != NULL && cwd != plane->parent) {
		bhnd_nvram_plane_t *parent;

		parent = cwd->parent;
		BHND_NVPLANE_UNLOCK_RO(cwd);
	
		/* Traverse into the parent plane (if any) */
		cwd = parent;
	}

	/* Return a caller-owned reference to the result, or NULL if resolution
	 * failed */
	return (resolved);
}

/**
 * Retrieve a list of all children currently connected to @p plane, returning
 * the list in @p children and the count in @p count.
 * 
 * The caller is responsible for releasing the resources allocated for the
 * returned list using bhnd_nvram_plane_free_children().
 * 
 * @param	plane		The NVRAM plane to query.
 * @param[out]	children	The list of child planes.
 * @param[out]	count		The number of planes returned.
 * @param	malloc_flags	Flags to be passed to malloc (see malloc(9)).
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_get_children(bhnd_nvram_plane_t *plane,
    bhnd_nvram_plane_t ***children, size_t *count, int malloc_flags)
{
	bhnd_nvram_plane_t	*child, **list;
	size_t			 num_attached;

	BHND_NVPLANE_LOCK_RO(plane);

	/* Determine the number of attached children */
	num_attached = 0;
	LIST_FOREACH(child, &plane->children, np_link) {
		/* Skip detached children */
		if (child->parent == NULL)
			continue;

		BHND_NV_ASSERT(child->parent == plane, ("dangling parent ref"));

		/* Increment count */
		if (num_attached == SIZE_MAX) {
			BHND_NVPLANE_UNLOCK_RO(plane);
			return (ENOMEM);
		}
		num_attached++;
	}

	/* Allocate child list */
	list = bhnd_nv_calloc(num_attached, sizeof(bhnd_nvram_plane_t *),
	    malloc_flags);
	if (list == NULL) {
		BHND_NVPLANE_UNLOCK_RO(plane);
		return (ENOMEM);
	}

	*children = list;
	*count = 0;

	/* Populate child list */
	LIST_FOREACH(child, &plane->children, np_link) {
		/* Skip detached children */
		if (child->parent == NULL)
			continue;

		BHND_NV_ASSERT(*count < num_attached, ("invalid count"));

		list[*count] = bhnd_nvram_plane_retain(child);
		(*count)++;
	}

	BHND_NVPLANE_UNLOCK_RO(plane);
	return (0);
}

/**
 * Free a list of child planes previously returned by
 * bhnd_nvram_plane_get_children().
 * 
 * @param	plane		The NVRAM plane previously passed to
 *				bhnd_nvram_plane_get_children().
 * @param	children	The list of child planes to be freed.
 * @param	count		The number of planes in @p children.
 */
void
bhnd_nvram_plane_free_children(bhnd_nvram_plane_t *plane,
    bhnd_nvram_plane_t **children, size_t count)
{
	for (size_t i = 0; i < count; i++)
		bhnd_nvram_plane_release(children[i]);

	bhnd_nv_free(children);
}

/**
 * Map @p provider into the given NVRAM plane.
 * 
 * @param	plane		The NVRAM plane into which @p provider will be
 * 				mapped.
 * @param	provider	The NVRAM provider to be mapped.
 * @param	malloc_flags	Flags to be passed to malloc (see malloc(9)).
 *
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval non-zero	If mapping the provider otherwise fails, a regular unix
 *			error code will be returned.
 */
int
bhnd_nvram_plane_map_provider(bhnd_nvram_plane_t *plane,
    bhnd_nvram_prov_t *provider, int malloc_flags)
{
	bhnd_nvram_plane_pmap_t	*pmap;
	int			 error;

	if (provider == NULL)
		BHND_NV_PANIC("NULL provider");

	/* Allocate provider mapping state */
	pmap = bhnd_nv_calloc(1, sizeof(*pmap), malloc_flags);
	if (pmap == NULL)
		return (ENOMEM);

	mtx_init(&pmap->pm_lock, "BHND NVRAM pm_req mutex", NULL, MTX_DEF);

	pmap->pm_prov = provider;
	pmap->pm_reqs = 0;

	/* Attempt to map into the plane */
	BHND_NVPLANE_LOCK_RW(plane);
	if (plane->pmap == NULL) {
		plane->pmap = pmap;
		error = 0;
	} else {
		error = EEXIST;
	}
	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Clean up unused mapping state on error */
	if (error) {
		mtx_destroy(&pmap->pm_lock);
		bhnd_nv_free(pmap);
	}

	return (error);
}

/**
 * Unmap @p provider from the NVRAM plane.
 * 
 * @param	plane		The NVRAM plane from which @p provider will be
 * 				unmapped.
 * @param	provider	The NVRAM provider to be unmapped, or NULL
 *				to unmap all currently mapped providers.
 */
void
bhnd_nvram_plane_unmap_provider(bhnd_nvram_plane_t *plane,
    bhnd_nvram_prov_t *provider)
{
	bhnd_nvram_plane_pmap_t	*pmap;

	BHND_NVPLANE_LOCK_RW(plane);

	/* Fetch the provider mapping (if any) */
	if ((pmap = plane->pmap) == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return;
	}

	/* Skip if current mapping does not match the specified provider */
	if (provider != NULL && plane->pmap->pm_prov != provider) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return;
	}

	/* Found match; disconnect mapping from the plane */
	plane->pmap = NULL;

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* The provider is no longer mapped by the plane; we only need to wait
	 * for any outstanding requests to complete.
	 * 
	 * Once the request count hits zero, no concurrent threads hold a
	 * reference to this mapping */
	mtx_lock(&pmap->pm_lock);
	while (pmap->pm_reqs > 0) {
		mtx_sleep(pmap, &pmap->pm_lock, 0, "bhnd_nvrq", 0);
	}
	mtx_unlock(&pmap->pm_lock);

	BHND_NV_ASSERT(pmap->pm_reqs == 0, ("requests active"));

	/* Clean up mapping state */
	mtx_destroy(&pmap->pm_lock);
	bhnd_nv_free(pmap);
}
