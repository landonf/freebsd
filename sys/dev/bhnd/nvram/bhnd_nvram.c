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

#include "bhnd_nvram_prov_if.h"


MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "BHND NVRAM data");


static bool			 bhnd_nvplane_validate_name(const char *name);

static int			 bhnd_nvplane_pmap_enter(
				     bhnd_nvram_pmap_t *pmap);
static void			 bhnd_nvplane_pmap_exit(
				     bhnd_nvram_pmap_t *pmap);
static void			 bhnd_nvplane_pmap_wait_reqs(
				     bhnd_nvram_pmap_t *pmap);

static int			 bhnd_nvplane_entry_enter(
				     bhnd_nvram_entry_t *entry,
				     bhnd_nvram_pmap_t **pmap,
				     bhnd_nvram_phandle_t *phandle);
static void			 bhnd_nvplane_entry_exit(
				     bhnd_nvram_entry_t *entry,
				     bhnd_nvram_pmap_t *pmap,
				     bhnd_nvram_phandle_t phandle);

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
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval EINVAL	if @p name is not a valid relative name.
 * @retval EEXIST	if @p name already exists in @p parent.
 */
int
bhnd_nvram_plane_new(bhnd_nvram_plane_t **plane, const char *name,
    bhnd_nvram_plane_t *parent)
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
	p = bhnd_nv_calloc(1, sizeof(*p), M_NOWAIT);
	if (p == NULL)
		return (ENOMEM);

	p->name = bhnd_nv_strdup(name, M_NOWAIT);
	if (p->name == NULL) {
		bhnd_nv_free(p);
		return (ENOMEM);
	}

	refcount_init(&p->refs, 1);
	p->pmap = NULL;

	BHND_NVPLANE_LOCK_INIT(p);
	LIST_INIT(&p->children);
	LIST_INIT(&p->entries);

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
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_get_children(bhnd_nvram_plane_t *plane,
    bhnd_nvram_plane_t ***children, size_t *count)
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
	    M_NOWAIT);
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
 *
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval non-zero	If mapping the provider otherwise fails, a regular unix
 *			error code will be returned.
 */
int
bhnd_nvram_plane_map_provider(bhnd_nvram_plane_t *plane,
    bhnd_nvram_prov_t *provider)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_entry_t	*entry;

	if (provider == NULL)
		BHND_NV_PANIC("NULL provider");

	/* Allocate provider mapping state */
	pmap = bhnd_nv_calloc(1, sizeof(*pmap), M_NOWAIT);
	if (pmap == NULL)
		return (ENOMEM);

	mtx_init(&pmap->lock, "BHND NVRAM pm_req mutex", NULL, MTX_DEF);

	pmap->prov = provider;
	pmap->reqs = 0;

	BHND_NVPLANE_LOCK_RW(plane);

	/* Plane must not already have a provider mapped. */
	if (plane->pmap != NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);

		mtx_destroy(&pmap->lock);
		bhnd_nv_free(pmap);

		return (EEXIST);
	}

	/* Connect the provider, associate all outstanding entries */
	plane->pmap = pmap;

	LIST_FOREACH(entry, &plane->entries, ne_link) {
		BHND_NV_ASSERT(entry->pmap == NULL, ("dangling pmap"));
		BHND_NV_ASSERT(entry->phandle == BHND_NVRAM_PHANDLE_NULL,
		    ("dangling phandle"));

		entry->pmap = pmap;
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (0);
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
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_entry_t	*entry;

	BHND_NVPLANE_LOCK_RW(plane);

	/* Fetch the provider mapping (if any) */
	if ((pmap = plane->pmap) == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return;
	}

	/* Skip if current mapping does not match the specified provider */
	if (provider != NULL && plane->pmap->prov != provider) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return;
	}

	/* Found match; invalidate all entries */
	BHND_NV_ASSERT(plane->pmap->prov == provider, ("invalid prov"));
	LIST_FOREACH(entry, &plane->entries, ne_link) {
		BHND_NV_ASSERT(entry->pmap == pmap, ("dangling pmap"));

		entry->pmap = NULL;
		entry->phandle = BHND_NVRAM_PHANDLE_NULL;
	}

	/* Disconnect from the plane */
	plane->pmap = NULL;

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* The provider is no longer mapped by the plane; we only need to wait
	 * for any outstanding requests to complete.
	 * 
	 * Once the request count hits zero, no concurrent threads hold a
	 * reference to this mapping */
	bhnd_nvplane_pmap_wait_reqs(pmap);
	BHND_NV_ASSERT(pmap->reqs == 0, ("requests active"));

	/* Clean up mapping state */
	mtx_destroy(&pmap->lock);
	bhnd_nv_free(pmap);
}

/**
 * Return an entry instance for @p path, allocating and registering a new
 * entry if no existing entry can be found.
 * 
 * The caller is responsible for releasing the returned entry via
 * bhnd_nvram_entry_release().
 */
static bhnd_nvram_entry_t *
bhnd_nvplane_get_entry(bhnd_nvram_plane_t *plane, const char *path)
{
	bhnd_nvram_entry_t *entry;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	LIST_FOREACH(entry, &plane->entries, ne_link) {
		if (strcmp(entry->path, path) == 0)
			return (bhnd_nvram_entry_retain(entry));
	}

	/* Not found; register a new entry */
	entry = bhnd_nv_calloc(1, sizeof(*entry), M_NOWAIT);
	if (entry == NULL)
		return (NULL);

	entry->path = bhnd_nv_strdup(path, M_NOWAIT);
	if (entry->path == NULL) {
		bhnd_nv_free(entry);
		return (NULL);
	}

	refcount_init(&entry->refs, 1);
	entry->plane = bhnd_nvram_plane_retain(plane);
	entry->pmap = plane->pmap;
	entry->phandle = BHND_NVRAM_PHANDLE_NULL;

	LIST_INSERT_HEAD(&plane->entries, entry, ne_link);
	return (entry);
}

/**
 * 
 */
int
bhnd_nvram_plane_find_entry(bhnd_nvram_plane_t *plane, const char *path,
    bhnd_nvram_entry_t **entry)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	bhnd_nvram_entry_t	*result;
	int			 error;

	BHND_NVPLANE_LOCK_RW(plane);

	/* All entry lookups fail without a provider */
	if (plane->pmap == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (ENODEV);
	}

	/* Retain or allocate an entry instance */
	if ((result = bhnd_nvplane_get_entry(plane, path)) == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (ENOMEM);
	}

	/* Claim the entry's phandle reference, forcing reacquisition of
	 * the reference below */
	phandle = result->phandle;
	result->phandle = BHND_NVRAM_PHANDLE_NULL;

	/* Ensure the provider will not be be unmapped after we drop
	 * our lock below */
	pmap = result->pmap;
	if ((error = bhnd_nvplane_pmap_enter(result->pmap))) {
		BHND_NVPLANE_UNLOCK_RW(plane);

		bhnd_nvram_entry_release(result);
		return (error);
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Drop the now unused phandle reference, and then release our
	 * reservation on the provider mapping */
	BHND_NVRAM_PROV_RELEASE_PATH(pmap->prov, phandle);
	bhnd_nvplane_pmap_exit(pmap);

	pmap = NULL;
	phandle = BHND_NVRAM_PHANDLE_NULL;

	/* Force reacquisition of the entry's phandle, verifying that the
	 * requested path still exists in the backing provider */
	if ((error = bhnd_nvplane_entry_enter(result, &pmap, &phandle))) {
		bhnd_nvram_entry_release(result);
		return (error);
	}

	bhnd_nvplane_entry_exit(result, pmap, phandle);

	/* Return the validated entry */
	*entry = result;
	return (0);
}

bhnd_nvram_entry_t *
bhnd_nvram_entry_retain(bhnd_nvram_entry_t *entry)
{
	refcount_acquire(&entry->refs);
	return (entry);
}

void
bhnd_nvram_entry_release(bhnd_nvram_entry_t *entry)
{
	/* We need to hold our lock to guarantee that the entry cannot
	 * be resurrected by the NVRAM plane's weak reference after the
	 * refcount hits zero */
	BHND_NVPLANE_LOCK_RW(entry->plane);

	if (!refcount_release(&entry->refs)) {
		BHND_NVPLANE_UNLOCK_RW(entry->plane);
		return;
	}

	BHND_NV_ASSERT(
	    bhnd_nvplane_get_entry(entry->plane, entry->path) == entry,
	    ("dangling entry reference"));

	/* Disconnect the entry from the plane */
	LIST_REMOVE(entry, ne_link);

	/* Release phandle reference (plane lock is held to prevent unmapping
	 * of the provider) */
	if (entry->pmap != NULL && entry->phandle != BHND_NVRAM_PHANDLE_NULL)
		BHND_NVRAM_PROV_RELEASE_PATH(entry->pmap->prov, entry->phandle);

	BHND_NVPLANE_UNLOCK_RW(entry->plane);

	/* Clean up remaining entry state */
	bhnd_nv_free(entry->path);
}

int
bhnd_nvram_get_children(bhnd_nvram_entry_t *entry,
    bhnd_nvram_entry_t ***children, size_t *count)
{
#if 0
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	bhnd_nvram_phandle_t	*phandles;
	bhnd_nvram_entry_t	**entries;
	char			**paths;
	size_t			 num_phandles;
	int			 error;

	if ((error = bhnd_nvplane_entry_enter(entry, &pmap, &phandle)))
		return (error);

	/* Fetch phandles for all children */
	error = BHND_NVRAM_PROV_GET_CHILDREN(pmap->prov, phandle, &phandles,
	    &num_phandles);
	if (error) {
		bhnd_nvplane_entry_exit(entry, pmap, phandle);
		return (error);
	}

	/* Fetch path names */
	paths = bhnd_nv_calloc(num_phandles, sizeof(*paths), M_NOWAIT);
	if (paths == NULL) {
		bhnd_nvplane_entry_exit(entry, pmap, phandle);
		return (ENOMEM);
	}


	/* Allocate and populate our entry list buffer */
	entries = bhnd_nv_calloc(num_phandles, sizeof(*entries), M_NOWAIT);
	if (entries == NULL) {
		bhnd_nvplane_entry_exit(entry, pmap, phandle);
		return (ENOMEM);
	}

	BHND_NVPLANE_LOCK_RW(plane);
	for (size_t i = 0; i < num_phandles; i++) {
		entries[i] = bhnd_nvplane_get_entry(plane, )
	}
	BHND_NVPLANE_UNLOCK_RW(plane);
#endif

	panic("TODO");
}

int
bhnd_nvram_free_children(bhnd_nvram_entry_t *entry,
    bhnd_nvram_entry_t **children, size_t count)
{
	panic("TODO");
}

int
bhnd_nvram_setprop(bhnd_nvram_entry_t *entry, const char *propname,
    const void *buf, size_t len, bhnd_nvram_type type)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	int			 error;

	if ((error = bhnd_nvplane_entry_enter(entry, &pmap, &phandle)))
		return (error);

	error = BHND_NVRAM_PROV_SETPROP(pmap->prov, phandle, propname, buf,
	    len, type);

	bhnd_nvplane_entry_exit(entry, pmap, phandle);

	return (error);
}

int
bhnd_nvram_delprop(bhnd_nvram_entry_t *entry, const char *propname)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	int			 error;

	if ((error = bhnd_nvplane_entry_enter(entry, &pmap, &phandle)))
		return (error);

	error = BHND_NVRAM_PROV_DELPROP(pmap->prov, phandle, propname);

	bhnd_nvplane_entry_exit(entry, pmap, phandle);

	return (error);
}

int
bhnd_nvram_getprop(bhnd_nvram_entry_t *entry, const char *propname, void *buf,
    size_t *len, bhnd_nvram_type type, bool search_parents)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	int			 error;

	if ((error = bhnd_nvplane_entry_enter(entry, &pmap, &phandle)))
		return (error);

	error = BHND_NVRAM_PROV_GETPROP(pmap->prov, phandle, propname, buf,
	    len, type, search_parents);

	bhnd_nvplane_entry_exit(entry, pmap, phandle);

	return (error);
}

int
bhnd_nvram_getprop_alloc(bhnd_nvram_entry_t *entry, const char *propname,
    void **buf, size_t *len, bhnd_nvram_type type, bool search_parents)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	void			*outp;
	size_t			 req_olen, olen;
	int			 error;

	if ((error = bhnd_nvplane_entry_enter(entry, &pmap, &phandle)))
		return (error);

	outp = NULL;
	do {
		if (outp != NULL)
			bhnd_nv_free(outp);

		/* Determine required buffer size */
		error = BHND_NVRAM_PROV_GETPROP(pmap->prov, phandle, propname,
		    NULL, &req_olen, type, search_parents);
		if (error) {
			goto cleanup;
		}

		/* Allocate and fetch to buffer */
		outp = bhnd_nv_malloc(req_olen, M_NOWAIT);
		if (outp == NULL) {
			error = ENOMEM;
			goto cleanup;
		}

		olen = req_olen;
		error = BHND_NVRAM_PROV_GETPROP(pmap->prov, phandle, propname,
		    outp, &olen, type, search_parents);
	} while (error == ENOMEM && olen > req_olen);

cleanup:
	bhnd_nvplane_entry_exit(entry, pmap, phandle);

	if (!error) {
		*buf = outp;
		*len = olen;
	} else {
		if (outp != NULL)
			bhnd_nv_free(outp);
	}

	return (error);
}

void
bhnd_nvram_getprop_free(void *buf)
{
	bhnd_nv_free(buf);
}

int
bhnd_nvram_copyprops(bhnd_nvram_entry_t *entry,
    struct bhnd_nvram_plist **plist)
{
	bhnd_nvram_pmap_t	*pmap;
	bhnd_nvram_phandle_t	 phandle;
	int			 error;

	if ((error = bhnd_nvplane_entry_enter(entry, &pmap, &phandle)))
		return (error);

	error = BHND_NVRAM_PROV_COPYPROPS(pmap->prov, phandle, plist);

	bhnd_nvplane_entry_exit(entry, pmap, phandle);

	return (error);
}

/**
 * Begin a new request against @p pmap, acquiring a usage reservation on the
 * given provider mapping; the backing provider reference is guaranteed to
 * remain valid until a matching call to bhnd_nvplane_pmap_exit().
 * 
 * @param pmap	The provider mapping.
 * 
 * @retval 0		success
 * @retval EAGAIN	if incrementing the use count would overflow.
 */
static int
bhnd_nvplane_pmap_enter(bhnd_nvram_pmap_t *pmap)
{
	mtx_lock(&pmap->lock);

	if (pmap->reqs >= BHND_NVPLANE_PROV_REQS_MAX) {
		mtx_unlock(&pmap->lock);
		return (EAGAIN);
	}

	pmap->reqs++;

	mtx_unlock(&pmap->lock);

	return (0);
}


/**
 * Release a usage reservation held on @p pmap.
 * 
 * @param pmap	The provider mapping.
 */
static void
bhnd_nvplane_pmap_exit(bhnd_nvram_pmap_t *pmap)
{
	mtx_lock(&pmap->lock);

	BHND_NV_ASSERT(pmap->reqs > 0, ("overrelease"));

	pmap->reqs--;
	if (pmap->reqs == 0)
		wakeup(pmap);

	mtx_unlock(&pmap->lock);
}

/**
 * Sleep until all requests have completed on @p pmap.
 * 
 * @param pmap	The provider mapping.
 */
static void
bhnd_nvplane_pmap_wait_reqs(bhnd_nvram_pmap_t *pmap)
{
	mtx_assert(&pmap->lock, SA_UNLOCKED);

	mtx_lock(&pmap->lock);
	while (pmap->reqs > 0) {
		mtx_sleep(pmap, &pmap->lock, 0, "bhnd_nvrq", 0);
	}
	mtx_unlock(&pmap->lock);
}

/**
 * Begin a new request against @p entry, acquiring a request reservation on the
 * mapped provider.
 * 
 * Upon request completion, bhnd_nvplane_entry_exit() must be called to release
 * all acquired resources.
 * 
 * @param	entry	The plane entry.
 * @param[out]	pmap	On success, the provider mapping for @p entry.
 * @param[out]	phandle	On success, the provider handle for @p entry.
 * 
 * @retval 0		success
 * @retval EAGAIN	if incrementing the use count would overflow.
 * @retval ENODEV	if the entry's path is no longer mapped by a provider.
 * @retval non-zero	if starting a request on @p entry otherwise fails, a
 *			regular unix error code will be returned.
 */
static int
bhnd_nvplane_entry_enter(bhnd_nvram_entry_t *entry, bhnd_nvram_pmap_t **pmap,
    bhnd_nvram_phandle_t *phandle)
{
	int error;

	BHND_NVPLANE_LOCK_ASSERT(entry->plane, SA_UNLOCKED);

	/*
	 * Fetch and retain the entry's backing provider mapping and
	 * phandle (if any).
	 */
	BHND_NVPLANE_LOCK_RO(entry->plane);

	/* Must have a provider mapping */
	if (entry->pmap == NULL) {
		BHND_NVPLANE_UNLOCK_RO(entry->plane);
		return (ENODEV);
	}

	/* Acquire a reservation on the provider mapping */
	if ((error = bhnd_nvplane_pmap_enter(entry->pmap))) {
		BHND_NVPLANE_UNLOCK_RO(entry->plane);
		return (error);
	}

	/* Save the provider and phandle references */
	*pmap = entry->pmap;
	*phandle = entry->phandle;
	if (*phandle != BHND_NVRAM_PHANDLE_NULL)
		BHND_NVRAM_PROV_RETAIN_PATH((*pmap)->prov, *phandle);

	BHND_NVPLANE_UNLOCK_RO(entry->plane);

	/*
	 * Lazy fallback resolution of the entry's phandle.
	 */
	if (*phandle == BHND_NVRAM_PHANDLE_NULL) {
		/* Try to open the path */
		error = BHND_NVRAM_PROV_OPEN_PATH((*pmap)->prov,
		    BHND_NVRAM_PHANDLE_NULL, entry->path, strlen(entry->path),
		    phandle);

		if (error) {
			bhnd_nvplane_pmap_exit(*pmap);
			return (error);
		}

		/* Try to update our entry with the resolved phandle */
		BHND_NVPLANE_LOCK_RW(entry->plane);
		if (entry->pmap == *pmap &&
		    entry->phandle == BHND_NVRAM_PHANDLE_NULL)
		{
			BHND_NVRAM_PROV_RETAIN_PATH((*pmap)->prov, *phandle);
			entry->phandle = *phandle;
		}
		BHND_NVPLANE_UNLOCK_RW(entry->plane);
	}

	/* The pmap and phandle are now valid; we transfer ownership of our
	 * open pmap reservation and the strong phandle reference to our
	 * caller */
	return (0);
}

/**
 * Terminate a request against @p entry, releasing all resources acquired
 * by bhnd_nvplane_entry_enter().
 * 
 * @param	entry	The plane entry.
 * @param[out]	pmap	The provider mapping returned by
 *			bhnd_nvplane_entry_enter().
 * @param[out]	phandle	The provider handle returned by
 *			bhnd_nvplane_entry_enter().
 * 
 * @retval 0		success
 * @retval EAGAIN	if incrementing the use count would overflow.
 * @retval ENODEV	if the entry's path is no longer mapped by a provider.
 * @retval non-zero	if starting a request on @p entry otherwise fails, a
 *			regular unix error code will be returned.
 */
static void
bhnd_nvplane_entry_exit(bhnd_nvram_entry_t *entry, bhnd_nvram_pmap_t *pmap,
    bhnd_nvram_phandle_t phandle)
{
	BHND_NVRAM_PROV_RELEASE_PATH(pmap->prov, phandle);
	bhnd_nvplane_pmap_exit(pmap);
}
