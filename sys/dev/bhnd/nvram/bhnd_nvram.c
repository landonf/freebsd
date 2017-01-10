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

#ifdef _KERNEL
#include <sys/malloc.h>
#include <sys/kernel.h>
#else /* !_KERNEL */
#include <errno.h>
#include <string.h>
#include <stdint.h>
#endif /* _KERNEL */

#include "bhnd_nvram_private.h"
#include "bhnd_nvramvar.h"

#ifdef _KERNEL
MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "BHND NVRAM data");
#endif /* _KERNEL */

static struct bhnd_nvpath_str	*bhnd_nvpath_str_new(const char *pathname,
				     size_t pathlen);
static struct bhnd_nvpath_str	*bhnd_nvpath_str_append(const char *pathname,
				     size_t pathlen, const char *name,
				     size_t namelen);
static void			 bhnd_nvpath_str_fini(
				     struct bhnd_nvpath_str *path);

static bhnd_nvram_consumer	*bhnd_nvram_consumer_new(
				     struct bhnd_nvram_plane *plane);
static void			 bhnd_nvram_consumer_fini(
				     struct bhnd_nvram_consumer *consumer);

static struct bhnd_nvram_link	*bhnd_nvram_link_new(const char *name,
				     size_t namelen,
				     struct bhnd_nvram_link *parent,
				     struct bhnd_nvram_provider *prov,
				     struct bhnd_nvpath_str *prov_path);
static void			 bhnd_nvram_link_free(
				     struct bhnd_nvram_link *link);

static void			 bhnd_nvram_link_clear_provider(
				     struct bhnd_nvram_link *link);

static int			 bhnd_nvram_link_insert(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *cwd,
				     const char *pathname, size_t pathlen,
				     struct bhnd_nvram_link **link);
static struct bhnd_nvram_link	*bhnd_nvram_link_resolve(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *cwd,
				     const char *pathname,
				     size_t pathlen);

static void			 bhnd_nvram_plane_fini(
				     struct bhnd_nvram_plane *plane);

static bool			 bhnd_nvram_plane_is_leaf(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link);
static void			 bhnd_nvram_plane_try_remove(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link);

static void			 bhnd_nvram_plane_remove_provider(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_provider *provider);

static void			 bhnd_nvram_provider_fini(
				     struct bhnd_nvram_provider *provider);

static int			 bhnd_nvram_provider_add_consumer(
				     struct bhnd_nvram_provider *provider,
				     struct bhnd_nvram_plane *plane,
				     size_t use_count);
static void			 bhnd_nvram_provider_remove_consumer(
				     struct bhnd_nvram_provider *provider,
				     struct bhnd_nvram_plane *plane,
				     size_t use_count);

static int			 bhnd_nvram_provider_busy(
				     struct bhnd_nvram_provider *provider) __attribute__((unused));
static int			 bhnd_nvram_provider_busy_locked(
				     struct bhnd_nvram_provider *provider);

static void			 bhnd_nvram_provider_unbusy(
				     struct bhnd_nvram_provider *provider) __attribute__((unused));
static void			 bhnd_nvram_provider_unbusy_locked(
				     struct bhnd_nvram_provider *provider);


#define	BHND_NVPROV_ASSERT_STATE(_prov, _state)		\
	BHND_NV_ASSERT((_prov)->state == (_state),	\
	    ("invalid state: %d", (_prov)->state))

#define	BHND_NVPROV_ASSERT_ACTIVE(_prov)		\
	BHND_NVPROV_ASSERT_STATE((_prov), BHND_NVRAM_PROV_ACTIVE)

/**
 * Allocate and initialize a reference-counted NVRAM path string.
 * 
 * The caller is responsible for releasing the returned path instance.
 * 
 * @param pathname	A fully qualified path in normalized (canonical) form.
 * @param pathlen	The length of @p pathname.
 * 
 * @retval NULL if allocation fails.
 */
static struct bhnd_nvpath_str *
bhnd_nvpath_str_new(const char *pathname, size_t pathlen)
{
	return (bhnd_nvpath_str_append(pathname, pathlen, NULL, 0));
}

/**
 * Allocate and return a new path string by appending @p name to @p pathname.
 * 
 * The returned path string will be fully normalized.
 * 
 * The caller is responsible for releasing the returned path instance.
 *
 * @param	pathname	The parent path.
 * @param	pathlen		The length of @p pathname.
 * @param	name		The path component to be appended.
 * @param	namelen		The length of @p name.
 * 
 * @retval non-NULL	success
 * @retval NULL		if allocation fails.
 */
static struct bhnd_nvpath_str *
bhnd_nvpath_str_append(const char *pathname, size_t pathlen, const char *name,
    size_t namelen)
{
	struct bhnd_nvpath_str	*path;
	char			*pathbuf;
	size_t			 bufsize, baselen;
	bool			 need_delim;

	/* Map NULL zero-length strings to valid empty string pointers */
	if (pathname == NULL) {
		BHND_NV_ASSERT(pathlen == 0, ("NULL pathname"));
		pathname = "";
	}

	if (name == NULL) {
		BHND_NV_ASSERT(namelen == 0, ("NULL name"));
		name = "";
	}

	/* Calculate the lengths minus any trailing NUL */
	namelen = strnlen(name, namelen);
	pathlen = strnlen(pathname, pathlen);

	/* Do we need to insert a '/' delimiter? Only necessary if both
	 * strings to be appended are non-empty. */
	need_delim = false;
	if (namelen > 0 && pathlen > 0) {
		if ((pathname[pathlen-1] != '/') && (name[0] != '/'))
			need_delim = true;
	}

	/* Determine total concatenated length of path + / + name + '\0' */
	baselen = pathlen;
	if (need_delim)
		baselen += 1; /* '/' */

	bufsize = baselen + namelen + 1 /* '\0' */;

	/* Produce concatenated path */
	if ((pathbuf = bhnd_nv_malloc(bufsize)) == NULL)
		return (NULL);

	strncpy(pathbuf, pathname, pathlen);
	if (need_delim) {
		BHND_NV_ASSERT(baselen > 0, ("invalid base length"));
		pathbuf[baselen - 1] = '/';
	}
	strncpy(pathbuf+baselen, name, namelen);

	BHND_NV_ASSERT(bufsize > 0, ("invalid bufsize"));
	pathbuf[bufsize-1] = '\0';

	/* If required, normalize path string */
	if (!bhnd_nvram_is_normalized_path(pathbuf, bufsize)) {
		char	*nbuf;

		if ((nbuf = bhnd_nv_malloc(bufsize)) == NULL) {
			bhnd_nv_free(pathbuf);
			return (NULL);
		}

		bhnd_nvram_normalize_path(pathbuf, bufsize, nbuf, bufsize);
		bhnd_nv_free(pathbuf);
		pathbuf = nbuf;
	}

	/* Allocate new path instance */
	if ((path = bhnd_nv_calloc(1, sizeof(*path))) == NULL) {
		bhnd_nv_free(pathbuf);
		return (NULL);
	}

	BHND_NVREF_INIT(&path->refs);
	path->pathname = pathbuf;
	path->pathlen = strlen(pathbuf);

	/* Populate path's base name */
	path->basename = bhnd_nvram_parse_path_filename(path->pathname,
	    path->pathlen, &path->baselen);

	return (path);
}

static void
bhnd_nvpath_str_fini(struct bhnd_nvpath_str *path)
{
	bhnd_nv_free(path->pathname);
}

/**
 * Allocate, initialize, and return a new NVRAM consumer record.
 * 
 * The caller is responsible for releasing the returned consumer record.
 * 
 * @param plane		The NVRAM plane associated with this consumer record.
 * 
 * @retval non-NULL	success.
 * @retval NULL		if allocation fails.
 */
static struct bhnd_nvram_consumer *
bhnd_nvram_consumer_new(struct bhnd_nvram_plane *plane)
{
	struct bhnd_nvram_consumer *consumer;

	consumer = bhnd_nv_calloc(1, sizeof(*consumer));
	if (consumer == NULL)
		return (NULL);

	BHND_NVREF_INIT(&consumer->refs);

	consumer->plane = BHND_NVREF_RETAIN_WEAK(plane, refs);
	consumer->uses = 0;

	return (consumer);
}


static void
bhnd_nvram_consumer_fini(struct bhnd_nvram_consumer *consumer)
{
	BHND_NVREF_RELEASE_WEAK(consumer->plane, refs);
}

/**
 * Allocate, initialize, and return a new NVRAM provider instance.
 * 
 * @param dev		The new provider's NVRAM device.
 * 
 * @retval non-NULL	success.
 * @retval NULL		if allocation fails.
 */
struct bhnd_nvram_provider *
bhnd_nvram_provider_new(device_t dev)
{
	struct bhnd_nvram_provider *provider;

	/* Allocate new provider instance */
	provider = bhnd_nv_calloc(1, sizeof(*provider));
	if (provider == NULL)
		return (NULL);

	provider->dev = dev;
	provider->state = BHND_NVRAM_PROV_ACTIVE;

	BHND_NVPROV_LOCK_INIT(provider);
	BHND_NVREF_INIT(&provider->refs);
	refcount_init(&provider->busy, 0);

	LIST_INIT(&provider->consumers);

	return (provider);
}

/**
 * Release a @p provider instance previously allocated via
 * bhnd_nvram_provider_new().
 * 
 * This function will:
 * 
 * - Mark the provider as unavailable, preventing any new requests from
 *   being dispatched to the device, and then sleep until all outstanding
 *   requests have completed.
 * - Remove all consumer references to the provider, including any paths
 *   registered via bhnd_nvram_register_paths(), or paths re-exported to
 *   additional NVRAM planes.
 * - Release the caller's strong reference to provider, allowing deallocation
 *   of the provider and associated resources.
 * 
 * Upon return, no further requests will be made to @p provider.
 */
void
bhnd_nvram_provider_destroy(struct bhnd_nvram_provider *provider)
{
	struct bhnd_nvram_consumer_list	 consumers;
	struct bhnd_nvram_consumer	*c, *cnext;

	LIST_INIT(&consumers);

	BHND_NVPROV_LOCK_RW(provider);

	/* Set stopping state; any calls to bhnd_nvram_provider_busy() after
	 * this point will fail with ENODEV */
	BHND_NVPROV_ASSERT_ACTIVE(provider);
	provider->state = BHND_NVRAM_PROV_STOPPING;

	/* Wait for all outstanding reservations to be released. */
	while (provider->busy > 0)
		BHND_NVPROV_LOCK_WAIT(provider);

	/*
	 * After this point, all outstanding requests have completed, and
	 * any concurrent access to the provider will terminate upon reading
	 * the provider's state.
	 * 
	 * Migrate provider to 'dead' state, denoting that all requests
	 * have terminated.
	 */
	provider->state = BHND_NVRAM_PROV_DEAD;

	/* Claim all outstanding consumer references */
	LIST_FOREACH_SAFE(c, &provider->consumers, nc_link, cnext) {
		LIST_REMOVE(c, nc_link);
		LIST_INSERT_HEAD(&consumers, c, nc_link);
	}

	BHND_NVPROV_UNLOCK_RW(provider);

	/* Ask all consumers to discard their references to this provider,
	 * and then discard our consumer references */
	LIST_FOREACH_SAFE(c, &consumers, nc_link, cnext) {
		struct bhnd_nvram_plane *plane;

		if ((plane = BHND_NVREF_PROMOTE_WEAK(c->plane, refs)) != NULL) {
			bhnd_nvram_plane_remove_provider(plane, provider) ;
			bhnd_nvram_plane_release(plane);
		}

		LIST_REMOVE(c, nc_link);
		BHND_NVREF_RELEASE(c, refs, bhnd_nvram_consumer_fini);
	}

	/* Release the caller's provider reference */
	BHND_NVREF_RELEASE(provider, refs, bhnd_nvram_provider_fini);
}

static void
bhnd_nvram_provider_fini(struct bhnd_nvram_provider *provider)
{
	/* Should not be possible to reach the finalization without
	 * first calling bhnd_nvram_provider_destroy() */
	BHND_NVPROV_ASSERT_STATE(provider, BHND_NVRAM_PROV_DEAD);

	BHND_NV_ASSERT(LIST_EMPTY(&provider->consumers), ("active consumers"));
	BHND_NV_ASSERT(provider->busy == 0, ("provider in use"));

	BHND_NVPROV_LOCK_DESTROY(provider);
}

/**
 * Add a consumer reference for @p plane to @p provider.
 * 
 * @param provider	The provider to be modified.
 * @param plane		The plane for which a consumer reference should be
 *			added.
 * @param use_count	The use count to be added to @p plane's consumer entry.
 * 
 * @retval 0		success.
 * @retval ERANGE	if @p num_uses would overflow.
 * @retval ENOMEM	if allocation fails.
 * @retval ENODEV	if @p provider is marked for removal.
 */
static int
bhnd_nvram_provider_add_consumer(struct bhnd_nvram_provider *provider,
    struct bhnd_nvram_plane *plane, size_t use_count)
{
	struct bhnd_nvram_consumer *c, *consumer;

	BHND_NVPROV_LOCK_RW(provider);

	/* Provider must be active */
	if (provider->state != BHND_NVRAM_PROV_ACTIVE) {
		BHND_NVPROV_UNLOCK_RW(provider);
		return (ENODEV);
	}

	/* Look for an existing consumer entry, if any */
	consumer = NULL;
	LIST_FOREACH(c, &provider->consumers, nc_link) {
		if (c->plane == plane) {
			consumer = c;
			break;
		}
	}

	/* If not found, try allocating a new entry */
	if (consumer == NULL) {
		consumer = bhnd_nvram_consumer_new(plane);
		if (consumer == NULL) {
			BHND_NVPROV_UNLOCK_RW(provider);
			return (ENOMEM);
		}

		LIST_INSERT_HEAD(&provider->consumers, consumer, nc_link);
	}

	/* Check for overflow */
	if (SIZE_MAX - consumer->uses < use_count) {
		BHND_NVPROV_UNLOCK_RW(provider);
		return (ERANGE);
	}

	consumer->uses += use_count;
	BHND_NVPROV_UNLOCK_RW(provider);
	return (0);
}

/**
 * Remove a consumer reference from @p provider.
 * 
 * @param provider	The provider to be modified.
 * @param plane		The plane for which a consumer reference should
 *			be removed.
 * @param use_count	The use count to be decremented.
 * 
 * @retval 0		success.
 * @retval ENOMEM	if allocation fails.
 */
static void
bhnd_nvram_provider_remove_consumer(struct bhnd_nvram_provider *provider,
    struct bhnd_nvram_plane *plane, size_t use_count)
{
	struct bhnd_nvram_consumer *c, *cnext;

	BHND_NVPROV_LOCK_RW(provider);

	/* If the provider is already dead, all consumer records have
	 * already been discarded, and remove requests can be ignored */
	if (provider->state == BHND_NVRAM_PROV_DEAD) {
		BHND_NVPROV_UNLOCK_RW(provider);
		return;
	}

	/* Look for the existing consumer entry */
	LIST_FOREACH_SAFE(c, &provider->consumers, nc_link, cnext) {
		if (c->plane != plane)
			continue;

		/* Found; decrement use count */
		if (c->uses < use_count)
			BHND_NV_PANIC("%zu < %zu", c->uses, use_count);

		c->uses -= use_count;

		/* If the count hits zero, deallocate the consumer entry */
		if (c->uses == 0) {
			LIST_REMOVE(c, nc_link);
			BHND_NVREF_RELEASE(c, refs, bhnd_nvram_consumer_fini);

			BHND_NVPROV_UNLOCK_RW(provider);
			return;
		}

		return;
	}

	BHND_NVPROV_UNLOCK_RW(provider);
	BHND_NV_PANIC("invalid consumer reference"); 
}

/**
 * Attempt to retain a reservation on provider, preventing removal of the
 * provider until a matching call to bhnd_nvram_provider_unbusy() is made.
 * 
 * @param provider	The provider to be marked as busy.
 * 
 * @retval 0		success
 * @retval ENODEV	if @p provider is marked for removal.
 */
static int
bhnd_nvram_provider_busy(struct bhnd_nvram_provider *provider)
{
	int error;

	BHND_NVPROV_LOCK_RD(provider);
	error = bhnd_nvram_provider_busy_locked(provider);
	BHND_NVPROV_UNLOCK_RD(provider);

	return (error);
}

static int
bhnd_nvram_provider_busy_locked(struct bhnd_nvram_provider *provider)
{
	BHND_NVPROV_LOCK_ASSERT(provider, SA_LOCKED);

	if (provider->state != BHND_NVRAM_PROV_ACTIVE)
		return (ENODEV);

	refcount_acquire(&provider->busy);
	return (0);
}

/**
 * Release a reservation on @p provider acquired via
 * bhnd_nvram_provider_busy()
 * 
 * @param provider	The provider to be marked as busy.
 * 
 * @retval 0		success
 * @retval ENODEV	if @p provider is marked for removal.
 */
static void
bhnd_nvram_provider_unbusy(struct bhnd_nvram_provider *provider)
{
	BHND_NVPROV_LOCK_RD(provider);
	bhnd_nvram_provider_unbusy_locked(provider);
	BHND_NVPROV_UNLOCK_RD(provider);
}

static void
bhnd_nvram_provider_unbusy_locked(struct bhnd_nvram_provider *provider)
{
	BHND_NVPROV_LOCK_ASSERT(provider, SA_LOCKED);
	BHND_NVPROV_ASSERT_ACTIVE(provider);

	if (refcount_release(&provider->busy)) {
		/* Wake up any threads waiting for the busy count to hit
		 * zero */
		BHND_NVPROV_LOCK_WAKEUP(provider);
	}
}


/**
 * Allocate, initialize, and return an unconnected adjacency list link node.
 * 
 * The caller is responsible for deallocating the returned link via
 * bhnd_nvram_link_free()
 *
 * @param name		The path name relative to @p parent.
 * @param namelen	The length of @p name.
 * @param parent	The link's parent path, or NULL if none.
 * @param prov		The link's provider, or NULL.
 * @param prov_path	If @p prov is non-NULL, the provider's canonical path
 *			for this link.
 * 
 * @retval non-NULL	success
 * @retval NULL		if allocation fails
 */
static struct bhnd_nvram_link *
bhnd_nvram_link_new(const char *name, size_t namelen,
    struct bhnd_nvram_link *parent, struct bhnd_nvram_provider *prov,
    struct bhnd_nvpath_str *prov_path)
{
	struct bhnd_nvram_link *link;

	BHND_NV_ASSERT(prov == NULL || prov_path != NULL,
		("prov/prov_path must be NULL or non-NULL"));

	link = bhnd_nv_calloc(1, sizeof(*link));
	if (link == NULL)
		return (NULL);

	LIST_INIT(&link->children);

	/* Allocate full path */
	if (parent == NULL) {
		link->path = bhnd_nvpath_str_new(name, namelen);
	} else {
		link->path = bhnd_nvpath_str_append(parent->path->pathname,
		    parent->path->pathlen, name, namelen);
	}

	if (link->path == NULL) {
		bhnd_nv_free(link);
		return (NULL);
	}

	BHND_NV_ASSERT(
	    bhnd_nvram_is_normalized_path(link->path->pathname,
		link->path->pathlen) &&
	    bhnd_nvram_is_qualified_path(link->path->pathname,
		link->path->pathlen),
	    ("invalid path: %s", link->path->pathname));

	/* Connect to parent */
	if (parent != NULL) {
		link->parent = parent;
		LIST_INSERT_HEAD(&parent->children, link, child_link);
	}

	/* Retain provider reference */
	if (prov != NULL) {
		link->prov = BHND_NVREF_RETAIN(prov, refs);
		link->prov_path = BHND_NVREF_RETAIN(prov_path, refs);
	}

	return (link);
}

/**
 * Free a link instance and all of its children. The link (and all of its
 * children) must not have an assigned provider.
 *
 * @param link	The link to be freed.
 */
static void
bhnd_nvram_link_free(struct bhnd_nvram_link *link)
{
	struct bhnd_nvram_link *cwd;

	/* Disconnect from the parent, preventing upward traversal from freeing
	 * the parent within the tree walking loop */
	if (link->parent != NULL) {
		LIST_REMOVE(link, child_link);
		link->parent = NULL;
	}

	/* Walk the tree without tail recursion, performing deallocation
	 * starting at leaf nodes. */
	cwd = link;
	while (cwd != NULL) {
		struct bhnd_nvram_link *parent;

		/* Must not have an active provider */
		BHND_NV_ASSERT(cwd->prov == NULL, ("active provider"));

		/* Once we hit a leaf node, free the node and then try
		 * to walk upwards */
		if (LIST_EMPTY(&cwd->children)) {
			parent = cwd->parent;

			/* Disconnect from the parent */
			if (parent != NULL)
				LIST_REMOVE(cwd, child_link);

			/* Release the path string */
			BHND_NVREF_RELEASE(cwd->path, refs,
			    bhnd_nvpath_str_fini);			

			/* Free the link allocation */
			bhnd_nv_free(cwd);

			/* Traverse the parent, if any */
			cwd = parent;
			continue;
		} else {
			/* Otherwise, traverse the first child looking for
			 * a leaf node */
			cwd = LIST_FIRST(&cwd->children);
		}
	}	
}

/**
 * Clear all provider-specific state in @p link.
 * 
 * @param link	The link to be modified.
 */
static void
bhnd_nvram_link_clear_provider(struct bhnd_nvram_link *link)
{
	BHND_NVREF_RELEASE(link->prov, refs, bhnd_nvram_provider_fini);
	BHND_NVREF_RELEASE(link->prov_path, refs, bhnd_nvpath_str_fini);

	link->prov = NULL;
	link->prov_path = NULL;
}

/**
 * Create or fetch the plane-specific adjacency list link for @p pathname
 * in @p plane, returning a borrowed reference to the link.
 *
 * @param	plane		The NVRAM plane.
 * @param	cwd		The 'current working directory' from which
 *				relative paths will be resolved, or NULL if
 *				only fully qualified paths should be permitted.
 * @param	pathname	The name of the NVRAM path entry to be added.
 * @param	pathlen		The length of @p pathname.
 * @param[out]	link		On success, the newly inserted link.
 *
 * @retval 0		success
 * @retval EINVAL	if @p cwd is NULL and @p pathname is a relative path.
 * @retval EINVAL	if @p pathname is invalid.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvram_link_insert(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *cwd, const char *pathname, size_t pathlen,
    struct bhnd_nvram_link **link)
{
	const char	*basename, *name;
	size_t		 namelen, baselen;
	int		 error;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	/* The basename must be non-empty, and must be a normalized path
	 * component (i.e. not one of ".", "..") */
	basename = bhnd_nvram_parse_path_filename(pathname, pathlen, &baselen);
	if (baselen == 0 || !bhnd_nvram_is_normalized_path(basename, baselen))
		return (EINVAL);

	/* Determine the initial search path */
	name = NULL;
	name = bhnd_nvram_parse_path_next(pathname, pathlen, name, &namelen);

	if (namelen == 1 && *name == '/') {
		cwd = plane->root;
	} else {
		if (cwd == NULL)
			return (EINVAL);

		/* restart path walking at the first element */
		name = NULL;
	}

	/* Walk the input path, resolving or creating all path components */
	name = NULL;
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		struct bhnd_nvram_link *next;

		/* Resolve or create the next path component's link */
		next = bhnd_nvram_link_resolve(plane, cwd, name, namelen);
		if (next == NULL) {
			/* Not found; create a new link node */
			next = bhnd_nvram_link_new(name, namelen, cwd, NULL,
			    NULL);
			if (next == NULL) {
				error = ENOMEM;
				goto failed;
			}
		}

		/* Replace cwd with new link and continue resolution */
		cwd = next;
	}

	*link = cwd;
	return (0);

failed:
	/* Recursively remove any unused leaf nodes allocated above */
	bhnd_nvram_plane_try_remove(plane, cwd);

	return (error);
}

/**
 * Resolve @p pathname to its plane-specific adjacency list link, or NULL if
 * not found.
 * 
 * @param	plane		The NVRAM plane.
 * @param	cwd		The 'current working directory' from which
 *				relative paths will be resolved, or NULL if
 *				only fully qualified paths should be permitted.
 * @param	pathname	The NVRAM path name to be resolved.
 * @param	pathlen		The length of @p pathname.
 */
static struct bhnd_nvram_link *
bhnd_nvram_link_resolve(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *cwd, const char *pathname, size_t pathlen)
{
	const char	*name;
	size_t		 namelen;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	/* Determine the initial search path */
	name = NULL;
	name = bhnd_nvram_parse_path_next(pathname, pathlen, name, &namelen);

	if (namelen == 1 && *name == '/') {
		cwd = plane->root;
	} else {
		if (cwd == NULL)
			return (NULL);

		/* restart path walking at the first element */
		name = NULL;
	}

	/* Walk the path tree */
	while ((name = bhnd_nvram_parse_path_next(pathname, pathlen, name,
	    &namelen)) != NULL)
	{
		struct bhnd_nvram_link *l, *child;

		/* cwd reference is a no-op */
		if (namelen == 1 && *name == '.')
			continue;

		/* handle parent references ('..') */
		if (namelen == 2 && name[0] == '.' && name[1] == '.') {
			/* Parent references are cyclical if we're already
			 * at the root path */
			if (cwd->parent == NULL)
				continue;

			/* Replace current path with parent path and continue
			 * searching */
			cwd = cwd->parent;
			continue;
		}

		/* locate name in the current path */
		child = NULL;
		LIST_FOREACH(l, &cwd->children, child_link) {
			if (strncmp(l->path->basename, name, namelen) != 0)
				continue;

			if (l->path->basename[namelen] != '\0')
				continue;

			/* Subpath matches */
			child = l;
			break;
		}

		/* Not found? */
		if (child == NULL)
			return (NULL);

		/* Replace current path with child path and continue
		 * searching */
		cwd = child;
	}

	return (cwd);
}

/**
 * Allocate and initialize an empty NVRAM plane.
 * 
 * The caller is responsible for releasing the returned NVRAM plane
 * via bhnd_nvram_plane_release().
 * 
 * @param	parent	The parent NVRAM plane, or NULL.
 * 
 * @retval NULL if allocation fails.
 */
struct bhnd_nvram_plane *
bhnd_nvram_plane_new(struct bhnd_nvram_plane *parent)
{
	struct bhnd_nvram_plane	*plane;

	plane = bhnd_nv_calloc(1, sizeof(*plane));
	if (plane == NULL)
		return (NULL);

	BHND_NVPLANE_LOCK_INIT(plane);
	BHND_NVREF_INIT(&plane->refs);
	LIST_INIT(&plane->children);

	plane->parent = NULL;

	for (size_t i = 0; i < nitems(plane->prov_map); i++)
		LIST_INIT(&plane->prov_map[i]);

	/* Create our persistent root entry */
	plane->root = bhnd_nvram_link_new("/", strlen("/"), NULL, NULL, NULL);
	if (plane->root == NULL) {
		bhnd_nvram_plane_release(plane);
		return (NULL);
	}

	/* Link to our parent plane */
	if (parent != NULL) {
		plane->parent = bhnd_nvram_plane_retain(parent);

		/* Insert weak reference into the parent's child list */
		BHND_NVPLANE_LOCK_RW(parent);

		BHND_NVREF_RETAIN_WEAK(plane, refs);
		LIST_INSERT_HEAD(&parent->children, plane, child_link);

		BHND_NVPLANE_UNLOCK_RW(parent);
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
	struct bhnd_nvram_plane *parent;

	BHND_NV_ASSERT(LIST_EMPTY(&plane->children), ("active children"));

	/* Disconnect all provider references */
	for (size_t i = 0; i < nitems(plane->prov_map); i++) {
		struct bhnd_nvram_link *link, *lnext;

		LIST_FOREACH_SAFE(link, &plane->prov_map[i], hash_link, lnext) {
			struct bhnd_nvram_provider *prov;

			prov = link->prov;
			BHND_NV_ASSERT(prov != NULL, ("missing provider"));
			
			/* Clear the link's provider state */
			bhnd_nvram_link_clear_provider(link);

			/* Remove from the plane's provider->link map */
			LIST_REMOVE(link, hash_link);

			/* Release the provider's consumer reference */
			bhnd_nvram_provider_remove_consumer(prov, plane, 1);
		}
	}

	/* Free our link nodes */
	bhnd_nvram_link_free(plane->root);

	/*
	 * Unlink ourselves from the parent plane.
	 */
	if ((parent = plane->parent) != NULL) {
		/* Remove from the parent's list of children */
		BHND_NVPLANE_LOCK_RW(parent);
		LIST_REMOVE(plane, child_link);
		BHND_NVPLANE_UNLOCK_RW(parent);

		/* Drop our parent's weak reference to this instance; this may
		 * deallocate our instance, and no member references should be
		 * made after this point */
		BHND_NVREF_RELEASE_WEAK(plane, refs);

		/* Drop our strong reference to the parent */
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
 * Return true if @p link is a leaf node and can be safely deallocated, false
 * otherwise.
 * 
 * A leaf node:
 * - Has no children.
 * - Has a NULL provider entry.
 * - Is not the root node ("/")
 * 
 * @param	plane	The NVRAM plane.
 * @param	link	The link to be deallocated.
 */
static bool
bhnd_nvram_plane_is_leaf(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	/* Must not be the root link */
	if (plane->root == link)
		return (false);

	BHND_NV_ASSERT(strcmp(link->path->pathname, "/") != 0,
	    ("non-root link with / path"));

	/* Must not have an active provider reference */
	if (link->prov != NULL)
		return (false);

	/* Must not have active children */
	if (!LIST_EMPTY(&link->children))
		return (false);

	return (true);
}

/**
 * If @p link is an unused leaf node (@see bhnd_nvram_link_is_leaf()),
 * free @p link and all associated resources.
 * 
 * If deallocating @p link results in its parent becoming a leaf node, the
 * parent will also be deallocated. This will be performed recursively, up
 * to (but not including) the persistent root ("/") link entry.
 * 
 * @param plane	The NVRAM plane.
 * @param link	The link to be removed.
 */
static void
bhnd_nvram_plane_try_remove(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	struct bhnd_nvram_link *parent;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	while (link != NULL && bhnd_nvram_plane_is_leaf(plane, link)) {
		/* Remove the link */
		parent = link->parent;
		bhnd_nvram_link_free(link);

		/* Try removing the parent */
		link = parent;
	}
}

/**
 * Remove all references to a terminated @p provider.
 * 
 * @param plane		The NVRAM plane.
 * @param provider	The provider to be removed.
 */
static void
bhnd_nvram_plane_remove_provider(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_provider *provider)
{
	struct bhnd_nvram_link	*link, *lnext;
	size_t			 bucket, num_refs;

	BHND_NVPLANE_LOCK_RW(plane);

	num_refs = 0;
	bucket = (uintptr_t)provider % nitems(plane->prov_map);
	LIST_FOREACH_SAFE(link, &plane->prov_map[bucket], hash_link, lnext) {
		/* Skip non-matching provider entries */
		if (link->prov != provider)
			continue;

		/* Clear link's provider state */
		bhnd_nvram_link_clear_provider(link);

		/* Remove from the plane's provider->link map */
		LIST_REMOVE(link, hash_link);

		/* Clean up any leaf link nodes */
		bhnd_nvram_plane_try_remove(plane, link);

		/* Increment total number of consumer references. */
		BHND_NV_ASSERT(num_refs < SIZE_MAX,
		    ("unrepresentable refcount"));
		num_refs++;
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* With the plane now unlocked, release our consumer reference(s) to
	 * the provider */
	bhnd_nvram_provider_remove_consumer(provider, plane, num_refs);
}

/**
 * Register NVRAM paths exported by @p provider.
 * 
 * @param	plane		The NVRAM plane in which @p pathnames will be
 *				registered.
 * @param	provider	The NVRAM provider for @p pathnames.
 * @param	pathnames	Normalized, fully qualified plane-specific path
 *				names to be mapped to @p provider.
 * @param	num_pathnames	The number of @p pathnames.
 * 
 * @retval 0		success.
 * @retval EINVAL	if duplicate paths are found in @p pathnames.
 * @retval EINVAL	if a path in @p pathnames is not a fully-qualified
 *			normalized path.
 * @retval EEXIST	if a path in @p pathnames is already registered in
 *			@p plane.
 * @retval ENODEV	if @p provider is marked for removal.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_register_paths(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_provider *provider, char *pathnames[],
    size_t num_pathnames)
{
	size_t	prov_bucket;
	int	error;

	/* Skip if no paths are provided */
	if (num_pathnames == 0)
		return (0);

	/* Validate all paths */
	for (size_t i = 0; i < num_pathnames; i++) {
		const char	*pathname;
		size_t		 pathlen;
		
		pathname = pathnames[i];
		pathlen = strlen(pathname);

		if (!bhnd_nvram_is_qualified_path(pathname, pathlen))
			return (EINVAL);

		if (!bhnd_nvram_is_normalized_path(pathname, pathlen))
			return (EINVAL);

		/* No duplicates */
		for (size_t j = 0; j < i; j++) {
			if (strcmp(pathname, pathnames[j]) == 0) {
				BHND_NV_LOG("duplicate path: %s\n", pathname);
				return (EINVAL);
			}
		}
	}

	/* Try to mark the provider as busy; this ensures that the provider
	 * will not perform termination while we modify the plane */
	if ((error = bhnd_nvram_provider_busy(provider)))
		return (error);

	/* Try to register consumer references for all paths */
	error = bhnd_nvram_provider_add_consumer(provider, plane,
	    num_pathnames);
	if (error) {
		bhnd_nvram_provider_unbusy(provider);
		return (error);
	}

	/*
	 * Register link nodes with the plane.
	 */
	BHND_NVPLANE_LOCK_RW(plane);

	prov_bucket = (uintptr_t)provider % nitems(plane->prov_map);

	/* Verify that none of the requested links have an existing provider
	 * attached */
	for (size_t i = 0; i < num_pathnames; i++) {
		struct bhnd_nvram_link	*link;
		const char		*pathname;
		size_t			 pathlen;

		pathname = pathnames[i];
		pathlen = strlen(pathname);

		/* Fetch or create a new link node */
		error = bhnd_nvram_link_insert(plane, NULL, pathname, pathlen,
		    &link);
		if (error == 0 && link->prov != NULL) {
			/* An existing link was found, but it already has a
			 * provider assigned */
			error = EEXIST;
		}

		/* On failure, we need to clean up all previously inserted
		 * links and return the error */
		if (error) {
			for (size_t j = 0; j < i; j++) {
				/* Clear link's provider state */
				bhnd_nvram_link_clear_provider(link);

				/* Remove from the plane's provider->link map */
				LIST_REMOVE(link, hash_link);

				/* Try to remove the link */
				bhnd_nvram_plane_try_remove(plane, link);
			}

			/* Drop our plane lock before modifying the provider */
			BHND_NVPLANE_UNLOCK_RW(plane);

			/* Release the consumer references we added above */
			bhnd_nvram_provider_remove_consumer(provider, plane,
			    num_pathnames);

			/* Release our reservation on the provider */
			bhnd_nvram_provider_unbusy(provider);

			return (error);
		}

		/* Assign new provider and mark as pending */
		link->prov = BHND_NVREF_RETAIN(provider, refs);
		link->prov_path = BHND_NVREF_RETAIN(link->path, refs);

		/* Insert in provider->link map */
		LIST_INSERT_HEAD(&plane->prov_map[prov_bucket], link,
		    hash_link);
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Release our reservation on the provider, allowing any pending
	 * termination operations to proceed (which could include removing
	 * the links we just added!) */
	bhnd_nvram_provider_unbusy(provider);

	return (0);
}

/**
 * Deregister NVRAM paths previously registered via
 * bhnd_nvram_register_paths().
 * 
 * If a path is currently mapped by a provider other than @p provider, the
 * request to deregister the path will be ignored.
 * 
 * @param	plane		The NVRAM plane in which @p pathnames will be
 *				registered.
 * @param	provider	The NVRAM provider for @p pathnames.
 * @param	pathnames	Normalized, fully qualified plane-specific
 *				path names to be unmapped from @p provider.
 * @param	num_pathnames	The number of @p pathnames.
 */
void
bhnd_nvram_deregister_paths(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_provider *provider, char *pathnames[],
    size_t num_pathnames)
{
	struct bhnd_nvram_link_list	*prov_map;
	struct bhnd_nvram_link		*link, *lnext;
	size_t				 bucket, use_count;

	BHND_NVPLANE_LOCK_RW(plane);

	use_count = 0;
	bucket = (uintptr_t)provider % nitems(plane->prov_map);
	prov_map = &plane->prov_map[bucket];

	LIST_FOREACH_SAFE(link, prov_map, hash_link, lnext) {
		/* Provider must match */
		if (link->prov != provider)
			continue;

		/* Path must match */
		for (size_t i = 0; i < num_pathnames; i++) {
			const char *pathname = pathnames[i];

			if (strcmp(pathname, link->path->pathname) != 0)
				continue;

			/* Clear link's provider state */
			bhnd_nvram_link_clear_provider(link);

			/* Remove from the plane's provider->link map */
			LIST_REMOVE(link, hash_link);

			/* Clean up any leaf link nodes */
			bhnd_nvram_plane_try_remove(plane, link);

			/*
			 * Increment total number of consumer references to
			 * be released.
			 * 
			 * bhnd_nvram_provider_add_consumer() guarantees that
			 * the number of active consumer references will never
			 * exceed than SIZE_MAX.
			 */
			BHND_NV_ASSERT(use_count < SIZE_MAX,
			    ("unrepresentable refcount"));
			use_count++;
		}
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* With the plane now unlocked, release our consumer reference(s) to
	 * the provider */
	bhnd_nvram_provider_remove_consumer(provider, plane, use_count);
}

#if 0

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

	BHND_NVPLANE_LOCK_RD();
	phandle = bhnd_nvram_phandle_open(plane->root, NULL, pathname,
	    strlen(pathname));
	BHND_NVPLANE_UNLOCK_RD();

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

#endif
