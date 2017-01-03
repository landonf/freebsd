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

static struct bhnd_nvpath	*bhnd_nvpath_new(const char *pathname,
				     size_t pathlen);
static struct bhnd_nvpath	*bhnd_nvpath_append(const char *pathname,
				     size_t pathlen, const char *name,
				     size_t namelen);

static void			 bhnd_nvpath_fini(struct bhnd_nvpath *path);

static struct bhnd_nvram_entry	*bhnd_nvram_entry_insert(
				     struct bhnd_nvram_provider *provider,
				     const char *pathname);
static struct bhnd_nvram_entry	*bhnd_nvram_entry_find(
				     struct bhnd_nvram_provider *provider,
				     const char *pathname);

static void			 bhnd_nvram_entry_fini(
				     struct bhnd_nvram_entry *entry);

static bool			 bhnd_nvram_entry_is_leaf(
				     struct bhnd_nvram_provider *provider,
				     struct bhnd_nvram_entry *entry);
static void			 bhnd_nvram_entry_try_remove(
				     struct bhnd_nvram_provider *provider,
				     struct bhnd_nvram_entry *entry);

static bhnd_nvram_consumer	*bhnd_nvram_consumer_new(
				     struct bhnd_nvram_plane *plane);
static void			 bhnd_nvram_consumer_fini(
				     struct bhnd_nvram_consumer *consumer);
static void			 bhnd_nvram_consumer_set_entry(
				     struct bhnd_nvram_consumer *consumer,
				     struct bhnd_nvram_entry *entry);

static bhnd_nvram_consumer	**bhnd_nvram_consumers_alloc(
				     struct bhnd_nvram_plane *plane,
				     size_t num_consumers);
static void			 bhnd_nvram_consumers_free(
				     struct bhnd_nvram_consumer **consumers,
				     size_t num_consumers);

static void			 bhnd_nvram_provider_fini(
				     struct bhnd_nvram_provider *provider);

static struct bhnd_nvram_link	*bhnd_nvram_link_new(const char *name,
				     size_t namelen, const char *parent,
				     size_t parentlen);

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

static bool			 bhnd_nvram_link_is_leaf(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link);
static void			 bhnd_nvram_link_remove(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link);
static void			 bhnd_nvram_link_try_remove(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link);

static struct bhnd_nvram_link	*bhnd_nvram_link_find_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *parent,
				     const char *name);

static bool			 bhnd_nvram_link_has_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *parent,
				     const char *name);

static void			 bhnd_nvram_plane_fini(
				     struct bhnd_nvram_plane *plane);

static bool			 bhnd_nvram_plane_is_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);
static void			 bhnd_nvram_plane_remove_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);


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
static struct bhnd_nvpath *
bhnd_nvpath_new(const char *pathname, size_t pathlen)
{
	return (bhnd_nvpath_append(pathname, pathlen, NULL, 0));
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
static struct bhnd_nvpath *
bhnd_nvpath_append(const char *pathname, size_t pathlen, const char *name,
    size_t namelen)
{
	struct bhnd_nvpath	*path;
	char			*pathbuf;
	size_t			 bufsize, baselen;
	bool			 need_delim;

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
	path->pathlen = pathlen;

	/* Populate path's base name */
	path->basename = bhnd_nvram_parse_path_filename(path->pathname,
	    path->pathlen, &path->baselen);

	return (path);
}

static void
bhnd_nvpath_fini(struct bhnd_nvpath *path)
{
	bhnd_nv_free(path->pathname);
}

/**
 * Allocate and initialize a new NVRAM path entry, registering it with
 * @p provider.
 * 
 * If @p pathname is already registered with @p provider, the existing entry
 * will be returned.
 * 
 * Returns a borrowed reference to the entry.
 * 
 * @param provider	The entry's provider.
 * @param pathname	The entry's fully qualified path in normalized
 *			(canonical) form.
 * 
 * @retval non-NULL	if an entry for @p pathname was found in @p provider.
 * @retval non-NULL	if an entry for @p pathname was allocated and registered
 *			in @p provider.
 * @retval NULL		if allocation fails.
 */
static struct bhnd_nvram_entry *
bhnd_nvram_entry_insert(struct bhnd_nvram_provider *provider,
    const char *pathname)
{
	struct bhnd_nvram_entry	*entry;
	size_t			 pathlen;

	BHND_NVPROV_LOCK_ASSERT(provider, SA_XLOCKED);
	BHND_NVPROV_ASSERT_ACTIVE(provider);

	/* Look for existing entry */
	entry = bhnd_nvram_entry_find(provider, pathname);
	if (entry != NULL)
		return (entry);

	/* Path must be fully qualified and in normal form */
	pathlen = strlen(pathname);
	if (!bhnd_nvram_is_normalized_path(pathname, strlen(pathname)) ||
	    !bhnd_nvram_is_qualified_path(pathname, pathlen))
	{
		BHND_NV_PANIC("invalid path: '%s'", pathname);
	}

	entry = bhnd_nv_calloc(1, sizeof(*entry));
	if (entry == NULL)
		return (NULL);

	LIST_INIT(&entry->consumers);
	BHND_NVREF_INIT(&entry->refs);

	entry->canon = bhnd_nvpath_new(pathname, strlen(pathname));
	if (entry->canon == NULL) {
		bhnd_nv_free(entry);
		return (NULL);
	}

	/* Add to provider's entry list, transfering our reference ownership */
	LIST_INSERT_HEAD(&provider->entries, entry, ne_link);

	return (entry);
}

static void
bhnd_nvram_entry_fini(struct bhnd_nvram_entry *entry)
{
	BHND_NV_ASSERT(LIST_EMPTY(&entry->consumers), ("active consumers"));

	BHND_NVREF_RELEASE_WEAK(entry->prov, refs);
	BHND_NVREF_RELEASE(entry->canon, refs, bhnd_nvpath_fini);
}

/**
 * Return true if @p entry has no consumers and can be safely deregistered
 * from @p provider.
 * 
 * @param provider	The NVRAM provider owning @p entry.
 * @param entry		The entry to be deregistered.
 */
static bool
bhnd_nvram_entry_is_leaf(struct bhnd_nvram_provider *provider,
    struct bhnd_nvram_entry *entry)
{
	struct bhnd_nvram_consumer *consumer;

	BHND_NVPROV_LOCK_ASSERT(provider, SA_LOCKED);
	BHND_NVPROV_ASSERT_ACTIVE(provider);

	/* Scan consumer list for non-zombie consumer references */
	LIST_FOREACH(consumer, &entry->consumers, nc_link) {
		/* Ignore zombies */
		if (BHND_NVREF_IS_ZOMBIE(consumer, refs))
			continue;

		/* Active consumer; entry is not a leaf */
		return (false);
	}

	/* No active consumers */
	return (true);
}


/**
 * If @p entry is an unused leaf entry (@see bhnd_nvram_entry_is_leaf()),
 * remove it from @p provider's list of entries.
 * 
 * @param provider	The NVRAM provider.
 * @param entry		The NVRAM entry.
 */
static void
bhnd_nvram_entry_try_remove(struct bhnd_nvram_provider *provider,
    struct bhnd_nvram_entry *entry)
{
	struct bhnd_nvram_consumer *consumer, *cnext;

	BHND_NVPROV_LOCK_ASSERT(provider, SA_XLOCKED);
	BHND_NVPROV_ASSERT_ACTIVE(provider);

	/* Perform cleanup of any zombie consumer records */
	LIST_FOREACH_SAFE(consumer, &entry->consumers, nc_link, cnext) {
		if (!BHND_NVREF_IS_ZOMBIE(consumer, refs))
			continue;

		/* Remove and drop our weak reference to the consumer record */
		LIST_REMOVE(consumer, nc_link);
		BHND_NVREF_RELEASE_WEAK(consumer, refs);
	}

	/* Must be a leaf entry */
	if (!bhnd_nvram_entry_is_leaf(provider, entry))
		return;

	/* The entry must not have already been removed from the provider's
	 * entry list. This may occur if a concurrent call to
	 * bhnd_nvram_entry_try_remove() was previously made, resulting in
	 * the entry being purged from the provider */
	if (bhnd_nvram_entry_find(provider, entry->canon->pathname) != entry)
		return;

	/* Remove */
	LIST_REMOVE(entry, ne_link);
	BHND_NVREF_RELEASE(entry, refs, bhnd_nvram_entry_fini);
}


/**
 * Return the first entry in @p provider matching @p pathname, or NULL
 * if not found.
 * 
 * @param	provider	The NVRAM provider to search.
 * @param	pathname 	The canonical path name to search for.
 */
static struct bhnd_nvram_entry *
bhnd_nvram_entry_find(struct bhnd_nvram_provider *provider,
    const char *pathname)
{
	struct bhnd_nvram_entry *entry;

	BHND_NVPROV_LOCK_ASSERT(provider, SA_LOCKED);
	BHND_NVPROV_ASSERT_ACTIVE(provider);

	LIST_FOREACH(entry, &provider->entries, ne_link) {
		if (strcmp(entry->canon->pathname, pathname) == 0)
			return (entry);
	}

	return (NULL);
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
	consumer->entry = NULL;

	return (consumer);
}


static void
bhnd_nvram_consumer_fini(struct bhnd_nvram_consumer *consumer)
{
	if (consumer->entry != NULL) {
		BHND_NVREF_RELEASE(consumer->entry, refs,
		    bhnd_nvram_entry_fini);
	}

	BHND_NVREF_RELEASE_WEAK(consumer->plane, refs);
}

/**
 * Set the entry associated with @p consumer. May only be set once, prior
 * to the consumer record being linked into the plane.
 * 
 * @param consumer	The empty consumer record to be modified.
 * @param entry		The entry to be set.
 */
static void
bhnd_nvram_consumer_set_entry(struct bhnd_nvram_consumer *consumer,
    struct bhnd_nvram_entry *entry)
{
	if (consumer->entry != NULL)
		BHND_NV_PANIC("cannot overwrite consumer entry");

	consumer->entry = BHND_NVREF_RETAIN(entry, refs);
}

/**
 * Allocate, initialize, and return an array of NVRAM consumer records.
 * 
 * The caller is responsible for freeing the returned array via
 * bhnd_nvram_consumers_free().
 * 
 * @param plane		The NVRAM plane associated with the consumer records.
 * @param num_consumers	The number of consumer records to allocate.
 * 
 * @retval non-NULL	success.
 * @retval NULL		if allocation fails.
 */
static struct bhnd_nvram_consumer **
bhnd_nvram_consumers_alloc(struct bhnd_nvram_plane *plane, size_t num_consumers)
{
	struct bhnd_nvram_consumer **consumers;

	consumers = bhnd_nv_calloc(num_consumers, sizeof(*consumers));
	if (consumers == NULL)
		return (NULL);

	for (size_t i = 0; i < num_consumers; i++) {
		consumers[i] = bhnd_nvram_consumer_new(plane);
		if (consumers[i] == NULL) {
			bhnd_nvram_consumers_free(consumers, i);
			return (NULL);
		}
	}

	return (consumers);
}

/**
 * Free an array of @p consumers previously returned by
 * bhnd_nvram_consumers_alloc().
 * 
 * @param consumers	An array of NVRAM consumer records previously allocated
 *			via bhnd_nvram_consumers_alloc().
 * @param num_consumers	The number of consumer records in @p consumers.
 */
static void
bhnd_nvram_consumers_free(struct bhnd_nvram_consumer **consumers,
    size_t num_consumers)
{
	/* Release all individual records */
	for (size_t i = 0; i < num_consumers; i++) {
		BHND_NVREF_RELEASE(consumers[i], refs,
		    bhnd_nvram_consumer_fini);
	}

	/* Free the array */
	bhnd_nv_free(consumers);
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

	LIST_INIT(&provider->entries);

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
	struct bhnd_nvram_entry *e, *enext;

	BHND_NVPROV_LOCK_RW(provider);

	/* Set stopping state; any threads acquiring a read lock after
	 * this point will immediately return ENODEV instead of querying
	 * the provider */
	BHND_NVPROV_ASSERT_ACTIVE(provider);
	provider->state = BHND_NVRAM_PROV_STOPPING;

	/* Wait for any outstanding requests to complete */
	while (provider->in_use > 0)
		BHND_NVPROV_LOCK_WAIT(provider);

	/* Set dead state */
	provider->state = BHND_NVRAM_PROV_DEAD;

	/* After this point, any threads accessing the provider will terminate
	 * upon reading the 'dead' state, and we can safely modify other
	 * structure members without a lock held. */
	BHND_NVPROV_UNLOCK_RW(provider);

	LIST_FOREACH_SAFE(e, &provider->entries, ne_link, enext) {
		struct bhnd_nvram_consumer *c, *cnext;

		LIST_FOREACH_SAFE(c, &e->consumers, nc_link, cnext) {
			struct bhnd_nvram_consumer	*consumer;
			struct bhnd_nvram_plane		*plane;
	
			/* Attempt to promote our weak consumer reference,
			 * and then drop the list entry's weak reference */
			consumer = BHND_NVREF_PROMOTE_WEAK(c, refs);

			BHND_NVREF_RELEASE_WEAK(c, refs);
			LIST_REMOVE(c, nc_link);

			if (consumer == NULL) {
				/* Consumer already dead */
				continue;
			}

			/* Attempt to promote our consumer's weak plane
			 * reference, and then drop our strong reference to
			 * the consumer */
			plane = BHND_NVREF_PROMOTE_WEAK(consumer->plane, refs);
			BHND_NVREF_RELEASE(consumer, refs,
			    bhnd_nvram_consumer_fini);

			if (plane == NULL) {
				/* Plane already dead; nothing left to do */
				continue;
			}

			// TODO: ask the plane to deregister the entry

			BHND_NVREF_RELEASE(plane, refs, bhnd_nvram_plane_fini);
		}
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

	BHND_NV_ASSERT(LIST_EMPTY(&provider->entries), ("active entries"));
	BHND_NV_ASSERT(provider->in_use == 0, ("provider in use"));

	BHND_NVPROV_LOCK_DESTROY(provider);
}



/**
 * Allocate, initialize, and return an unconnected adjacency list link node.
 * 
 * The caller is responsible for releasing the returned link via
 * bhnd_nvram_link_remove().
 *
 * @param name		The relative path name to append to @p pathname.
 * @param namelen	The length of @p name.
 * @param pathname	The parent path of @p name, or NULL.
 * @param pathlen	The length of @p pathname.
 * 
 * @retval non-NULL	success
 * @retval NULL		if allocation fails
 */
static struct bhnd_nvram_link *
bhnd_nvram_link_new(const char *name, size_t namelen, const char *pathname,
    size_t pathlen)
{
	struct bhnd_nvram_link *link;

	link = bhnd_nv_calloc(1, sizeof(*link));
	if (link == NULL)
		return (NULL);

	link->consumer = NULL;
	link->parent = NULL;
	LIST_INIT(&link->children);

	link->path = bhnd_nvpath_append(pathname, pathlen, name, namelen);
	if (link->path == NULL) {
		bhnd_nv_free(link);
		return (NULL);
	}

	return (link);
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
			next = bhnd_nvram_link_new(name, namelen,
			    cwd->path->pathname, cwd->path->pathlen);
			if (next == NULL) {
				error = ENOMEM;
				goto failed;
			}

			/* Link with parent */
			BHND_NV_ASSERT(!bhnd_nvram_link_has_child(plane, cwd,
			    next->path->basename),
			    ("duplicate child %s\n", next->path->pathname));

			next->parent = cwd;
			LIST_INSERT_HEAD(&cwd->children, next, nl_link);
		}

		/* Replace cwd with new link and continue resolution */
		cwd = next;
	}

	*link = cwd;
	return (0);

failed:
	/* Recursively remove any unused leaf nodes allocated above */
	bhnd_nvram_link_try_remove(plane, cwd);

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
		LIST_FOREACH(l, &cwd->children, nl_link) {
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
bhnd_nvram_link_is_leaf(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	/* Must not be the root link */
	if (plane->root == link)
		return (false);

	BHND_NV_ASSERT(strcmp(link->path->pathname, "/") != 0,
	    ("non-root link with / path"));

	/* Must not have an active consumer record */
	if (link->consumer != NULL)
		return (false);

	/* Must not have active children */
	if (!LIST_EMPTY(&link->children))
		return (false);

	return (true);
}

/**
 * Remove @p link from @p plane and free all associated resources. The
 * link must be an unused leaf node (@see bhnd_nvram_link_is_leaf()),
 * 
 * If removing @p link results in its parent becoming a leaf node, the
 * parent will also be removed. This will be performed recursively, up
 * to (but not including) the persistent root ("/") link entry.
 * 
 * @param	plane	The NVRAM plane.
 * @param	link	The link to be deallocated.
 */
static void
bhnd_nvram_link_remove(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	while (1) {
		struct bhnd_nvram_link *parent;

		BHND_NV_ASSERT(bhnd_nvram_link_is_leaf(plane, link),
		    ("cannot free active link"));

		/* Remove from parent */
		if ((parent = link->parent) != NULL) {
			struct bhnd_nvram_link *l;

			l = bhnd_nvram_link_find_child(plane, link->parent,
			    link->path->basename);
			if (l != link)
				BHND_NV_PANIC("non-symmetric binary relation");

			LIST_REMOVE(link, nl_link);
		}

		/* Release path reference */
		BHND_NVREF_RELEASE(link->path, refs, bhnd_nvpath_fini);

		/* No parent to recursively free */
		if (parent == NULL)
			break;

		/* Do not recursively free the plane's root node */
		if (plane->root == parent)
			break;

		/* Do not free non-leaf/non-empty parents */
		if (!bhnd_nvram_link_is_leaf(plane, parent))
			break;

		link = parent;
	}
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
bhnd_nvram_link_try_remove(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	if (!bhnd_nvram_link_is_leaf(plane, link))
		return;

	bhnd_nvram_link_remove(plane, link);
}

/**
 * Return the child with @p name in @p parent, or NULL if not found.
 * 
 * @param	plane	The NVRAM plane.
 * @param	parent	The NVRAM link to query.
 * @param	name	The relative name to search for.
 */
static struct bhnd_nvram_link *
bhnd_nvram_link_find_child(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *parent, const char *name)
{
	struct bhnd_nvram_link *l;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_LOCKED);

	LIST_FOREACH(l, &parent->children, nl_link) {
		if (strcmp(l->path->basename, name) == 0)
			return (l);
	}

	/* Not found */
	return (NULL);
}

/**
 * Return true if a child with @p name is found in @p parent, false otherwise.
 * 
 * @param	plane	The NVRAM plane.
 * @param	parent	The NVRAM link to query.
 * @param	name	The relative name to search for.
 */
static bool
bhnd_nvram_link_has_child(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *parent, const char *name)
{
	return (bhnd_nvram_link_find_child(plane, parent, name) != NULL);
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

	LIST_INIT(&plane->children);
	BHND_NVPLANE_LOCK_INIT(plane);
	BHND_NVREF_INIT(&plane->refs);
	plane->parent = NULL;

	/* Create our persistent root entry */
	plane->root = bhnd_nvram_link_new("/", strlen("/"), NULL, 0);
	if (plane->root == NULL) {
		bhnd_nvram_plane_release(plane);
		return (NULL);
	}

	/* Link to our parent plane */
	if (parent != NULL) {
		plane->parent = bhnd_nvram_plane_retain(parent);

		/* Insert reference into the parent's child list */
		BHND_NVPLANE_LOCK_RW(parent);

		LIST_INSERT_HEAD(&parent->children,
		    BHND_NVREF_RETAIN_WEAK(plane, refs), child_link);

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
	BHND_NV_ASSERT(LIST_EMPTY(&plane->root->children), ("active links"));

	/*
	 * Remove ourselves from the parent plane.
	 * 
	 * This may deallocate our instance pointer. No member references to
	 * this plane should be made after this point.
	 */
	if ((parent = plane->parent) != NULL) {
		BHND_NVPLANE_LOCK_RW(parent);
		bhnd_nvram_plane_remove_child(parent, plane);
		BHND_NVPLANE_UNLOCK_RW(parent);

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

	LIST_FOREACH(p, &plane->children, child_link) {
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
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	if (!bhnd_nvram_plane_is_child(plane, child))
		BHND_NV_PANIC("plane is not a direct child of parent");

	LIST_REMOVE(child, child_link);
	BHND_NVREF_RELEASE_WEAK(child, refs);
}

/**
 * Register NVRAM paths exported by @p provider.
 * 
 * @param	plane		The NVRAM plane in which @p pathnames will be
 *				registered.
 * @param	provider	The NVRAM provider for @p pathnames.
 * @param	pathnames	Normalized, fully qualified path names to be
 *				mapped to @p provider.
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
    struct bhnd_nvram_provider *provider, char **pathnames,
    size_t num_pathnames)
{
	struct bhnd_nvram_entry		**entries;
	struct bhnd_nvram_consumer	**consumers;
	int				 error;

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

	/*
	 * Reserve all required provider entries for the paths.
	 */
	BHND_NVPROV_LOCK_RW(provider);

	// TODO: mark provider as busy to prevent destruction

	/* Provider must be running */
	if (provider->state != BHND_NVRAM_PROV_ACTIVE) {
		BHND_NVPROV_UNLOCK_RW(provider);
		return (ENODEV);
	}

	/* Register provider entries and consumer records for all paths */
	entries = bhnd_nv_calloc(num_pathnames, sizeof(entries[0]));
	if (entries == NULL)
		return (ENOMEM);

	consumers = bhnd_nvram_consumers_alloc(plane, num_pathnames);
	if (consumers == NULL) {
		bhnd_nv_free(entries);
		return (ENOMEM);
	}

	for (size_t i = 0; i < num_pathnames; i++) {
		struct bhnd_nvram_entry	*entry;
		const char		*pathname;
		
		pathname = pathnames[i];

		/* Fetch or create the provider entry */
		entry = bhnd_nvram_entry_insert(provider, pathname);
		if (entry == NULL) {
			error = ENOMEM;
			goto failed;
		}

		/* Retain our own strong reference to the entry */
		entries[i] = BHND_NVREF_RETAIN(entry, refs);

		/* Attach the consumer record to the entry */
		bhnd_nvram_consumer_set_entry(consumers[i], entry);

		BHND_NVREF_RETAIN_WEAK(consumers[i], refs);
		LIST_INSERT_HEAD(&entry->consumers, consumers[i], nc_link);
	}

	BHND_NVPROV_UNLOCK_RW(provider);

	/*
	 * Register plane-specific link nodes for all paths.
	 */
	BHND_NVPLANE_LOCK_RW(plane);

	/* Add the link nodes, verifying that none of the paths have an existing
	 * consumer record attached */
	for (size_t i = 0; i < num_pathnames; i++) {
		const char		*pathname;
		struct bhnd_nvram_link	*link;

		pathname = pathnames[i];

		/* Fetch or create a new link node */
		error = bhnd_nvram_link_insert(plane, NULL, pathname,
		    strlen(pathname), &link);
		if (error) {
			BHND_NVPLANE_UNLOCK_RW(plane);
			goto failed;
		}

		/* The link must not have a consumer record already assigned */
		if (link->consumer != NULL) {
			BHND_NVPLANE_UNLOCK_RW(plane);
			error = EEXIST;
			goto failed;
		}
	}

	/* Now that we know the modification will succeed, attach the consumer
	 * record to all link nodes */
	for (size_t i = 0; i < num_pathnames; i++) {
		const char		*pathname;
		struct bhnd_nvram_link	*link;

		pathname = pathnames[i];

		/* Fetch the link node validated/created above */
		link = bhnd_nvram_link_resolve(plane, NULL, pathname,
		    strlen(pathname));
		BHND_NV_ASSERT(link != NULL, ("'%s' link missing", pathname));

		/* Attach the consumer record */
		link->consumer = BHND_NVREF_RETAIN(consumers[i], refs);
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Clean up our local consumer record references. After this, there
	 * will exist only one strong reference to the consumer record. */
	bhnd_nvram_consumers_free(consumers, num_pathnames);

	/* Release all locally held strong references to our entries */
	for (size_t i = 0; i < num_pathnames; i++)
		BHND_NVREF_RELEASE(entries[i], refs, bhnd_nvram_entry_fini);

	bhnd_nv_free(entries);

	// TODO: unbusy provider to allow destruction

	return (0);

failed:
	/* Clean up our local consumer record references */
	bhnd_nvram_consumers_free(consumers, num_pathnames);

	/* Clean up our previously added provider entries */
	BHND_NVPROV_LOCK_RW(provider);
	for (size_t i = 0; i < num_pathnames && entries[i] != NULL; i++) {
		struct bhnd_nvram_entry	*entry;
		const char		*pathname;

		pathname = pathnames[i];
		entry = entries[i];

		/* Try to remove the entry */
		bhnd_nvram_entry_try_remove(provider, entry);

		/* Release our strong entry reference */
		BHND_NVREF_RELEASE(entry, refs, bhnd_nvram_entry_fini);
	}
	BHND_NVPROV_UNLOCK_RW(provider);

	bhnd_nv_free(entries);

	/* Clean up any previously added plane-specific adjacency list links */
	BHND_NVPLANE_LOCK_RW(plane);
	for (size_t i = 0; i < num_pathnames; i++) {
		const char		*pathname;
		struct bhnd_nvram_link	*link;

		pathname = pathnames[i];

		link = bhnd_nvram_link_resolve(plane, NULL, pathname,
		    strlen(pathname));

		/* The link may have already been removed */
		if (link == NULL)
			continue;

		/* The link may now be unused; try to remove it */
		bhnd_nvram_link_try_remove(plane, link);
	}
	BHND_NVPLANE_UNLOCK_RW(plane);

	return (error);
}

#if 0

static struct sx bhnd_nvplane_lock;
SX_SYSINIT(bhnd_nvram_topology, &bhnd_nvplane_lock, "BHND NVRAM topology lock");

#define	BHND_NVPLANE_LOCK_RD()			sx_slock(&bhnd_nvplane_lock)
#define	BHND_NVPLANE_UNLOCK_RD()		sx_sunlock(&bhnd_nvplane_lock)
#define	BHND_NVPLANE_LOCK_RW()			sx_xlock(&bhnd_nvplane_lock)
#define	BHND_NVPLANE_UNLOCK_RW()		sx_xunlock(&bhnd_nvplane_lock)
#define	BHND_NVPLANE_LOCK_ASSERT(what)		\
	sx_assert(&bhnd_nvplane_lock, what)

static void			 bhnd_nvram_plane_fini(
				     struct bhnd_nvram_plane *plane);

static bool			 bhnd_nvram_plane_is_child(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_plane *child);t
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
 * @retval NULL if allocation fails.
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

	/* Register with parent, if any */
	if (parent == NULL) {
		plane->parent = NULL;
	} else {
		/* Retain strong parent reference */
		plane->parent = bhnd_nvram_plane_retain(plane->parent);
	
		/* Add weak reference to our parent's child list */
		BHND_NVPLANE_LOCK_RW();

		BHND_NVREF_RETAIN_WEAK(plane, refs);
		LIST_INSERT_HEAD(&parent->children, plane, children_link);

		BHND_NVPLANE_UNLOCK_RW();
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
	 * If this is the root plane, we're responsible for deallocating
	 * the shared topology lock.
	 *
	 * Otherwise, we need to deregister ourselves our parent plane.
	 */
	if ((parent = plane->parent) != NULL) {
		/* 
		 * Drop parent's reference to the plane instance.
		 * 
		 * This may deallocate our instance entirely, and no member
		 * references to this plane should be made after this point.
		 */
		bhnd_nvram_plane_remove_child(parent, plane);

		/* Drop our strong reference to the parent */
		bhnd_nvram_plane_release(parent);
	}
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

	BHND_NVPLANE_LOCK_ASSERT(SA_LOCKED);

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
	BHND_NVPLANE_LOCK_RW();

	if (!bhnd_nvram_plane_is_child(plane, child))
		BHND_NV_PANIC("plane is not a direct child of parent");

	LIST_REMOVE(plane, children_link);

	BHND_NVPLANE_UNLOCK_RW();

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

	BHND_NVPLANE_LOCK_RW();

	/* Device already registered? */
	if (bhnd_nvram_plane_find_device(plane, dev) != NULL) {
		BHND_NVPLANE_UNLOCK_RW();
		return (EEXIST);
	}

	/* Allocate new provider instance */
	prov = bhnd_nvram_device_prov_new(dev);
	if (prov == NULL) {
		BHND_NVPLANE_UNLOCK_RW();
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

		BHND_NVPLANE_UNLOCK_RW();
		return (error);
	}

	BHND_NVPLANE_UNLOCK_RW();

	return (0);
}

/**
 * Search for @p dev in @p plane, returning its provider entry if found.
 */
static struct bhnd_nvram_prov *
bhnd_nvram_plane_find_device(struct bhnd_nvram_plane *plane, device_t dev)
{
	struct bhnd_nvram_prov *prov;

	BHND_NVPLANE_LOCK_ASSERT(SA_LOCKED);

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

	BHND_NVPLANE_LOCK_RW();

	prov = bhnd_nvram_plane_find_device(plane, dev);
	if (prov == NULL) {
		/* Ignore request to deregister unrecognized provider */
		BHND_NVPLANE_UNLOCK_RW();
		return (0);
	}

	error = bhnd_nvram_plane_remove_provider(plane, prov);

	BHND_NVPLANE_UNLOCK_RW();

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
	BHND_NVPLANE_LOCK_ASSERT(SA_XLOCKED);

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

	BHND_NVPLANE_LOCK_RW();

	prov = bhnd_nvram_plane_find_device(plane, dev);
	if (prov == NULL) {
		BHND_NVPLANE_UNLOCK_RW();
		return (EINVAL);
	}

	error = bhnd_nvram_prov_add_paths(plane, prov, pathnames,
	    num_pathnames);

	BHND_NVPLANE_UNLOCK_RW();

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

	BHND_NVPLANE_LOCK_ASSERT(SA_XLOCKED);

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

	BHND_NVPLANE_LOCK_ASSERT(SA_LOCKED);

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

	BHND_NVPLANE_LOCK_ASSERT(SA_XLOCKED);

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

	BHND_NVPLANE_LOCK_RW();

	prov = bhnd_nvram_plane_find_device(plane, dev);
	if (prov == NULL) {
		BHND_NVPLANE_UNLOCK_RW();
		return (EINVAL);
	}

	error = bhnd_nvram_prov_remove_paths(plane, prov, pathnames,
	    num_pathnames);

	BHND_NVPLANE_UNLOCK_RW();

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
		BHND_NVPLANE_LOCK_RW();

		/* Path must not already exist */
		if (bhnd_nvram_phandle_has_child(p->parent, p->name)) {
			BHND_NVPLANE_UNLOCK_RW();
			error = EEXIST;
			goto failed;
		}

		/* Insert weak reference into the parent's child list */
		LIST_INSERT_HEAD(&p->parent->children,
		    BHND_NVREF_RETAIN_WEAK(p, refs), children_link);

		/* Save a strong reference to our parent */
		p->parent = bhnd_nvram_path_retain(parent);

		BHND_NVPLANE_UNLOCK_RW();
	}

	/* Transfer ownership of the new instance to the caller */
	*phandle = p;
	return (0);

failed:
	BHND_NV_ASSERT(p->parent == NULL, ("stale parent reference"));
	BHND_NV_ASSERT(LIST_EMPTY(&p->children), ("active children"));

	if (p->pathname != NULL)
		bhnd_nv_free(p->pathname);

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

	bhnd_nv_free(phandle->pathname);

	if (phandle->parent != NULL) {
		/* Remove our parent's weak reference to this path */
		BHND_NVPLANE_LOCK_RW();

		if (!bhnd_nvram_phandle_is_child(phandle->parent, phandle))
			BHND_NV_PANIC("path is not direct child of parent");

		LIST_REMOVE(phandle, children_link);

		BHND_NVPLANE_UNLOCK_RW();

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

	BHND_NVPLANE_LOCK_ASSERT(SA_LOCKED);

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

			/* Parent references are cyclical if we're already
			 * at the root path */
			if (cwd->parent == NULL)
				continue;

			/* Retain reference to parent path */
			parent = bhnd_nvram_path_retain(cwd->parent);

			/* Replace current path with parent path and continue
			 * searching */
			bhnd_nvram_path_release(cwd);
			cwd = parent;

			continue;
		}

		/* locate name in the current path */
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

		/* Not found? */
		if (child == NULL) {
			bhnd_nvram_path_release(cwd);
			return (NULL);
		}

		/* Replace current path with child path and continue
		 * searching */
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
		next = bhnd_nvram_phandle_open(root, cwd, name, nafmelen);
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

	BHND_NVPLANE_LOCK_ASSERT(SA_LOCKED);

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

	BHND_NVPLANE_LOCK_ASSERT(SA_LOCKED);

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
	BHND_NVPLANE_LOCK_ASSERT(SA_XLOCKED);
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
	BHND_NVPLANE_LOCK_ASSERT(SA_XLOCKED);

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
