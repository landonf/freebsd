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

static int			 bhnd_nvram_provider_busy(
				     struct bhnd_nvram_provider *provider) __attribute__((unused));
static int			 bhnd_nvram_provider_busy_locked(
				     struct bhnd_nvram_provider *provider);

static void			 bhnd_nvram_provider_unbusy(
				     struct bhnd_nvram_provider *provider) __attribute__((unused));
static void			 bhnd_nvram_provider_unbusy_locked(
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
static struct bhnd_nvram_link	*bhnd_nvram_plane_resolve_entry(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_entry *entry);
static void			 bhnd_nvram_plane_remove_entry(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_entry *entry);


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

	entry->canon = bhnd_nvpath_new(pathname, strlen(pathname));
	if (entry->canon == NULL) {
		bhnd_nv_free(entry);
		return (NULL);
	}

	entry->prov = BHND_NVREF_RETAIN_WEAK(provider, refs);
	LIST_INIT(&entry->consumers);
	BHND_NVREF_INIT(&entry->refs);

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
	refcount_init(&provider->busy, 0);

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

	/* Set stopping state; any calls to bhnd_nvram_provider_busy() after
	 * this point will fail with ENODEV */
	BHND_NVPROV_ASSERT_ACTIVE(provider);
	provider->state = BHND_NVRAM_PROV_STOPPING;

	/* Wait for all outstanding reservations to be released. */
	while (provider->busy > 0)
		BHND_NVPROV_LOCK_WAIT(provider);

	/* Set dead state */
	BHND_NV_ASSERT(provider->busy == 0, ("busy %u", provider->busy));
	provider->state = BHND_NVRAM_PROV_DEAD;

	/*
	 * After this point, any concurrent access to the provider will
	 * terminate upon reading the provider's state.
	 *
	 * We can now safely modify other structure members without a lock
	 * held.
	 */
	BHND_NVPROV_UNLOCK_RW(provider);

	LIST_FOREACH_SAFE(e, &provider->entries, ne_link, enext) {
		struct bhnd_nvram_consumer *c, *cnext;

		LIST_FOREACH_SAFE(c, &e->consumers, nc_link, cnext) {
			struct bhnd_nvram_consumer	*consumer;
			struct bhnd_nvram_plane		*plane;
	
			/* Attempt to promote our weak consumer reference */
			consumer = BHND_NVREF_PROMOTE_WEAK(c, refs);

			/* Remove consumer from list */
			LIST_REMOVE(c, nc_link);
			BHND_NVREF_RELEASE_WEAK(c, refs);

			/* If the consumer record is already dead, nothing
			 * left to do. */
			if (consumer == NULL)
				continue;

			/* Try to fetch a strong reference to the plane  */
			plane = BHND_NVREF_PROMOTE_WEAK(consumer->plane, refs);

			/* Drop our consumer reference */
			BHND_NVREF_RELEASE(consumer, refs,
			    bhnd_nvram_consumer_fini);

			/* If the plane is already dead, nothing left to do */
			if (plane == NULL)
				continue;

			/* Ask the plane to deregister the entry, and then
			 * drop our plane reference */
			BHND_NVPLANE_LOCK_RW(plane);
			bhnd_nvram_plane_remove_entry(plane, e);
			BHND_NVPLANE_UNLOCK_RW(plane);

			BHND_NVREF_RELEASE(plane, refs, bhnd_nvram_plane_fini);
		}

		/* Drop entry */
		LIST_REMOVE(e, ne_link);
		BHND_NVREF_RELEASE(e, refs, bhnd_nvram_entry_fini);
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
	BHND_NV_ASSERT(provider->busy == 0, ("provider in use"));

	BHND_NVPROV_LOCK_DESTROY(provider);
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
 * The caller is responsible for releasing the returned link via
 * bhnd_nvram_link_remove()
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
 * Free a link instance and all of its children, adding any unused consumer
 * records to the plane's free list.
 * 
 * If removing @p link results in its parent becoming an empty leaf node, the
 * parent will also be removed. This will be performed recursively, up
 * to (but not including) the persistent root ("/") link entry.
 *
 * @param plane	The NVRAM plane.
 * @param link	The link to be freed.
 */
static void
bhnd_nvram_link_remove(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	struct bhnd_nvram_link	*cwd;
	bool			 in_orig_tree;

	in_orig_tree = true;

	/* We don't need a lock during finalization */
	if (!BHND_NVREF_IS_ZOMBIE(plane, refs))
		BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

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

		/* Once we hit a leaf node, free the node and then try
		 * to walk upwards */
		if (LIST_EMPTY(&cwd->children)) {
			parent = cwd->parent;

			/*
			 * Once we hit the link originally passed to
			 * bhnd_nvram_link_free(), we've freed all requested
			 * nodes.
			 *
			 * After this point, we should only traverse into
			 * parent nodes if they are empty leafs that can be
			 * safely garbage collected
			 */
			if (cwd == link)
				in_orig_tree = false;

			/* Disconnect from the parent */
			if (parent != NULL)
				LIST_REMOVE(cwd, child_link);

			/* Move the consumer record to the free list */
			if (cwd->consumer) {
				LIST_INSERT_HEAD(&plane->freelist,
				    cwd->consumer, free_link);
				cwd->consumer = NULL;
			}

			/* Release the path string */
			BHND_NVREF_RELEASE(cwd->path, refs, bhnd_nvpath_fini);			

			/* Free the link allocation */
			bhnd_nv_free(cwd);

			/* Resume traversal at the parent node? */
			cwd = NULL;
			if (!in_orig_tree && parent != NULL) {
				/* Do not remove non-leaf parents */
				if (!bhnd_nvram_link_is_leaf(plane, parent))
					break;
				
				/* Do not remove the plane's persistent root
				 * node */
				if (plane->root == parent)
					break;
			}

			cwd = parent;
			continue;
		} 

		/* Otherwise, recursively evaluate the first child */
		cwd = LIST_FIRST(&cwd->children);
	}	
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
			LIST_INSERT_HEAD(&cwd->children, next, child_link);
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

	LIST_FOREACH(l, &parent->children, child_link) {
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

	BHND_NVPLANE_LOCK_INIT(plane);
	BHND_NVREF_INIT(&plane->refs);
	LIST_INIT(&plane->children);
	LIST_INIT(&plane->freelist);

	plane->parent = NULL;

	for (size_t i = 0; i < nitems(plane->map); i++)
		LIST_INIT(&plane->map[i]);

	/* Create our persistent root entry */
	plane->root = bhnd_nvram_link_new("/", strlen("/"), NULL, 0);
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

	/* Drop our persistent root link */
	bhnd_nvram_link_remove(plane, plane->root);

	/* Disconnect all consumer records */
	// TODO

	/*
	 * Unlink ourselves from the parent plane.
	 */
	if ((parent = plane->parent) != NULL) {
		bool release_child;

		/* Remove from the parent's list of children */
		BHND_NVPLANE_LOCK_RW(parent);

		release_child = bhnd_nvram_plane_is_child(parent, plane);
		if (release_child)
			LIST_REMOVE(plane, child_link);

		BHND_NVPLANE_UNLOCK_RW(parent);

		/* Drop our parent's weak reference to this instance; this may
		 * deallocate our instance, and no member references should be
		 * made after this point */
		if (release_child)
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
 * Return the plane-specific adjacency link for @p entry, or NULL if not
 * found.
 * 
 * @param plane	The NVRAM plane to be queried.
 * @param entry	The NVRAM entry for which the plane's adjacency link should
 *		be returned.
 */
static struct bhnd_nvram_link *
bhnd_nvram_plane_resolve_entry(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_entry *entry)
{
	struct bhnd_nvram_link	*link;
	size_t			 bucket;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	bucket = (uintptr_t)entry % nitems(plane->map);
	LIST_FOREACH(link, &plane->map[bucket], hash_link) {
		if (link->consumer == NULL)
			continue;

		if (link->consumer->entry != entry)
			continue;

		/* Found entry */
		return (link);
	}

	/* Not found */
	return (NULL);
}

/**
 * Discard the plane-specific adjacency link for @p entry, if any.
 * 
 * @param plane	The NVRAM plane to be modified.
 * @param entry	The NVRAM entry for which the plane's adjacency link should
 *		be removed.
 */
static void
bhnd_nvram_plane_remove_entry(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_entry *entry)
{
	struct bhnd_nvram_link *link;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	/* Find the link */
	link = bhnd_nvram_plane_resolve_entry(plane, entry);
	if (link == NULL)
		return;

	/* Discard the consumer record */
	BHND_NVREF_RELEASE(link->consumer, refs, bhnd_nvram_consumer_fini);
	link->consumer = NULL;

	/* Remove the hash table entry */
	LIST_REMOVE(link, hash_link);
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
    struct bhnd_nvram_provider *provider, char *pathnames[],
    size_t num_pathnames)
{
	struct bhnd_nvram_entry		**entries;
	struct bhnd_nvram_consumer	**consumers;
	int				 error;
	bool				 prov_busy;

	entries = NULL;
	consumers = NULL;
	prov_busy = false;

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

	/* Mark provider as busy, verifying that the provider is currently
	 * active and will remain so until we release our reservation. */
	error = bhnd_nvram_provider_busy_locked(provider);
	if (error == 0) {
		prov_busy = true;
	} else {
		BHND_NVPROV_UNLOCK_RW(provider);
		goto failed;
	}

	/* Register provider entries and consumer records for all paths */
	entries = bhnd_nv_calloc(num_pathnames, sizeof(entries[0]));
	if (entries == NULL) {
		BHND_NVPROV_UNLOCK_RW(provider);
		error = ENOMEM;
		goto failed;
	}

	consumers = bhnd_nvram_consumers_alloc(plane, num_pathnames);
	if (consumers == NULL) {
		BHND_NVPROV_UNLOCK_RW(provider);
		error = ENOMEM;
		goto failed;
	}

	for (size_t i = 0; i < num_pathnames; i++) {
		struct bhnd_nvram_entry	*entry;
		const char		*pathname;
		
		pathname = pathnames[i];

		/* Fetch or create the provider entry */
		entry = bhnd_nvram_entry_insert(provider, pathname);
		if (entry == NULL) {
			BHND_NVPROV_UNLOCK_RW(provider);
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

	/* An entry may only be registered once in a given plane */
	for (size_t i = 0; i < num_pathnames; i++) {
		if (bhnd_nvram_plane_resolve_entry(plane, entries[i]) == NULL)
			continue;

		/* Entry already linked into the plane */
		BHND_NVPLANE_UNLOCK_RW(plane);
		error = EEXIST;
		goto failed;
	}

	/* Add the adjacency list nodes, verifying that none of the links have
	 * an existing consumer record attached */
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
		struct bhnd_nvram_link	*link;
		struct bhnd_nvram_entry	*entry;
		const char		*pathname;
		size_t			 bucket;

		pathname = pathnames[i];

		/* Fetch the link node created (or fetched) above */
		link = bhnd_nvram_link_resolve(plane, NULL, pathname,
		    strlen(pathname));
		BHND_NV_ASSERT(link != NULL, ("'%s' link missing", pathname));

		/* Attach the consumer record */
		link->consumer = BHND_NVREF_RETAIN(consumers[i], refs);
		BHND_NV_ASSERT(link->consumer->entry != NULL, ("NULL entry"));

		/* Insert the entry in our entry -> link table */
		entry = link->consumer->entry;
		bucket = (uintptr_t)entry % nitems(plane->map);
		LIST_INSERT_HEAD(&plane->map[bucket], link, hash_link);
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Clean up our local consumer record references. After this, there
	 * will exist only one strong reference to the consumer record. */
	bhnd_nvram_consumers_free(consumers, num_pathnames);

	/* Release all locally held strong references to our entries */
	for (size_t i = 0; i < num_pathnames; i++)
		BHND_NVREF_RELEASE(entries[i], refs, bhnd_nvram_entry_fini);

	bhnd_nv_free(entries);

	/* Release our reservation on the provider */
	bhnd_nvram_provider_unbusy_locked(provider);

	return (0);

failed:
	/* Clean up our local consumer record references */
	if (consumers != NULL)
		bhnd_nvram_consumers_free(consumers, num_pathnames);

	if (entries != NULL) {
		BHND_NVPROV_LOCK_RW(provider);

		/* Clean up our previously added provider entries */
		for (size_t i = 0; i < num_pathnames; i++) {
			struct bhnd_nvram_entry	*entry = entries[i];

			if (entries[i] == NULL)
				continue;

			/* Try to remove the entry */
			bhnd_nvram_entry_try_remove(provider, entry);

			/* Release our strong entry reference */
			BHND_NVREF_RELEASE(entry, refs, bhnd_nvram_entry_fini);
		}

		/* Release our reservation on the provider */
		if (prov_busy)
			bhnd_nvram_provider_unbusy_locked(provider);

		BHND_NVPROV_UNLOCK_RW(provider);

		bhnd_nv_free(entries);
	}

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
