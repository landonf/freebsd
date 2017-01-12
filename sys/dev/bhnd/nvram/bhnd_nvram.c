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

static int			 bhnd_nvpath_new(struct bhnd_nvpath **path,
				     const char *pathname, size_t pathlen);
static int			 bhnd_nvpath_append_name(
				     struct bhnd_nvpath **path,
				     struct bhnd_nvpath *parent,
				     const char *name, size_t namelen);

struct bhnd_nvpath		*bhnd_nvpath_retain(struct bhnd_nvpath *path);
void				 bhnd_nvpath_release(struct bhnd_nvpath *path);

static void			 bhnd_nvpath_fini(struct bhnd_nvpath *path);

static bhnd_nvram_consumer	*bhnd_nvram_consumer_new(
				     struct bhnd_nvram_plane *plane);
static void			 bhnd_nvram_consumer_fini(
				     struct bhnd_nvram_consumer *consumer);

static struct bhnd_nvram_entry	*bhnd_nvram_entry_new(
				     struct bhnd_nvram_provider *provider,
				     struct bhnd_nvpath *path);
static void			 bhnd_nvram_entry_fini(
				     struct bhnd_nvram_entry *entry);

static void			 bhnd_nvram_link_free(
				     struct bhnd_nvram_link *link);

static int			 bhnd_nvram_link_set_entry(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link,
				     struct bhnd_nvram_entry *entry);
static void			 bhnd_nvram_link_clear_entry(
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvram_link *link);

static int			 bhnd_nvram_link_insert(
				     struct bhnd_nvram_link **link,
				     struct bhnd_nvram_plane *plane,
				     struct bhnd_nvpath *path,
				     struct bhnd_nvram_entry *entry);
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
 * @param[out]	path		On success, the newly allocated NVRAM path
 *				string.
 * @param	pathname	A fully qualified, normalized path.
 * @param	pathlen		The length of @p pathname.
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval EINVAL	if @p pathname is not in fully-qualified, normalized
 *			form.
 */
static int
bhnd_nvpath_new(struct bhnd_nvpath **path, const char *pathname,
    size_t pathlen)
{
	struct bhnd_nvpath *p;

	/* Validate path */
	if (!bhnd_nvram_is_qualified_path(pathname, pathlen))
		return (EINVAL);

	if (!bhnd_nvram_is_normalized_path(pathname, pathlen))
		return (EINVAL);

	/* Allocate new path instance */
	if ((p = bhnd_nv_calloc(1, sizeof(*p))) == NULL)
		return (ENOMEM);

	BHND_NVREF_INIT(&p->refs);

	/* Copy path name */
	p->pathname = bhnd_nv_strndup(pathname, pathlen);
	if (p->pathname == NULL) {
		bhnd_nv_free(p);
		return (ENOMEM);
	}
	p->pathlen = strlen(p->pathname);

	/* Populate path's base name */
	p->basename = bhnd_nvram_parse_path_filename(p->pathname, p->pathlen,
	    &p->baselen);

	/* Success */
	*path = p;
	return (0);
}

/**
 * Allocate and return a new path by appending @p name to @p parent.
 * 
 * @param[out]	path	On success, the new path instance.
 * @param	parent	The parent path.
 * @param	name	The single path component to append to @p parent.
 * @param	namelen	The length of @p name.
 * 
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval EINVAL	if @p name is not a valid, relative path component.
 */
static int
bhnd_nvpath_append_name(struct bhnd_nvpath **path, struct bhnd_nvpath *parent,
    const char *name, size_t namelen)
{
	char	*buf;
	size_t	 bufsize;
	int	 error;

	/* Cannot contain ".." or ".", etc */
	if (!bhnd_nvram_is_normalized_path(name, namelen))
		return (EINVAL);

	/* Must not contain path delimiter */
	if (memchr(name, '/', namelen) != NULL)
		return (EINVAL);

	/* Calculate namelen minus any trailing NUL */
	namelen = strnlen(name, namelen);

	/* Determine total concatenated length of parent + '/' + name + '\0' */
	bufsize = namelen + 1; /* '\0' */

	if (SIZE_MAX - (parent->pathlen + 1 /* '/' */) < bufsize)
		return (ENOMEM); /* size_t would overflow */

	bufsize += parent->pathlen + 1;

	/* Produce concatenated path */
	if ((buf = bhnd_nv_malloc(bufsize)) == NULL)
		return (ENOMEM);

	strcpy(buf, parent->pathname);

	BHND_NV_ASSERT(parent->pathlen > 0, ("invalid base length"));
	buf[parent->pathlen] = '/';

	strncpy(buf+parent->pathlen+1, name, namelen);

	BHND_NV_ASSERT(bufsize > 0, ("invalid bufsize"));
	buf[bufsize-1] = '\0';

	/* Allocate new path instance */
	error = bhnd_nvpath_new(path, buf, bufsize);
	bhnd_nv_free(buf);

	return (error);
}

struct bhnd_nvpath *
bhnd_nvpath_retain(struct bhnd_nvpath *path)
{
	return (BHND_NVREF_RETAIN(path, refs));
}

void
bhnd_nvpath_release(struct bhnd_nvpath *path)
{
	BHND_NVREF_RELEASE(path, refs, bhnd_nvpath_fini);
}

static void
bhnd_nvpath_fini(struct bhnd_nvpath *path)
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
	consumer->use_count = 0;

	return (consumer);
}


static void
bhnd_nvram_consumer_fini(struct bhnd_nvram_consumer *consumer)
{
	BHND_NVREF_RELEASE_WEAK(consumer->plane, refs);
}

/**
 * Allocate, initialize, and return a new NVRAM entry.
 * 
 * The caller is responsible for releasing the returned entry.
 * 
 * @param provider	The NVRAM entry's data provider.
 * @param path		The NVRAM entry's canonical (provider-defined) path.
 * 
 * @retval non-NULL	success.
 * @retval NULL		if allocation fails.
 */
static struct bhnd_nvram_entry *
bhnd_nvram_entry_new(struct bhnd_nvram_provider *provider,
    struct bhnd_nvpath *path)
{
	struct bhnd_nvram_entry *entry;

	entry = bhnd_nv_calloc(1, sizeof(*entry));
	if (entry == NULL)
		return (NULL);

	BHND_NVREF_INIT(&entry->refs);

	entry->provider = BHND_NVREF_RETAIN(provider, refs);
	entry->path = bhnd_nvpath_retain(path);

	return (entry);
}

static void
bhnd_nvram_entry_fini(struct bhnd_nvram_entry *entry)
{
	BHND_NVREF_RELEASE(entry->provider, refs, bhnd_nvram_provider_fini);
	bhnd_nvpath_release(entry->path);
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
	provider->busy = 0;

	BHND_NVPROV_LOCK_INIT(provider);
	BHND_NVREF_INIT(&provider->refs);

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
 *   registered via bhnd_nvram_plane_register_paths() or
 *   bhnd_nvram_plane_map_entry().
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
 * @retval EBUSY	if @p num_uses would overflow the use count; consumer
 *			registration will not be possible until an existing
 *			consumer entry is removed.
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
	if (SIZE_MAX - consumer->use_count < use_count) {
		BHND_NVPROV_UNLOCK_RW(provider);
		return (EBUSY);
	}

	consumer->use_count += use_count;
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
		if (c->use_count < use_count)
			BHND_NV_PANIC("%zu < %zu", c->use_count, use_count);

		c->use_count -= use_count;

		/* If the count hits zero, deallocate the consumer entry */
		if (c->use_count == 0) {
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
 * @retval EAGAIN	if incrementing @p provider's busy count would trigger
 *			an overflow.
 */
static int
bhnd_nvram_provider_busy(struct bhnd_nvram_provider *provider)
{
	int error;

	BHND_NVPROV_LOCK_RW(provider);
	error = bhnd_nvram_provider_busy_locked(provider);
	BHND_NVPROV_UNLOCK_RW(provider);

	return (error);
}

static int
bhnd_nvram_provider_busy_locked(struct bhnd_nvram_provider *provider)
{
	BHND_NVPROV_LOCK_ASSERT(provider, SA_XLOCKED);

	if (provider->state != BHND_NVRAM_PROV_ACTIVE)
		return (ENODEV);

	if (provider->busy == SIZE_MAX)
		return (EAGAIN);

	provider->busy++;

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
	BHND_NVPROV_LOCK_RW(provider);
	bhnd_nvram_provider_unbusy_locked(provider);
	BHND_NVPROV_UNLOCK_RW(provider);
}

static void
bhnd_nvram_provider_unbusy_locked(struct bhnd_nvram_provider *provider)
{
	BHND_NVPROV_LOCK_ASSERT(provider, SA_XLOCKED);
	BHND_NVPROV_ASSERT_ACTIVE(provider);

	BHND_NV_ASSERT(provider->busy > 0, ("busy count underflow"));
	provider->busy--;

	if (provider->busy == 0) {
		/* Wake up any threads waiting for the busy count to hit zero */
		BHND_NVPROV_LOCK_WAKEUP(provider);
	}
}

/**
 * Free a link instance and all of its children. The link (and all of its
 * children) must not have a backing NVRAM entry.
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

		/* Must not have a backing entry assigned */
		BHND_NV_ASSERT(cwd->entry == NULL, ("active link"));

		/* Once we hit a leaf node, free the node and then try
		 * to walk upwards */
		if (LIST_EMPTY(&cwd->children)) {
			parent = cwd->parent;

			/* Disconnect from the parent */
			if (parent != NULL)
				LIST_REMOVE(cwd, child_link);

			/* Release the path string */
			bhnd_nvpath_release(cwd->path);

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
 * Attempt to set the backing entry reference in @p link.
 * 
 * @param plane	The NVRAM plane.
 * @param link	The link to be modified.
 * @param entry	The new entry reference to be set.
 * 
 * @retval 0		success
 * @retval EEXIST	if @p link has an existing entry reference.
 */
static int
bhnd_nvram_link_set_entry(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link, struct bhnd_nvram_entry *entry)
{
	size_t bucket;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	if (link->entry != NULL)
		return (EEXIST);

	/* Save entry reference */
	link->entry = BHND_NVREF_RETAIN(entry, refs);

	/* Insert in provider->link map */
	bucket = (uintptr_t)entry->provider % nitems(plane->prov_map);
	LIST_INSERT_HEAD(&plane->prov_map[bucket], link, hash_link);

	return (0);
}

/**
 * Clear the backing entry reference in @p link.
 * 
 * @param plane	The NVRAM plane.
 * @param link	The link to be modified.
 */
static void
bhnd_nvram_link_clear_entry(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_link *link)
{
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	if (link->entry == NULL)
		return;

	/* Clear entry reference */
	BHND_NVREF_RELEASE(link->entry, refs, bhnd_nvram_entry_fini);
	link->entry = NULL;

	/* Remove from provider->link map */
	LIST_REMOVE(link, hash_link);
}

/**
 * Create or fetch the plane-specific adjacency list link for @p pathname
 * in @p plane, returning a borrowed reference to the link.
 *
 * @param	plane		The NVRAM plane.
 * @param	path		The NVRAM path entry to be created or fetched.
 * @param	entry		The backing entry to be set on the link, or
 *				NULL if none.
 * @param[out]	link		On success, the newly inserted link.
 *
 * @retval 0		success
 * @retval EINVAL	if @p cwd is NULL and @p pathname is a relative path.
 * @retval EINVAL	if @p pathname is invalid.
 * @retval EEXIST	if @p entry is non-NULL and an entry is already
 *			registered for the existing adjacency list link at
 *			@p pathname.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvram_link_insert(struct bhnd_nvram_link **link,
    struct bhnd_nvram_plane *plane, struct bhnd_nvpath *path,
    struct bhnd_nvram_entry *entry)
{
	struct bhnd_nvram_link	*cwd;
	const char		*name;
	size_t			 namelen;
	int			 error;

	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	cwd = plane->root;

	/* Walk the input path, resolving or creating all path components */
	name = NULL;
	while ((name = bhnd_nvram_parse_path_next(path->pathname, path->pathlen,
	    name, &namelen)) != NULL)
	{
		struct bhnd_nvram_link	*next;

		/* Resolve or create the next path component's link */
		next = bhnd_nvram_link_resolve(plane, cwd, name, namelen);
		if (next == NULL) {
			struct bhnd_nvpath *next_path;

			/* Allocate path instance */
			error = bhnd_nvpath_append_name(&next_path, cwd->path,
			    name, namelen);
			if (error)
				goto failed;

			/* Allocate new link instance */
			next = bhnd_nv_calloc(1, sizeof(*next));
			if (next == NULL) {
				bhnd_nvpath_release(next_path);

				error = ENOMEM;
				goto failed;
			}

			LIST_INIT(&next->children);

			/* Transfer ownership of next_path to the link */
			next->path = next_path;

			/* Connect to parent */
			next->parent = cwd;
			LIST_INSERT_HEAD(&cwd->children, next, child_link);
		}

		/* Replace cwd with new link and continue resolution */
		cwd = next;
	}

	/* Try to set the entry reference, if any */
	if (entry != NULL) {
		if ((error = bhnd_nvram_link_set_entry(plane, cwd, entry)))
			goto failed;
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

	/* Create our persistent root link */
	plane->root = bhnd_nv_calloc(1, sizeof(*plane->root));
	if (plane->root == NULL) {
		bhnd_nvram_plane_release(plane);
		return (NULL);
	}

	LIST_INIT(&plane->root->children);
	if (bhnd_nvpath_new(&plane->root->path, "/", strlen("/")) != 0) {
		bhnd_nv_free(plane->root);
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

			BHND_NV_ASSERT(link->entry != NULL, ("missing entry"));

			/* Save a reference to the provider */
			prov = BHND_NVREF_RETAIN(link->entry->provider, refs);

			/* Clear the link's entry state */
			bhnd_nvram_link_clear_entry(plane, link);

			/* Release the provider's reference to this consumer */
			bhnd_nvram_provider_remove_consumer(prov, plane, 1);

			/* Release our provider reference */
			BHND_NVREF_RELEASE(prov, refs,
			    bhnd_nvram_provider_fini);
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

	/* Must not have a backing NVRAM entry */
	if (link->entry != NULL)
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
		BHND_NV_ASSERT(link->entry != NULL, ("missing entry"));

		/* Skip non-matching provider entries */
		if (link->entry->provider != provider)
			continue;

		/* Clear link's provider state and remove from the 
		 * provider -> link map */
		bhnd_nvram_link_clear_entry(plane, link);

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
bhnd_nvram_plane_register_paths(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_provider *provider, char *pathnames[],
    size_t num_pathnames)
{
	size_t	prov_bucket, num_links;
	int	error;

	/* Skip if no paths are provided */
	if (num_pathnames == 0)
		return (0);

	/* Check for duplicate paths */
	for (size_t i = 0; i < num_pathnames; i++) {
		const char *pathname = pathnames[i];

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

	num_links = 0;
	for (size_t i = 0; i < num_pathnames; i++) {
		struct bhnd_nvram_entry	*entry;
		struct bhnd_nvram_link	*link;
		struct bhnd_nvpath	*path;

		/* Allocate path instance */
		error = bhnd_nvpath_new(&path, pathnames[i],
		    strlen(pathnames[i]));
		if (error)
			goto failed;

		/* Allocate entry for this path */
		entry = bhnd_nvram_entry_new(provider, path);
		bhnd_nvpath_release(path);

		if (entry == NULL) {
			error = ENOMEM;
			goto failed;
		}

		/* Fetch or create a new link node */
		error = bhnd_nvram_link_insert(&link, plane, entry->path,
		    entry);
		BHND_NVREF_RELEASE(entry, refs, bhnd_nvram_entry_fini);

		if (error)
			goto failed;

		/* Increment count of successfully added links */
		num_links++;
	}

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Release our reservation on the provider, allowing any pending
	 * termination operations to proceed (which could include removing
	 * the links we just added!) */
	bhnd_nvram_provider_unbusy(provider);

	return (0);

failed:
	/* Lock must still be held */
	BHND_NVPLANE_LOCK_ASSERT(plane, SA_XLOCKED);

	/* Clean up all previously inserted links */
	for (size_t i = 0; i < num_links; i++) {
		struct bhnd_nvram_link	*link;
		const char		*pathname;
		size_t			 pathlen;

		pathname = pathnames[i];
		pathlen = strlen(pathname);
		link = bhnd_nvram_link_resolve(plane, NULL, pathname, pathlen);

		BHND_NV_ASSERT(link != NULL, ("missing link"));

		/* Clear link's backing entry */
		bhnd_nvram_link_clear_entry(plane, link);

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

/**
 * Deregister NVRAM paths previously registered via
 * bhnd_nvram_plane_register_paths().
 * 
 * If a path is currently mapped by a provider other than @p provider, the
 * request to deregister the path will be ignored.
 * 
 * @param	plane		The NVRAM plane in which @p pathnames will be
 *				registered.
 * @param	provider	The NVRAM provider for @p pathnames.
 * @param	pathnames	Normalized, fully qualified path names to be
 *				unmapped from @p provider.
 * @param	num_pathnames	The number of @p pathnames.
 */
void
bhnd_nvram_plane_deregister_paths(struct bhnd_nvram_plane *plane,
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
		struct bhnd_nvram_entry *entry;

		/* Provider must match */
		if ((entry = link->entry) == NULL)
			continue;

		if (entry->provider != provider)
			continue;

		/* Path must match */
		for (size_t i = 0; i < num_pathnames; i++) {
			const char *pathname = pathnames[i];

			if (strcmp(pathname, entry->path->pathname) != 0)
				continue;

			/* Clear link's provider state */
			bhnd_nvram_link_clear_entry(plane, link);

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

/**
 * Map an NVRAM entry into @p plane at @p pathname.
 * 
 * @param	plane		The NVRAM plane in which @p entry will be mapped.
 * @param	entry		The NVRAM entry to be mapped.
 * @param	pathname	Normalized, fully qualified path name at which
 *				@p entry should be mapped in @p plane.
 * 
 * @retval 0		success.
 * @retval EEXIST	if @p pathname is already registered in @p plane.
 * @retval ENODEV	if the provider for @p entry is marked for removal.
 * @retval ENOMEM	if allocation fails.
 */
int
bhnd_nvram_plane_map_entry(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_entry *entry, const char *pathname)
{
	struct bhnd_nvram_link	*link;
	struct bhnd_nvpath	*path;
	int			 error;

	/* Parse and wrap the path name; this also verifies that the string
	 * is a normalized, fully-qualified path */
	error = bhnd_nvpath_new(&path, pathname, strlen(pathname));
	if (error)
		return (error);

	/* Try to mark the provider as busy; this ensures that the provider
	 * will not perform termination while we modify the plane */
	if ((error = bhnd_nvram_provider_busy(entry->provider))) {
		bhnd_nvpath_release(path);
		return (error);
	}

	/* Register a consumer reference for the new mapping */
	error = bhnd_nvram_provider_add_consumer(entry->provider, plane, 1);
	if (error) {
		bhnd_nvpath_release(path);
		bhnd_nvram_provider_unbusy(entry->provider);
		return (error);
	}

	/* Fetch or create a new link node, and set the backing entry */
	BHND_NVPLANE_LOCK_RW(plane);

	error = bhnd_nvram_link_insert(&link, plane, entry->path, entry);
	bhnd_nvpath_release(path);

	if (error)
		goto failed;

	BHND_NVPLANE_UNLOCK_RW(plane);

	/* Release our reservation on the provider, allowing any pending
	 * termination operations to proceed (which could include removing
	 * the links we just added!) */
	bhnd_nvram_provider_unbusy(entry->provider);

	return (0);

failed:
	/* Release the consumer reference we added above */
	bhnd_nvram_provider_remove_consumer(entry->provider, plane, 1);

	/* Release our reservation on the provider */
	bhnd_nvram_provider_unbusy(entry->provider);

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
