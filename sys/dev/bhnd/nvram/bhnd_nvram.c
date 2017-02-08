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

#include "bhnd_nvram_prov_if.h"

#include "bhnd_nvramvar.h"
#include "bhnd_nvram_private.h"

MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "BHND NVRAM data");

static bhnd_nvram_entry	*bhnd_nvram_entry_new(bhnd_nvram_plane *plane,
			     bhnd_nvram_provider *provider,
			     bhnd_nvram_phandle phandle);


/**
 * Allocate and initialize a new NVRAM provider instance.
 * 
 * @param[out]	provider	On success, the newly initialized provider
 *				instance.
 * @param	cls		The NVRAM provider class.
 * @param	init_params	Class-specific initialization parameters.
 * 

 * @retval ENOMEM	If allocation fails
 * @retval non-zero	If initializing the provider instance otherwise fails,
 *			a regular unix error code will be returned.
 */
int
bhnd_nvram_provider_new(bhnd_nvram_provider **provider,
    bhnd_nvram_provider_class_t cls, const void *init_params)
{
	bhnd_nvram_provider	*p;
	int			 error;

	p = bhnd_nv_calloc(1, sizeof(*p) + cls->size, M_WAITOK);
	if (p == NULL)
		return (ENOMEM);

	kobj_init((kobj_t)p, cls);
	refcount_init(&p->refs, 1);

	if ((error = BHND_NVRAM_PROV_INIT(p, init_params))) {
		bhnd_nv_free(p);
		return (error);
	}

	*provider = p;
	return (0);
}

/**
 * Return the private instance state for @p provider.
 * 
 * @param provider	The NVRAM provider to be queried.
 */
void *
bhnd_nvram_provider_get_softc(bhnd_nvram_provider *provider)
{
	return (provider->softc);
}

/**
 * Retain a reference to an NVRAM provider.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_provider_release().
 * 
 * @param provider	The NVRAM provider to be retained.
 * 
 * @return Returns the @p provider argument for convenience.
 */
bhnd_nvram_provider *
bhnd_nvram_provider_retain(bhnd_nvram_provider *provider)
{
	BHND_NV_ASSERT(provider->refs > 0, ("retained a zombie provider"));
	refcount_acquire(&provider->refs);

	return (provider);
}

/**
 * Release a reference to an NVRAM provider.
 * 
 * @param provider The NVRAM provider to be released.
 */
void
bhnd_nvram_provider_release(bhnd_nvram_provider *provider)
{
	if (!refcount_release(&provider->refs))
		return;

	BHND_NVRAM_PROV_FINI(provider);
	bhnd_nv_free(provider);
}

/**
 * Allocate a new NVRAM plane with @p name.
 * 
 * The caller is responsible for releasing the returned instance via
 * bhnd_nvram_plane_release().
 * 
 * @param name	The name to be assigned to the returned NVRAM plane.
 *
 * @retval 0		success
 * @retval ENOMEM	if allocation fails.
 * @retval EINVAL	if @p name is not a valid relative name.
 * @retval EEXIST	if @p name already exists in @p parent.
 */
bhnd_nvram_plane *
bhnd_nvram_plane_new(const char *name)
{
	bhnd_nvram_plane *plane;

	/* Allocate new plane instance */
	plane = bhnd_nv_calloc(1, sizeof(*plane), M_NOWAIT);
	if (plane == NULL)
		return (NULL);

	plane->name = bhnd_nv_strdup(name, M_NOWAIT);
	if (plane->name == NULL) {
		bhnd_nv_free(plane);
		return (NULL);
	}

	refcount_init(&plane->refs, 1);
	plane->provider = NULL;

	BHND_NVPLANE_LOCK_INIT(plane);

	return (plane);
}

/**
 * Retain a strong reference to @p plane, returning @p plane to the caller.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_plane_release().
 * 
 * @param	plane	The NVRAM plane to be retained.
 */
bhnd_nvram_plane *
bhnd_nvram_plane_retain(bhnd_nvram_plane *plane)
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
bhnd_nvram_plane_release(bhnd_nvram_plane *plane)
{
	if (!refcount_release(&plane->refs))
		return;

	if (plane->provider != NULL)
		bhnd_nvram_provider_release(plane->provider);

	bhnd_nv_free(plane->name);
	BHND_NVPLANE_LOCK_DESTROY(plane);
}

/**
 * Return a borrowed reference to the name assigned to @p plane.
 * 
 * @param plane	The NVRAM plane to query.
 */
const char *
bhnd_nvram_plane_get_name(bhnd_nvram_plane *plane)
{
	/* Immutable, no locking required */
	return (plane->name);
}

/**
 * Set the NVRAM plane's provider.
 * 
 * @param	plane		The NVRAM plane into which @p provider will be
 * 				mapped.
 * @param	provider	The NVRAM provider to be mapped.
 *
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval EEXIST	If a provider has already been mapped for @p plane.
 * @retval non-zero	If setting the plane provider otherwise fails, a regular
 *			unix error code will be returned.
 */
int
bhnd_nvram_plane_set_provider(bhnd_nvram_plane *plane,
    bhnd_nvram_provider *provider)
{
	BHND_NVPLANE_LOCK_RW(plane);

	/* Plane must not already have a provider mapped. */
	if (plane->provider != NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (EEXIST);
	}

	/* Connect the provider; will be used for all future entry resolution */
	plane->provider = provider;

	BHND_NVPLANE_UNLOCK_RW(plane);

	return (0);
}

/**
 * Open and return a caller-owned reference to the entry at @p path in @p plane.
 * 
 * The caller assumes ownership of the returned entry, and is responsible for
 * releasing it via bhnd_nvram_release()
 * 
 * @param	plane		The NVRAM plane.
 * @param	pathname	The path to be opened.
 * @param[out]	entry		On success, a caller-owned reference to the
 *				entry at @p path.
 *
 * @retval 0		success
 * @retval ENOMEM	If allocation fails
 * @retval ENXIO	If no provider has been set for @p plane.
 * @retval ENOENT	If @p path is not found in @p plane.
 * @retval non-zero	If opening @p path otherwise fails, a regular
 *			unix error code will be returned.
 */
int
bhnd_nvram_plane_open_path(bhnd_nvram_plane *plane, const char *path,
    bhnd_nvram_entry **entry)
{
	bhnd_nvram_provider	*provider;
	bhnd_nvram_phandle	 phandle;
	bhnd_nvram_entry	*result;
	int			 error;

	/*
	 * Fetch and retain the current provider.
	 */
	BHND_NVPLANE_LOCK_RO(plane);

	if ((provider = plane->provider) == NULL) {
		BHND_NVPLANE_UNLOCK_RW(plane);
		return (ENXIO);
	}

	bhnd_nvram_provider_retain(provider);

	BHND_NVPLANE_UNLOCK_RO(plane);

	/* Attempt to resolve the path */
	error = BHND_NVRAM_PROV_OPEN_PATH(provider, path, strlen(path),
	    &phandle);
	if (error) {
		bhnd_nvram_provider_release(provider);
		return (error);
	}

	/* Allocate entry instance for the caller */
	result = bhnd_nvram_entry_new(plane, provider, phandle);
	bhnd_nvram_provider_release(provider);

	if (result == NULL)
		return (ENOMEM);

	*entry = result;
	return (0);
}

/**
 * Allocate a new entry.
 * 
 * The caller is responsible for releasing the returned entry via
 * bhnd_nvram_release().
 * 
 * @retval non-NULL	success
 * @retval NULL		if allocation fails
 */
static bhnd_nvram_entry *
bhnd_nvram_entry_new(bhnd_nvram_plane *plane, bhnd_nvram_provider *provider,
    bhnd_nvram_phandle phandle)
{
	bhnd_nvram_entry *entry;

	entry = bhnd_nv_calloc(1, sizeof(*entry), M_NOWAIT);
	if (entry == NULL)
		return (NULL);

	refcount_init(&entry->refs, 1);
	entry->plane = bhnd_nvram_plane_retain(plane);
	entry->provider = bhnd_nvram_provider_retain(provider);

	BHND_NVRAM_PROV_RETAIN_PATH(provider, phandle);
	entry->phandle = phandle;

	return (entry);
}

/**
 * Retain a new reference to an NVRAM entry.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_release().
 * 
 * @param entry	The entry to be retained.
 * 
 * @return Returns the @p entry argument for convenience.
 */
bhnd_nvram_entry *
bhnd_nvram_retain(bhnd_nvram_entry *entry)
{
	refcount_acquire(&entry->refs);
	return (entry);
}

/**
 * Release a caller-owned reference to the given @p entry.
 *
 * @param entry	The caller-owned entry to be released.
 */
void
bhnd_nvram_release(bhnd_nvram_entry *entry)
{
	if (!refcount_release(&entry->refs))
		return;

	BHND_NVRAM_PROV_RELEASE_PATH(entry->provider, entry->phandle);
	bhnd_nvram_provider_release(entry->provider);
	bhnd_nvram_plane_release(entry->plane);

	bhnd_nv_free(entry);
}

/**
 * Return a borrowed reference to the entry's plane.
 * 
 * @param entry	The entry to query.
 */
bhnd_nvram_plane *
bhnd_nvram_get_plane(bhnd_nvram_entry *entry)
{
	return (entry->plane);
}

/**
 * Return a borrowed reference to the entry's backing provider.
 * 
 * @param entry	The entry to query.
 */
bhnd_nvram_provider *
bhnd_nvram_get_provider(bhnd_nvram_entry *entry)
{
	return (entry->provider);
}

/**
 * Return a borrowed reference to the entry's backing provider handle.
 * 
 * @param entry	The entry to query.
 */
bhnd_nvram_phandle
bhnd_nvram_get_phandle(bhnd_nvram_entry *entry)
{
	return (entry->phandle);
}

/**
 * Return a borrowed reference to the entry's fully-qualified,
 * canonical path.
 *
 * @param entry	The entry to query.
 */
const char *
bhnd_nvram_get_pathname(bhnd_nvram_entry *entry)
{
	// TODO
	panic("unimplemented");
}

/**
 * Retrieve a list of all direct children of @p entry, returning the entry
 * references in @p children and the count in @p count.
 *
 * The resources allocated for @p children should be freed using
 * bhnd_nvram_free_children().
 *
 * @param	entry		The entry to query.
 * @param[out]	children	On success, all child entries.
 * @param[out]	count		The number of entries in @p children.
 *
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval ENOENT	If the path described by @p entry does not exist.
 */
int
bhnd_nvram_get_children(bhnd_nvram_entry *entry,
    bhnd_nvram_entry ***children, size_t *count)
{
	bhnd_nvram_phandle	*phandles;
	bhnd_nvram_entry	**entries;
	size_t			 num_entries;
	int			 error;

	/* Fetch phandles for all children */
	error = BHND_NVRAM_PROV_GET_CHILDREN(entry->provider, entry->phandle,
	    &phandles, &num_entries);
	if (error)
		return (error);

	/* No children? */
	if (num_entries == 0) {
		*children = NULL;
		*count = 0;
		return (0);
	}

	/* Allocate and populate our entry array */
	entries = bhnd_nv_calloc(num_entries, sizeof(*entries), M_NOWAIT);
	if (entries == NULL) {
		BHND_NVRAM_PROV_FREE_CHILDREN(entry->provider, phandles,
		    num_entries);
		return (ENOMEM);
	}

	for (size_t i = 0; i < num_entries; i++) {
		entries[i] = bhnd_nvram_entry_new(entry->plane, entry->provider,
		    phandles[i]);

		if (entries[i] == NULL) {
			/* Clean up partially populated entry array */
			for (size_t j = 0; j < i; j++)
				bhnd_nvram_release(entries[j]);
			bhnd_nv_free(entries);

			/* Clean up phandle array */
			BHND_NVRAM_PROV_FREE_CHILDREN(entry->provider, phandles,
			    num_entries);

			return (ENOMEM);
		}
	}

	/* Clean up our local phandle array; the entries allocated above
	 * retain a strong reference to these phandles */
	BHND_NVRAM_PROV_FREE_CHILDREN(entry->provider, phandles, num_entries);

	*children = entries;
	*count = num_entries;

	return (0);
}


/**
 * Free resources allocated in a previous call to
 * bhnd_nvram_get_children().
 *
 * @param	entry		The NVRAM parent entry.
 * @param	children	A list of children allocated by @p entry. 
 * @param	count		The number of entries in @p children.
 */
void
bhnd_nvram_free_children(bhnd_nvram_entry *entry,
    bhnd_nvram_entry **children, size_t count)
{
	if (count == 0) {
		BHND_NV_ASSERT(*children == NULL, ("non-NULL empty array"));
		return;
	}

	for (size_t i = 0; i < count; i++) {
		BHND_NV_ASSERT(entry->provider == children[i]->provider,
		    ("provider mismatch"));
		bhnd_nvram_release(children[i]);
	}

	bhnd_nv_free(children);
}

/**
 * Insert or update a property value in @p entry.
 * 
 * @param	entry		The NVRAM entry to be updated.
 * @param	propname	The property name.
 * @param[out]	buf		The new property value.
 * @param	len		The size of @p buf.
 * @param	type		The data type of @p buf.
 *
 * @retval 0		success
 * @retval ENOATTR	If @p propname is not a known property name, and the
 *			definition of arbitrary property names within @p entry
 *			is unsupported.
 * @retval ENOENT	If the path described by @p entry does not exist.
 * @retval EFTYPE	If @p propname cannot be set to the given value or
 *			value type.
 * @retval ERANGE	If @p buf cannot be represented within the value range
 *			required by @p propname.
 * @retval non-zero	If setting the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_setprop(bhnd_nvram_entry *entry, const char *propname,
    const void *buf, size_t len, bhnd_nvram_type type)
{
	return (BHND_NVRAM_PROV_SETPROP(entry->provider, entry->phandle,
	    propname, buf, len, type));
}

/**
 * Delete a property value in @p entry.
 * 
 * @param	entry		The NVRAM entry to be updated.
 * @param	propname	The property name.
 *
 * @retval 0		success
 * @retval ENOATTR	If @p propname is not found in @p entry.
 * @retval ENOENT	If the path described by @p entry does not exist.
 * @retval EINVAL	If @p propname is read-only.
 * @retval non-zero	If deleting the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_delprop(bhnd_nvram_entry *entry, const char *propname)
{
	return (BHND_NVRAM_PROV_DELPROP(entry->provider, entry->phandle,
	    propname));
}

/**
 * Read a property value from @p entry.
 *
 * @param		entry		The NVRAM entry to be queried.
 * @param		propname	The property name.
 * @param[out]		buf		On success, the property value will be
 *					written to this buffer. This argment may
 *					be NULL if the value is not desired.
 * @param[in,out]	len		The maximum capacity of @p buf. On
 *					success, will be set to the actual size
 *					of the requested property. This argment
 *					may be NULL if the value is not
 *					desired (e.g. to test for the existence
 *					of a property).
 * @param		type		The data type to be written to @p buf.
 * @param		search_parents	If true, recursively search @p entry
 *					and its parents for the first entry
 *					containing a property matching
 *					@p propname.
 *
 * @retval 0		success
 * @retval ENOATTR	If @p propname is not found in @p entry (or in a parent
 *			of @p entry, if search_parents is specified).
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 * @retval ENOENT	If the path described by @p entry does not exist.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_getprop(bhnd_nvram_entry *entry, const char *propname,
    void *buf, size_t *len, bhnd_nvram_type type, bool search_parents)
{
	return (BHND_NVRAM_PROV_GETPROP(entry->provider, entry->phandle,
	    propname, buf, len, type, search_parents));
}

/**
 * Read a property value from @p entry, automatically allocating a buffer
 * to store the result.
 * 
 * On success, the caller is responsible for freeing the allocated value
 * via bhnd_nvram_prop_free().
 *
 * @param		entry		The NVRAM entry to be queried.
 * @param		propname	The property name.
 * @param[out]		buf		On success, will be set to a pointer to
 *					a newly allocated buffer containing the
 *					requested property value.
 * @param[in,out]	len		On success, will be set to the actual
 *					size of the property value in @p buf.
 * @param		type		The data type to be written to @p buf.
 * @param		search_parents	If true, recursively search @p entry
 *					and its parents for the first entry
 *					containing a property matching
 *					@p propname.
 *
 * @retval 0		success
 * @retval ENOATTR	If @p propname is not found in @p entry (or in a parent
 *			of @p entry, if search_parents is specified).
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 * @retval ENOENT	If the path described by @p entry does not exist.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_getprop_alloc(bhnd_nvram_entry *entry, const char *propname,
    void **buf, size_t *len, bhnd_nvram_type type, bool search_parents)
{
	void	*outp;
	size_t	 req_olen, olen;
	int	 error;

	outp = NULL;
	do {
		if (outp != NULL)
			bhnd_nv_free(outp);

		/* Determine required buffer size */
		error = BHND_NVRAM_PROV_GETPROP(entry->provider, entry->phandle,
		    propname, NULL, &req_olen, type, search_parents);
		if (error)
			goto failed;

		/* Allocate and fetch to buffer */
		outp = bhnd_nv_malloc(req_olen, M_NOWAIT);
		if (outp == NULL) {
			error = ENOMEM;
			goto failed;
		}

		olen = req_olen;
		error = BHND_NVRAM_PROV_GETPROP(entry->provider, entry->phandle,
		    propname, outp, &olen, type, search_parents);
	} while (error == ENOMEM && olen > req_olen);

	if (error)
		goto failed;

	*buf = outp;
	*len = olen;
	return (0);

failed:
	if (outp != NULL)
		bhnd_nv_free(outp);

	return (error);
}

/**
 * Free a property value buffer previously allocated by
 * bhnd_nvram_getprop_alloc().
 *
 * @param buf	The buffer to be freed.
 */
void
bhnd_nvram_prop_free(void *buf)
{
	bhnd_nv_free(buf);
}

/**
 * Read and return all properties defined in @p entry.
 * 
 * The caller is responsible for releasing the result via
 * @p bhnd_nvram_plist_release().
 * 
 * @param	entry	The NVRAM entry to be queried.
 * @param[out]	plist	On success, a property list containing a copy of all
 *			properties defined in @p entry.
 *
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval ENOENT	If the path described by @p entry does not exist.
 * @retval non-zero	If reading the property values otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_copyprops(bhnd_nvram_entry *entry,
    struct bhnd_nvram_plist **plist)
{
	return (BHND_NVRAM_PROV_COPYPROPS(entry->provider, entry->phandle,
	    plist));
}
