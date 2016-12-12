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
#include <sys/refcount.h>

#ifdef _KERNEL

#include <sys/malloc.h>
#include <sys/kernel.h>

#else /* !_KERNEL */

#include <errno.h>
#include <stdlib.h>

#endif /* _KERNEL */

#include "bhnd_nvram_private.h"

#include "bhnd_nvramvar.h"

#ifdef _KERNEL
MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");
#endif

/**
 * Allocate and initialize an empty NVRAM plane.
 * 
 * The caller is responsible for releasing the returned NVRAM plane
 * via bhnd_nvram_plane_release().
 * 
 * @retval NULL if allocation failed.
 */
struct bhnd_nvram_plane *
bhnd_nvram_plane_new(void)
{
	struct bhnd_nvram_plane	*plane;

	plane = bhnd_nv_calloc(1, sizeof(*plane));
	if (plane == NULL)
		return (NULL);

	return (plane);
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
	BHND_NV_ASSERT(plane->refs >= 1, ("refcount over-release"));
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
bhnd_nvram_plane_release(struct bhnd_nvram_plane *plane)
{
	BHND_NV_ASSERT(plane->refs >= 1, ("refcount over-release"));

	if (!refcount_release(&plane->refs))
		return;

	bhnd_nv_free(plane);
}

/**
 * Register a new NVRAM provider with @p plane.
 * 
 * TODO
 */
int
bhnd_nvram_plane_register(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov)
{
	// TODO
	return (ENXIO);
}

/**
 * Deregister a NVRAM provider and all associated paths.
 * 
 * TODO
 */
int
bhnd_nvram_plane_deregister(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov)
{
	// TODO
	return (ENXIO);
}

/**
 * Register a new NVRAM path.
 * 
 * TODO
 */
int
bhnd_nvram_plane_register_path(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, const char *path)
{
	// TODO
	return (ENXIO);
}

/**
 * Deregister an NVRAM path.
 * 
 * TODO
 */
int
bhnd_nvram_plane_deregister_path(struct bhnd_nvram_plane *plane,
    struct bhnd_nvram_prov *prov, const char *path)
{
	// TODO
	return (ENXIO);
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
 * Open and return a caller-owned reference to the parent of @p phandle,
 * if any.
 * @p path.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing it via bhnd_nvram_plane_release_path().
 *
 * @param	phandle	The path whose parent is to be opened.
 *
 * @retval non-NULL	a caller-owned reference to the parent of @p phandle.
 * @retval NULL		if @p phandle is the root of the path hierarchy.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_parent(bhnd_nvram_phandle *phandle)
{
	if (phandle->parent == NULL)
		return (NULL);

	return (bhnd_nvram_plane_retain_path(phandle->parent));
}

/**
 * Open and return a caller-owned reference to the given fully qualified
 * @p path.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing it via bhnd_nvram_plane_release_path().
 * 
 * @param	plane	The NVRAM plane containing @p path.
 * @param	path	The fully qualified path to be opened.
 *
 * @retval non-NULL	if @p path is found in @p plane.
 * @retval NULL		if @p path is not found in @p plane.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_open_path(struct bhnd_nvram_plane *plane, const char *path)
{
	// TODO
	return (NULL);
}

/**
 * Retain a new reference to an open path handle, returning @p phandle
 * to the caller.
 * 
 * The caller is responsible for releasing their reference ownership via
 * bhnd_nvram_plane_close_path().
 * 
 * @param	phandle	The path handle to be retained.
 * 
 * @return Returns the @p phandle argument for convenience.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_retain_path(bhnd_nvram_phandle *phandle)
{
	BHND_NV_ASSERT(phandle->refs >= 1, ("refcount over-release"));

	refcount_acquire(&phandle->refs);

	return (phandle);
}

/**
 * Close a previously opened or retained path reference.
 *
 * @param	phandle	The path handle to be closed.
 */
void
bhnd_nvram_plane_close_path(bhnd_nvram_phandle *phandle)
{
	BHND_NV_ASSERT(phandle->refs >= 1, ("refcount over-release"));

	if (!refcount_release(&phandle->refs))
		return;

	/* Release our plane reference */
	bhnd_nvram_plane_release(phandle->plane);

	bhnd_nv_free(phandle);
}

/**
 * Recursively search @p phandle and its parents for the given property,
 * opening and returning a path handle for the first path containing a property
 * matching @p propname, or NULL if not found.
 * 
 * The caller assumes ownership of the returned path handle, and is responsible
 * for releasing the reference via bhnd_nvram_plane_close_path().
 * 
 * @param	phandle		The path at which to start the search.
 * @param	propname	The property to search for.
 */
bhnd_nvram_phandle *
bhnd_nvram_plane_findprop_path(bhnd_nvram_phandle *phandle,
    const char *propname)
{
	// TODO
	return (NULL);
}

int
bhnd_nvram_plane_setprop(bhnd_nvram_phandle *phandle, const char *propname,
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
 * @retval ENODEV	If the underlying provider for @p phandle is
 *			unavailable.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_plane_getprop(bhnd_nvram_phandle *phandle, const char *propname,
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
 * @retval ENODEV	If the underlying provider for @p phandle is
 *			unavailable.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
int
bhnd_nvram_plane_getprop_alloc(bhnd_nvram_phandle *phandle,
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
		error = bhnd_nvram_plane_getprop(phandle, propname, NULL, len,
		    type);
		if (error)
			return (error);

		/* Free previously allocated buffer, if any */
		if (*buf != NULL)
			bhnd_nv_free(buf);

#ifdef _KERNEL
		*buf = malloc(*len, M_BHND_NVRAM, flags);
#else
		*buf = bhnd_nv_malloc(*len);
#endif
		if (buf == NULL)
			return (ENOMEM);

		/* Write to output buffer */
		olen = *len;
		error = bhnd_nvram_plane_getprop(phandle, propname, buf, len,
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
bhnd_nvram_plane_getprop_free(void *buf)
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
bhnd_nvram_plane_getprops_copy(bhnd_nvram_phandle *phandle)
{
	// TODO
	return (NULL);
}
