#-
# Copyright (c) 2016-2017 Landon Fuller <landon@landonf.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# $FreeBSD$

#include <sys/types.h>
#include <sys/bus.h>

#include <dev/bhnd/bhnd.h>
#include <dev/bhnd/nvram/bhnd_nvramvar.h>

INTERFACE bhnd_nvram_prov;

#
# bhnd(4) NVRAM provider interface.
#
# Generic interface to a tree of named NVRAM entries, each with an associated
# key/value property dictionary.
#

HEADER {
	/* forward declarations */
	struct bhnd_nvram_plist;
}

CODE {
	#include "bhnd_nvram_prov_if.h"

	static int
	bhnd_nvram_prov_null_sync(bhnd_nvram_prov_t *prov, bool forced)
	{
		panic("bhnd_nvram_prov_sync unimplemented");
	}

	static int
	bhnd_nvram_prov_null_open_path(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t cwd, const char *pathname, size_t pathlen,
	    bhnd_nvram_phandle_t *phandle)
	{
		panic("bhnd_nvram_prov_open_path unimplemented");
	}

	static bhnd_nvram_phandle_t
	bhnd_nvram_prov_null_retain_path(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t phandle)
	{
		panic("bhnd_nvram_prov_retain_path unimplemented");
	}

	static void
	bhnd_nvram_prov_null_release_path(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t phandle)
	{
		panic("bhnd_nvram_prov_release_path unimplemented");
	}

	static int
	bhnd_nvram_prov_null_get_children(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t **children, size_t *count)
	{
		panic("bhnd_nvram_prov_get_children unimplemented");
	}

	static int
	bhnd_nvram_prov_null_free_children(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t *children, size_t count)
	{
		panic("bhnd_nvram_prov_free_children unimplemented");
	}

	static int
	bhnd_nvram_prov_null_setprop(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t phandle, const char *propname, const void *buf,
	    size_t len, bhnd_nvram_type type)
	{
		panic("bhnd_nvram_prov_setprop unimplemented");
	}

	static int
	bhnd_nvram_prov_null_delprop(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t phandle, const char *propname)
	{
		panic("bhnd_nvram_prov_delprop unimplemented");
	}

	static int
	bhnd_nvram_prov_null_getprop(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t phandle, const char *propname, void *buf,
	    size_t *len, bhnd_nvram_type type, bool search_parents)
	{
		panic("bhnd_nvram_prov_getprop unimplemented");
	}

	static int
	bhnd_nvram_prov_null_copyprops(bhnd_nvram_prov_t *prov,
	    bhnd_nvram_phandle_t phandle, struct bhnd_nvram_plist **plist)
	{
		panic("bhnd_nvram_prov_copyprops unimplemented");
	}
}

/**
 * Request that pending changes be written out to the provider's non-volatile
 * storage.
 *
 * @param	prov	The NVRAM provider.
 * @param	forced	If true, perform the write immediately. If false, the
 *			NVRAM provider may arbitrarily delay or coalesce writes
 *			to prevent unnecessary (or actively malicious) flash
 *			memory wear.
 *
 * @retval 0		success
 * @retval ENOSPC	There is insufficient space to write the NVRAM image,
 *			and no changes were written.
 * @retval EIO		An I/O error occured writing to non-volatile storage.
 * @retval non-zero	If writing to non-volatile storage otherwise fails, a
 *			regular unix error code will be returned.
 */
METHOD int sync {
	bhnd_nvram_prov_t	*prov;
	bool			 forced;
} DEFAULT bhnd_nvram_prov_null_sync;

/**
 * Open and return a handle for the given @p path.
 * 
 * The caller assumes ownership of the returned provider handle, and is
 * responsible for releasing it via BHND_NVRAM_PROV_RELEASE_PATH()
 * 
 * @param	prov	The NVRAM provider.
 * @param	cwd	The provider handle from which @p path will be resolved,
 *			or BHND_NVRAM_PHANDLE_NULL to perform resolution from
 *			the root path.
 * @param	path	The path to be opened relative to @p cwd.
 * @param	pathlen	The length of @p path.
 * @param[out]	phandle	On success, a caller-owned reference to @p path.
 *
 * @retval 0		success
 * @retval ENOMEM	If allocation fails
 * @retval ENODEV	If @p path or @p cwd are not found.
 * @retval non-zero	If opening @p path otherwise fails, a regular
 *			unix error code will be returned.
 */
METHOD int open_path {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 cwd;
	const char		*path;
	size_t			 pathlen;
	bhnd_nvram_phandle_t	*phandle;
} DEFAULT bhnd_nvram_prov_null_open_path;

/**
 * Retain a new reference to an open provider handle.
 * 
 * The caller is responsible for releasing their reference ownership via
 * BHND_NVRAM_PROV_RELEASE_PATH().
 * 
 * @param prov		The NVRAM provider.
 * @param phandle	The provider handle to be retained.
 * 
 * @return Returns the @p phandle argument for convenience.
 */
METHOD bhnd_nvram_phandle_t retain_path {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 phandle;
} DEFAULT bhnd_nvram_prov_null_retain_path;

/**
 * Release a caller-owned reference to the given @p phandle.
 *
 * @param prov		The NVRAM provider.
 * @param phandle	The caller-owned provider handle to be released.
 */
METHOD void release_path {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 phandle;
} DEFAULT bhnd_nvram_prov_null_release_path;

/**
 * Retrieve a list of all direct children of @p phandle, returning the list
 * in @p children and the count in @p count.
 *
 * The resources allocated for the list should be freed using
 * BHND_NVRAM_PROV_FREE_CHILDREN().
 *
 * @param	prov		The NVRAM provider.
 * @param	phandle		The provider handle to be queried.
 * @param[out]	children	The list of child provider handles.
 * @param[out]	count		The number of handles in @p children.
 *
 * @retval 0		success
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 * @retval ENODEV	If @p phandle is no longer mapped in @p prov.
 */
METHOD int get_children {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	**children;
	size_t			*count;
} DEFAULT bhnd_nvram_prov_null_get_children;

/**
 * Free resources allocated in a previous call to
 * BHND_NVRAM_PROV_GET_CHILDREN().
 *
 * @param	erom		The erom parser instance.
 * @param	cores		A core table allocated by @p erom. 
 */
METHOD void free_children {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	*children;
	size_t			 count;
} DEFAULT bhnd_nvram_prov_null_free_children;

/**
 * Insert or update a property value in @p phandle.
 * 
 * @param	prov		The NVRAM provider.
 * @param	phandle		The provider handle to be updated.
 * @param	propname	The property name.
 * @param[out]	buf		The new property value.
 * @param	len		The size of @p buf.
 * @param	type		The data type of @p buf.
 *
 * @retval 0		success
 * @retval ENOENT	If @p propname is not a known property name, and the
 *			definition of arbitrary property names is unsupported
 *			by @p phandle.
 * @retval ENODEV	If @p phandle is no longer mapped in @p prov.
 * @retval EINVAL	If @p propname is read-only.
 * @retval EFTYPE	If @p propname cannot be set to the given value or
 *			value type.
 * @retval ERANGE	If @p buf cannot be represented within the value range
 *			required by @p propname.
 * @retval non-zero	If setting the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
METHOD int setprop {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 phandle;
	const char		*propname;
	const void		*buf;
	size_t			 len;
	bhnd_nvram_type		 type;
} DEFAULT bhnd_nvram_prov_null_setprop;

/**
 * Delete a property value in @p phandle.
 * 
 * @param	prov		The NVRAM provider.
 * @param	phandle		The provider handle to be updated.
 * @param	propname	The property name.
 *
 * @retval 0		success
 * @retval ENOENT	If @p propname is not found in @p phandle.
 * @retval ENODEV	If @p phandle is no longer mapped in @p prov.
 * @retval EINVAL	If @p propname is read-only.
 * @retval non-zero	If deleting the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
METHOD int delprop {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 phandle;
	const char		*propname;
} DEFAULT bhnd_nvram_prov_null_delprop;

/**
 * Read a property value from @p phandle.
 *
 * @param		prov		The NVRAM provider.
 * @param		phandle		The provider handle to be queried.
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
 * @param		search_parents	If true ... TODO
 *
 * @retval 0		success
 * @retval ENOENT	If @p propname is not found in @p phandle (or in a
 *			parent of @p phandle, if BHND_NVRAM_FLAG_SEARCH_PARENTS
 *			is specified).
 * @retval ENOMEM	If @p buf is non-NULL and a buffer of @p len is too
 *			small to hold the requested value.
 * @retval ENODEV	If @p phandle is no longer mapped in @p prov.
 * @retval EFTYPE	If the property value cannot be coerced to @p type.
 * @retval ERANGE	If value coercion would overflow @p type.
 * @retval non-zero	If reading the property value otherwise fails, a
 *			regular unix error code will be returned.
 */
 METHOD int getprop {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 phandle;
	const char		*propname;
	const void		*buf;
	size_t			*len;
	bhnd_nvram_type		 type;
	bool			 search_parents;
} DEFAULT bhnd_nvram_prov_null_getprop;

/**
 * Read and return all properties from @p phandle.
 * 
 * The caller is responsible for releasing the result via
 * @p bhnd_nvram_plist_release().
 * 
 * @param	prov	The NVRAM provider.
 * @param	phandle	The provider handle to be queried.
 * @param[out]	plist	On success, a property list containing a copy of all
 *			properties defined in @p phandle.
 *
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval ENODEV	If @p phandle is no longer mapped in @p prov.
 */
METHOD int copyprops {
	bhnd_nvram_prov_t	*prov;
	bhnd_nvram_phandle_t	 phandle;
	struct bhnd_nvram_plist	**plist;
} DEFAULT bhnd_nvram_prov_null_copyprops;
