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
#include <sys/hash.h>
#include <sys/queue.h>

#ifdef _KERNEL

#include <sys/ctype.h>
#include <sys/systm.h>

#else /* !_KERNEL */

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#endif /* _KERNEL */

#include "bhnd_nvram_private.h"
#include "bhnd_nvram_datavar.h"

#include "bhnd_nvram_storevar.h"

/*
 * BHND NVRAM Store
 *
 * Manages in-memory and persistent representations of NVRAM data.
 */

typedef enum {
	BHND_NVSTORE_NAME_INTERNAL,
	BHND_NVSTORE_NAME_EXTERNAL
} bhnd_nvstore_name_type;

static int			 bhnd_nvstore_parse_data(
				     struct bhnd_nvram_store *sc);

static int			 bhnd_nvstore_parse_path_entries(
				     struct bhnd_nvram_store *sc);

static bhnd_nvstore_index	*bhnd_nvstore_index_new(size_t capacity);
static void			 bhnd_nvstore_index_free(
				     bhnd_nvstore_index *index);
static int			 bhnd_nvstore_index_append(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_index *index,
				     void *cookiep);
static int			 bhnd_nvstore_index_prepare(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_index *index);
static void			*bhnd_nvstore_index_lookup(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_index *index,
				     const char *name);

static bhnd_nvstore_path	*bhnd_nvstore_get_root_path(
				    struct bhnd_nvram_store *sc);
static bool			 bhnd_nvstore_is_root_path(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_path *path);

static void			*bhnd_nvstore_path_data_next(
				      struct bhnd_nvram_store *sc,
				      bhnd_nvstore_path *path, void **indexp);
static void			*bhnd_nvstore_path_data_lookup(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_path *path,
				     const char *name);

static bhnd_nvstore_alias	*bhnd_nvstore_find_alias(
				     struct bhnd_nvram_store *sc,
				     const char *path);
static bhnd_nvstore_alias	*bhnd_nvstore_get_alias(
				     struct bhnd_nvram_store *sc,
				     u_long alias_val);

static bhnd_nvstore_path	*bhnd_nvstore_get_path(
				     struct bhnd_nvram_store *sc,
				     const char *path, size_t path_len);
static bhnd_nvstore_path	*bhnd_nvstore_resolve_path_alias(
				     struct bhnd_nvram_store *sc,
				     u_long aval);

static bhnd_nvstore_path	*bhnd_nvstore_var_get_path(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_name_info *info);
static int			 bhnd_nvstore_var_register_path(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_name_info *info,
				     void *cookiep);

static int			 bhnd_nvstore_register_path(
				     struct bhnd_nvram_store *sc,
				     const char *path, size_t path_len);
static int			 bhnd_nvstore_register_alias(
				     struct bhnd_nvram_store *sc,
				     const bhnd_nvstore_name_info *info,
				     void *cookiep);

static const char		*bhnd_nvstore_parse_relpath(const char *parent,
				     const char *child);
static int			 bhnd_nvstore_parse_name_info(const char *name,
				     bhnd_nvstore_name_type name_type,
				     uint32_t data_caps,
				     bhnd_nvstore_name_info *info);

/**
 * Allocate and initialize a new NVRAM data store instance.
 *
 * The caller is responsible for deallocating the instance via
 * bhnd_nvram_store_free().
 * 
 * @param[out] store On success, a pointer to the newly allocated NVRAM data
 * instance.
 * @param data The NVRAM data to be managed by the returned NVRAM data store
 * instance.
 *
 * @retval 0 success
 * @retval non-zero if an error occurs during allocation or initialization, a
 * regular unix error code will be returned.
 */
int
bhnd_nvram_store_new(struct bhnd_nvram_store **store,
    struct bhnd_nvram_data *data)
{
	struct bhnd_nvram_store *sc;
	int			 error;

	/* Allocate new instance */
	sc = bhnd_nv_calloc(1, sizeof(*sc));
	if (sc == NULL)
		return (ENOMEM);

	BHND_NVSTORE_LOCK_INIT(sc);
	BHND_NVSTORE_LOCK(sc);

	/* Initialize path hash table */
	sc->num_paths = 0;
	for (size_t i = 0; i < nitems(sc->paths); i++)
		LIST_INIT(&sc->paths[i]);

	/* Initialize alias hash table */
	sc->num_aliases = 0;
	for (size_t i = 0; i < nitems(sc->aliases); i++)
		LIST_INIT(&sc->aliases[i]);

	/* Retain the NVRAM data */
	sc->data = bhnd_nvram_data_retain(data);
	sc->data_caps = bhnd_nvram_data_caps(data);

	/* Register required root path */
	error = bhnd_nvstore_register_path(sc, BHND_NVSTORE_ROOT_PATH,
	    BHND_NVSTORE_ROOT_PATH_LEN);
	if (error)
		goto cleanup;

	sc->root_path = bhnd_nvstore_get_path(sc, BHND_NVSTORE_ROOT_PATH,
	    BHND_NVSTORE_ROOT_PATH_LEN);
	BHND_NV_ASSERT(sc->root_path, ("missing root path"));

	/* Parse all variables vended by our backing NVRAM data instance,
	 * generating all path entries, alias entries, and variable indexes */
	if ((error = bhnd_nvstore_parse_data(sc)))
		goto cleanup;

	*store = sc;

	BHND_NVSTORE_UNLOCK(sc);
	return (0);

cleanup:
	BHND_NVSTORE_UNLOCK(sc);
	bhnd_nvram_store_free(sc);
	return (error);
}

/**
 * Allocate and initialize a new NVRAM data store instance, parsing the
 * NVRAM data from @p io.
 *
 * The caller is responsible for deallocating the instance via
 * bhnd_nvram_store_free().
 * 
 * The NVRAM data mapped by @p io will be copied, and @p io may be safely
 * deallocated after bhnd_nvram_store_new() returns.
 * 
 * @param[out] store On success, a pointer to the newly allocated NVRAM data
 * instance.
 * @param io An I/O context mapping the NVRAM data to be copied and parsed.
 * @param cls The NVRAM data class to be used when parsing @p io, or NULL
 * to perform runtime identification of the appropriate data class.
 *
 * @retval 0 success
 * @retval non-zero if an error occurs during allocation or initialization, a
 * regular unix error code will be returned.
 */
int
bhnd_nvram_store_parse_new(struct bhnd_nvram_store **store,
    struct bhnd_nvram_io *io, bhnd_nvram_data_class *cls)
{
	struct bhnd_nvram_data	*data;
	int			 error;


	/* Try to parse the data */
	if ((error = bhnd_nvram_data_new(cls, &data, io)))
		return (error);

	/* Try to create our new store instance */
	error = bhnd_nvram_store_new(store, data);
	bhnd_nvram_data_release(data);

	return (error);
}

/**
 * Free an NVRAM store instance, releasing all associated resources.
 * 
 * @param sc A store instance previously allocated via
 * bhnd_nvram_store_new().
 */
void
bhnd_nvram_store_free(struct bhnd_nvram_store *sc)
{
	
	/* Clean up alias hash table */
	for (size_t i = 0; i < nitems(sc->aliases); i++) {
		bhnd_nvstore_alias *alias, *anext;
		LIST_FOREACH_SAFE(alias, &sc->aliases[i], na_link, anext)
			bhnd_nv_free(alias);
	}

	/* Clean up path hash table */
	for (size_t i = 0; i < nitems(sc->paths); i++) {
		bhnd_nvstore_path *path, *pnext;
		LIST_FOREACH_SAFE(path, &sc->paths[i], np_link, pnext) {
			if (path->index != NULL)
				bhnd_nvstore_index_free(path->index);

			bhnd_nvram_plist_release(path->pending);
			bhnd_nv_free(path->path_str);
			bhnd_nv_free(path);
		}
	}

	if (sc->data != NULL)
		bhnd_nvram_data_release(sc->data);

	BHND_NVSTORE_LOCK_DESTROY(sc);
	bhnd_nv_free(sc);
}

/** Maximum alias ('devpathXX=') variable name length */
#define	BHND_NVSTORE_ALIAS_VARNAME_MAXLEN	\
	(sizeof("devpath") + sizeof(u_long)*3)

/**
 * Merge exported per-path variables (uncommitted, committed, or both) into 
 * the empty @p merged property list.
 * 
 * @param	sc	The NVRAM store instance.
 * @param	path	The NVRAM path to be exported.
 * @param	merged	The property list to populate with the merged results.
 * @param	flags	Export flags. See BHND_NVSTORE_EXPORT_*.
 * 
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval non-zero	If merging the variables defined in @p path otherwise
 *			fails, a regular unix error code will be returned.
 */
static int
bhnd_nvstore_merge_exported(struct bhnd_nvram_store *sc,
    bhnd_nvstore_path *path, bhnd_nvram_plist *merged, uint32_t flags)
{
	void	*cookiep, *idxp;
	int	 error;

	/* Populate merged list with all pending variables */
	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_UNCOMMITTED)) {
		error = bhnd_nvram_plist_append_list(merged, path->pending);
		if (error)
			return (error);
	}

	/* Skip merging committed variables? */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMMITTED))
		return (0);

	/* Merge in the committed changes */
	idxp = NULL;
	while ((cookiep = bhnd_nvstore_path_data_next(sc, path, &idxp))) {
		const char	*name;
		bhnd_nvram_val	*val;

		/* Fetch the variable name */
		name = bhnd_nvram_data_getvar_name(sc->data, cookiep);

		/* Trim device path prefix */
		if (sc->data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS)
			name = bhnd_nvram_trim_path_name(name);

		/* Skip if already defined in pending changes */
		if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_UNCOMMITTED)) {
			if (bhnd_nvram_plist_contains(path->pending, name))
				continue;
		}

		/* Skip if higher precedence value was already defined. This
		 * may occur if the underlying data store contains duplicate
		 * keys; iteration will always return the definition with
		 * the highest precedence first */
		if (bhnd_nvram_plist_contains(merged, name))
			continue;

		/* Fetch the variable's value representation */
		if ((error = bhnd_nvram_data_copy_val(sc->data, cookiep, &val)))
			return (error);

		/* Add to path variable list */
		error = bhnd_nvram_plist_append_val(merged, name, val);
		bhnd_nvram_val_release(val);
		if (error)
			return (error);
	}

	return (0);
}

/**
 * Find a free alias value for @p path, and append the devpathXX alias
 * declaration to @p plist.
 * 
 * @param	sc		The NVRAM store instance.
 * @param	path		The NVRAM path for which a devpath alias
 *				variable should be produced.
 * @param	devpath		The devpathXX path value for @p path.
 * @param	plist		The property list to which @p path's devpath
 *				variable will be appended.
 * @param[out]	alias_val	On success, will be set to the alias value
 *				allocated for @p path.
 * 
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval non-zero	If merging the variables defined in @p path otherwise
 *			fails, a regular unix error code will be returned.
 */
static int
bhnd_nvstore_export_devpath_alias(struct bhnd_nvram_store *sc,
    bhnd_nvstore_path *path, const char *devpath, bhnd_nvram_plist *plist,
    u_long *alias_val)
{
	bhnd_nvstore_alias	*alias;
	char			*pathvar;
	int			 error;

	*alias_val = 0;

	/* Prefer alias value already reserved for this path. */
	alias = bhnd_nvstore_find_alias(sc, path->path_str);
	if (alias != NULL) {
		*alias_val = alias->alias;

		/* Allocate devpathXX variable name */
		bhnd_nv_asprintf(&pathvar, "devpath%lu", *alias_val);
		if (pathvar == NULL)
			return (ENOMEM);

		/* Append alias variable to property list */
		error = bhnd_nvram_plist_append_string(plist, pathvar, devpath);

		BHND_NV_ASSERT(error != EEXIST, ("reserved alias %lu:%s in use",
		   * alias_val, path->path_str));

		bhnd_nv_free(pathvar);
		return (error);
	}

	/* Find the next free devpathXX alias entry */
	while (1) {
		/* Skip existing reserved alias values */
		while (bhnd_nvstore_get_alias(sc, *alias_val) != NULL) {
			if (*alias_val == ULONG_MAX)
				return (ENOMEM);

			(*alias_val)++;
		}

		/* Allocate devpathXX variable name */
		bhnd_nv_asprintf(&pathvar, "devpath%lu", *alias_val);
		if (pathvar == NULL)
			return (ENOMEM);

		/* If not in-use, we can terminate the search */
		if (!bhnd_nvram_plist_contains(plist, pathvar))
			break;

		/* Keep searching */
		bhnd_nv_free(pathvar);

		if (*alias_val == ULONG_MAX)
			return (ENOMEM);

		(*alias_val)++;
	}

	/* Append alias variable to property list */
	error = bhnd_nvram_plist_append_string(plist, pathvar, devpath);

	bhnd_nv_free(pathvar);
	return (error);
}

/**
 * Export a single @p child path's properties, appending the result to @p plist.
 * 
 * @param	sc		The NVRAM store instance.
 * @param	top		The root NVRAM path being exported.
 * @param	child		The NVRAM path to be exported.
 * @param	plist		The property list to which @p child's exported
 *				properties should be appended.
 * @param	flags		Export flags. See BHND_NVSTORE_EXPORT_*.
 * 
 * @retval 0		success
 * @retval ENOMEM	If allocation fails.
 * @retval non-zero	If merging the variables defined in @p path otherwise
 *			fails, a regular unix error code will be returned.
 */
static int
bhnd_nvram_store_export_child(struct bhnd_nvram_store *sc,
    bhnd_nvstore_path *top, bhnd_nvstore_path *child, bhnd_nvram_plist *plist,
    uint32_t flags)
{
	bhnd_nvram_plist	*path_vars;
	bhnd_nvram_prop		*prop;
	const char		*relpath;
	char			*prefix, *namebuf;
	size_t			 prefix_len, relpath_len;
	size_t			 namebuf_size, num_props;
	bool			 emit_compact_devpath;
	int			 error;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	prefix = NULL;
	num_props = 0;
	path_vars = NULL;
	namebuf = NULL;

	/* Determine the path relative to the top-level path */
	relpath = bhnd_nvstore_parse_relpath(top->path_str, child->path_str);
	if (relpath == NULL) {
		/* Skip -- not a child of the root path */
		return (0);
	}
	relpath_len = strlen(relpath);

	/* Skip sub-path if export of children was not requested,  */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_CHILDREN) && relpath_len > 0)
		return (0);

	/* Skip if no children are to be exported */
	if ((!BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMMITTED) ||
	     child->num_vars == 0) &&
	    (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_UNCOMMITTED) ||
	     bhnd_nvram_plist_count(child->pending) == 0))
	{
		return (0);
	}

	/* Determine appropriate device path encoding */
	emit_compact_devpath = false;
	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMPACT_DEVPATHS)) {
		/* Re-encode as compact (if non-empty path) */
		if (relpath_len > 0)
			emit_compact_devpath = true;
	} else if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_EXPAND_DEVPATHS)) {
		/* Re-encode with fully expanded device path */
		emit_compact_devpath = false;
	} else {
		/* Preserve existing encoding of this path */
		if (bhnd_nvstore_find_alias(sc, child->path_str) != NULL)
			emit_compact_devpath = true;
	}

	/* Allocate variable device path prefix to use for all property names,
	 * and if using compact encoding, emit the devpathXX= variable */
	prefix = NULL;
	prefix_len = 0;
	if (emit_compact_devpath) {
		u_long	alias_val;
		int	len;

		/* Reserve an alias value and append the devpathXX= variable to
		 * the property list */
		error = bhnd_nvstore_export_devpath_alias(sc, child, relpath,
		    plist, &alias_val);
		if (error)
			return (error);

		/* Allocate variable name prefix */
		len = bhnd_nv_asprintf(&prefix, "%lu:", alias_val);
		if (prefix == NULL)
			return (ENOMEM);
	
		prefix_len = len;
	} else if (relpath_len > 0) {
		int len;

		/* Allocate the variable name prefix, appending '/' to the
		 * relative path */
		len = bhnd_nv_asprintf(&prefix, "%s/", relpath);
		if (prefix == NULL)
			return (ENOMEM);

		prefix_len = len;
	}

	/* Merge committed and uncommitted changes */
	if ((path_vars = bhnd_nvram_plist_new()) == NULL) {
		error = ENOMEM;
		goto cleanup;
	}

	error = bhnd_nvstore_merge_exported(sc, child, path_vars, flags);
	if (error)
		goto cleanup;

	/* If prefixing of variable names is required, allocate a name
	 * formatting buffer */
	namebuf_size = 0;
	if (prefix != NULL) {
		size_t	maxlen;

		/* Find the maximum name length */
		maxlen = 0;
		prop = NULL;
		while ((prop = bhnd_nvram_plist_next(path_vars, prop))) {
			const char *name;

			name = bhnd_nvram_prop_name(prop);
			maxlen = bhnd_nv_ummax(strlen(name), maxlen);
		}

		/* Allocate name buffer (path-prefix + name + '\0') */
		namebuf_size = prefix_len + maxlen + 1;
		namebuf = bhnd_nv_malloc(namebuf_size);
		if (namebuf == NULL) {
			error = ENOMEM;
			goto cleanup;
		}
	}

	/* Append all path variables to the export plist, prepending the
	 * device-path prefix to the variable names, if required */
	prop = NULL;
	while ((prop = bhnd_nvram_plist_next(path_vars, prop)) != NULL) {
		const char *name;

		/* NULL property values denote a pending deletion */
		if (bhnd_nvram_prop_type(prop) == BHND_NVRAM_TYPE_NULL)
			continue;

		/* Prepend device prefix to the variable name */
		name = bhnd_nvram_prop_name(prop);
		if (prefix != NULL) {
			int len;

			/*
			 * Write prefixed variable name to our name buffer.
			 * 
			 * We precalcuate the size when scanning all names 
			 * above, so this should always succeed.
			 */
			len = snprintf(namebuf, namebuf_size, "%s%s", prefix,
			    name);
			if (len < 0 || (size_t)len >= namebuf_size)
				BHND_NV_PANIC("invalid max_name_len");

			name = namebuf;
		}

		/* Add property to export plist */
		error = bhnd_nvram_plist_append_val(plist, name,
		    bhnd_nvram_prop_val(prop));
		if (error)
			goto cleanup;
	}

	/* Success */
	error = 0;

cleanup:
	if (prefix != NULL)
		bhnd_nv_free(prefix);

	if (namebuf != NULL)
		bhnd_nv_free(namebuf);

	if (path_vars != NULL)
		bhnd_nvram_plist_release(path_vars);

	return (error);
}

/**
 * Export a flat NVRAM property list representation of all NVRAM properties
 * at @p path.
 * 
 * @param	sc	The NVRAM store instance.
 * @param	path	The NVRAM path to export, or NULL to select the root
 *			path.
  * @param[out]	plist	On success, will be set to the newly allocated property
 *			list. The caller is responsible for releasing this value
 *			via bhnd_nvram_plist_release().
 * @param	flags	Export flags. See BHND_NVSTORE_EXPORT_*.
 * 
 * @retval 0		success
 * @retval EINVAL	If @p flags is invalid.
 * @retval ENOENT	The requested path was not found.
 * @retval ENOMEM	If allocation fails.
 * @retval non-zero	If export of  @p path otherwise fails, a regular unix
 *			error code will be returned.
 */
int
bhnd_nvram_store_export(struct bhnd_nvram_store *sc, const char *path,
    bhnd_nvram_plist **plist, uint32_t flags)
{
	bhnd_nvstore_path	*top;
	int			 error;
	
	*plist = NULL;

	BHND_NVSTORE_LOCK(sc);

	/* Default to exporting root path */
	if (path == NULL)
		path = BHND_NVSTORE_ROOT_PATH;

	/* Default to exporting all properties */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMMITTED) &&
	    !BHND_NVSTORE_GET_FLAG(flags, EXPORT_UNCOMMITTED))
	{
		flags |= BHND_NVSTORE_EXPORT_ALL_VARS;
	}

	/* Report incompatible device path flags */
	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMPACT_DEVPATHS) &&
	    BHND_NVSTORE_GET_FLAG(flags, EXPORT_EXPAND_DEVPATHS))
	{
		return (EINVAL);
	}

	/* Fetch referenced path */
	top = bhnd_nvstore_get_path(sc, path, strlen(path));
	if (top == NULL) {
		error = ENOENT;
		goto cleanup;
	}

	/* Allocate new, empty property list */
	if ((*plist = bhnd_nvram_plist_new()) == NULL) {
		error = ENOMEM;
		goto cleanup;
	}

	/* Export the top-level path first */
	error = bhnd_nvram_store_export_child(sc, top, top, *plist, flags);
	if (error)
		goto cleanup;

	/* Attempt to export any children of the root path */
	for (size_t i = 0; i < nitems(sc->paths); i++) {
		bhnd_nvstore_path *child;

		LIST_FOREACH(child, &sc->paths[i], np_link) {
			/* Top-level path was already exported */
			if (child == top)
				continue;

			error = bhnd_nvram_store_export_child(sc, top,
			    child, *plist, flags);
			if (error)
				goto cleanup;
		}
	}
	
	/* Export complete */
	error = 0;

cleanup:
	BHND_NVSTORE_UNLOCK(sc);

	if (error && *plist != NULL) {
		bhnd_nvram_plist_release(*plist);
		*plist = NULL;
	}

	return (error);
}

/**
 * Parse all variables vended by our backing NVRAM data instance,
 * generating all path entries, alias entries, and variable indexes.
 * 
 * @param	sc	The NVRAM store instance to be initialized with
 *			paths, aliases, and data parsed from its backing
 *			data.
 *
 * @retval 0		success
 * @retval non-zero	if an error occurs during parsing, a regular unix error
 *			code will be returned.
 */
static int
bhnd_nvstore_parse_data(struct bhnd_nvram_store *sc)
{
	const char	*name;
	void		*cookiep;
	int		 error;

	/* Parse and register all device paths and path aliases. This enables
	 * resolution of _forward_ references to device paths aliases when
	 * scanning variable entries below */
	if ((error = bhnd_nvstore_parse_path_entries(sc)))
		return (error);

	/* Calculate the per-path variable counts, and report dangling alias
	 * references as an error. */
	cookiep = NULL;
	while ((name = bhnd_nvram_data_next(sc->data, &cookiep))) {
		bhnd_nvstore_path	*path;
		bhnd_nvstore_name_info	 info;

		/* Parse the name info */
		error = bhnd_nvstore_parse_name_info(name,
		    BHND_NVSTORE_NAME_INTERNAL, sc->data_caps, &info);
		if (error)
			return (error);

		switch (info.type) {
		case BHND_NVSTORE_VAR:
			/* Fetch referenced path */
			path = bhnd_nvstore_var_get_path(sc, &info);
			if (path == NULL) {
				BHND_NV_LOG("variable '%s' has dangling "
					    "path reference\n", name);
				return (EFTYPE);
			}

			/* Increment path variable count */
			if (path->num_vars == SIZE_MAX) {
				BHND_NV_LOG("more than SIZE_MAX variables in "
				    "path %s\n", path->path_str);
				return (EFTYPE);
			}
			path->num_vars++;
			break;

		case BHND_NVSTORE_ALIAS_DECL:
			/* Skip -- path alias already parsed and recorded */
			break;
		}
	}

	/* If the backing NVRAM data instance vends only a single root ("/")
	 * path, we may be able to skip generating an index for the root
	 * path */
	if (sc->num_paths == 1) {
		bhnd_nvstore_path *path;

		/* If the backing instance provides its own name-based lookup
		 * indexing, we can skip generating a duplicate here */
		if (sc->data_caps & BHND_NVRAM_DATA_CAP_INDEXED)
			return (0);

		/* If the sole root path contains fewer variables than the
		 * minimum indexing threshhold, we do not need to generate an
		 * index */
		path = bhnd_nvstore_get_root_path(sc);
		if (path->num_vars < BHND_NV_IDX_VAR_THRESHOLD)
			return (0);
	}

	/* Allocate per-path index instances */
	for (size_t i = 0; i < nitems(sc->paths); i++) {
		bhnd_nvstore_path	*path;

		LIST_FOREACH(path, &sc->paths[i], np_link) {
			path->index = bhnd_nvstore_index_new(path->num_vars);
			if (path->index == NULL)
				return (ENOMEM);
		}
	}

	/* Populate per-path indexes */
	cookiep = NULL;
	while ((name = bhnd_nvram_data_next(sc->data, &cookiep))) {
		bhnd_nvstore_name_info	 info;
		bhnd_nvstore_path	*path;

		/* Parse the name info */
		error = bhnd_nvstore_parse_name_info(name,
		    BHND_NVSTORE_NAME_INTERNAL, sc->data_caps, &info);
		if (error)
			return (error);

		switch (info.type) {
		case BHND_NVSTORE_VAR:
			/* Fetch referenced path */
			path = bhnd_nvstore_var_get_path(sc, &info);
			BHND_NV_ASSERT(path != NULL,
			    ("dangling path reference"));

			/* Append to index */
			error = bhnd_nvstore_index_append(sc, path->index,
			    cookiep);
			if (error)
				return (error);
			break;

		case BHND_NVSTORE_ALIAS_DECL:
			/* Skip */
			break;
		}
	}

	/* Prepare indexes for querying */
	for (size_t i = 0; i < nitems(sc->paths); i++) {
		bhnd_nvstore_path	*path;

		LIST_FOREACH(path, &sc->paths[i], np_link) {
			error = bhnd_nvstore_index_prepare(sc, path->index);
			if (error)
				return (error);
		}
	}

	return (0);
}


/**
 * Parse and register path and path alias entries for all declarations found in
 * the NVRAM data backing @p nvram.
 * 
 * @param sc		The NVRAM store instance.
 *
 * @retval 0		success
 * @retval non-zero	If parsing fails, a regular unix error code will be
 *			returned.
 */
static int
bhnd_nvstore_parse_path_entries(struct bhnd_nvram_store *sc)
{
	const char	*name;
	void		*cookiep;
	int		 error;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Skip path registration if the data source does not support device
	 * paths. */
	if (!(sc->data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS)) {
		BHND_NV_ASSERT(sc->root_path != NULL, ("missing root path"));
		return (0);
	}

	/* Otherwise, parse and register all paths and path aliases */
	cookiep = NULL;
	while ((name = bhnd_nvram_data_next(sc->data, &cookiep))) {
		bhnd_nvstore_name_info info;

		/* Parse the name info */
		error = bhnd_nvstore_parse_name_info(name,
		    BHND_NVSTORE_NAME_INTERNAL, sc->data_caps, &info);
		if (error)
			return (error);

		/* Register the path */
		error = bhnd_nvstore_var_register_path(sc, &info, cookiep);
		if (error) {
			BHND_NV_LOG("failed to register path for %s: %d\n",
			    name, error);
			return (error);
		}
	}

	return (0);
}

/**
 * Allocate and initialize a new index instance with @p capacity.
 * 
 * The caller is responsible for deallocating the instance via
 * bhnd_nvstore_index_free().
 * 
 * @param	capacity	The maximum number of variables to be indexed.
 * 
 * @retval non-NULL	success
 * @retval NULL		if allocation failed.
 */
static bhnd_nvstore_index *
bhnd_nvstore_index_new(size_t capacity)
{
	bhnd_nvstore_index	*index;
	size_t			 bytes;

	/* Allocate and populate variable index */
	bytes = sizeof(struct bhnd_nvstore_index) + (sizeof(void *) * capacity);
	index = bhnd_nv_malloc(bytes);
	if (index == NULL) {
		BHND_NV_LOG("error allocating %zu byte index\n", bytes);
		return (NULL);
	}

	index->count = 0;
	index->capacity = capacity;

	return (index);
}

/**
 * Free an index instance, releasing all associated resources.
 * 
 * @param	index	An index instance previously allocated via
 *			bhnd_nvstore_index_new().
 */
static void
bhnd_nvstore_index_free(bhnd_nvstore_index *index)
{
	bhnd_nv_free(index);
}

/**
 * Append a new NVRAM variable's @p cookiep value to @p index.
 * 
 * After one or more append requests, the index must be prepared via
 * bhnd_nvstore_index_prepare() before any indexed lookups are performed.
 *
 * @param	sc	The NVRAM store from which NVRAM values will be queried.
 * @param	index	The index to be modified.
 * @param	cookiep	The cookiep value (as provided by the backing NVRAM
 *			data instance of @p sc) to be included in @p index.
 * 
 * @retval 0		success
 * @retval ENOMEM	if appending an additional entry would exceed the
 *			capacity of @p index.
 */
static int
bhnd_nvstore_index_append(struct bhnd_nvram_store *sc,
    bhnd_nvstore_index *index, void *cookiep)
{
	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	if (index->count >= index->capacity)
		return (ENOMEM);

	index->cookiep[index->count] = cookiep;
	index->count++;
	return (0);
}

/* sort function for bhnd_nvstore_index_prepare() */
static int
bhnd_nvstore_idx_cmp(void *ctx, const void *lhs, const void *rhs)
{
	struct bhnd_nvram_store	*sc;
	void			*l_cookiep, *r_cookiep;
	const char		*l_str, *r_str;
	const char		*l_name, *r_name;
	int			 order;

	sc = ctx;
	l_cookiep = *(void * const *)lhs;
	r_cookiep = *(void * const *)rhs;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Fetch string pointers from the cookiep values */
	l_str = bhnd_nvram_data_getvar_name(sc->data, l_cookiep);
	r_str = bhnd_nvram_data_getvar_name(sc->data, r_cookiep);

	/* Trim device path prefixes */
	if (sc->data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS) {
		l_name = bhnd_nvram_trim_path_name(l_str);
		r_name = bhnd_nvram_trim_path_name(r_str);
	} else {
		l_name = l_str;
		r_name = r_str;
	}

	/* Perform comparison */
	order = strcmp(l_name, r_name);
	if (order != 0 || lhs == rhs)
		return (order);

	/* If the backing data incorrectly contains variables with duplicate
	 * names, we need a sort order that provides stable behavior.
	 * 
	 * Since Broadcom's own code varies wildly on this question, we just
	 * use a simple precedence rule: The first declaration of a variable
	 * takes precedence. */
	return (bhnd_nvram_data_getvar_order(sc->data, l_cookiep, r_cookiep));
}

/**
 * Prepare @p index for querying via bhnd_nvstore_index_lookup().
 * 
 * After one or more append requests, the index must be prepared via
 * bhnd_nvstore_index_prepare() before any indexed lookups are performed.
 *
 * @param	sc	The NVRAM store from which NVRAM values will be queried.
 * @param	index	The index to be prepared.
 * 
 * @retval 0		success
 * @retval non-zero	if preparing @p index otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvstore_index_prepare(struct bhnd_nvram_store *sc,
    bhnd_nvstore_index *index)
{
	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Sort the index table */
	qsort_r(index->cookiep, index->count, sizeof(index->cookiep[0]), sc,
	    bhnd_nvstore_idx_cmp);

	return (0);
}

/**
 * Return a borrowed reference to the root path node.
 * 
 * @param	sc	The NVRAM store.
 */
static bhnd_nvstore_path *
bhnd_nvstore_get_root_path(struct bhnd_nvram_store *sc)
{
	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);
	return (sc->root_path);
}

/**
 * Return true if @p path is the root path node.
 * 
 * @param	sc	The NVRAM store.
 * @param	path	The path to query.
 */
static bool
bhnd_nvstore_is_root_path(struct bhnd_nvram_store *sc, bhnd_nvstore_path *path)
{
	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);
	return (sc->root_path == path);
}

/**
 * Iterate over all variable cookiep values retrievable from the backing
 * data store in @p path.
 * 
 * @warning Pending changes in @p path are ignored by this function.
 *
 * @param		sc	The NVRAM store.
 * @param		path	The NVRAM path to be iterated.
 * @param[in,out]	indexp	A pointer to an opaque indexp value previously
 *				returned by bhnd_nvstore_path_data_next(), or a
 *				NULL value to begin iteration.
 *
 * @return Returns the next variable name, or NULL if there are no more
 * variables defined in @p path.
 */
static void *
bhnd_nvstore_path_data_next(struct bhnd_nvram_store *sc,
     bhnd_nvstore_path *path, void **indexp)
{
	void **index_ref;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* No index */
	if (path->index == NULL) {
		/* An index is required for all non-empty, non-root path
		 * instances */
		BHND_NV_ASSERT(bhnd_nvstore_is_root_path(sc, path),
		    ("missing index for non-root path %s", path->path_str));

		/* Iterate NVRAM data directly, using the NVRAM data's cookiep
		 * value as our indexp context */
		if ((bhnd_nvram_data_next(sc->data, indexp)) == NULL)
			return (NULL);

		return (*indexp);
	}

	/* Empty index */
	if (path->index->count == 0)
		return (NULL);

	if (*indexp == NULL) {
		/* First index entry */
		index_ref = &path->index->cookiep[0];
	} else {
		size_t idxpos;

		/* Advance to next index entry */
		index_ref = *indexp;
		index_ref++;

		/* Hit end of index? */
		BHND_NV_ASSERT(index_ref > path->index->cookiep,
		    ("invalid indexp"));

		idxpos = (index_ref - path->index->cookiep);
		if (idxpos >= path->index->count)
			return (NULL);
	}

	/* Provide new index position */
	*indexp = index_ref;

	/* Return the data's cookiep value */
	return (*index_ref);
}

/**
 * Perform an lookup of @p name in the backing NVRAM data for @p path,
 * returning the associated cookiep value, or NULL if the variable is not found
 * in the backing NVRAM data.
 * 
 * @warning Pending changes in @p path are ignored by this function.
 * 
 * @param	sc	The NVRAM store from which NVRAM values will be queried.
 * @param	path	The path to be queried.
 * @param	name	The variable name to be queried.
 * 
 * @retval non-NULL	success
 * @retval NULL		if @p name is not found in @p index.
 */
static void *
bhnd_nvstore_path_data_lookup(struct bhnd_nvram_store *sc,
    bhnd_nvstore_path *path, const char *name)
{
	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* No index */
	if (path->index == NULL) {
		/* An index is required for all non-empty, non-root path
		 * instances */
		BHND_NV_ASSERT(bhnd_nvstore_is_root_path(sc, path),
		    ("missing index for non-root path %s", path->path_str));

		/* Look up directly in NVRAM data */
		return (bhnd_nvram_data_find(sc->data, name));
	}

	/* Otherwise, delegate to an index-based lookup */
	return (bhnd_nvstore_index_lookup(sc, path->index, name));
}

/**
 * Perform an index lookup of @p name, returning the associated cookiep
 * value, or NULL if the variable does not exist.
 * 
 * @param	sc	The NVRAM store from which NVRAM values will be queried.
 * @param	index	The index to be queried.
 * @param	name	The variable name to be queried.
 * 
 * @retval non-NULL	success
 * @retval NULL		if @p name is not found in @p index.
 */
static void *
bhnd_nvstore_index_lookup(struct bhnd_nvram_store *sc,
    bhnd_nvstore_index *index, const char *name)
{
	void		*cookiep;
	const char	*indexed_name;
	size_t		 min, mid, max;
	uint32_t	 data_caps;
	int		 order;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);
	BHND_NV_ASSERT(index != NULL, ("NULL index"));

	/*
	 * Locate the requested variable using a binary search.
	 */
	if (index->count == 0)
		return (NULL);

	data_caps = sc->data_caps;
	min = 0;
	max = index->count - 1;

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;
		cookiep = index->cookiep[mid];

		/* Fetch variable name */
		indexed_name = bhnd_nvram_data_getvar_name(sc->data, cookiep);

		/* Trim any path prefix */
		if (data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS)
			indexed_name = bhnd_nvram_trim_path_name(indexed_name);

		/* Determine which side of the partition to search */
		order = strcmp(indexed_name, name);
		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search (non-empty) lower partition */
			if (mid == 0)
				break;
			max = mid - 1;
		} else if (order == 0) {
			size_t	idx;

			/*
			 * Match found.
			 * 
			 * If this happens to be a key with multiple definitions
			 * in the backing store, we need to find the entry with
			 * the highest declaration precedence.
			 * 
			 * Duplicates are sorted in order of descending
			 * precedence; to find the highest precedence entry,
			 * we search backwards through the index.
			 */
			idx = mid;
			while (idx > 0) {
				void		*dup_cookiep;
				const char	*dup_name;

				/* Fetch preceding index entry */
				idx--;
				dup_cookiep = index->cookiep[idx];
				dup_name = bhnd_nvram_data_getvar_name(sc->data,
				    dup_cookiep);

				/* Trim any path prefix */
				if (data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS) {
					dup_name = bhnd_nvram_trim_path_name(
					    dup_name);
				}

				/* If no match, current cookiep is the variable
				 * definition with the highest precedence */
				if (strcmp(indexed_name, dup_name) != 0)
					return (cookiep);

				/* Otherwise, prefer this earlier definition,
				 * and keep searching for a higher-precedence
				 * definitions */
				cookiep = dup_cookiep;
			}

			return (cookiep);
		}
	}

	/* Not found */
	return (NULL);
}

/**
 * Read an NVRAM variable.
 *
 * @param		sc	The NVRAM parser state.
 * @param		name	The NVRAM variable name.
 * @param[out]		outp	On success, the requested value will be written
 *				to this buffer. This argment may be NULL if
 *				the value is not desired.
 * @param[in,out]	olen	The capacity of @p outp. On success, will be set
 *				to the actual size of the requested value.
 * @param		otype	The requested data type to be written to
 *				@p outp.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable was not found.
 * @retval ENOMEM	If @p outp is non-NULL and a buffer of @p olen is too
 *			small to hold the requested value.
 * @retval non-zero	If reading @p name otherwise fails, a regular unix
 *			error code will be returned.
  */
int
bhnd_nvram_store_getvar(struct bhnd_nvram_store *sc, const char *name,
    void *outp, size_t *olen, bhnd_nvram_type otype)
{
	bhnd_nvstore_name_info	 info;
	bhnd_nvstore_path	*path;
	bhnd_nvram_prop		*prop;
	void			*cookiep;
	int			 error;

	BHND_NVSTORE_LOCK(sc);

	/* Parse the variable name */
	error = bhnd_nvstore_parse_name_info(name, BHND_NVSTORE_NAME_EXTERNAL,
	    sc->data_caps, &info);
	if (error)
		goto finished;

	/* Fetch the variable's enclosing path entry */
	if ((path = bhnd_nvstore_var_get_path(sc, &info)) == NULL) {
		error = ENOENT;
		goto finished;
	}

	/* Search uncommitted changes first */
	prop = bhnd_nvram_plist_get_prop(path->pending, info.name);
	if (prop != NULL) {
		/* Found in uncommitted change list */
		if (bhnd_nvram_prop_type(prop) == BHND_NVRAM_TYPE_NULL) {
			/* NULL property values denote a pending deletion */
			error = ENOENT;
		} else {
			error = bhnd_nvram_prop_encode(prop, outp, olen, otype);
		}

		goto finished;
	}

	/* Search the backing NVRAM data */
	cookiep = bhnd_nvstore_path_data_lookup(sc, path, info.name);
	if (cookiep != NULL) {
		/* Found in backing store */
		error = bhnd_nvram_data_getvar(sc->data, cookiep, outp, olen,
		     otype);
		goto finished;
	}

	/* Not found */
	error = ENOENT;

finished:
	BHND_NVSTORE_UNLOCK(sc);
	return (error);
}

static int
bhnd_nvram_store_setval_common(struct bhnd_nvram_store *sc, const char *name,
    bhnd_nvram_val *value)
{
	bhnd_nvram_val		*prop_val;
	bhnd_nvstore_path	*path;
	const char		*full_name;
	char			*namebuf;
	void			*cookiep;
	bhnd_nvstore_name_info	 info;
	int			 error;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	prop_val = NULL;
	namebuf = NULL;

	/* Parse the variable name */
	error = bhnd_nvstore_parse_name_info(name, BHND_NVSTORE_NAME_EXTERNAL,
	    sc->data_caps, &info);
	if (error)
		return (error);

	/* Fetch the variable's enclosing path entry */
	if ((path = bhnd_nvstore_var_get_path(sc, &info)) == NULL)
		return (error);

	/* Determine the full path-prefixed name for the variable */
	cookiep = bhnd_nvstore_index_lookup(sc, path->index, info.name);
	if (cookiep != NULL) {
		/* Variable is already defined in the backing data; use the
		 * existing variable name */
		full_name = bhnd_nvram_data_getvar_name(sc->data, cookiep);
	} else if (path == sc->root_path) {
		/* No prefix required for root path */
		full_name = name;
	} else {
		bhnd_nvstore_alias	*alias;
		int			 len;

		/* New variable is being set; we need to determine the
		 * appropriate path prefix */
		alias = bhnd_nvstore_find_alias(sc, path->path_str);
		if (alias != NULL) {
			/* Use <alias>:name */
			len = bhnd_nv_asprintf(&namebuf, "%lu:%s", alias->alias,
			    info.name);
		} else {
			/* Use path/name */
			len = bhnd_nv_asprintf(&namebuf, "%s/%s",
			    path->path_str, info.name);
		}

		if (len < 0)
			return (ENOMEM);

		full_name = namebuf;
	}

	/* Allow the data store to filter the NVRAM value */
	error = bhnd_nvram_data_filter_setvar(sc->data, full_name, value,
	    &prop_val);
	if (error) {
		BHND_NV_LOG("cannot set property %s: %d\n", full_name, error);
		goto cleanup;
	}

	/* Add relative variable name to the per-path change list */
	error = bhnd_nvram_plist_replace_val(path->pending, info.name,
	    prop_val);
	if (error)
		goto cleanup;

	/* Success */
	error = 0;

cleanup:
	if (namebuf != NULL)
		bhnd_nv_free(namebuf);

	if (prop_val != NULL)
		bhnd_nvram_val_release(prop_val);

	return (error);
}

/**
 * Set an NVRAM variable.
 * 
 * @param	sc	The NVRAM parser state.
 * @param	name	The NVRAM variable name.
 * @param	value	The new value.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable @p name was not found.
 * @retval EINVAL	If @p value is invalid.
 */
int
bhnd_nvram_store_setval(struct bhnd_nvram_store *sc, const char *name,
    bhnd_nvram_val *value)
{
	int error;

	BHND_NVSTORE_LOCK(sc);
	error = bhnd_nvram_store_setval_common(sc, name, value);
	BHND_NVSTORE_UNLOCK(sc);

	return (error);
}

/**
 * Set an NVRAM variable.
 * 
 * @param		sc	The NVRAM parser state.
 * @param		name	The NVRAM variable name.
 * @param[out]		inp	The new value.
 * @param[in,out]	ilen	The size of @p inp.
 * @param		itype	The data type of @p inp.
 *
 * @retval 0		success
 * @retval ENOENT	The requested variable @p name was not found.
 * @retval EINVAL	If the new value is invalid.
 */
int
bhnd_nvram_store_setvar(struct bhnd_nvram_store *sc, const char *name,
    const void *inp, size_t ilen, bhnd_nvram_type itype)
{
	bhnd_nvram_val	val;
	int		error;

	error = bhnd_nvram_val_init(&val, NULL, inp, ilen, itype,
	    BHND_NVRAM_VAL_FIXED|BHND_NVRAM_VAL_BORROW_DATA);
	if (error) {
		BHND_NV_LOG("error initializing value: %d\n", error);
		return (EINVAL);
	}

	BHND_NVSTORE_LOCK(sc);
	error = bhnd_nvram_store_setval_common(sc, name, &val);
	BHND_NVSTORE_UNLOCK(sc);

	bhnd_nvram_val_release(&val);

	return (error);
}

/**
 * Return the device path entry registered for @p path, if any.
 * 
 * @param	sc		The NVRAM store to be queried.
 * @param	path		The device path to search for.
 * @param	path_len	The length of @p path.
 *
 * @retval non-NULL	if found.
 * @retval NULL		if not found.
 */
static bhnd_nvstore_path *
bhnd_nvstore_get_path(struct bhnd_nvram_store *sc, const char *path,
    size_t path_len)
{
	bhnd_nvstore_path_list	*plist;
	bhnd_nvstore_path	*p;
	uint32_t		 h;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Use hash lookup */
	h = hash32_strn(path, path_len, HASHINIT);
	plist = &sc->paths[h % nitems(sc->paths)];

	LIST_FOREACH(p, plist, np_link) {
		/* Check for prefix match */
		if (strncmp(p->path_str, path, path_len) != 0)
			continue;

		/* Check for complete match */
		if (strnlen(path, path_len) != strlen(p->path_str))
			continue;

		return (p);
	}

	/* Not found */
	return (NULL);
}

/**
 * Resolve @p aval to its corresponding device path entry, if any.
 * 
 * @param	sc		The NVRAM store to be queried.
 * @param	aval		The device path alias value to search for.
 *
 * @retval non-NULL	if found.
 * @retval NULL		if not found.
 */
static bhnd_nvstore_path *
bhnd_nvstore_resolve_path_alias(struct bhnd_nvram_store *sc, u_long aval)
{
	bhnd_nvstore_alias *alias;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Fetch alias entry */
	if ((alias = bhnd_nvstore_get_alias(sc, aval)) == NULL)
		return (NULL);

	return (alias->path);
}

/**
 * Register a device path entry for the path referenced by variable name
 * @p info, if any.
 *
 * @param	sc		The NVRAM store to be updated.
 * @param	info		The NVRAM variable name info.
 * @param	cookiep		The NVRAM variable's cookiep value.
 *
 * @retval 0		if the path was successfully registered, or an identical
 *			path or alias entry exists.
 * @retval EEXIST	if a conflicting entry already exists for the path or
 *			alias referenced by @p info.
 * @retval ENOENT	if @p info contains a dangling alias reference.
 * @retval EINVAL	if @p info contains an unsupported bhnd_nvstore_var_type
 *			and bhnd_nvstore_path_type combination.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvstore_var_register_path(struct bhnd_nvram_store *sc,
    bhnd_nvstore_name_info *info, void *cookiep)
{
	switch (info->type) {
	case BHND_NVSTORE_VAR:
		/* Variable */
		switch (info->path_type) {
		case BHND_NVSTORE_PATH_STRING:
			/* Variable contains a full path string
			 * (pci/1/1/varname); register the path */
			return (bhnd_nvstore_register_path(sc,
			    info->path.str.value, info->path.str.value_len));

		case BHND_NVSTORE_PATH_ALIAS:
			/* Variable contains an alias reference (0:varname).
			 * There's no path to register */
			return (0);
		}

		BHND_NV_PANIC("unsupported path type %d", info->path_type);
		break;

	case BHND_NVSTORE_ALIAS_DECL:
		/* Alias declaration */
		return (bhnd_nvstore_register_alias(sc, info, cookiep));
	}

	BHND_NV_PANIC("unsupported var type %d", info->type);
}

/**
 * Resolve the device path entry referenced referenced by @p info.
 *
 * @param	sc		The NVRAM store to be updated.
 * @param	info		Variable name information descriptor containing
 *				the path or path alias to be resolved.
 *
 * @retval non-NULL	if found.
 * @retval NULL		if not found.
 */
static bhnd_nvstore_path *
bhnd_nvstore_var_get_path(struct bhnd_nvram_store *sc,
    bhnd_nvstore_name_info *info)
{
	switch (info->path_type) {
	case BHND_NVSTORE_PATH_STRING:
		return (bhnd_nvstore_get_path(sc, info->path.str.value,
		    info->path.str.value_len));
	case BHND_NVSTORE_PATH_ALIAS:
		return (bhnd_nvstore_resolve_path_alias(sc,
		    info->path.alias.value));
	}

	BHND_NV_PANIC("unsupported path type %d", info->path_type);
}

/**
 * Return the device path alias entry registered for @p alias_val, if any.
 * 
 * @param	sc		The NVRAM store to be queried.
 * @param	alias_val	The alias value to search for.
 *
 * @retval non-NULL	if found.
 * @retval NULL		if not found.
 */
static bhnd_nvstore_alias *
bhnd_nvstore_get_alias(struct bhnd_nvram_store *sc, u_long alias_val)
{
	bhnd_nvstore_alias_list	*alist;
	bhnd_nvstore_alias	*alias;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Can use hash lookup */
	alist = &sc->aliases[alias_val % nitems(sc->aliases)];
	LIST_FOREACH(alias, alist, na_link) {
		if (alias->alias == alias_val)
			return (alias);			
	}

	/* Not found */
	return (NULL);
}

/**
 * Return the device path alias entry registered for @p path, if any.
 * 
 * @param	sc	The NVRAM store to be queried.
 * @param	path	The alias path to search for.
 *
 * @retval non-NULL	if found.
 * @retval NULL		if not found.
 */
static bhnd_nvstore_alias *
bhnd_nvstore_find_alias(struct bhnd_nvram_store *sc, const char *path)
{
	bhnd_nvstore_alias *alias;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Have to scan the full table */
	for (size_t i = 0; i < nitems(sc->aliases); i++) {
		LIST_FOREACH(alias, &sc->aliases[i], na_link) {
			if (strcmp(alias->path->path_str, path) == 0)
				return (alias);			
		}
	}

	/* Not found */
	return (NULL);
}

/**
 * Register a device path entry for @p path.
 * 
 * @param	sc		The NVRAM store to be updated.
 * @param	path_str	The absolute device path string.
 * @param	path_slen	The length of @p path_str.
 * 
 * @retval 0		if the path was successfully registered, or an identical
 *			path/alias entry already exists.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvstore_register_path(struct bhnd_nvram_store *sc, const char *path_str,
    size_t path_slen)
{
	bhnd_nvstore_path_list	*plist;
	bhnd_nvstore_path	*path;
	uint32_t		 h;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	/* Can't represent more than SIZE_MAX paths */
	if (sc->num_paths == SIZE_MAX)
		return (ENOMEM);

	/* Already exists? */
	if (bhnd_nvstore_get_path(sc, path_str, path_slen) != NULL)
		return (0);

	/* Allocate new entry */
	path = bhnd_nv_malloc(sizeof(*path));
	if (path == NULL)
		return (ENOMEM);

	path->index = NULL;
	path->num_vars = 0;

	if ((path->pending = bhnd_nvram_plist_new()) == NULL) {
		bhnd_nv_free(path);
		return (ENOMEM);
	}

	if ((path->path_str = bhnd_nv_strndup(path_str, path_slen)) == NULL) {
		bhnd_nvram_plist_release(path->pending);
		bhnd_nv_free(path);
		return (ENOMEM);
	}

	/* Insert in path hash table */
	h = hash32_str(path->path_str, HASHINIT);
	plist = &sc->paths[h % nitems(sc->paths)];
	LIST_INSERT_HEAD(plist, path, np_link);

	/* Increment path count */
	sc->num_paths++;

	return (0);
}

/**
 * Register a device path alias for an NVRAM 'devpathX' variable.
 * 
 * The path value for the alias will be fetched from the backing NVRAM data.
 * 
 * @param	sc	The NVRAM store to be updated.
 * @param	info	The NVRAM variable name info.
 * @param	cookiep	The NVRAM variable's cookiep value.
 * 
 * @retval 0		if the alias was successfully registered, or an
 *			identical alias entry exists.
 * @retval EEXIST	if a conflicting alias or path entry already exists.
 * @retval EINVAL	if @p info is not a BHND_NVSTORE_ALIAS_DECL or does
 *			not contain a BHND_NVSTORE_PATH_ALIAS entry.
 * @retval ENOMEM	if allocation fails.
 */
static int
bhnd_nvstore_register_alias(struct bhnd_nvram_store *sc,
    const bhnd_nvstore_name_info *info, void *cookiep)
{
	bhnd_nvstore_alias_list	*alist;
	bhnd_nvstore_alias	*alias;
	bhnd_nvstore_path	*path;
	char			*path_str;
	size_t			 path_slen;
	int			 error;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	path_str = NULL;
	alias = NULL;

	/* Can't represent more than SIZE_MAX aliases */
	if (sc->num_aliases == SIZE_MAX)
		return (ENOMEM);

	/* Must be an alias declaration */
	if (info->type != BHND_NVSTORE_ALIAS_DECL)
		return (EINVAL);

	if (info->path_type != BHND_NVSTORE_PATH_ALIAS)
		return (EINVAL);

	/* Fetch the devpath variable's value length */
	error = bhnd_nvram_data_getvar(sc->data, cookiep, NULL, &path_slen,
	    BHND_NVRAM_TYPE_STRING);
	if (error)
		return (ENOMEM);

	/* Allocate path string buffer */
	if ((path_str = bhnd_nv_malloc(path_slen)) == NULL)
		return (ENOMEM);

	/* Decode to our new buffer */
	error = bhnd_nvram_data_getvar(sc->data, cookiep, path_str, &path_slen,
	    BHND_NVRAM_TYPE_STRING);
	if (error)
		goto failed;

	/* Trim trailing '/' character(s) from the path length */
	path_slen = strnlen(path_str, path_slen);
	while (path_slen > 0 && path_str[path_slen-1] == '/') {
		path_str[path_slen-1] = '\0';
		path_slen--;
	}

	/* Is a conflicting alias entry already registered for this alias
	 * value? */
	alias = bhnd_nvstore_get_alias(sc, info->path.alias.value);
	if (alias != NULL) {
		if (alias->cookiep != cookiep ||
		    strcmp(alias->path->path_str, path_str) != 0)
		{
			error = EEXIST;
			goto failed;
		}
	}

	/* Is a conflicting entry already registered for the alias path? */
	if ((alias = bhnd_nvstore_find_alias(sc, path_str)) != NULL) {
		if (alias->alias != info->path.alias.value ||
		    alias->cookiep != cookiep ||
		    strcmp(alias->path->path_str, path_str) != 0)
		{
			error = EEXIST;
			goto failed;
		}
	}

	/* Get (or register) the target path entry */
	path = bhnd_nvstore_get_path(sc, path_str, path_slen);
	if (path == NULL) {
		error = bhnd_nvstore_register_path(sc, path_str, path_slen);
		if (error)
			goto failed;

		path = bhnd_nvstore_get_path(sc, path_str, path_slen);
		BHND_NV_ASSERT(path != NULL, ("missing registered path"));
	}

	/* Allocate alias entry */
	alias = bhnd_nv_calloc(1, sizeof(*alias));
	if (alias == NULL) {
		error = ENOMEM;
		goto failed;
	}

	alias->path = path;
	alias->cookiep = cookiep;
	alias->alias = info->path.alias.value;

	/* Insert in alias hash table */
	alist = &sc->aliases[alias->alias % nitems(sc->aliases)];
	LIST_INSERT_HEAD(alist, alias, na_link);

	/* Increment alias count */
	sc->num_aliases++;

	bhnd_nv_free(path_str);
	return (0);

failed:
	if (path_str != NULL)
		bhnd_nv_free(path_str);

	if (alias != NULL)
		bhnd_nv_free(alias);

	return (error);
}

/**
 * If @p child is equal to or a child path of @p parent, return a pointer to
 * @p child's path component(s) relative to @p parent; otherwise, return NULL.
 */
static const char *
bhnd_nvstore_parse_relpath(const char *parent, const char *child)
{
	size_t prefix_len;

	/* All paths have an implicit leading '/'; this allows us to treat
	 * our manufactured root path of "/" as a prefix to all NVRAM-defined
	 * paths (which do not necessarily include a leading '/' */
	if (*parent == '/')
		parent++;

	if (*child == '/')
		child++;

	/* Is parent a prefix of child? */
	prefix_len = strlen(parent);
	if (strncmp(parent, child, prefix_len) != 0)
		return (NULL);

	/* A zero-length prefix matches everything */
	if (prefix_len == 0)
		return (child);

	/* Is child equal to parent? */
	if (child[prefix_len] == '\0')
		return (child + prefix_len);

	/* Is child actually a child of parent? */
	if (child[prefix_len] == '/')
		return (child + prefix_len + 1);

	/* No match (e.g. parent=/foo..., child=/fooo...) */
	return (NULL);
}

/**
 * Parse a raw NVRAM variable name and return its @p entry_type, its
 * type-specific @p prefix (e.g. '0:', 'pci/1/1', 'devpath'), and its
 * type-specific @p suffix (e.g. 'varname', '0').
 * 
 * @param	name		The NVRAM variable name to be parsed. This
 *				value must remain valid for the lifetime of
 *				@p info.
 * @param	type		The NVRAM name type -- either INTERNAL for names
 *				parsed from backing NVRAM data, or EXTERNAL for
 *				names provided by external NVRAM store clients.
 * @param	data_caps	The backing NVRAM data capabilities
 *				(see bhnd_nvram_data_caps()).
 * @param[out]	info		On success, the parsed variable name info.
 * 
 * @retval 0		success
 * @retval non-zero	if parsing @p name otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvstore_parse_name_info(const char *name, bhnd_nvstore_name_type type,
    uint32_t data_caps, bhnd_nvstore_name_info *info)
{
	const char	*p;
	char		*endp;

	/* Skip path parsing? */
	if (data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS) {
		/* devpath declaration? (devpath0=pci/1/1) */
		if (strncmp(name, "devpath", strlen("devpath")) == 0) {
			u_long alias;

			/* Perform standard validation on the relative
			 * variable name */
			if (type != BHND_NVSTORE_NAME_INTERNAL &&
			    !bhnd_nvram_validate_name(name))
			{
				return (ENOENT);
			}

			/* Parse alias value that should follow a 'devpath'
			 * prefix */
			p = name + strlen("devpath");
			alias = strtoul(p, &endp, 10);
			if (endp != p && *endp == '\0') {
				info->type = BHND_NVSTORE_ALIAS_DECL;
				info->path_type = BHND_NVSTORE_PATH_ALIAS;
				info->name = name;
				info->path.alias.value = alias;

				return (0);
			}
		}

		/* device aliased variable? (0:varname) */
		if (bhnd_nv_isdigit(*name)) {
			u_long alias;

			/* Parse '0:' alias prefix */
			alias = strtoul(name, &endp, 10);
			if (endp != name && *endp == ':') {
				/* Perform standard validation on the relative
				 * variable name */
				if (type != BHND_NVSTORE_NAME_INTERNAL &&
				    !bhnd_nvram_validate_name(name))
				{
					return (ENOENT);
				}

				info->type = BHND_NVSTORE_VAR;
				info->path_type = BHND_NVSTORE_PATH_ALIAS;

				/* name follows 0: prefix */
				info->name = endp + 1;
				info->path.alias.value = alias;

				return (0);
			}
		}

		/* device variable? (pci/1/1/varname) */
		if ((p = strrchr(name, '/')) != NULL) {
			const char	*path, *relative_name;
			size_t		 path_len;

			/* Determine the path length; 'p' points at the last
			 * path separator in 'name' */
			path_len = p - name;
			path = name;

			/* The relative variable name directly follows the
			 * final path separator '/' */
			relative_name = path + path_len + 1;

			/* Now that we calculated the name offset, exclude all
			 * trailing '/' characters from the path length */
			while (path_len > 0 && path[path_len-1] == '/')
				path_len--;

			/* Perform standard validation on the relative
			 * variable name */
			if (type != BHND_NVSTORE_NAME_INTERNAL &&
			    !bhnd_nvram_validate_name(relative_name))
			{
				return (ENOENT);
			}

			/* Initialize result with pointers into the name
			 * buffer */
			info->type = BHND_NVSTORE_VAR;
			info->path_type = BHND_NVSTORE_PATH_STRING;
			info->name = relative_name;
			info->path.str.value = path;
			info->path.str.value_len = path_len;

			return (0);
		}
	}

	/* If all other parsing fails, the result is a simple variable with
	 * an implicit path of "/" */
	if (type != BHND_NVSTORE_NAME_INTERNAL &&
	    !bhnd_nvram_validate_name(name))
	{
		/* Invalid relative name */
		return (ENOENT);
	}

	info->type = BHND_NVSTORE_VAR;
	info->path_type = BHND_NVSTORE_PATH_STRING;
	info->name = name;
	info->path.str.value = BHND_NVSTORE_ROOT_PATH;
	info->path.str.value_len = BHND_NVSTORE_ROOT_PATH_LEN;

	return (0);
}
