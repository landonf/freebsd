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

#include <sys/ctype.h>
#include <sys/hash.h>
#include <sys/queue.h>
#include <sys/systm.h>

#include <machine/_inttypes.h>

#include "bhnd_nvram_private.h"
#include "bhnd_nvram_datavar.h"
#include "bhnd_nvram_storevar.h"

#include "bhnd_nvram_prov_if.h"

/*
 * BHND NVRAM Store
 * 
 * Vends an NVRAM provider interface to a backing bhnd_nvram_data instance.
 */

static void			 bhnd_nvram_store_fini(
				     struct bhnd_nvram_provider *prov);

static int			 bhnd_nvstore_parse_data(
				     struct bhnd_nvram_store *sc);

static int			 bhnd_nvstore_parse_path_entries(
				     struct bhnd_nvram_store *sc);

static int			 bhnd_nvstore_export_path(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_path *path,
				     bhnd_nvram_data_class **cls,
				     bhnd_nvram_plist **props,
				     bhnd_nvram_plist **options,
				     uint32_t flags);

static int			 bhnd_nvram_store_export_child(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_path *top,
				     bhnd_nvstore_path *child,
				     bhnd_nvram_plist *plist,
				     uint32_t flags);

static int			 bhnd_nvstore_export_merge(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_path *path,
				     bhnd_nvram_plist *merged,
				     uint32_t flags);

static int			 bhnd_nvstore_export_devpath_alias(
				     struct bhnd_nvram_store *sc,
				     bhnd_nvstore_path *path,
				     const char *devpath,
				     bhnd_nvram_plist *plist,
				     u_long *alias_val);

static int
bhnd_nvram_store_init(struct bhnd_nvram_provider *prov, const void *params)
{
	struct bhnd_nvram_store				*sc;
	const struct bhnd_nvram_store_init_params	*init_params;
	int						 error;

	// TODO: save dev reference

	sc = bhnd_nvram_provider_get_softc(prov);
	init_params = params;

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
	sc->data = bhnd_nvram_data_retain(init_params->data);
	sc->data_caps = bhnd_nvram_data_caps(init_params->data);
	sc->data_opts = bhnd_nvram_data_options(init_params->data);
	if (sc->data_opts != NULL) {
		bhnd_nvram_plist_retain(sc->data_opts);
	} else {
		sc->data_opts = bhnd_nvram_plist_new();
		if (sc->data_opts == NULL) {
			error = ENOMEM;
			goto cleanup;
		}
	}

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

	BHND_NVSTORE_UNLOCK(sc);
	return (0);

cleanup:
	BHND_NVSTORE_UNLOCK(sc);
	bhnd_nvram_store_fini(prov);
	return (error);
}

static void
bhnd_nvram_store_fini(struct bhnd_nvram_provider *prov)
{
	struct bhnd_nvram_store *sc;

	sc = bhnd_nvram_provider_get_softc(prov);

	/* Clean up alias hash table */
	for (size_t i = 0; i < nitems(sc->aliases); i++) {
		bhnd_nvstore_alias *alias, *anext;
		LIST_FOREACH_SAFE(alias, &sc->aliases[i], na_link, anext)
			bhnd_nv_free(alias);
	}

	/* Clean up path hash table */
	for (size_t i = 0; i < nitems(sc->paths); i++) {
		bhnd_nvstore_path *path, *pnext;
		LIST_FOREACH_SAFE(path, &sc->paths[i], np_link, pnext)
			bhnd_nvstore_path_release(path);
	}

	if (sc->data != NULL)
		bhnd_nvram_data_release(sc->data);

	if (sc->data_opts != NULL)
		bhnd_nvram_plist_release(sc->data_opts);

	BHND_NVSTORE_LOCK_DESTROY(sc);
}

static int
bhnd_nvram_store_sync(struct bhnd_nvram_provider *prov, bool forced)
{
	struct bhnd_nvram_store *sc;

	sc = bhnd_nvram_provider_get_softc(prov);

	BHND_NVSTORE_LOCK(sc);
	if (sc->dev == NULL) {
		BHND_NVSTORE_UNLOCK(sc);
		return (ENXIO);
	}

	// TODO: request sync
	panic("unimplemented");

	BHND_NVSTORE_UNLOCK(sc);
	return (0);
}

static int
bhnd_nvram_store_open_path(bhnd_nvram_provider *prov, const char *pathname,
    size_t pathlen, bhnd_nvram_phandle *phandle)
{
	struct bhnd_nvram_store	*sc;
	bhnd_nvstore_path	*path;

	sc = bhnd_nvram_provider_get_softc(prov);

	pathname = bhnd_nvstore_parse_external_path(pathname, &pathlen);
	if (pathname == NULL)
		return (ENOENT);

	BHND_NVSTORE_LOCK(sc);
	if ((path = bhnd_nvstore_get_path(sc, pathname, pathlen)) != NULL) {
		bhnd_nvstore_path_retain(path);
	}
	BHND_NVSTORE_UNLOCK(sc);

	if (path == NULL)
		return (ENOENT);

	*phandle = (bhnd_nvram_phandle)path;
	return (0);
}

static void
bhnd_nvram_store_retain_path(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle phandle)
{
	bhnd_nvstore_path_retain((bhnd_nvstore_path *)phandle);
}

static void
bhnd_nvram_store_release_path(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle phandle)
{
	bhnd_nvstore_path_release((bhnd_nvstore_path *)phandle);
}

static int
bhnd_nvram_store_get_pathname(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle phandle, char *buf, size_t *len)
{
	struct bhnd_nvram_store	*sc;
        bhnd_nvstore_path       *path;
	size_t			 limit;

	sc = bhnd_nvram_provider_get_softc(prov);
	path = (bhnd_nvstore_path *)phandle;

	limit = *len;

	BHND_NVSTORE_LOCK(sc);

	/* Determine required buffer size */
	*len = strlen(path->qual_path_str) + 1;

	if (buf == NULL) {
		BHND_NVSTORE_UNLOCK(sc);
		return (0);
	} else if (*len > limit) {
		BHND_NVSTORE_UNLOCK(sc);
		return (ENOMEM);
	}

	strcpy(buf, path->qual_path_str);

	BHND_NVSTORE_UNLOCK(sc);

	return (0);
}

static int
bhnd_nvram_store_get_children(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle phandle, bhnd_nvram_phandle **children, size_t *count)
{
	// TODO
	return (ENXIO);
}

static void
bhnd_nvram_store_free_children(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle *children, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		bhnd_nvstore_path *path;

		path = (bhnd_nvstore_path *)children[i];
		bhnd_nvstore_path_release(path);
	}

	bhnd_nv_free(children);
}

static int
bhnd_nvram_store_setprop(bhnd_nvram_provider *prov, bhnd_nvram_phandle phandle,
    const char *propname, const void *buf, size_t len, bhnd_nvram_type type)
{
	struct bhnd_nvram_store	*sc;
	bhnd_nvstore_path       *path;
	bhnd_nvram_val		 val;
	int			 error;

	sc = bhnd_nvram_provider_get_softc(prov);
	path = (bhnd_nvstore_path *)phandle;

	if (!bhnd_nvram_validate_name(propname))
		return (ENOATTR);

	/* Wrap the property value */
	error = bhnd_nvram_val_init(&val, NULL, buf, len, type,
	    BHND_NVRAM_VAL_FIXED|BHND_NVRAM_VAL_BORROW_DATA);
	if (error) {
		BHND_NV_LOG("error initializing value: %d\n", error);
		return (EINVAL);
	}

	/* Register the update */
	BHND_NVSTORE_LOCK(sc);
	error = bhnd_nvstore_path_register_update(sc, path, propname, &val);
	BHND_NVSTORE_UNLOCK(sc);

	bhnd_nvram_val_release(&val);

	return (error);
}

static int
bhnd_nvram_store_delprop(bhnd_nvram_provider *prov, bhnd_nvram_phandle phandle,
    const char *propname)
{
	struct bhnd_nvram_store	*sc;
        bhnd_nvstore_path       *path;
	bhnd_nvram_val		 val;
        int                      error;

	sc = bhnd_nvram_provider_get_softc(prov);
	path = (bhnd_nvstore_path *)phandle;

	if (!bhnd_nvram_validate_name(propname))
		return (ENOATTR);

	/* Register the update */
	BHND_NVSTORE_LOCK(sc);
	error = bhnd_nvstore_path_register_update(sc, path, propname,
	    BHND_NVRAM_VAL_NULL);
	BHND_NVSTORE_UNLOCK(sc);

	bhnd_nvram_val_release(&val);

	return (error);
}

static int
bhnd_nvram_store_getprop(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle phandle, const char *propname, void *buf, size_t *len,
    bhnd_nvram_type type, bool search_parents)
{
	struct bhnd_nvram_store	*sc;
        bhnd_nvstore_path       *path;
	bhnd_nvram_prop		*prop;
	void			*cookiep;
        int                      error;

	sc = bhnd_nvram_provider_get_softc(prov);
	path = (bhnd_nvstore_path *)phandle;

	BHND_NVSTORE_LOCK(sc);

	do {
		const char	*parent;
		size_t		 plen;

		error = ENOATTR;

		/* Search uncommitted updates first */
		prop = bhnd_nvstore_path_get_update(sc, path, propname);
		if (prop != NULL) {
			if (bhnd_nvram_prop_is_null(prop)) {
				/* NULL values denote a pending deletion */
				error = ENOATTR;
			} else {
				error = bhnd_nvram_prop_encode(prop, buf, len,
				    type);
			}

			break;
		}

		/* Search the backing NVRAM data */
		cookiep = bhnd_nvstore_path_data_lookup(sc, path, propname);
		if (cookiep != NULL) {
			/* Found in backing store */
			error = bhnd_nvram_data_getvar(sc->data, cookiep, buf,
			    len, type);
			break;
		}

		/* Not found -- recursively search parent? */
		if (!search_parents)
			break;

		parent = path->path_str;
		plen = bhnd_nvram_parse_path_dirlen(parent, strlen(parent));
		path = bhnd_nvstore_get_path(sc, parent, plen);
	} while (path != NULL);

	BHND_NVSTORE_UNLOCK(sc);
	return (error);
}

static int
bhnd_nvram_store_copyprops(bhnd_nvram_provider *prov,
    bhnd_nvram_phandle phandle, struct bhnd_nvram_plist **plist)
{
	struct bhnd_nvram_store	*sc;
        bhnd_nvstore_path       *path;
        int                      error;

	sc = bhnd_nvram_provider_get_softc(prov);
	path = (bhnd_nvstore_path *)phandle;

	BHND_NVSTORE_LOCK(sc);
	error = bhnd_nvstore_export_path(sc, path, NULL, plist, NULL,
	    BHND_NVSTORE_EXPORT_COMMITTED|BHND_NVSTORE_EXPORT_UNCOMMITTED);
	BHND_NVSTORE_UNLOCK(sc);

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
		error = bhnd_nvstore_parse_name_info(name, sc->data_caps,
		    &info);
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
		error = bhnd_nvstore_parse_name_info(name, sc->data_caps,
		    &info);
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
		printf("SKIP PATH PARSING");
		BHND_NV_ASSERT(sc->root_path != NULL, ("missing root path"));
		return (0);
	}

	/* Otherwise, parse and register all paths and path aliases */
	cookiep = NULL;
	while ((name = bhnd_nvram_data_next(sc->data, &cookiep))) {
		bhnd_nvstore_name_info info;

		/* Parse the name info */
		error = bhnd_nvstore_parse_name_info(name, sc->data_caps,
		    &info);
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
bhnd_nvstore_export_merge(struct bhnd_nvram_store *sc,
    bhnd_nvstore_path *path, bhnd_nvram_plist *merged, uint32_t flags)
{
	void	*cookiep, *idxp;
	int	 error;

	/* Populate merged list with all pending variables */
	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_UNCOMMITTED)) {
		bhnd_nvram_prop *prop;

		prop = NULL;
		while ((prop = bhnd_nvram_plist_next(path->pending, prop))) {
			/* Skip variables marked for deletion */
			if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_DELETED)) {
				if (bhnd_nvram_prop_is_null(prop))
					continue;
			}

			/* Append to merged list */
			error = bhnd_nvram_plist_append(merged, prop);
			if (error)
				return (error);
		}
	}

	/* Skip merging committed variables? */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMMITTED))
		return (0);

	/* Merge in the committed NVRAM variables */
	idxp = NULL;
	while ((cookiep = bhnd_nvstore_path_data_next(sc, path, &idxp))) {
		const char	*name;
		bhnd_nvram_val	*val;

		/* Fetch the variable name */
		name = bhnd_nvram_data_getvar_name(sc->data, cookiep);

		/* Trim device path prefix */
		if (sc->data_caps & BHND_NVRAM_DATA_CAP_DEVPATHS)
			name = bhnd_nvram_trim_path_name(name);

		/* Skip if already defined in pending updates */
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
		bhnd_nv_asprintf(&pathvar, M_NOWAIT, "devpath%lu", *alias_val);
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
		bhnd_nv_asprintf(&pathvar, M_NOWAIT, "devpath%lu", *alias_val);
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

	/* Collect all variables to be included in the export */
	if ((path_vars = bhnd_nvram_plist_new()) == NULL)
		return (ENOMEM);

	if ((error = bhnd_nvstore_export_merge(sc, child, path_vars, flags))) {
		bhnd_nvram_plist_release(path_vars);
		return (error);
	}

	/* Skip if no children are to be exported */
	if (bhnd_nvram_plist_count(path_vars) == 0) {
		bhnd_nvram_plist_release(path_vars);
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
	} else if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_PRESERVE_DEVPATHS)) {
		/* Preserve existing encoding of this path */
		if (bhnd_nvstore_find_alias(sc, child->path_str) != NULL)
			emit_compact_devpath = true;
	} else {
		BHND_NV_LOG("invalid device path flag: %#" PRIx32, flags);
		error = EINVAL;
		goto finished;
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
			goto finished;

		/* Allocate variable name prefix */
		len = bhnd_nv_asprintf(&prefix, M_NOWAIT, "%lu:", alias_val);
		if (prefix == NULL) {
			error = ENOMEM;
			goto finished;
		}
	
		prefix_len = len;
	} else if (relpath_len > 0) {
		int len;

		/* Allocate the variable name prefix, appending '/' to the
		 * relative path */
		len = bhnd_nv_asprintf(&prefix, M_NOWAIT, "%s/", relpath);
		if (prefix == NULL) {
			error = ENOMEM;
			goto finished;
		}

		prefix_len = len;
	}

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
		namebuf = bhnd_nv_malloc(namebuf_size, M_NOWAIT);
		if (namebuf == NULL) {
			error = ENOMEM;
			goto finished;
		}
	}

	/* Append all path variables to the export plist, prepending the
	 * device-path prefix to the variable names, if required */
	prop = NULL;
	while ((prop = bhnd_nvram_plist_next(path_vars, prop)) != NULL) {
		const char *name;

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
			goto finished;
	}

	/* Success */
	error = 0;

finished:
	if (prefix != NULL)
		bhnd_nv_free(prefix);

	if (namebuf != NULL)
		bhnd_nv_free(namebuf);

	if (path_vars != NULL)
		bhnd_nvram_plist_release(path_vars);

	return (error);
}

/**
 * Filter and validate the given export @p flags, returning the updated flags
 * in @p oflags.
 */
static int
bhnd_nvstore_filter_export_flags(uint32_t flags, uint32_t *oflags)
{
	size_t num_dpath_flags = 0;

	/* Default to exporting all properties */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMMITTED) &&
	    !BHND_NVSTORE_GET_FLAG(flags, EXPORT_UNCOMMITTED))
	{
		flags |= BHND_NVSTORE_EXPORT_ALL_VARS;
	}

	/* Default to preserving the current device path encoding */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMPACT_DEVPATHS) &&
	    !BHND_NVSTORE_GET_FLAG(flags, EXPORT_EXPAND_DEVPATHS))
	{
		flags |= BHND_NVSTORE_EXPORT_PRESERVE_DEVPATHS;
	}

	/* Exactly one device path encoding flag must be set */
	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_COMPACT_DEVPATHS))
		num_dpath_flags++;

	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_EXPAND_DEVPATHS))
		num_dpath_flags++;

	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_PRESERVE_DEVPATHS))
		num_dpath_flags++;

	if (num_dpath_flags != 1)
		return (EINVAL);

	/* If EXPORT_DELETED is set, EXPORT_UNCOMMITTED must be set too */
	if (BHND_NVSTORE_GET_FLAG(flags, EXPORT_DELETED) &&
	    !BHND_NVSTORE_GET_FLAG(flags, EXPORT_DELETED))
	{
		return (EINVAL);
	}

	/* Flags are valid */
	*oflags = flags;
	return (0);
}

/**
 * Export a flat, ordered NVRAM property list representation of all NVRAM
 * properties at @p path.
 * 
 * @param	sc	The NVRAM store instance.
 * @param	path	The NVRAM path to export.
 * @param[out]	cls	On success, will be set to the backing data class
 *			of @p sc. If the data class is are not desired,
 *			a NULL pointer may be provided.
 * @param[out]	props	On success, will be set to a caller-owned property
 *			list containing the exported properties. The caller is
 *			responsible for releasing this value via
 *			bhnd_nvram_plist_release().
 * @param[out]	options	On success, will be set to a caller-owned property
 *			list containing the current NVRAM serialization options
 *			for @p sc. The caller is responsible for releasing this
 *			value via bhnd_nvram_plist_release().
 * @param	flags	Export flags. See BHND_NVSTORE_EXPORT_*.
 * 
 * @retval 0		success
 * @retval EINVAL	If @p flags is invalid.
 * @retval ENOENT	The requested path was not found.
 * @retval ENOMEM	If allocation fails.
 * @retval non-zero	If export of  @p path otherwise fails, a regular unix
 *			error code will be returned.
 */
static int
bhnd_nvstore_export_path(struct bhnd_nvram_store *sc, bhnd_nvstore_path *path,
    bhnd_nvram_data_class **cls, bhnd_nvram_plist **props,
    bhnd_nvram_plist **options, uint32_t flags)
{
	bhnd_nvram_plist	*unordered;
	bhnd_nvram_prop		*prop;
	const char		*name;
	void			*cookiep;
	int			 error;

	BHND_NVSTORE_LOCK_ASSERT(sc, MA_OWNED);

	*props = NULL;
	unordered = NULL;
	if (options != NULL)
		*options = NULL;

	/* Validate and filter the export flags */
	if ((error = bhnd_nvstore_filter_export_flags(flags, &flags)))
		return (error);

	/* Allocate new, empty property list */
	if ((unordered = bhnd_nvram_plist_new()) == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Export the top-level path first */
	error = bhnd_nvram_store_export_child(sc, path, path, unordered, flags);
	if (error)
		goto failed;

	/* If requested, export any children */
	for (size_t i = 0; i < nitems(sc->paths); i++) {
		bhnd_nvstore_path *child;

		LIST_FOREACH(child, &sc->paths[i], np_link) {
			/* Top-level path was already exported */
			if (child == path)
				continue;

			error = bhnd_nvram_store_export_child(sc, path, child,
			    unordered, flags);
			if (error)
				goto failed;
		}
	}

	/* If requested, provide the current class and serialization options */
	if (cls != NULL)
		*cls = bhnd_nvram_data_get_class(sc->data);

	if (options != NULL)
		*options = bhnd_nvram_plist_retain(sc->data_opts);

	/*
	 * If we're re-encoding device paths, don't bother preserving the
	 * existing NVRAM variable order; our variable names will not match
	 * the existing backing NVRAM data.
	 */
	if (!BHND_NVSTORE_GET_FLAG(flags, EXPORT_PRESERVE_DEVPATHS)) {
		*props = unordered;
		unordered = NULL;

		goto finished;
	}

	/* 
	 * Re-order the flattened output to match the existing NVRAM variable
	 * ordering.
	 * 
	 * We append all new variables at the end of the input; this should
	 * reduce the delta that needs to be written (e.g. to flash) when
	 * committing NVRAM updates, and should result in a serialization
	 * identical to the input serialization if uncommitted updates are
	 * excluded from the export.
	 */
	if ((*props = bhnd_nvram_plist_new()) == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Using the backing NVRAM data ordering to order all variables
	 * currently defined in the backing store */ 
	cookiep = NULL;
	while ((name = bhnd_nvram_data_next(sc->data, &cookiep))) {
		prop = bhnd_nvram_plist_get_prop(unordered, name);
		if (prop == NULL)
			continue;

		/* Append to ordered result */
		if ((error = bhnd_nvram_plist_append(*props, prop)))
			goto failed;
	
		/* Remove from unordered list */
		bhnd_nvram_plist_remove(unordered, name);
	}

	/* Any remaining variables are new, and should be appended to the
	 * end of the export list */
	prop = NULL;
	while ((prop = bhnd_nvram_plist_next(unordered, prop)) != NULL) {
		if ((error = bhnd_nvram_plist_append(*props, prop)))
			goto failed;
	}

	/* Export complete */
finished:
	if (unordered != NULL)
		bhnd_nvram_plist_release(unordered);

	return (0);

failed:
	if (unordered != NULL)
		bhnd_nvram_plist_release(unordered);

	if (options != NULL && *options != NULL)
		bhnd_nvram_plist_release(*options);

	if (*props != NULL)
		bhnd_nvram_plist_release(*props);

	return (error);
}

/**
 * Export a flat, ordered NVRAM property list representation of all NVRAM
 * properties at @p path.
 * 
 * @param	sc	The NVRAM store instance.
 * @param	path	The NVRAM path to export, or NULL to select the root
 *			path.
 * @param[out]	cls	On success, will be set to the backing data class
 *			of @p sc. If the data class is are not desired,
 *			a NULL pointer may be provided.
 * @param[out]	props	On success, will be set to a caller-owned property
 *			list containing the exported properties. The caller is
 *			responsible for releasing this value via
 *			bhnd_nvram_plist_release().
 * @param[out]	options	On success, will be set to a caller-owned property
 *			list containing the current NVRAM serialization options
 *			for @p sc. The caller is responsible for releasing this
 *			value via bhnd_nvram_plist_release().
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
    bhnd_nvram_data_class **cls, bhnd_nvram_plist **props,
    bhnd_nvram_plist **options, uint32_t flags)
{
	bhnd_nvram_plist	*unordered;
	bhnd_nvstore_path	*top;
	size_t			 pathlen;
	int			 error;
	
	*props = NULL;
	unordered = NULL;
	if (options != NULL)
		*options = NULL;

	/* Default to exporting root path */
	if (path == NULL)
		path = BHND_NVSTORE_ROOT_PATH;

	pathlen = strlen(path);
	path = bhnd_nvstore_parse_external_path(path, &pathlen);
	if (path == NULL) {
		/* Path is not a valid fully-qualified external path */
		return (ENOENT);
	}

	BHND_NVSTORE_LOCK(sc);

	/* Fetch referenced path */
	top = bhnd_nvstore_get_path(sc, path, pathlen);
	if (top == NULL) {
		error = ENOENT;
	} else {
		error = bhnd_nvstore_export_path(sc, top, cls, props, options,
		    flags);
	}

	BHND_NVSTORE_UNLOCK(sc);

	return (error);
}

/**
 * Encode all NVRAM properties at @p path, using the @p store's current NVRAM
 * data format.
 * 
 * @param	sc	The NVRAM store instance.
 * @param	path	The NVRAM path to export, or NULL to select the root
 *			path.
 * @param[out]	data	On success, will be set to the newly serialized value.
 *			The caller is responsible for freeing this value
 *			via bhnd_nvram_io_free().
 * @param	flags	Export flags. See BHND_NVSTORE_EXPORT_*.
 *
 * @retval 0		success
 * @retval EINVAL	If @p flags is invalid.
 * @retval ENOENT	The requested path was not found.
 * @retval ENOMEM	If allocation fails.
 * @retval non-zero	If serialization of  @p path otherwise fails, a regular
 *			unix error code will be returned.
 */
int
bhnd_nvram_store_serialize(struct bhnd_nvram_store *sc, const char *path,
   struct bhnd_nvram_io **data,  uint32_t flags)
{
	bhnd_nvram_plist	*props;
	bhnd_nvram_plist	*options;
	bhnd_nvram_data_class	*cls;
	struct bhnd_nvram_io	*io;
	void			*outp;
	size_t			 olen;
	int			 error;

	props = NULL;
	options = NULL;
	io = NULL;

	/* Perform requested export */
	error = bhnd_nvram_store_export(sc, path, &cls, &props, &options,
	    flags);
	if (error)
		return (error);

	/* Determine serialized size */
	error = bhnd_nvram_data_serialize(cls, props, options, NULL, &olen);
	if (error)
		goto failed;

	/* Allocate output buffer */
	if ((io = bhnd_nvram_iobuf_empty(olen, olen)) == NULL) {
		error = ENOMEM;
		goto failed;
	}

	/* Fetch write pointer */
	if ((error = bhnd_nvram_io_write_ptr(io, 0, &outp, olen, NULL)))
		goto failed;

	/* Perform serialization */
	error = bhnd_nvram_data_serialize(cls, props, options, outp, &olen);
	if (error)
		goto failed;

	if ((error = bhnd_nvram_io_setsize(io, olen)))
		goto failed;

	/* Success */
	bhnd_nvram_plist_release(props);
	bhnd_nvram_plist_release(options);

	*data = io;
	return (0);

failed:
	if (props != NULL)
		bhnd_nvram_plist_release(props);

	if (options != NULL)
		bhnd_nvram_plist_release(options);

	if (io != NULL)
		bhnd_nvram_io_free(io);

	return (error);
}


static kobj_method_t bhnd_nvram_store_methods[] = {
	KOBJMETHOD(bhnd_nvram_prov_init,		bhnd_nvram_store_init),
	KOBJMETHOD(bhnd_nvram_prov_fini,		bhnd_nvram_store_fini),
	KOBJMETHOD(bhnd_nvram_prov_sync,		bhnd_nvram_store_sync),
	KOBJMETHOD(bhnd_nvram_prov_open_path,		bhnd_nvram_store_open_path),
	KOBJMETHOD(bhnd_nvram_prov_get_pathname,	bhnd_nvram_store_get_pathname),
	KOBJMETHOD(bhnd_nvram_prov_retain_path,		bhnd_nvram_store_retain_path),
	KOBJMETHOD(bhnd_nvram_prov_release_path,	bhnd_nvram_store_release_path),
	KOBJMETHOD(bhnd_nvram_prov_get_children,	bhnd_nvram_store_get_children),
	KOBJMETHOD(bhnd_nvram_prov_free_children,	bhnd_nvram_store_free_children),
	KOBJMETHOD(bhnd_nvram_prov_setprop,		bhnd_nvram_store_setprop),
	KOBJMETHOD(bhnd_nvram_prov_delprop,		bhnd_nvram_store_delprop),
	KOBJMETHOD(bhnd_nvram_prov_getprop,		bhnd_nvram_store_getprop),
	KOBJMETHOD(bhnd_nvram_prov_copyprops,		bhnd_nvram_store_copyprops),

	KOBJMETHOD_END
};

DEFINE_CLASS_0(bhnd_nvram_store, bhnd_nvram_store_provider, bhnd_nvram_store_methods, sizeof(struct bhnd_nvram_store));
