/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
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

#include <sys/hash.h>
#include <sys/param.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include "bhnd_nvram_common.h"

#include "bhnd_nvram_map_data.h"

/*
 * Common NVRAM/SPROM support, including NVRAM variable map
 * lookup.
 */

MALLOC_DEFINE(M_BHND_NVRAM, "bhnd_nvram", "bhnd nvram data");

/*
 * CRC-8 lookup table used to checksum SPROM and NVRAM data via
 * bhnd_nvram_crc8().
 * 
 * Generated with following parameters:
 * 	polynomial:	CRC-8 (x^8 + x^7 + x^6 + x^4 + x^2 + 1)
 * 	reflected bits:	false
 * 	reversed:	true
 */
const uint8_t bhnd_nvram_crc8_tab[] = {
	0x00, 0xf7, 0xb9, 0x4e, 0x25, 0xd2, 0x9c, 0x6b, 0x4a, 0xbd, 0xf3,
	0x04, 0x6f, 0x98, 0xd6, 0x21, 0x94, 0x63, 0x2d, 0xda, 0xb1, 0x46,
	0x08, 0xff, 0xde, 0x29, 0x67, 0x90, 0xfb, 0x0c, 0x42, 0xb5, 0x7f,
	0x88, 0xc6, 0x31, 0x5a, 0xad, 0xe3, 0x14, 0x35, 0xc2, 0x8c, 0x7b,
	0x10, 0xe7, 0xa9, 0x5e, 0xeb, 0x1c, 0x52, 0xa5, 0xce, 0x39, 0x77,
	0x80, 0xa1, 0x56, 0x18, 0xef, 0x84, 0x73, 0x3d, 0xca, 0xfe, 0x09,
	0x47, 0xb0, 0xdb, 0x2c, 0x62, 0x95, 0xb4, 0x43, 0x0d, 0xfa, 0x91,
	0x66, 0x28, 0xdf, 0x6a, 0x9d, 0xd3, 0x24, 0x4f, 0xb8, 0xf6, 0x01,
	0x20, 0xd7, 0x99, 0x6e, 0x05, 0xf2, 0xbc, 0x4b, 0x81, 0x76, 0x38,
	0xcf, 0xa4, 0x53, 0x1d, 0xea, 0xcb, 0x3c, 0x72, 0x85, 0xee, 0x19,
	0x57, 0xa0, 0x15, 0xe2, 0xac, 0x5b, 0x30, 0xc7, 0x89, 0x7e, 0x5f,
	0xa8, 0xe6, 0x11, 0x7a, 0x8d, 0xc3, 0x34, 0xab, 0x5c, 0x12, 0xe5,
	0x8e, 0x79, 0x37, 0xc0, 0xe1, 0x16, 0x58, 0xaf, 0xc4, 0x33, 0x7d,
	0x8a, 0x3f, 0xc8, 0x86, 0x71, 0x1a, 0xed, 0xa3, 0x54, 0x75, 0x82,
	0xcc, 0x3b, 0x50, 0xa7, 0xe9, 0x1e, 0xd4, 0x23, 0x6d, 0x9a, 0xf1,
	0x06, 0x48, 0xbf, 0x9e, 0x69, 0x27, 0xd0, 0xbb, 0x4c, 0x02, 0xf5,
	0x40, 0xb7, 0xf9, 0x0e, 0x65, 0x92, 0xdc, 0x2b, 0x0a, 0xfd, 0xb3,
	0x44, 0x2f, 0xd8, 0x96, 0x61, 0x55, 0xa2, 0xec, 0x1b, 0x70, 0x87,
	0xc9, 0x3e, 0x1f, 0xe8, 0xa6, 0x51, 0x3a, 0xcd, 0x83, 0x74, 0xc1,
	0x36, 0x78, 0x8f, 0xe4, 0x13, 0x5d, 0xaa, 0x8b, 0x7c, 0x32, 0xc5,
	0xae, 0x59, 0x17, 0xe0, 0x2a, 0xdd, 0x93, 0x64, 0x0f, 0xf8, 0xb6,
	0x41, 0x60, 0x97, 0xd9, 0x2e, 0x45, 0xb2, 0xfc, 0x0b, 0xbe, 0x49,
	0x07, 0xf0, 0x9b, 0x6c, 0x22, 0xd5, 0xf4, 0x03, 0x4d, 0xba, 0xd1,
	0x26, 0x68, 0x9f
};

/**
 * Return the size of type @p type.
 * 
 * @param type NVRAM data type.
 * @result the byte width of @p type.
 */
size_t
bhnd_nvram_type_width(bhnd_nvram_type type)
{
	switch (type) {
	case BHND_NVRAM_TYPE_INT8:
	case BHND_NVRAM_TYPE_UINT8:
	case BHND_NVRAM_TYPE_CHAR:
		return (sizeof(uint8_t));

	case BHND_NVRAM_TYPE_INT16:
	case BHND_NVRAM_TYPE_UINT16:
		return (sizeof(uint16_t));

	case BHND_NVRAM_TYPE_INT32:
	case BHND_NVRAM_TYPE_UINT32:
		return (sizeof(uint32_t));
	}

	/* Quiesce gcc4.2 */
	panic("bhnd nvram type %u unknown", type);
}

/**
 * Find and return the variable definition for @p varname, if any.
 * 
 * @param varname variable name
 * 
 * @retval bhnd_nvram_vardefn If a valid definition for @p varname is found.
 * @retval NULL If no definition for @p varname is found. 
 */
const struct bhnd_nvram_vardefn *
bhnd_nvram_find_vardefn(const char *varname)
{
	size_t	min, mid, max;
	int	order;

	/*
	 * Locate the requested variable using a binary search.
	 * 
	 * The variable table is guaranteed to be sorted in lexicographical
	 * order (using the 'C' locale for collation rules)
	 */
	min = 0;
	mid = 0;
	max = nitems(bhnd_nvram_vardefs) - 1;

	while (max >= min) {
		/* Select midpoint */
		mid = (min + max) / 2;

		/* Determine which side of the partition to search */
		order = strcmp(bhnd_nvram_vardefs[mid].name, varname);
		if (order < 0) {
			/* Search upper partition */
			min = mid + 1;
		} else if (order > 0) {
			/* Search lower partition */
			max = mid - 1;
		} else if (order == 0) {
			/* Match found */
			return (&bhnd_nvram_vardefs[mid]);
		}
	}

	/* Not found */
	return (NULL);
}

/**
 * Initialize a new variable hash table with @p nelements.
 * 
 * @param map Hash table instance to be initialized.
 * @param nelements The number of hash table buckets to allocate.
 * @param flags Hash table flags (HASH_*).
 */
int
bhnd_nvram_varmap_init(struct bhnd_nvram_varmap *map, size_t nelements,
    int flags)
{
	map->table = hashinit_flags(nelements, M_BHND_NVRAM, &map->mask,
	    flags);
	if (map->table == NULL)
		return (ENOMEM);

	return (0);
}

/**
 * Deallocate all resources associated with @p map.
 * 
 * @param map Hash table to be deallocated.
 */
void
bhnd_nvram_varmap_free(struct bhnd_nvram_varmap *map)
{
	struct bhnd_nvram_tuple *t, *tnext;

	/* Free all elements */
	for (size_t i = 0; i <= map->mask; i++) {
		LIST_FOREACH_SAFE(t, &map->table[i], t_link, tnext) {
			LIST_REMOVE(t, t_link);
			bhnd_nvram_tuple_free(t);
		}
	}

	/* Free hash table */
	hashdestroy(map->table, M_BHND_NVRAM, map->mask);
}

/**
 * Add a variable entry to @p map.
 * 
 * @param map Hash table to modify.
 * @param name Variable name.
 * @param value Variable value.
 * 
 * @retval 0 success
 * @retval ENOMEM unable to allocate new entry
 */
int
bhnd_nvram_varmap_add(struct bhnd_nvram_varmap *map, const char *name,
    const char *value)
{
	struct bhnd_nvram_tuples	*head;
	struct bhnd_nvram_tuple		*t;

	/* Locate target bucket */
	head = &map->table[hash32_str(name, HASHINIT) & map->mask];

	/* Allocate new entry */
	if ((t = bhnd_nvram_tuple_alloc(name, value)) == NULL)
		return (ENOMEM);

	/* Remove any existing entry */
	bhnd_nvram_varmap_remove(map, name);

	/* Insert new entry */
	LIST_INSERT_HEAD(head, t, t_link);
	return (0);
}

/**
 * Remove @p map in @p tuples, if it exists.
 * 
 * @param map Hash table to modify.
 * @param key Key to remove.
 * 
 * @retval 0 success
 * @retval ENOENT If @p name is not found in @p map.
 */
int
bhnd_nvram_varmap_remove(struct bhnd_nvram_varmap *map, const char *name)
{
	struct bhnd_nvram_tuples	*head;
	struct bhnd_nvram_tuple		*t;
	size_t				 name_len;

	/* Locate target bucket */
	head = &map->table[hash32_str(name, HASHINIT) & map->mask];
	name_len = strlen(name);

	LIST_FOREACH(t, head, t_link) {
		if (t->name_len != name_len)
			continue;

		if (strncmp(t->name, name, name_len) != 0)
			continue;

		LIST_REMOVE(t, t_link);
		bhnd_nvram_tuple_free(t);
		return (0);
	}

	/* Not found */
	return (ENOENT);
}

/**
 * Search for @p name in @p map.
 * 
 * @param map Hash table to modify.
 * @param name Variable name.
 * @param name_len Length of @p name, not including trailing NUL.
 * 
 * @retval bhnd_nvram_tuple If @p name is found in @p map.
 * @retval NULL If @p name is not found.
 */
struct bhnd_nvram_tuple *
bhnd_nvram_varmap_find(struct bhnd_nvram_varmap *map, const char *name,
    size_t name_len)
{
	struct bhnd_nvram_tuples	*head;
	struct bhnd_nvram_tuple		*t;

	head = &map->table[hash32_str(name, HASHINIT) & map->mask];

	LIST_FOREACH(t, head, t_link) {
		if (t->name_len != name_len)
			continue;

		if (strncmp(t->name, name, name_len) != 0)
			continue;

		/* Match */
		return (t);
	}

	/* not found */
	return (NULL);
}

/**
 * Check for @p name in @p map.
 * 
 * @param map Hash table to modify.
 * @param name Variable name.
 * @param name_len Length of @p name, not including trailing NUL.
 * 
 * @retval true If @p name is found in @p tuples.
 * @retval false If @p name is not found.
 */
bool bhnd_nvram_varmap_contains(struct bhnd_nvram_varmap *map,
    const char *name, size_t name_len)
{
	return (bhnd_nvram_varmap_find(map, name, name_len) != NULL);
}

/**
 * Allocate a new tuple with @p name and @p value.
 * 
 * @param name Variable name.
 * @param value Variable value.
 * 
 * @retval bhnd_nvram_tuple success.
 * @retval NULL if allocation fails.
 */
struct bhnd_nvram_tuple *
bhnd_nvram_tuple_alloc(const char *name, const char *value)
{
	struct bhnd_nvram_tuple *t;

	t = malloc(sizeof(*t), M_BHND_NVRAM, M_NOWAIT);
	if (t == NULL)
		return (NULL);

	t->name_len = strlen(name);
	t->name = malloc(t->name_len+1, M_BHND_NVRAM, M_NOWAIT);

	t->value_len = strlen(value);
	t->value = malloc(t->value_len+1, M_BHND_NVRAM, M_NOWAIT);

	if (t->name == NULL || t->value == NULL)
		goto failed;

	strcpy(t->name, name);
	strcpy(t->value, value);

	return (t);

failed:
	if (t->name != NULL)
		free(t->name, M_BHND_NVRAM);

	if (t->value != NULL)
		free(t->value, M_BHND_NVRAM);

	free(t, M_BHND_NVRAM);

	return (NULL);
}

void
bhnd_nvram_tuple_free(struct bhnd_nvram_tuple *tuple)
{
	free(tuple->name, M_BHND_NVRAM);
	free(tuple->value, M_BHND_NVRAM);
	free(tuple, M_BHND_NVRAM);
}
