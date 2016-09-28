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
#include <sys/systm.h>

#include "bhnd_nvram_io.h"

#include "bhnd_nvram_parservar.h"
#include "bhnd_nvram_parser.h"

#include "bhnd_nvram_common.h"

/* Limit a size_t value to a suitable range for use as a printf string field
 * width */
#define	NVRAM_PRINT_WIDTH(_len)			\
	((_len) > (BHND_NVRAM_VAL_MAXLEN*2) ?	\
	    (BHND_NVRAM_VAL_MAXLEN*2) :		\
	    (int)(_len))

/**
 * Parse a 'name=value' string.
 * 
 * @param env The string to be parsed.
 * @param env_len The length of @p envp.
 * @param delim The delimiter used in @p envp. This will generally be '='.
 * @param[out] name On success, a pointer to the name string.
 * @param[out] name_len On success, the length of the name substring.
 * @param[out] value On success, a pointer to the value substring.
 * @param[out] value_len On success, the length of the value substring.
 * 
 * @retval 0 success
 * @retval EINVAL if parsing @p envp fails.
 */
int
bhnd_nvram_parse_env(const char *env, size_t env_len, char delim,
    const char **name, size_t *name_len, const char **value, size_t *value_len)
{
	const char *p;

	/* Name */
	if ((p = memchr(env, delim, env_len)) == NULL) {
		printf("%s: delimiter '%c' not found in '%.*s'\n",
		    __FUNCTION__, delim, NVRAM_PRINT_WIDTH(env_len), env);
		return (EINVAL);
	}

	*name = env;
	*name_len = p - env;

	/* Skip delim */
	p++;

	/* Vaue */
	*value = p;
	*value_len = env_len - (p - env);

	return (0);
}
