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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_NVRAM_BHND_NVRAM_PARSER_VAR_H_
#define _BHND_NVRAM_BHND_NVRAM_PARSER_VAR_H_

#include <sys/param.h>

#include "bhnd_nvram_io.h"

#include "bhnd_nvram_parser.h"

/** @see bhnd_nvram_parser_probe() */
typedef int (bhnd_nvram_parser_op_probe)(struct bhnd_nvram_io *io);

/** @see bhnd_nvram_parser_new() */
typedef int (bhnd_nvram_parser_op_new)(struct bhnd_nvram_parser **nv,
    struct bhnd_nvram_io *io);

/** @see bhnd_nvram_parser_free() */
typedef void (bhnd_nvram_parser_op_free)(struct bhnd_nvram_parser *nv);

/**
 * NVRAM parser class.
 */
struct bhnd_nvram_parser_class {
	bhnd_nvram_parser_op_probe	*op_probe;
	bhnd_nvram_parser_op_new	*op_new;
	bhnd_nvram_parser_op_free	*op_free;
};

/**
 * NVRAM parser instance.
 */
struct bhnd_nvram_parser {
	const struct bhnd_nvram_parser_class	*cls;
};

int	bhnd_nvram_parse_env(const char *env, size_t env_len, char delim,
	    const char **name, size_t *name_len, const char **value,
	    size_t *value_len);

/**
 * Define a bhnd_nvram_parser_class with name @p _n.
 */
#define	BHND_NVRAM_PARSER_DEFN(_n)					\
	static bhnd_nvram_parser_op_probe				\
	    bhnd_nvram_ ## _n ## _probe;				\
	static bhnd_nvram_parser_op_new					\
	    bhnd_nvram_ ## _n ## _new;					\
	static bhnd_nvram_parser_op_free				\
	    bhnd_nvram_ ## _n ## _free;					\
									\
	struct bhnd_nvram_parser_class bhnd_nvram_parser_## _n##_class =\
	{								\
		.op_probe	= bhnd_nvram_ ## _n ## _probe,		\
		.op_new		= bhnd_nvram_ ## _n ## _new,		\
		.op_free	= bhnd_nvram_ ## _n ## _free,		\
	};

#endif /* _BHND_NVRAM_BHND_NVRAM_PARSER_VAR_H_ */
