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
 * 
 * $FreeBSD$
 */

#ifndef _BHND_NVRAM_BHND_NVRAM_COMMON_H_
#define _BHND_NVRAM_BHND_NVRAM_COMMON_H_

#ifdef _KERNEL
#include <sys/param.h>
#include <sys/malloc.h>
#else
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#endif

#include "bhnd_nvram.h"

struct bhnd_nvram_vardefn;

#ifdef _KERNEL
MALLOC_DECLARE(M_BHND_NVRAM);
#endif

/** NVRAM data type string representations */
typedef enum {
	BHND_NVRAM_SFMT_HEX	= 1,	/**< hex format */
	BHND_NVRAM_SFMT_DEC	= 2,	/**< decimal format */
	BHND_NVRAM_SFMT_MACADDR	= 3,	/**< mac address (canonical form, hex octets,
					     separated with ':') */
	BHND_NVRAM_SFMT_LEDDC	= 4,	/**< LED PWM duty-cycle (2 bytes -- on/off) */
	BHND_NVRAM_SFMT_CCODE	= 5	/**< count code format (2-3 ASCII chars, or hex string) */
} bhnd_nvram_sfmt;

size_t				 bhnd_nvram_type_width(bhnd_nvram_type type);
const struct bhnd_nvram_vardefn	*bhnd_nvram_find_vardefn(const char *varname);
const struct bhnd_nvram_vardefn	*bhnd_nvram_get_vardefn(size_t id);
size_t				 bhnd_nvram_get_vardefn_id(
				     const struct bhnd_nvram_vardefn *defn);

bool				 bhnd_nvram_validate_name(const char *name,
				     size_t name_len);

/** NVRAM variable flags */
enum {
	BHND_NVRAM_VF_ARRAY	= (1<<0),	/**< variable is an array */
	BHND_NVRAM_VF_MFGINT	= (1<<1),	/**< mfg-internal variable; should not be externally visible */
	BHND_NVRAM_VF_IGNALL1	= (1<<2)	/**< hide variable if its value has all bits set. */
};


/** NVRAM variable definition */
struct bhnd_nvram_vardefn {
	const char		*name;	  	/**< variable name */
	const char		*desc;		/**< human readable description, or NULL */
	const char		*help;		/**< human readable help text, or NULL */
	bhnd_nvram_type		 type;	 	/**< base data type */
	uint8_t			 nelem;		/**< array element count if BHND_NVRAM_VF_ARRAY,
						     otherwise 1 */
	bhnd_nvram_sfmt		 sfmt;		/**< string format */
	uint32_t		 flags;		/**< BHND_NVRAM_VF_* flags */
};

extern const struct bhnd_nvram_vardefn bhnd_nvram_vardefns[];
extern const size_t bhnd_nvram_num_vardefns;

/** SPROM layout flags */
enum {
	/**
	 * SPROM layout does not have magic identification value.
	 *
	 * This applies to SPROM revisions 1-3, where the actual
	 * layout must be determined by looking for a matching sromrev
	 * at the expected offset, and then verifying the CRC to ensure
	 * that the match was not a false positive.
	 */
	SPROM_LAYOUT_MAGIC_NONE	= (1<<0),	
};

/**
 * SPROM layout descriptor.
 */
struct bhnd_sprom_layout {
	size_t		 size;		/**< SPROM image size, in bytes */
	uint8_t		 rev;		/**< SPROM revision */
	uint8_t		 flags;		/**< layout flags (SPROM_LAYOUT_*) */
	size_t		 srev_offset;	/**< offset to SROM revision */
	size_t		 magic_offset;	/**< offset to magic value */
	uint16_t	 magic_value;	/**< expected magic value */
	const uint8_t	*bindings;	/**< SPROM binding opcode table */
	size_t		 bindings_size;	/**< SPROM binding opcode table size */
	uint16_t	 num_vars;	/**< total number of variables defined
					     for this layout by the binding
					     table */
};

extern const struct bhnd_sprom_layout bhnd_sprom_layouts[];
extern const size_t bhnd_sprom_num_layouts;

/*
 * Most opcodes are provided with two variants:
 *
 * - Standard:	The opcode's data directly follows the opcode. The data type
 * 		(SPROM_OPCODE_DATA_*) is encoded in the opcode immediate (IMM).
 * - Immediate:	The opcode's data is encoded directly in the opcode immediate
 *		(IMM).
 */ 
#define	SPROM_OPC_MASK			0xF0	/**< operation mask */
#define	SPROM_IMM_MASK			0x0F	/**< immediate value mask */
#define	SPROM_IMM_MAX			SPROM_IMM_MASK
#define	  SPROM_OP_DATA_U8		  0x00	/**< data is u8 */
#define	  SPROM_OP_DATA_U8_SCALED	  0x01	/**< data is u8; multiply by type width */
#define	  SPROM_OP_DATA_U16		  0x02	/**< data is u16-le */
#define	  SPROM_OP_DATA_U32		  0x03	/**< data is u32-le */
#define	  SPROM_OP_DATA_I8		  0x04	/**< data is i8 */
#define	SPROM_OPCODE_EXT		0x00	/**< extended opcodes defined in IMM */
#define	SPROM_OPCODE_EOF		0x00	/**< marks end of opcode stream */
#define	SPROM_OPCODE_NELEM		0x01	/**< variable array element count follows as U8 */
#define	SPROM_OPCODE_VAR_END		0x02	/**< marks end of variable definition */
#define	SPROM_OPCODE_VAR_IMM		0x10	/**< variable ID (immediate) */
#define	SPROM_OPCODE_VAR_REL_IMM	0x20	/**< relative variable ID (last ID + immediate) */
#define	SPROM_OPCODE_VAR		0x30	/**< variable ID */
#define	SPROM_OPCODE_REV_IMM		0x40	/**< revision range (immediate) */
#define	SPROM_OPCODE_REV_RANGE		0x50	/**< revision range (8-bit range)*/
#define	  SPROM_OP_REV_RANGE_MAX	  0x0F	/**< maximum representable SROM revision */
#define	  SPROM_OP_REV_START_MASK	  0xF0
#define	  SPROM_OP_REV_START_SHIFT	  4
#define	  SPROM_OP_REV_END_MASK	 	  0x0F
#define	  SPROM_OP_REV_END_SHIFT	  0
#define	SPROM_OPCODE_MASK_IMM		0x60	/**< value mask (immediate) */
#define	SPROM_OPCODE_MASK		0x70	/**< value mask */
#define	SPROM_OPCODE_SHIFT_IMM		0x80	/**< value shift (unsigned immediate, multipled by 2) */
#define	SPROM_OPCODE_SHIFT		0x90	/**< value shift */
#define	SPROM_OPCODE_OFFSET_REL_IMM	0xA0	/**< relative input offset (last offset + (immediate * type width) */
#define	SPROM_OPCODE_OFFSET		0xB0	/**< input offset */
#define	SPROM_OPCODE_TYPE_IMM		0xC0	/**< input type (BHND_NVRAM_TYPE_*) (immediate) */
#define	SPROM_OPCODE_DO_BIND		0xD0	/**< bind current value, advance input/output offsets as per IMM */
#define	  SPROM_OP_BIND_SKIP_IN_MASK	  0x03	/**< the number of input elements to advance after the bind */
#define	  SPROM_OP_BIND_SKIP_IN_SHIFT	  0
#define	  SPROM_OP_BIND_SKIP_IN_SIGN	 (1<<2)	/**< SKIP_IN sign bit */
#define	  SPROM_OP_BIND_SKIP_OUT_MASK	  0x08	/**< the number of output elements to advance after the bind */
#define	  SPROM_OP_BIND_SKIP_OUT_SHIFT	  3
#define	SPROM_OPCODE_DO_BINDN_IMM	0xE0	/**< bind IMM times, advancing input/output offsets by one element each time */
#define	SPROM_OPCODE_DO_BINDN		0xF0	/**< bind multiple times, advancing input/output offsets as per
						     SPROM_OP_BIND_SKIP_IN/SPROM_OP_BIND_SKIP_OUT IMM values.
						     The U8 element count follows. */

/** Evaluates to true if opcode is an extended opcode */
#define SPROM_OPCODE_IS_EXT(_opcode)	\
    (((_opcode) & SPROM_OPC_MASK) == SPROM_OPCODE_EXT)

/** Return the opcode constant for a simple or extended opcode */
#define SPROM_OPCODE_OP(_opcode)	\
    (SPROM_OPCODE_IS_EXT(_opcode) ? (_opcode) : ((_opcode) & SPROM_OPC_MASK))

/** Return the opcode immediate for a simple opcode, or zero if this is
  * an extended opcode  */
#define SPROM_OPCODE_IMM(_opcode)	\
    (SPROM_OPCODE_IS_EXT(_opcode) ? 0 : ((_opcode) & SPROM_IMM_MASK))

/** Evaluates to true if the given opcode produces an implicit
 *  SPROM_OPCODE_VAR_END instruction for any open variable */
#define	SPROM_OP_IS_IMPLICIT_VAR_END(_opcode)			\
    (((_opcode) == SPROM_OPCODE_VAR_IMM)		||	\
     ((_opcode) == SPROM_OPCODE_VAR_REL_IMM)	||	\
     ((_opcode) == SPROM_OPCODE_VAR)		||	\
     ((_opcode) == SPROM_OPCODE_REV_IMM)		||	\
     ((_opcode) == SPROM_OPCODE_REV_RANGE))

/** Evaluates to true if the given opcode is either an explicit
  * SPROM_OPCODE_VAR_END instruction, or is an opcode that produces an
  * implicit terminatation of any open variable */
#define	SPROM_OP_IS_VAR_END(_opcode)		\
     (((_opcode) == SPROM_OPCODE_VAR_END) ||	\
     SPROM_OP_IS_IMPLICIT_VAR_END(_opcode))

/** maximum representable immediate value */
#define	SPROM_OP_IMM_MAX	SPROM_IMM_MASK

/** maximum representable SROM revision */
#define	SPROM_OP_REV_MAX	MAX(SPROM_OP_REV_RANGE_MAX, SPROM_IMM_MAX)

#endif /* _BHND_NVRAM_BHND_NVRAM_COMMON_H_ */
