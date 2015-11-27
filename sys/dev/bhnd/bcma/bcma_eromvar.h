/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * $FreeBSD$
 */

#ifndef	_BCMA_BCMA_EROMVAR_H_
#define	_BCMA_BCMA_EROMVAR_H_

#include <dev/bhnd/bhnd.h>

#include "bcmavar.h"

/**
 * EROM read context.
 */
struct bcma_erom {
	device_t	 dev;		/**< the owning device. */
	struct resource	*res;		/**< resource mapping the EROM table. */
	bus_size_t	 start;		/**< EROM table offset */
	bus_size_t	 offset;	/**< current read offset */
};

/** EROM core descriptor. */
struct bcma_erom_core {
	uint16_t	vendor;		/**< core's designer */
	uint16_t	device;		/**< core's device identifier */
	uint16_t	rev;		/**< core's hardware revision */
	u_long		num_mport;	/**< number of master port descriptors */
	u_long		num_dport;	/**< number of slave port descriptors */
	u_long		num_mwrap;	/**< number of master wrapper slave port descriptors */
	u_long		num_swrap;	/**< number of slave wrapper slave port descriptors */
};

/** EROM master port descriptor. */
struct bcma_erom_mport {
	uint8_t		port_num;	/**< the port number (bus-unique) */
	uint8_t		port_vid;	/**< the port VID. A single physical
					     master port may have multiple VIDs;
					     the canonical port address is
					     composed of the port number + the
					     port VID */
};

/** EROM slave port region descriptor. */
struct bcma_erom_sport_region {
	uint8_t		port_num;	/**< the slave port mapping this region */
	uint8_t		port_type;	/**< the mapping port's type */
	bcma_addr_t	base_addr;	/**< region base address */
	bcma_addr_t	size;		/**< region size */
};

int		bcma_erom_open(struct resource *resource, bus_size_t offset,
		    struct bcma_erom *erom);

int		bcma_erom_peek32(struct bcma_erom *erom, uint32_t *entry);
bus_size_t	bcma_erom_tell(struct bcma_erom *erom);
void		bcma_erom_seek(struct bcma_erom *erom, bus_size_t offset);
void		bcma_erom_reset(struct bcma_erom *erom);

int		bcma_erom_seek_core_index(struct bcma_erom *erom,
		    u_int core_index);
int		bcma_erom_parse_core(struct bcma_erom *erom,
		    struct bcma_erom_core *core);

int		bcma_erom_parse_mport(struct bcma_erom *erom,
		    struct bcma_erom_mport *mport);

int		bcma_erom_parse_sport_region(struct bcma_erom *erom,
		    struct bcma_erom_sport_region *region);

int		bcma_erom_get_core_info(struct bcma_erom *erom,
		    struct bhnd_core_info **cores,
		    u_int *num_cores);

int		bcma_erom_parse_corecfg(struct bcma_erom *erom,
		    struct bcma_corecfg **result);

#endif /* _BCMA_BCMA_EROMVAR_H_ */
