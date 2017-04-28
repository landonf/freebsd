/*-
 * Copyright (c) 2017 Landon Fuller <landonf@FreeBSD.org>
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

#ifndef	_MIPS_BROADCOM_BCM_DISK_CFE_H_
#define	_MIPS_BROADCOM_BCM_DISK_CFE_H_

#include <sys/param.h>
#include <sys/queue.h>

#define	BCM_CFE_DUNIT_MAX	64		/**< maximum CFE device unit */
#define	BCM_CFE_DNAME_MAX	64		/**< maximum CFE device name length */

#define	BCM_CFE_PALIGN_MIN	0x1000		/**< minimum partition alignment */

SLIST_HEAD(bcm_cfe_parts, bcm_cfe_part);
SLIST_HEAD(bcm_cfe_disks, bcm_cfe_disk);

int			 bcm_cfe_probe_disks(struct bcm_cfe_disks *result);
struct bcm_cfe_disk	*bcm_cfe_disk_new(const char *drvname, u_int unit);
void			 bcm_cfe_disk_free(struct bcm_cfe_disk *disk);

/**
 * CFE-probed partition description.
 */
struct bcm_cfe_part {
	char		*devname;	/**< CFE device name (e.g. 'nflash0.boot') */
	const char	*label;		/**< CFE partition label */
	off_t		 offset;	/**< partition offset, or OFF_MAX if unknown */
	off_t		 size;		/**< partition size, or 0 if unknown */

	SLIST_ENTRY(bcm_cfe_part) cp_link;
};

/**
 * CFE-probed block device description.
 */
struct bcm_cfe_disk {
	const char		*drvname;			/**< CFE driver class name (e.g. 'nflash') */
	u_int			 unit;				/**< CFE device unit */
	struct bcm_cfe_parts	 parts;				/**< identified partitions */

	SLIST_ENTRY(bcm_cfe_disk) cd_link;
};

#endif /* _MIPS_BROADCOM_BCM_DISK_CFE_H_ */
