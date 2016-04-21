/*-
 * Copyright (c) 2016 Landon Fuller <landon@landonf.org>
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

#ifndef	_BHND_NVRAM_SPROMREG_H_
#define	_BHND_NVRAM_SPROMREG_H_

#define	BHND_SPROMSZ_R1_3	128	/**< SPROM image size (rev 1-3) */
#define	BHND_SPROMSZ_R4_8_9	440	/**< SPROM image size (rev 4, 8-9) */
#define	BHND_SPROMSZ_R10	460	/**< SPROM image size (rev 10) */ 
#define	BHND_SPROMSZ_R11	468	/**< SPROM image size (rev 11) */

/** Maximum supported SPROM image size */
#define	BHND_SPROMSZ_MAX	BHND_SPROMSZ_R11

/** SPROM signature (rev 4) */
#define	BHND_SPROMSIG_R4	0x5372			
#define	BHND_SPROMSIG_R4_OFF	64	/**< SPROM signature offset (rev 4) */

/** SPROM signature (rev 8, 9) */
#define	BHND_SPROMSIG_R8_9	BHND_SPROMSIG_R4
#define	BHND_SPROMSIG_R8_9_OFF	128	/**< SPROM signature offset (rev 8-9) */

/** SPROM signature (rev 10) */
#define	BHND_SPROMSIG_R10	BHND_SPROMSIG_R4
#define	BHND_SPROMSIG_R10_OFF	438	/**< SPROM signature offset (rev 10) */

/** SPROM signature (rev 11) */
#define	BHND_SPROMSIG_R11	0x0634
#define	BHND_SPROMSIG_R11_OFF	128	/**< SPROM signature offset (rev 11) */


#endif /* _BHND_NVRAM_SPROMREG_H_ */
