/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 *
 * Portions of this software were developed by Landon Fuller
 * under sponsorship from the FreeBSD Foundation.
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

#ifndef _BHND_PMU_BHND_PMU_H_
#define _BHND_PMU_BHND_PMU_H_

#include <sys/types.h>

#include <dev/bhnd/bhnd.h>

struct bhnd_regulator;

/**
 * bhnd_pmu(4) regulator identifiers.
 *
 * TODO: Allow strings instead?
 */
typedef enum bhnd_regulator_name {
	BHND_REGULATOR_CLDO		= 1,	/**< digital core LDO */
	BHND_REGULATOR_LDO1		= 1,	/**< digital core LDO */
	BHND_REGULATOR_PAREF_LDO	= 2,	/**< PA reference LDO */
	BHND_REGULATOR_CBUCK		= 3,	/**< digital core buck regulator */
	BHND_REGULATOR_LNLDO1		= 4,	/**< low-noise LDO#1 */
	BHND_REGULATOR_LNLDO2		= 5,	/**< low-noise LDO#2 */
} bhnd_regulator_name;

/**
 * bhnd_pmu(4) regulator modes.
 */
typedef enum bhnd_regulator_mode {
	BHND_REGULATOR_MODE_DEFAULT	= 0,	/**< default output mode */
	BHND_REGULATOR_MODE_PWM		= 1,	/**< PWM output mode */
	BHND_REGULATOR_MODE_BURST	= 2,	/**< burst output mode */
	BHND_REGULATOR_MODE_SEL1	= 3,	/**< selector #1 asserted */
} bhnd_regulator_mode;

/**
 * bhnd_pmu(4) regulator attributes
 */
typedef enum bhnd_regulator_attr {
	BHND_REGULATOR_ATTR_VOUT		= 0,	/**< output voltage in millivolts */
	BHND_REGULATOR_ATTR_VOUT_RAW		= 1,	/**< output voltage as raw register value */
	BHND_REGULATOR_ATTR_PWRUP_LATCH_CTRL	= 2,	/**< power-up latch control */
} bhnd_regulator_attr;

void	bhnd_regulator_release(struct bhnd_regulator *regulator);
int	bhnd_regulator_setattr(struct bhnd_regulator *regulator,
	    bhnd_regulator_attr attr, uint32_t value);

/**
 * Per-core PMU register information.
 */
struct bhnd_core_pmu_info {
	device_t		 pm_dev;	/**< core device */
	device_t		 pm_pmu;	/**< PMU device */
	struct bhnd_resource	*pm_res;	/**< Resource containing PMU
						     register block for this
						     device (if any). */
	bus_size_t		 pm_regs;	/**< Offset to PMU register
						  *  block in @p pm_res */
};

/**
 * bhnd_pmu(4) regulator reference.
 */
struct bhnd_regulator {
	uintptr_t	id;	/**< PMU-assigned opaque regulator ID */
	device_t	pmu;	/**< PMU provider reference */
};

#endif /* _BHND_PMU_BHND_PMU_H_ */
