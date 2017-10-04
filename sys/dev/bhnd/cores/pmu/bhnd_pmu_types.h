/*-
 * Copyright (c) 2017 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Landon Fuller under sponsorship from
 * the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _BHND_CORES_PMU_BHND_PMU_TYPES_H_
#define _BHND_CORES_PMU_BHND_PMU_TYPES_H_

#include <sys/types.h>

/**
 * bhnd_pmu(4) regulators.
 */
typedef enum bhnd_pmu_regulator {
	BHND_PMU_REG_CLDO	= 1,	/**< digital core LDO */
	BHND_PMU_REG_LDO1	= 1,	/**< digital core LDO */
	BHND_PMU_REG_PAREF_LDO	= 2,	/**< PA reference LDO */
	BHND_PMU_REG_CBUCK	= 3,	/**< digital core buck regulator */
	BHND_PMU_REG_LNLDO1	= 4,	/**< low-noise LDO#1 */
	BHND_PMU_REG_LNLDO2	= 5,	/**< low-noise LDO#2 */
} bhnd_pmu_regulator;

/**
 * bhnd_pmu(4) regulator modes.
 */
typedef enum bhnd_pmu_regulator_mode {
	BHND_PMU_REGMODE_DEFAULT	= 0,	/**< default output mode */
	BHND_PMU_REGMODE_MODE_PWM	= 1,	/**< PWM output mode */
	BHND_PMU_REGMODE_MODE_BURST	= 2,	/**< burst output mode */
} bhnd_pmu_regulator_mode;

/**
 * bhnd_pmu(4) regulator attributes
 */
typedef enum bhnd_pmu_regulator_attr {
	BHND_PMU_REGULATOR_ATTR_VOUT			= 0,	/**< output voltage in millivolts */
	BHND_PMU_REGULATOR_ATTR_VOUT_RAW		= 1,	/**< raw output voltage value */
	BHND_PMU_REGULATOR_ATTR_PWRUP_LATCH_CTRL	= 2,	/**< power-up latch control */
} bhnd_pmu_regulator_attr;

#endif /* _BHND_CORES_PMU_BHND_PMU_TYPES_H_ */
