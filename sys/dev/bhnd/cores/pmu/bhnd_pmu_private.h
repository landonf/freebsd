/*-
 * Copyright (c) 2016 Landon Fuller <landonf@FreeBSD.org>
 * Copyright (C) 2010, Broadcom Corporation.
 * All rights reserved.
 *
 * This file is derived from the hndpmu.h header contributed by Broadcom 
 * to to the Linux staging repository, as well as later revisions of hndpmu.h
 * distributed with the Asus RT-N16 firmware source code release.
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

#ifndef _BHND_CORES_PMU_BHND_PMU_PRIVATE_H_
#define _BHND_CORES_PMU_BHND_PMU_PRIVATE_H_

#include <sys/types.h>

#include "bhnd_pmuvar.h"


/* FVCO frequencies, in Hz */
enum {
	FVCO_880	= 880	* 1000,	/**< 880MHz */
	FVCO_1760	= 1760	* 1000,	/**< 1760MHz */
	FVCO_1440	= 1440	* 1000,	/**< 1440MHz */
	FVCO_960	= 960	* 1000,	/**< 960MHz */
};

enum {
	SET_LDO_VOLTAGE_LDO1		= 1,
	SET_LDO_VOLTAGE_LDO2		= 2,
	SET_LDO_VOLTAGE_LDO3		= 3,
	SET_LDO_VOLTAGE_PAREF		= 4,
	SET_LDO_VOLTAGE_CLDO_PWM	= 5,
	SET_LDO_VOLTAGE_CLDO_BURST	= 6,
	SET_LDO_VOLTAGE_CBUCK_PWM	= 7,
	SET_LDO_VOLTAGE_CBUCK_BURST	= 8,
	SET_LDO_VOLTAGE_LNLDO1		= 9,
	SET_LDO_VOLTAGE_LNLDO2_SEL	= 10,
};

void		bhnd_pmu_init(struct bhnd_pmu_softc *sc);
void		bhnd_pmu_pll_init(struct bhnd_pmu_softc *sc, uint32_t xtalfreq);
void		bhnd_pmu_res_init(struct bhnd_pmu_softc *sc);
void		bhnd_pmu_swreg_init(struct bhnd_pmu_softc *sc);

uint32_t	bhnd_pmu_force_ilp(struct bhnd_pmu_softc *sc, bool force);

uint32_t	bhnd_pmu_si_clock(struct bhnd_pmu_softc *sc);
uint32_t	bhnd_pmu_cpu_clock(struct bhnd_pmu_softc *sc);
uint32_t	bhnd_pmu_mem_clock(struct bhnd_pmu_softc *sc);
uint32_t	bhnd_pmu_alp_clock(struct bhnd_pmu_softc *sc);
uint32_t	bhnd_pmu_ilp_clock(struct bhnd_pmu_softc *sc);

void		bhnd_pmu_set_switcher_voltage(struct bhnd_pmu_softc *sc,
		    uint8_t bb_voltage, uint8_t rf_voltage);
void		bhnd_pmu_set_ldo_voltage(struct bhnd_pmu_softc *sc,
		    uint8_t ldo, uint8_t voltage);
uint16_t	bhnd_pmu_fast_pwrup_delay(struct bhnd_pmu_softc *sc);
void		bhnd_pmu_rcal(struct bhnd_pmu_softc *sc);
void		bhnd_pmu_spuravoid(struct bhnd_pmu_softc *sc,
		    uint8_t spuravoid);

bool		bhnd_pmu_is_otp_powered(struct bhnd_pmu_softc *sc);
uint32_t	bhnd_pmu_measure_alpclk(struct bhnd_pmu_softc *sc);

void		bhnd_pmu_radio_enable(struct bhnd_pmu_softc *sc,
		    device_t d11core, bool enable);

uint32_t	bhnd_pmu_waitforclk_on_backplane(struct bhnd_pmu_softc *sc,
		    uint32_t clk, uint32_t delay);

int		bhnd_pmu_otp_power(struct bhnd_pmu_softc *sc, bool on);
void		bhnd_pmu_sdiod_drive_strength_init(struct bhnd_pmu_softc *sc,
		    uint32_t drivestrength);

void		bhnd_pmu_paref_ldo_enable(struct bhnd_pmu_softc *sc,
		    bool enable);

#endif /* _BHND_CORES_PMU_BHND_PMU_PRIVATE_H_ */
