/*-
 * Copyright (c) 2015 Landon Fuller <landon@landonf.org>
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

#ifndef _BHND_CORES_PMU_BHND_PMUVAR_H_
#define _BHND_CORES_PMU_BHND_PMUVAR_H_

#include <sys/types.h>
#include <sys/rman.h>

DECLARE_CLASS(bhnd_pmu_driver);
extern devclass_t bhnd_pmu_devclass;

int	bhnd_pmu_probe(device_t dev);
int	bhnd_pmu_attach(device_t dev);
int	bhnd_pmu_detach(device_t dev);
int	bhnd_pmu_suspend(device_t dev);
int	bhnd_pmu_resume(device_t dev);

// XXX begin needs cleanup

struct bhnd_pmu_softc;

#define SET_LDO_VOLTAGE_LDO1	1
#define SET_LDO_VOLTAGE_LDO2	2
#define SET_LDO_VOLTAGE_LDO3	3
#define SET_LDO_VOLTAGE_PAREF	4
#define SET_LDO_VOLTAGE_CLDO_PWM	5
#define SET_LDO_VOLTAGE_CLDO_BURST	6
#define SET_LDO_VOLTAGE_CBUCK_PWM	7
#define SET_LDO_VOLTAGE_CBUCK_BURST	8
#define SET_LDO_VOLTAGE_LNLDO1	9
#define SET_LDO_VOLTAGE_LNLDO2_SEL	10

extern void bhnd_pmu_init(struct bhnd_pmu_softc *sih);
extern void bhnd_pmu_chip_init(struct bhnd_pmu_softc *sih);
extern void bhnd_pmu_pll_init(struct bhnd_pmu_softc *sih, uint32_t xtalfreq);
extern void bhnd_pmu_res_init(struct bhnd_pmu_softc *sih);
extern void bhnd_pmu_swreg_init(struct bhnd_pmu_softc *sih);

extern uint32_t bhnd_pmu_force_ilp(struct bhnd_pmu_softc *sih, bool force);

extern uint32_t bhnd_pmu_si_clock(struct bhnd_pmu_softc *sih);
extern uint32_t bhnd_pmu_cpu_clock(struct bhnd_pmu_softc *sih);
extern uint32_t bhnd_pmu_mem_clock(struct bhnd_pmu_softc *sih);
extern uint32_t bhnd_pmu_alp_clock(struct bhnd_pmu_softc *sih);
extern uint32_t bhnd_pmu_ilp_clock(struct bhnd_pmu_softc *sih);

extern void bhnd_pmu_set_switcher_voltage(struct bhnd_pmu_softc *sih,
					uint8_t bb_voltage, uint8_t rf_voltage);
extern void bhnd_pmu_set_ldo_voltage(struct bhnd_pmu_softc *sih, uint8_t ldo, uint8_t voltage);
extern uint16_t bhnd_pmu_fast_pwrup_delay(struct bhnd_pmu_softc *sih);
extern void bhnd_pmu_rcal(struct bhnd_pmu_softc *sih);
extern void bhnd_pmu_spuravoid(struct bhnd_pmu_softc *sih, uint8_t spuravoid);

extern bool bhnd_pmu_is_otp_powered(struct bhnd_pmu_softc *sih);
extern uint32_t bhnd_pmu_measure_alpclk(struct bhnd_pmu_softc *sih);

extern void bhnd_pmu_sprom_enable(struct bhnd_pmu_softc *sih, bool enable);

extern void bhnd_pmu_radio_enable(struct bhnd_pmu_softc *sih, bool enable);
extern uint32_t bhnd_pmu_waitforclk_on_backplane(struct bhnd_pmu_softc *sih, uint32_t clk, uint32_t delay);

extern void bhnd_pmu_otp_power(struct bhnd_pmu_softc *sih, bool on);
extern void si_sdiod_drive_strength_init(struct bhnd_pmu_softc *sih, uint32_t drivestrength);

// XXX end needs cleanup

/* 
 * BHND PMU device quirks / features
 */
enum {
	/** No quirks */
	BPMU_QUIRK_NONE			= 0,
};

/**
 * bhnd_pmu driver instance state.
 */
struct bhnd_pmu_softc {
	device_t		 dev;
	uint32_t		 quirks;	/**< device quirk flags */

	uint32_t		 caps;		/**< pmu capability flags. */

	struct bhnd_resource	*res;		/**< pmu register block. */
	int			 rid;		/**< pmu register RID */

	struct mtx		 mtx;		/**< state mutex */
};

#define	BPMU_LOCK_INIT(sc) \
	mtx_init(&(sc)->mtx, device_get_nameunit((sc)->dev), \
	    "BHND chipc driver lock", MTX_DEF)
#define	BPMU_LOCK(sc)				mtx_lock(&(sc)->mtx)
#define	BPMU_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)
#define	BPMU_LOCK_ASSERT(sc, what)		mtx_assert(&(sc)->mtx, what)
#define	BPMU_LOCK_DESTROY(sc)			mtx_destroy(&(sc)->mtx)

#endif /* _BHND_CORES_PMU_BHND_PMUVAR_H_ */
