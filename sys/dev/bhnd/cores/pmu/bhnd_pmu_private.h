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

/* Register I/O */
#define	BHND_PMU_READ_4(_sc, _reg)	(_sc)->io->rd4((_reg), (_sc)->io_ctx)
#define	BHND_PMU_WRITE_4(_sc, _reg, _val)	\
	(_sc)->io->wr4((_reg), (_val), (_sc)->io_ctx)

#define	BHND_PMU_AND_4(_sc, _reg, _val)		\
	BHND_PMU_WRITE_4((_sc), (_reg),		\
	    BHND_PMU_READ_4((_sc), (_reg)) & (_val))
#define	BHND_PMU_OR_4(_sc, _reg, _val)		\
	BHND_PMU_WRITE_4((_sc), (_reg),		\
	    BHND_PMU_READ_4((_sc), (_reg)) | (_val))

/* Indirect register support */
#define	BHND_PMU_IND_READ(_sc, _src, _reg)			\
	bhnd_pmu_ind_read((_sc)->io, (_sc)->io_ctx,		\
	    BHND_PMU_ ## _src ## _ADDR, BHND_PMU_ ## _src ## _DATA, (_reg))
#define	BHND_PMU_IND_WRITE(_sc, _src, _reg, _val, _mask)	\
	bhnd_pmu_ind_write((_sc)->io, (_sc)->io_ctx,		\
	    BHND_PMU_ ## _src ## _ADDR,				\
	    BHND_PMU_ ## _src ## _DATA, (_reg), (_val), (_mask))

/* Chip Control indirect registers */
#define	BHND_PMU_CCTRL_READ(_sc, _reg)			\
	BHND_PMU_IND_READ((_sc), CHIPCTL, (_reg))
#define	BHND_PMU_CCTRL_WRITE(_sc, _reg, _val, _mask)	\
	BHND_PMU_IND_WRITE((_sc), CHIPCTL, (_reg), (_val), (_mask))

/* Register Control indirect registers */
#define	BHND_PMU_REGCTRL_READ(_sc, _reg)			\
	BHND_PMU_IND_READ((_sc), REG_CONTROL, (_reg))
#define	BHND_PMU_REGCTRL_WRITE(_sc, _reg, _val, _mask)	\
	BHND_PMU_IND_WRITE((_sc), REG_CONTROL, (_reg), (_val), (_mask))

/* PLL Control indirect registers */
#define	BHND_PMU_PLL_READ(_sc, _reg)			\
	BHND_PMU_IND_READ((_sc), PLL_CONTROL, (_reg))
#define	BHND_PMU_PLL_WRITE(_sc, _reg, _val, _mask)	\
	BHND_PMU_IND_WRITE((_sc), PLL_CONTROL, (_reg), (_val), (_mask))

/**
 * Spin waiting until the clkst register matches @p _val with @p _mask,
 * or a maximum transition timeout occurs.
 * 
 * Returns true on success, false on timeout.
 */
#define	BHND_PMU_WAIT_CLKST(_sc, _val, _mask)			\
	bhnd_pmu_wait_clkst((_sc), (_sc)->dev, (_sc)->res,	\
	    BHND_CLK_CTL_ST, (_val), (_mask))


#define	BHND_PMU_LOG(_sc, _fmt, ...)	do {			\
	if (_sc->dev != NULL)					\
		device_printf(_sc->dev, _fmt, ##__VA_ARGS__);	\
	else							\
		printf(_fmt, ##__VA_ARGS__);			\
} while (0)

#define	BHND_PMU_DEBUG(_sc, _fmt, ...)	do {			\
	if (bootverbose) {					\
		BHND_PMU_LOG(_sc, "%s: "_fmt, __FUNCTION__,	\
		    ##__VA_ARGS__);				\
	}							\
} while (0)


/**
 * PMU common resource identifiers
 */
typedef enum {
	BHND_PMU_RSRC_PLL_PU		= 0,	/**< PLL power */
	BHND_PMU_RSRC_PA_REF_LDO	= 1,	/**< PA reference LDO */
	BHND_PMU_RSRC_INVALID	= UINT8_MAX
} bhnd_pmu_rsrc;

/* 
 * BHND PMU device quirks / features
 */
enum {
	/** No quirks */
	BPMU_QUIRK_NONE			= 0,

	/** On BCM4328-derived chipsets, the CLK_CTL_ST register CCS_HTAVAIL
	 *  and CCS_ALPAVAIL bits are swapped; the BHND_CCS0_* constants should
	 *  be used. */
	BPMU_QUIRK_CLKCTL_CCS0		= (1<<0),
};

/**
 * PMU resource ID lookup table.
 */
struct bhnd_pmu_rsrc_map {
	bhnd_pmu_rsrc	rsrc;		/**< common resource identifier */
	uint8_t		rsrc_num;	/**< chip-specific resource number */
};

#define	BHND_PMU_RSRC(_rsrc, _num)	\
	{ BHND_PMU_RSRC_ ## _rsrc, _num } 

#define	BHND_PMU_RSRC_NUM_INVALID	(sizeof(uint32_t) * 8)

#define	BHND_PMU_RSRC_MAP_END		\
    { BHND_PMU_RSRC_INVALID, BHND_PMU_RSRC_NUM_INVALID }

#define	BHND_PMU_RSRC_MAP_IS_END(_m)	\
	((_m)->rsrc == BHND_PMU_RSRC_INVALID)

/**
 * PMU hardware configuration.
 */
struct bhnd_pmu_hwcfg {
	struct bhnd_chip_match		 hwreq;		/**< chip match descriptor */
	uint32_t			 quirks;	/**< PMU quirks (see BPMU_QUIRK_*) */
	uint32_t			 alpfreq;	/**< fixed ALP frequency (kHz), or 0 */
	uint32_t			 bpfreq;	/**< fixed backplane frequency (kHz), or 0 */
	uint32_t			 cpufreq;	/**< fixed cpu frequency (kHz), or 0 */
	uint32_t			 xtalfreq;	/**< fixed xtal frequency (kHz), or 0 */
	const struct bhnd_pmu_ops	*ops;		/**< device-specific callbacks */
	const struct bhnd_pmu_rsrc_map	*rsrc_map;	/**< resource number mappings */
};

#define	BHND_PMU_HWCFG_END			\
	{ { BHND_MATCH_ANY }, 0, 0, 0, 0, 0, NULL }

#define	BHND_PMU_HWCFG_IS_END(_dev)		\
	(BHND_MATCH_IS_ANY(&(_dev)->hwreq) &&	\
	 (_dev)->rsrc_map == NULL)

/**
 * Probe the PMU hardware mapped by @p io and @p io_ctx, returning a standard
 * newbus device probe result (see BUS_PROBE_*), and the probed @p hwcfg on
 * success.
 *
 * @param[out]	hwcfg	On success, the probed PMU hardware configuration.
 * @param	io	PMU I/O callbacks.
 * @param	io_ctx	PMU I/O callback context.
 * @param	cid	The device's chip identifier.
 *
 * @retval 0		if this is the only possible device enumeration
 *			parser for the probed bus.
 * @retval negative	if the probe succeeds, a negative value should be
 *			returned; the parser returning the highest negative
 *			value will be selected to handle device enumeration.
 * @retval ENXIO	If the bhnd bus type is not handled by this parser.
 * @retval positive	if an error occurs during probing, a regular unix error
 *			code will be returned.
 */
typedef int		(bhnd_pmu_op_probe)(const struct bhnd_pmu_hwcfg **hwcfg,
			     const struct bhnd_pmu_io *io, void *io_ctx,
			     const struct bhnd_chipid *cid);

/**
 * Initialize PMU hardware for the given target frequency.
 * 
 * @param sc		PMU query instance.
 * @param xtalfreq	Target crystal frequency.
 */
typedef int		(bhnd_pmu_op_init)(struct bhnd_pmu_softc *sc,
			     uint32_t xtalfreq);

/**
 * Query and return a clock frequency, in Hz.
 * 
 * @param 	sc	PMU query instance.
 * @param[out]	hz	On success, the clock's frequency in Hz.
 * 
 * @retval 0		success
 * @retval ENODEV	if the clock does not exist.
 * @retval non-zero	if fetching the clock frequency otherwise fails, a
 *			regular unix error code will be returned.
 */
typedef int		(bhnd_pmu_op_clock_get)(struct bhnd_pmu_query *sc,
			     uint32_t *hz);

/**
 * PMU revision-specific operation callbacks.
 */
struct bhnd_pmu_ops {
	bhnd_pmu_op_probe	*probe;		/**< probe PMU hardware */
	bhnd_pmu_op_init	*init;		/**< initialize PMU hardware */
	bhnd_pmu_op_clock_get	*get_bp_clock;	/**< return the backplane clock frequency, in Hz */
	bhnd_pmu_op_clock_get	*get_cpu_clock;	/**< return the CPU clock frequency, in Hz */
	bhnd_pmu_op_clock_get	*get_mem_clock;	/**< return the memory clock frequency, in Hz */
	bhnd_pmu_op_clock_get	*get_alp_clock;	/**< return the ALP/XTAL clock frequency, in Hz */
};


/** Registered PMU operation callbacks */
SET_DECLARE(bhnd_pmu_ops_set, struct bhnd_pmu_ops);

/** Declare a set of PMU operation callbacks */
#define BHND_PMU_OPS_DECLARE(ops)   DATA_SET(bhnd_pmu_ops_set, ops)

/** FVCO frequencies, in Hz */
enum {
	FVCO_880	= 880	* 1000,	/**< 880MHz */
	FVCO_1760	= 1760	* 1000,	/**< 1760MHz */
	FVCO_1440	= 1440	* 1000,	/**< 1440MHz */
	FVCO_960	= 960	* 1000,	/**< 960MHz */
};

/** LDO voltage tunables */
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

int				bhnd_pmu_hwcfg_probe(
				    const struct bhnd_pmu_hwcfg **hwcfg,
				    const struct bhnd_pmu_io *io, void *io_ctx,
				    const struct bhnd_chipid *cid);

const struct bhnd_pmu_hwcfg	*bhnd_pmu_hwcfg_lookup(
				     const struct bhnd_chipid *cid,
				     const struct bhnd_pmu_hwcfg *entries,
				     size_t entry_size);

bool				 bhnd_pmu_rsrc_lookup(
				     const struct bhnd_pmu_hwcfg *hwcfg,
				     bhnd_pmu_rsrc rsrc, uint8_t *rsrc_num);

int				 bhnd_pmu_rsrc_on(struct bhnd_pmu_softc *sc,
				     bhnd_pmu_rsrc rsrc);
int				 bhnd_pmu_rsrc_off(struct bhnd_pmu_softc *sc,
				     bhnd_pmu_rsrc rsrc);
int				 bhnd_pmu_rsrc_uptime(struct bhnd_pmu_softc *sc,
				     bhnd_pmu_rsrc rsrc, uint32_t *uptime);

uint32_t	bhnd_pmu_ind_read(const struct bhnd_pmu_io *io, void *io_ctx,
		    bus_size_t addr, bus_size_t data, uint32_t reg);
void		bhnd_pmu_ind_write(const struct bhnd_pmu_io *io, void *io_ctx,
		    bus_size_t addr, bus_size_t data, uint32_t reg,
		    uint32_t val, uint32_t mask);

bool		bhnd_pmu_wait_clkst(struct bhnd_pmu_softc *sc, device_t dev,
		    struct bhnd_resource *r, bus_size_t clkst_reg,
		    uint32_t value, uint32_t mask);

int		bhnd_pmu_init(struct bhnd_pmu_softc *sc);
int		bhnd_pmu_pll_init(struct bhnd_pmu_softc *sc, uint32_t xtalfreq);
int		bhnd_pmu_res_init(struct bhnd_pmu_softc *sc);
void		bhnd_pmu_swreg_init(struct bhnd_pmu_softc *sc);

uint32_t	bhnd_pmu_force_ilp(struct bhnd_pmu_softc *sc, bool force);

void		bhnd_pmu_set_switcher_voltage(struct bhnd_pmu_softc *sc,
		    uint8_t bb_voltage, uint8_t rf_voltage);
void		bhnd_pmu_set_ldo_voltage(struct bhnd_pmu_softc *sc,
		    uint8_t ldo, uint8_t voltage);
int		bhnd_pmu_fast_pwrup_delay(struct bhnd_pmu_softc *sc,
		    uint16_t *pwrup_delay);
void		bhnd_pmu_rcal(struct bhnd_pmu_softc *sc);
void		bhnd_pmu_spuravoid(struct bhnd_pmu_softc *sc,
		    uint8_t spuravoid);

bool		bhnd_pmu_is_otp_powered(struct bhnd_pmu_softc *sc);
uint32_t	bhnd_pmu_measure_alpclk(struct bhnd_pmu_softc *sc);

int		bhnd_pmu_radio_enable(struct bhnd_pmu_softc *sc,
		    device_t d11core, bool enable);

uint32_t	bhnd_pmu_waitforclk_on_backplane(struct bhnd_pmu_softc *sc,
		    uint32_t clk, uint32_t delay);

int		bhnd_pmu_otp_power(struct bhnd_pmu_softc *sc, bool on);
void		bhnd_pmu_sdiod_drive_strength_init(struct bhnd_pmu_softc *sc,
		    uint32_t drivestrength);

void		bhnd_pmu_paref_ldo_enable(struct bhnd_pmu_softc *sc,
		    bool enable);

#endif /* _BHND_CORES_PMU_BHND_PMU_PRIVATE_H_ */
