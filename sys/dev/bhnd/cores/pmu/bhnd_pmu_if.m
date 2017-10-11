#-
# Copyright (c) 2016 Landon Fuller <landon@landonf.org>
# Copyright (c) 2017 The FreeBSD Foundation
# All rights reserved.
#
# Portions of this software were developed by Landon Fuller
# under sponsorship from the FreeBSD Foundation.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# $FreeBSD$

#include <sys/types.h>
#include <sys/bus.h>

#include <dev/bhnd/bhnd.h>

INTERFACE bhnd_pmu;

#
# bhnd(4) PMU interface.
#
# Provides an interface to the PMU hardware found on modern bhnd(4) chipsets.
#

HEADER {
	#include <dev/bhnd/cores/pmu/bhnd_pmu_types.h>

	struct bhnd_core_pmu_info;
}

/**
 * Return the current value of a PMU chipctrl register.
 *
 * @param dev A bhnd(4) PMU device.
 * @param reg The PMU chipctrl register to be read.
 *
 * Drivers should only use function for functionality that is not
 * available via another bhnd_chipc() function.
 *
 * @returns The chipctrl register value, or 0 if undefined by this hardware.
 */
METHOD uint32_t read_chipctrl {
	device_t dev;
	uint32_t reg;
}

/**
 * Write @p value with @p mask to a PMU chipctrl register.
 *
 * @param dev A bhnd(4) PMU device.
 * @param reg The PMU chipctrl register to be written.
 * @param value The value to write.
 * @param mask The mask of bits to be written from @p value.
 *
 * Drivers should only use function for functionality that is not
 * available via another bhnd_pmu() function.
 */
METHOD void write_chipctrl {
	device_t dev;
	uint32_t reg;
	uint32_t value;
	uint32_t mask;
}

/**
 * Return the current value of a PMU regulator control register.
 *
 * @param dev A bhnd(4) PMU device.
 * @param reg The PMU regctrl register to be read.
 *
 * Drivers should only use function for functionality that is not
 * available via another bhnd_chipc() function.
 *
 * @returns The regctrl register value, or 0 if undefined by this hardware.
 */
METHOD uint32_t read_regctrl {
	device_t dev;
	uint32_t reg;
};

/**
 * Write @p value with @p mask to a PMU regulator control register.
 *
 * @param dev A bhnd(4) PMU device.
 * @param reg The PMU regctrl register to be written.
 * @param value The value to write.
 * @param mask The mask of bits to be written from @p value.
 *
 * Drivers should only use function for functionality that is not
 * available via another bhnd_pmu() function.
 */
METHOD void write_regctrl {
	device_t dev;
	uint32_t reg;
	uint32_t value;
	uint32_t mask;
};

/**
 * Return the current value of a PMU PLL control register.
 *
 * @param dev A bhnd(4) PMU device.
 * @param reg The PMU pllctrl register to be read.
 *
 * Drivers should only use function for functionality that is not
 * available via another bhnd_chipc() function.
 *
 * @returns The pllctrl register value, or 0 if undefined by this hardware.
 */
METHOD uint32_t read_pllctrl {
	device_t dev;
	uint32_t reg;
};

/**
 * Write @p value with @p mask to a PMU PLL control register.
 *
 * @param dev A bhnd(4) PMU device.
 * @param reg The PMU pllctrl register to be written.
 * @param value The value to write.
 * @param mask The mask of bits to be written from @p value.
 *
 * Drivers should only use function for functionality that is not
 * available via another bhnd_pmu() function.
 */
METHOD void write_pllctrl {
	device_t dev;
	uint32_t reg;
	uint32_t value;
	uint32_t mask;
};

/**
 * Request that the PMU configure itself for a given hardware-specific
 * spuravoid mode.
 *
 * @param dev PMU device.
 * @param spuravoid The requested mode.
 *
 * @retval 0 success
 * @retval ENODEV If @p regulator is not supported by this driver.
 */
 METHOD int request_spuravoid {
	 device_t dev;
	 bhnd_pmu_spuravoid spuravoid;
 }

/**
 * Set a hardware-specific output voltage register value for @p regulator.
 *
 * @param dev PMU device.
 * @param regulator Regulator to be configured.
 * @param value The raw voltage register value.
 *
 * @retval 0 success
 * @retval ENODEV If @p regulator is not supported by this driver.
 */
 METHOD int set_voltage_raw {
	 device_t dev;
	 bhnd_pmu_regulator regulator;
	 uint32_t value;
 }

/**
 * Enable the given @p regulator.
 *
 * @param dev PMU device.
 * @param regulator Regulator to be enabled.
 *
 * @retval 0 success
 * @retval ENODEV If @p regulator is not supported by this driver.
 */
METHOD int enable_regulator {
	device_t dev;
	bhnd_pmu_regulator regulator;
};

/**
 * Disable the given @p regulator.
 *
 * @param dev PMU device.
 * @param regulator Regulator to be disabled.
 *
 * @retval 0 success
 * @retval ENODEV If @p regulator is not supported by this driver.
 */
METHOD int disable_regulator {
	device_t dev;
	bhnd_pmu_regulator regulator;
};

/**
 * Return the frequency for @p clock in Hz, if known.
 *
 * @param dev PMU device.
 * @param clock The clock to be queried.
 * @param[out] freq On success, the frequency of @p clock in Hz.
 * 
 * @retval 0 success
 * @retval ENODEV If the frequency for @p clock is not available.
 */
METHOD int get_clock_freq {
	device_t dev;
	bhnd_clock clock;
	uint32_t *freq;
};

/**
 * Return the transition latency required for @p clock in microseconds, if
 * known.
 *
 * The BHND_CLOCK_HT latency value is suitable for use as the D11 core's
 * 'fastpwrup_dly' value.
 *
 * @param	dev	PMU device.
 * @param	clock	The clock to be queried for transition latency.
 * @param[out]	latency	On success, the transition latency of @p clock in
 *			microseconds.
 * 
 * @retval 0 success
 * @retval ENODEV If the transition latency for @p clock is not available.
 */
METHOD int get_clock_latency {
	device_t dev;
	bhnd_clock clock;
	u_int *latency;
};

/**
 * Return the PMU's maximum state transition latency in microseconds.
 *
 * This upper bound may be used to busy-wait on PMU clock and resource state
 * transitions.
 *
 * @param dev PMU device.
 */
METHOD u_int get_max_transition_latency {
	device_t dev;
};
