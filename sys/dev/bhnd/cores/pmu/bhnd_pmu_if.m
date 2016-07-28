#-
# Copyright (c) 2016 Landon Fuller <landon@landonf.org>
# All rights reserved.
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
# Provides a common PMU and clock control interface.
#

HEADER {
	struct bhnd_core_pmu_info;
}

/** 
 * Enabling routing of @p clock (or faster) to a requesting device.
 *
 * @param pmu PMU device.
 * @param pinfo PMU info for requesting device.
 * @param clock Clock requested.
 *
 * @retval 0 success
 * @retval ENODEV If an unsupported clock was requested.
 */
METHOD int request_clock {
	device_t			 pmu;
	struct bhnd_core_pmu_info	*pinfo;
	bhnd_clock			 clock;
};


/** 
 * Request that @p clocks be powered on behalf of a requesting device.
 *
 * This will power any clock sources (XTAL, PLL, etc,) required by
 * @p clocks and wait until they are ready, discarding any previous
 * requests from the @p pinfo device.
 *
 * Requests from multiple devices are aggregated by the PMU.
 *
 * @param pmu PMU device.
 * @param pinfo PMU info for requesting device.
 * @param clocks Clocks requested.
 *
 * @retval 0 success
 * @retval ENODEV If an unsupported clock was requested.
 */
METHOD int enable_clocks {
	device_t			 pmu;
	struct bhnd_core_pmu_info	*pinfo;
	uint32_t			 clocks;
};
