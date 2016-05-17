/*-
 * Copyright (c) 2016 Michael Zhilin <mizhka@gmail.com>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/errno.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/queue.h>

#include <machine/resource.h>
#include <dev/bhnd/bhndvar.h>
#include <dev/bhnd/bhnd_debug.h>

#include "bus_if.h"
#include "chipc.h"
#include "chipcvar.h"
#include "chipcreg.h"

static void	chipc_print_capabilities(struct chipc_capabilities* capabilities);

static void
chipc_print_capabilities(struct chipc_capabilities* capabilities)
{

	BHND_DEBUG("UARTs: 0x%01x", capabilities->num_uarts);
	BHND_DEBUG("BigEngian: 0x%01x", capabilities->is_bigend);
	BHND_DEBUG("UART-GPIO: 0x%01x", capabilities->uart_gpio);
	BHND_DEBUG("UART Clock: 0x%01x", capabilities->uart_clock);
	BHND_DEBUG("Flash type: 0x%x", capabilities->flash_type);

	BHND_DEBUG("External buses: 0x%x",  capabilities->external_buses);
	BHND_DEBUG("Power control: 0x%01x", capabilities->power_control);
	BHND_DEBUG("JTAG master: 0x%01x", capabilities->jtag_master);

	BHND_DEBUG("PLL Type: 0x%x", capabilities->pll_type);
	BHND_DEBUG("OTP size: 0x%01x", capabilities->otp_size);
	BHND_DEBUG("Is 64bit? 0x%01x", capabilities->is_64bit);
	BHND_DEBUG("Boot ROM: 0x%01x", capabilities->boot_rom);
	BHND_DEBUG("PMU: 0x%01x", capabilities->pmu);
	BHND_DEBUG("ECI: 0x%01x", capabilities->eci);
	BHND_DEBUG("SPROM: 0x%01x", capabilities->sprom);
	BHND_DEBUG("NFLASH: 0x%01x", capabilities->nflash);
}

void
chipc_parse_capabilities(struct chipc_capabilities* capabilities,
		uint32_t caps)
{

	capabilities->num_uarts = GET_BITS(caps, CHIPC_CAP_UARTS);
	capabilities->is_bigend = GET_BITS(caps, CHIPC_CAP_MIPSEB);
	capabilities->uart_gpio = GET_BITS(caps, CHIPC_CAP_UARTGPIO);
	capabilities->uart_clock= GET_BITS(caps, CHIPC_CAP_UCLKSEL);
	capabilities->flash_type= GET_BITS(caps, CHIPC_CAP_FLASH);

	capabilities->external_buses = GET_BITS(caps, CHIPC_CAP_EXTBUS);
	capabilities->power_control = GET_BITS(caps, CHIPC_CAP_PWR_CTL);
	capabilities->jtag_master = GET_BITS(caps, CHIPC_CAP_JTAGP);

	capabilities->pll_type = GET_BITS(caps, CHIPC_CAP_PLL);
	capabilities->otp_size = GET_BITS(caps, CHIPC_CAP_OTP_SIZE);
	capabilities->is_64bit = GET_BITS(caps, CHIPC_CAP_BKPLN64);
	capabilities->boot_rom = GET_BITS(caps, CHIPC_CAP_ROM);
	capabilities->pmu = GET_BITS(caps, CHIPC_CAP_PMU);
	capabilities->eci = GET_BITS(caps, CHIPC_CAP_ECI);
	capabilities->sprom = GET_BITS(caps, CHIPC_CAP_SPROM);
	capabilities->nflash = GET_BITS(caps, CHIPC_CAP_NFLASH);

	if(bootverbose)
		chipc_print_capabilities(capabilities);
}
