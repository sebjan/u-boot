/*
 * (C) Copyright 2010
 * Texas Instruments Incorporated, <www.ti.com>
 * Steve Sakoman  <steve@sakoman.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>

#include "panda_mux_data.h"

DECLARE_GLOBAL_DATA_PTR;

const struct omap_sysinfo sysinfo = {
	"Board: OMAP4 Panda\n"
};

/**
 * @brief board_init
 *
 * @return 0
 */
int board_init(void)
{
	gpmc_init();

	gd->bd->bi_arch_number = MACH_TYPE_OMAP4_PANDA;
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */

	return 0;
}

/*
 * Panda has a usb nic with no ethernet rom so generate a macaddr
 * from the SOC die-id and set the usbethaddr env var to that
 * value.
 */
int board_eth_init(bd_t *bis)
{
	u8 macaddr[6];
	char usbethaddr[20];

	/*
	 * NB: The 1 here has been found to generate an address
	 * consistent with the kernel.
	 */
	omap4_die_id_to_ethernet_mac(macaddr, 1);
	sprintf (usbethaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
		 macaddr[0], macaddr[1],
		 macaddr[2], macaddr[3],
		 macaddr[4], macaddr[5]) ;
	setenv ("usbethaddr", usbethaddr);
	return 0;
}

/**
 * @brief misc_init_r - Configure Panda board specific configurations
 * such as power configurations, ethernet initialization as phase2 of
 * boot sequence
 *
 * @return 0
 */
int misc_init_r(void)
{
	return 0;
}

void set_muxconf_regs_non_essential(void)
{
	/*
	 * Enable USB phy ref clock before reconfiguring pins because
	 * the phy seems to get in a bad state otherwise.
	 */
#define OMAP44XX_SCRM_BASE		0x4a30a000
#define OMAP44XX_SCRM_ALTCLKSRC		(OMAP44XX_SCRM_BASE + 0x110)
#define OMAP44XX_SCRM_AUXCLK1		(OMAP44XX_SCRM_BASE + 0x314)
#define OMAP44XX_SCRM_AUXCLK3		(OMAP44XX_SCRM_BASE + 0x31c)
	/* enable software ioreq */
	sr32(OMAP44XX_SCRM_AUXCLK3, 8, 1, 0x1);
	/* set for sys_clk (38.4MHz) */
	sr32(OMAP44XX_SCRM_AUXCLK3, 1, 2, 0x0);
	/* set divisor to 2 */
	sr32(OMAP44XX_SCRM_AUXCLK3, 16, 4, 0x1);
	/* set the clock source to active */
	sr32(OMAP44XX_SCRM_ALTCLKSRC, 0, 1, 0x1);
	/* enable clocks */
	sr32(OMAP44XX_SCRM_ALTCLKSRC, 2, 2, 0x3);

	do_set_mux(CONTROL_PADCONF_CORE, core_padconf_array_non_essential,
		   sizeof(core_padconf_array_non_essential) /
		   sizeof(struct pad_conf_entry));

	do_set_mux(CONTROL_PADCONF_WKUP, wkup_padconf_array_non_essential,
		   sizeof(wkup_padconf_array_non_essential) /
		   sizeof(struct pad_conf_entry));
}

#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0);
	return 0;
}
#endif
