/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Aneesh V	<aneesh@ti.com>
 *	Steve Sakoman	<steve@sakoman.com>
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
#include <asm/io.h>
#include <asm/arch/sys_proto.h>

/*
 *  get_device_type(): tell if GP/HS/EMU/TST
 */
u32 get_device_type(void)
{
	return 0;
}

/*
 * get_board_rev() - get board revision
 */
u32 get_board_rev(void)
{
	return 0x20;
}

/*
 * Print CPU information
 */
int print_cpuinfo(void)
{

	puts("CPU  : OMAP4430\n");

	return 0;
}

/*
 * this uses the unique per-cpu info from the cpu fuses set at factory to
 * generate a 6-byte MAC address.  Two bits in the generated code are used
 * to elaborate the generated address into four, so it can be used on multiple
 * network interfaces.
 */

void omap4_die_id_to_ethernet_mac(u8 *mac, int subtype)
{
	struct ctrl_id *id_base = (struct ctrl_id *)(CTRL_BASE + 0x200);
	u32 idcode;
	u32 id[4];

	idcode = readl(&id_base->idcode);
	id[0] = readl(&id_base->die_id_0);
	id[1] = readl(&id_base->die_id_1);
	id[2] = readl(&id_base->die_id_2);
	id[3] = readl(&id_base->die_id_3);

	mac[0] = id[2];
	mac[1] = id[2] >> 8;
	mac[2] = id[1];
	mac[3] = id[1] >> 8;
	mac[4] = id[1] >> 16;
	mac[5] = id[1] >> 24;
	/* XOR other chip-specific data with ID */
	idcode ^= id[3];

	mac[0] ^= idcode;
	mac[1] ^= idcode >> 8;
	mac[2] ^= idcode >> 16;
	mac[3] ^= idcode >> 24;

	/* allow four MACs from this same basic data */
	mac[1] = (mac[1] & ~0xc0) | ((subtype & 3) << 6);

	/* mark it as not multicast and outside official 80211 MAC namespace */
	mac[0] = (mac[0] & ~1) | 2;
}
