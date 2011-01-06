/*
 * (C) Copyright 2004-2009 Texas Instruments, <www.ti.com>
 * Kishore Kadiyala <kishore.kadiyala@ti.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mem.h>

#if (CONFIG_FASTBOOT)
#include <fastboot.h>
#endif

void board_mmc_init(void)
{
#if (CONFIG_FASTBOOT)

	/* Partitons on EMMC preasent on OMAP4SDP required for Fastboot*/
	fastboot_ptentry ptn[10] = {
		{
			.name   = "ptable",
			.start  = 0x00, /*Sector Start */
			.length = 128*1024, /*128KB */
			.flags  = 0,
		},
		{
			.name   = "xloader",
			.start  = 0x100, /*Sector Start */
			.length = 128*1024, /*128KB */
			.flags  = 0,
		},
		{
			.name   = "bootloader",
			.start  = 0x200, /*Sector Start */
			.length = 256*1024, /*256KB */
			.flags  = 0,
		},
		{
			.name   = "environment",
			.start  = 0x400,  /* Sector Start */
			.length = 128*1024, /*128KB */
			.flags  = FASTBOOT_PTENTRY_FLAGS_WRITE_ENV,
		},
		{
			.name   = "boot",
			.start  = 0x500,  /* Sector Start */
			.length = 14*1024*1024, /*14MB */
			.flags  = 0,
		},
		{
			.name   = "recovery",
			.start  = 0x7500, /*Sector Start */
			.length = 14*1024*1024, /*14MB */
			.flags  = 0,
		},
		{
			.name   = "system",
			.start  = 0xE800,  /* Sector Start */
			.length = 384*1024*1024, /*384MB */
			.flags  = 0,
		},
		{
			.name   = "userdata",
			.start  = 0xCE800,  /* Sector Start */
			.length = 512*1024*1024, /*512MB */
			.flags  = 0,
		},
		{
			.name   = "cache",
			.start  = 0x1CE800,  /* Sector Start */
			.length = 128*1024*1024, /*128MB */
			.flags  = 0,
		},
		{
			.name   = "media",
			.start  = 0x20E800,  /* Sector Start */
			.length = 829*1024*1024, /*829MB */
			.flags  = 0,
		},
		/* Rest of the RAW Partitions can start from Sector start 0x27B622 */
	};
	int i;
	for (i = 0; i < 10; i++)
		fastboot_flash_add_ptn(&ptn[i]);
#endif
}
