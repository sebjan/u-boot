/*
 * (C) Copyright 2003
 * Kyle Harris, kharris@nexus-tech.net
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
#include <command.h>

#if (CONFIG_COMMANDS & CFG_CMD_MMC)

#include <mmc.h>

int do_mmc (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong src_addr, dst_addr, size;
	char *cmd;

	if (argc == 1) {
		if (strncmp(argv[0],"mmcinit",7) !=0) {
			goto mmc_cmd_usage;
		} else {
			if (mmc_init (1) != 0) {
                        	printf ("No MMC card found\n");
                       		return 1;
                	}
		}
	}

	cmd = argv[1];
	if (strncmp(cmd, "read", 4) != 0 && strncmp(cmd, "write", 5) != 0
					&& strncmp(cmd, "erase", 5) != 0)
		goto mmc_cmd_usage;

	if (strcmp(cmd, "erase") == 0) {
		if (argc == 4) {
			src_addr = simple_strtoul(argv[2], NULL, 16);
			dst_addr = simple_strtoul(argv[3], NULL, 16);
		} else if (argc == 3) {
			/* TODO dst_addr should be end of CARD */
			src_addr = simple_strtoul(argv[2], NULL, 16);
			dst_addr = 0;
		} else {
			/* TODO dst_addr should be end of CARD */
			src_addr = 0;
			dst_addr = 0;
		}
		mmc_erase(src_addr,dst_addr);
	}
	src_addr = simple_strtoul(argv[2], NULL, 16);
        dst_addr = simple_strtoul(argv[3], NULL, 16);
        size = simple_strtoul(argv[4], NULL, 16);
	if (strcmp(cmd, "read") == 0) {
		mmc_read(src_addr,(unsigned char *)dst_addr,size);
	}
	if (strcmp(cmd, "write") == 0) {
		mmc_write((unsigned char *)src_addr,dst_addr, (int)size);
	}
	return 0;

mmc_cmd_usage:
	printf ("Usage:\n%s\n", cmdtp->usage);
	return 1;
}

U_BOOT_CMD(mmcinit, 5, 1, do_mmc,
	"mmcinit - init mmc card in FAT Mode\n"
	"mmc read src dst size - Read MMC src to RAM dst\n"
	"mmc write src dst size - WRITE RAM src to MMC dst\n"
	"mmc erase src  - ERASE entire MMC Card\n"
	"mmc erase src [dst] - ERASE from  src to end of card\n",
	NULL);
#endif	/* CFG_CMD_MMC */
