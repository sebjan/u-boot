/*
 * (C) Copyright 2011 Freescale Semiconductor, Inc.
 * Based on drivers/misc/fsl_pmic.c
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

#include <config.h>
#include <common.h>
#include <i2c.h>
#include <asm/errno.h>
#include <linux/types.h>

static int check_param(u32 reg, u32 write)
{
	if (reg > 142 || write > 1) {
		printf("<reg num> = %d is invalid. Should be less then 142\n",
			reg);
		return -1;
	}

	return 0;
}

static u32 pmic_reg(u32 reg, u32 val, u32 write)
{
	unsigned char buf[1] = { 0 };
	u32 ret_val = 0;

	if (check_param(reg, write))
		return -1;

	if (write) {
		buf[0] = (val) & 0xff;
		if (i2c_write(CONFIG_SYS_DIALOG_PMIC_I2C_ADDR, reg, 1, buf, 1))
			return -1;
	} else {
		if (i2c_read(CONFIG_SYS_DIALOG_PMIC_I2C_ADDR, reg, 1, buf, 1))
			return -1;
		ret_val = buf[0];
	}

	return ret_val;
}

void pmic_reg_write(u32 reg, u32 value)
{
	pmic_reg(reg, value, 1);
}

u32 pmic_reg_read(u32 reg)
{
	return pmic_reg(reg, 0, 0);
}

static void pmic_dump(int numregs)
{
	u32 val;
	int i;

	for (i = 0; i < numregs; i++) {
		val = pmic_reg_read(i);
		if (!(i % 8))
			printf("\n0x%02x: ", i);
		printf("%02x ", val);
	}
	puts("\n");
}

int do_pmic(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *cmd;
	int nregs;
	u32 val;

	/* at least two arguments please */
	if (argc < 2)
		return cmd_usage(cmdtp);

	cmd = argv[1];
	if (strcmp(cmd, "dump") == 0) {
		if (argc < 3)
			return cmd_usage(cmdtp);

		nregs = simple_strtoul(argv[2], NULL, 16);
		pmic_dump(nregs);
		return 0;
	}
	if (strcmp(cmd, "write") == 0) {
		if (argc < 4)
			return cmd_usage(cmdtp);

		nregs = simple_strtoul(argv[2], NULL, 16);
		val = simple_strtoul(argv[3], NULL, 16);
		pmic_reg_write(nregs, val);
		return 0;
	}
	/* No subcommand found */
	return 1;
}

U_BOOT_CMD(
	pmic,	CONFIG_SYS_MAXARGS, 1, do_pmic,
	"Dialog PMIC (DA905x)",
	"dump [numregs] dump registers\n"
	"pmic write <reg> <value> - write register"
);
