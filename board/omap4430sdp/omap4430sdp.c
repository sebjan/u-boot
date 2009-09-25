/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
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
#include <asm/arch/cpu.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <i2c.h>
#include <asm/mach-types.h>
#if (CONFIG_COMMANDS & CFG_CMD_NAND) && defined(CFG_NAND_LEGACY)
#include <linux/mtd/nand_legacy.h>
#endif

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b" : "=r" (loops) : "0"(loops));
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init();		/* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP_4430SDP;
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */

	return 0;
}

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access
 * (GP Device only)
 *****************************************/
void secure_unlock_mem(void)
{
	/* Permission values for registers -Full fledged permissions to all */
	#define UNLOCK_1 0xFFFFFFFF
	#define UNLOCK_2 0x00000000
	#define UNLOCK_3 0x0000FFFF

	/* Protection Module Register Target APE (PM_RT)*/
	__raw_writel(UNLOCK_1, RT_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_1, RT_READ_PERMISSION_0);
	__raw_writel(UNLOCK_1, RT_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, RT_ADDR_MATCH_1);

	__raw_writel(UNLOCK_3, GPMC_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, OCM_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, OCM_ADDR_MATCH_2);

	/* IVA Changes */
	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_1);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_1);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_2);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_2);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_2);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_3);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_3);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_3);

	__raw_writel(UNLOCK_1, SMS_RG_ATT0); /* SDRC region 0 public */
}


/**********************************************************
 * Routine: secureworld_exit()
 * Description: If chip is EMU and boot type is external
 *		configure secure registers and exit secure world
 *  general use.
 ***********************************************************/

void secureworld_exit(void)
{
	unsigned long i;

	/* configrue non-secure access control register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 2" : "=r" (i));
	/* enabling co-processor CP10 and CP11 accesses in NS world */
	__asm__ __volatile__("orr %0, %0, #0xC00" : "=r"(i));
	/* allow allocation of locked TLBs and L2 lines in NS world */
	/* allow use of PLE registers in NS world also */
	__asm__ __volatile__("orr %0, %0, #0x70000" : "=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 2" : "=r" (i));

	/*
	 * Enable IBE in ACR register.ASA is disabled following
	 * recommendation from ARM
	 */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 1" : "=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x40" : "=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 1" : "=r" (i));

	/* Exiting secure world */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 0" : "=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x31" : "=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 0" : "=r" (i));
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP/EMU(special) type, unlock the SRAM for
 *  general use.
 ***********************************************************/
int get_boot_type(void);
void v7_flush_dcache_all(int , int);
void setup_auxcr(int , int);

void try_unlock_memory(void)
{
	int mode;
	int in_sdram = running_in_sdram();

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T*/
	mode = get_device_type();
	if (mode == GP_DEVICE)
		secure_unlock_mem();

	/* If device is EMU and boot is XIP external booting
	 * Unlock firewalls and disable L2 and put chip
	 * out of secure world
	 */
	/* Assuming memories are unlocked by the demon who put us in SDRAM */
	if ((mode <= EMU_DEVICE) && (get_boot_type() == 0x1F)
		&& (!in_sdram)) {
		secure_unlock_mem();
		secureworld_exit();
	}

	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called path is with SRAM stack.
 **********************************************************/

void s_init(void)
{
	int external_boot = 0;
	int in_sdram = running_in_sdram();

#ifdef CONFIG_4430SDP
	in_sdram = 0;  /* allow setup from memory for Virtio */
#endif
	watchdog_init();

	external_boot = (get_boot_type() == 0x1F) ? 1 : 0;
	/* Right now flushing at low MPU speed. Need to move after clock init */
	v7_flush_dcache_all(get_device_type(), external_boot);

	try_unlock_memory();

#ifndef CONFIG_ICACHE_OFF
	icache_enable();
#endif

	set_muxconf_regs();
	delay(100);

	/* Writing to AuxCR in U-boot using SMI for GP/EMU DEV */
	/* Currently SMI in Kernel on ES2 devices seems to have an isse
	 * Once that is resolved, we can postpone this config to kernel
	 */
	setup_auxcr(get_device_type(), external_boot);
/*
	prcm_init();

	per_clocks_enable();
*/

	if (!in_sdram)
		dmm_init();

}

/*******************************************************
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 ********************************************************/
int misc_init_r(void)
{
#ifdef CONFIG_DRIVER_OMAP44XX_I2C
	i2c_init(CFG_I2C_SPEED, CFG_I2C_SLAVE);
#endif
	return 0;
}

/******************************************************
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 ******************************************************/
void wait_for_command_complete(unsigned int wd_base)
{
	int pending = 1;
	do {
		pending = __raw_readl(wd_base + WWPS);
	} while (pending);
}

/****************************************
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 *****************************************/
void watchdog_init(void)
{
	/* There are 3 watch dogs WD1=Secure, WD2=MPU, WD3=IVA. WD1 is
	 * either taken care of by ROM (HS/EMU) or not accessible (GP).
	 * We need to take care of WD2-MPU or take a PRCM reset.  WD3
	 * should not be running and does not generate a PRCM reset.
	 */

	sr32(CM_FCLKEN_WKUP, 5, 1, 1);
	sr32(CM_ICLKEN_WKUP, 5, 1, 1);
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5); /* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/*******************************************************************
 * Routine:ether_init
 * Description: take the Ethernet controller out of reset and wait
 *  		   for the EEPROM load to complete.
 ******************************************************************/
void ether_init(void)
{
#ifdef CONFIG_DRIVER_LAN91C96
	int cnt = 20;

	__raw_writew(0x0, LAN_RESET_REGISTER);
	do {
		__raw_writew(0x1, LAN_RESET_REGISTER);
		udelay(100);
		if (cnt == 0)
			goto h4reset_err_out;
		--cnt;
	} while (__raw_readw(LAN_RESET_REGISTER) != 0x1);

	cnt = 20;

	do {
		__raw_writew(0x0, LAN_RESET_REGISTER);
		udelay(100);
		if (cnt == 0)
			goto h4reset_err_out;
		--cnt;
	} while (__raw_readw(LAN_RESET_REGISTER) != 0x0000);
	udelay(1000);

	*((volatile unsigned char *)ETH_CONTROL_REG) &= ~0x01;
	udelay(1000);

       h4reset_err_out:
	return;
#endif
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init(void)
{
    #define NOT_EARLY 0
    DECLARE_GLOBAL_DATA_PTR;
	unsigned int size0 = 0, size1 = 0;
	u32 mtype, btype;

	btype = get_board_type();
	mtype = get_mem_type();
	/* fixme... dont know why this func is crashing in ZeBu */
	display_board_info(btype);
    /* If a second bank of DDR is attached to CS1 this is
     * where it can be started.  Early init code will init
     * memory on CS0.
     */
	if ((mtype == DDR_COMBO) || (mtype == DDR_STACKED))
		do_dmm_init(0x30, NOT_EARLY);

	size0 = SZ_128M;
	size1 = 0;

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = size0;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_1+size0;
	gd->bd->bi_dram[1].size = size1;

	return 0;
}


#define		CP(x)	(CONTROL_PADCONF_##x)
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */


/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
       /* REVISIT */
	return;
}

/******************************************************************************
 * Routine: update_mux()
 * Description:Update balls which are different between boards.  All should be
 *             updated to match functionality.  However, I'm only updating ones
 *             which I'll be using for now.  When power comes into play they
 *             all need updating.
 *****************************************************************************/
void update_mux(u32 btype, u32 mtype)
{
	/* REVISIT  */
	return;

}

#if (CONFIG_COMMANDS & CFG_CMD_NAND) && defined(CFG_NAND_LEGACY)
/**********************************************************
 * Routine: nand+_init
 * Description: Set up nand for nand and jffs2 commands
 *********************************************************/
void nand_init(void)
{
	/* REVISIT */
	return;
}
#endif

