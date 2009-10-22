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
#include <asm/arch/mux.h>
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
	/* TODO: Disable Watchdog's here if needed.
	watchdog_init();
	*/

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

	prcm_init();

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
#if 0
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
#endif

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

#define		OMAP44XX_WKUP_CTRL_BASE		0x4A31E000
#if 1
#define M0_SAFE M0
#define M1_SAFE M1
#define M2_SAFE M2
#define M4_SAFE M4
#define M7_SAFE M7
#define M3_SAFE M3
#define M5_SAFE M5
#define M6_SAFE M6
#else
#define M0_SAFE M7
#define M1_SAFE M7
#define M2_SAFE M7
#define M4_SAFE M7
#define M7_SAFE M7
#define M3_SAFE M7
#define M5_SAFE M7
#define M6_SAFE M7
#endif
#define		MV(OFFSET, VALUE)\
			__raw_writew((VALUE), OMAP44XX_CTRL_BASE + (OFFSET));
#define		MV1(OFFSET, VALUE)\
			__raw_writew((VALUE), OMAP44XX_WKUP_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)
#define		WK(x)	(CONTROL_WKUP_##x)
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

#define MUX_DEFAULT_OMAP4() \
	MV(CP(GPMC_AD0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M3)) /* gpio_32 */ \
	MV(CP(GPMC_AD13),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_37 */ \
	MV(CP(GPMC_AD14),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_38 */ \
	MV(CP(GPMC_AD15),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_39 */ \
	MV(CP(GPMC_A18),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row6 */ \
	MV(CP(GPMC_A19),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row7 */ \
	MV(CP(GPMC_A22),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col6 */ \
	MV(CP(GPMC_A23),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col7 */ \
	MV(CP(GPMC_NOE),	(PTU | OFF_EN | OFF_OUT_PTD | M1)) /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_cmd */ \
	MV(CP(CSI21_DX4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_75 */ \
	MV(CP(CSI21_DY4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_76 */ \
	MV(CP(CAM_SHUTTER),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_shutter */ \
	MV(CP(CAM_STROBE),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_strobe */ \
	MV(CP(CAM_GLOBALRESET),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_83 */ \
	MV(CP(USBB1_ULPITLL_CLK),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_clk */ \
	MV(CP(USBB1_ULPITLL_STP),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* usbb1_ulpiphy_stp */ \
	MV(CP(USBB1_ULPITLL_DIR),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dir */ \
	MV(CP(USBB1_ULPITLL_NXT),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_nxt */ \
	MV(CP(USBB1_ULPITLL_DAT0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat0 */ \
	MV(CP(USBB1_ULPITLL_DAT1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat1 */ \
	MV(CP(USBB1_ULPITLL_DAT2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat2 */ \
	MV(CP(USBB1_ULPITLL_DAT3),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat3 */ \
	MV(CP(USBB1_ULPITLL_DAT4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat4 */ \
	MV(CP(USBB1_ULPITLL_DAT5),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat5 */ \
	MV(CP(USBB1_ULPITLL_DAT6),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat6 */ \
	MV(CP(USBB1_ULPITLL_DAT7),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat7 */ \
	MV(CP(USBB1_HSIC_DATA),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_strobe */ \
	MV(CP(SDMMC1_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat4 */ \
	MV(CP(SDMMC1_DAT5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat5 */ \
	MV(CP(SDMMC1_DAT6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat6 */ \
	MV(CP(SDMMC1_DAT7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat7 */ \
	MV(CP(ABE_MCBSP2_CLKX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_clkx */ \
	MV(CP(ABE_MCBSP2_DR),	(IEN | OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dr */ \
	MV(CP(ABE_MCBSP2_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dx */ \
	MV(CP(ABE_MCBSP2_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_fsx */ \
	MV(CP(ABE_MCBSP1_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp1_dx */ \
	MV(CP(ABE_MCBSP1_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp1_fsx */ \
	MV(CP(UART2_CTS),	(PTU | IEN | M0)) /* uart2_cts */ \
	MV(CP(UART2_RTS),	(M0)) /* uart2_rts */ \
	MV(CP(UART2_RX),	(IEN | M0)) /* uart2_rx */ \
	MV(CP(UART2_TX),	(M0)) /* uart2_tx */ \
	MV(CP(I2C1_SCL),	(PTU | IEN | M0)) /* i2c1_scl */ \
	MV(CP(I2C1_SDA),	(PTU | IEN | M0)) /* i2c1_sda */ \
	MV(CP(I2C2_SCL),	(PTU | IEN | M0)) /* i2c2_scl */ \
	MV(CP(I2C2_SDA),	(PTU | IEN | M0)) /* i2c2_sda */ \
	MV(CP(I2C3_SCL),	(PTU | IEN | M0)) /* i2c3_scl */ \
	MV(CP(I2C3_SDA),	(PTU | IEN | M0)) /* i2c3_sda */ \
	MV(CP(I2C4_SCL),	(PTU | IEN | M0)) /* i2c4_scl */ \
	MV(CP(I2C4_SDA),	(PTU | IEN | M0)) /* i2c4_sda */ \
	MV(CP(MCSPI1_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_clk */ \
	MV(CP(MCSPI1_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_somi */ \
	MV(CP(MCSPI1_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_simo */ \
	MV(CP(MCSPI1_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_cs0 */ \
	MV(CP(MCSPI1_CS2),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_139 */ \
	MV(CP(UART3_CTS_RCTX),	(PTU | IEN | M0)) /* uart3_tx */ \
	MV(CP(UART3_RTS_SD),	(M0)) /* uart3_rts_sd */ \
	MV(CP(UART3_RX_IRRX),	(IEN | M0)) /* uart3_rx */ \
	MV(CP(UART3_TX_IRTX),	(M0)) /* uart3_tx */ \
	MV(CP(SDMMC5_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc5_clk */ \
	MV(CP(SDMMC5_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_cmd */ \
	MV(CP(SDMMC5_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat0 */ \
	MV(CP(SDMMC5_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat1 */ \
	MV(CP(SDMMC5_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat2 */ \
	MV(CP(SDMMC5_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat3 */ \
	MV(CP(MCSPI4_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_simo */ \
	MV(CP(MCSPI4_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_somi */ \
	MV(CP(MCSPI4_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_cs0 */ \
	MV(CP(UART4_RX),	(IEN | M0)) /* uart4_rx */ \
	MV(CP(UART4_TX),	(M0)) /* uart4_tx */ \
	MV(CP(USBB2_ULPITLL_CLK),	(M3)) /* gpio_157 */ \
	MV(CP(USBB2_ULPITLL_STP),	(M5)) /* dispc2_data23 */ \
	MV(CP(USBB2_ULPITLL_DIR),	(M5)) /* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT),	(M5)) /* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0),	(M5)) /* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1),	(M5)) /* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2),	(M5)) /* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3),	(M5)) /* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4),	(M5)) /* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5),	(M5)) /* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6),	(M5)) /* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7),	(M5)) /* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_169 */ \
	MV(CP(USBB2_HSIC_STROBE),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_170 */ \
	MV(CP(UNIPRO_TX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col0 */ \
	MV(CP(UNIPRO_TY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col1 */ \
	MV(CP(UNIPRO_TX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col2 */ \
	MV(CP(UNIPRO_TY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col3 */ \
	MV(CP(UNIPRO_TX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col4 */ \
	MV(CP(UNIPRO_TY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col5 */ \
	MV(CP(UNIPRO_RX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row0 */ \
	MV(CP(UNIPRO_RY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row1 */ \
	MV(CP(UNIPRO_RX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row2 */ \
	MV(CP(UNIPRO_RY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row3 */ \
	MV(CP(UNIPRO_RX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row4 */ \
	MV(CP(UNIPRO_RY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row5 */ \
	MV(CP(USBA0_OTG_CE),	(PTU | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* usba0_otg_ce */ \
	MV(CP(USBA0_OTG_DP),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dp */ \
	MV(CP(USBA0_OTG_DM),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dm */ \
	MV(CP(SYS_NIRQ1),	(PTU | IEN | M0)) /* sys_nirq1 */ \
	MV(CP(SYS_NIRQ2),	(PTU | IEN | M0)) /* sys_nirq2 */ \
	MV(CP(DPM_EMU3),	(M5)) /* dispc2_data10 */ \
	MV(CP(DPM_EMU4),	(M5)) /* dispc2_data9 */ \
	MV(CP(DPM_EMU5),	(M5)) /* dispc2_data16 */ \
	MV(CP(DPM_EMU6),	(M5)) /* dispc2_data17 */ \
	MV(CP(DPM_EMU7),	(M5)) /* dispc2_hsync */ \
	MV(CP(DPM_EMU8),	(M5)) /* dispc2_pclk */ \
	MV(CP(DPM_EMU9),	(M5)) /* dispc2_vsync */ \
	MV(CP(DPM_EMU10),	(M5)) /* dispc2_de */ \
	MV(CP(DPM_EMU11),	(M5)) /* dispc2_data8 */ \
	MV(CP(DPM_EMU12),	(M5)) /* dispc2_data7 */ \
	MV(CP(DPM_EMU13),	(M5)) /* dispc2_data6 */ \
	MV(CP(DPM_EMU14),	(M5)) /* dispc2_data5 */ \
	MV(CP(DPM_EMU15),	(M5)) /* dispc2_data4 */ \
	MV(CP(DPM_EMU16),	(M5)) /* dispc2_data3/dmtimer8_pwm_evt */ \
	MV(CP(DPM_EMU17),	(M5)) /* dispc2_data2 */ \
	MV(CP(DPM_EMU18),	(M5)) /* dispc2_data1 */ \
	MV(CP(DPM_EMU19),	(M5)) /* dispc2_data0 */ \
	MV1(WK(PAD1_SR_SCL),	(PTU | IEN | M0)) /* sr_scl */ \
	MV1(WK(PAD0_SR_SDA),	(PTU | IEN | M0)) /* sr_sda */ \
	//MV1(WK(PAD0_JTAG_NTRST),	(IEN | M0)) /* jtag_ntrst */ \
	MV1(WK(PAD1_JTAG_TCK),	(IEN | M0)) /* jtag_tck */ \
	MV1(WK(PAD0_JTAG_RTCK),	(M0)) /* jtag_rtck */ \
	MV1(WK(PAD1_JTAG_TMS_TMSC),	(IEN | M0)) /* jtag_tms_tmsc */ \
	MV1(WK(PAD0_JTAG_TDI),	(IEN | M0)) /* jtag_tdi */ \
	MV1(WK(PAD1_JTAG_TDO),	(M0)) 		  /* jtag_tdo */
#define MUX_DEFAULT_OMAP4_ALL() \
	MV(CP(GPMC_AD0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M3)) /* gpio_32 */ \
	MV(CP(GPMC_AD9),	(M3_SAFE)) /* gpio_33 */ \
	MV(CP(GPMC_AD10),	(M3_SAFE)) /* gpio_34 */ \
	MV(CP(GPMC_AD11),	(M3_SAFE)) /* gpio_35 */ \
	MV(CP(GPMC_AD12),	(M3_SAFE)) /* gpio_36 */ \
	MV(CP(GPMC_AD13),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_37 */ \
	MV(CP(GPMC_AD14),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_38 */ \
	MV(CP(GPMC_AD15),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_39 */ \
	MV(CP(GPMC_A16),	(M3_SAFE)) /* gpio_40 */ \
	MV(CP(GPMC_A17),	(M3_SAFE)) /* gpio_41 */ \
	MV(CP(GPMC_A18),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row6 */ \
	MV(CP(GPMC_A19),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row7 */ \
	MV(CP(GPMC_A20),	(M3_SAFE)) /* gpio_44 */ \
	MV(CP(GPMC_A21),	(M3_SAFE)) /* gpio_45 */ \
	MV(CP(GPMC_A22),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col6 */ \
	MV(CP(GPMC_A23),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col7 */ \
	MV(CP(GPMC_A24),	(M3_SAFE)) /* gpio_48 */ \
	MV(CP(GPMC_A25),	(M3_SAFE)) /* gpio_49 */ \
	MV(CP(GPMC_NCS0),	(M3_SAFE)) /* gpio_50 */ \
	MV(CP(GPMC_NCS1),	(M3_SAFE)) /* gpio_51 */ \
	MV(CP(GPMC_NCS2),	(M3_SAFE)) /* gpio_52 */ \
	MV(CP(GPMC_NCS3),	(M3_SAFE)) /* gpio_53 */ \
	MV(CP(GPMC_NWP),	(M3_SAFE)) /* gpio_54 */ \
	MV(CP(GPMC_CLK),	(M3_SAFE)) /* gpio_55 */ \
	MV(CP(GPMC_NADV_ALE),	(M3_SAFE)) /* gpio_56 */ \
	MV(CP(GPMC_NOE),	(PTU | OFF_EN | OFF_OUT_PTD | M1)) /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* sdmmc2_cmd */ \
	MV(CP(GPMC_NBE0_CLE),	(M3_SAFE)) /* gpio_59 */ \
	MV(CP(GPMC_NBE1),	(M3_SAFE)) /* gpio_60 */ \
	MV(CP(GPMC_WAIT0),	(M3_SAFE)) /* gpio_61 */ \
	MV(CP(GPMC_WAIT1),	(M3_SAFE)) /* gpio_62 */ \
	MV(CP(C2C_DATA11),	(M4_SAFE)) /* gpio_100 */ \
	MV(CP(C2C_DATA12),	(M2_SAFE)) /* dsi1_te0 */ \
	MV(CP(C2C_DATA13),	(M4_SAFE)) /* gpio_102 */ \
	MV(CP(C2C_DATA14),	(M2_SAFE)) /* dsi2_te0 */ \
	MV(CP(C2C_DATA15),	(M4_SAFE)) /* gpio_104 */ \
	MV(CP(HDMI_HPD),	(M0_SAFE)) /* hdmi_hpd */ \
	MV(CP(HDMI_CEC),	(M0_SAFE)) /* hdmi_cec */ \
	MV(CP(HDMI_DDC_SCL),	(M0_SAFE)) /* hdmi_ddc_scl */ \
	MV(CP(HDMI_DDC_SDA),	(M0_SAFE)) /* hdmi_ddc_sda */ \
	MV(CP(CSI21_DX0),	(M0_SAFE)) /* csi21_dx0 */ \
	MV(CP(CSI21_DY0),	(M0_SAFE)) /* csi21_dy0 */ \
	MV(CP(CSI21_DX1),	(M0_SAFE)) /* csi21_dx1 */ \
	MV(CP(CSI21_DY1),	(M0_SAFE)) /* csi21_dy1 */ \
	MV(CP(CSI21_DX2),	(M0_SAFE)) /* csi21_dx2 */ \
	MV(CP(CSI21_DY2),	(M0_SAFE)) /* csi21_dy2 */ \
	MV(CP(CSI21_DX3),	(M7_SAFE)) /* csi21_dx3 */ \
	MV(CP(CSI21_DY3),	(M7_SAFE)) /* csi21_dy3 */ \
	MV(CP(CSI21_DX4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_75 */ \
	MV(CP(CSI21_DY4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M7)) /* gpi_76 */ \
	MV(CP(CSI22_DX0),	(M0_SAFE)) /* csi22_dx0 */ \
	MV(CP(CSI22_DY0),	(M0_SAFE)) /* csi22_dy0 */ \
	MV(CP(CSI22_DX1),	(M0_SAFE)) /* csi22_dx1 */ \
	MV(CP(CSI22_DY1),	(M0_SAFE)) /* csi22_dy1 */ \
	MV(CP(CAM_SHUTTER),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_shutter */ \
	MV(CP(CAM_STROBE),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* cam_strobe */ \
	MV(CP(CAM_GLOBALRESET),	(PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* gpio_83 */ \
	MV(CP(USBB1_ULPITLL_CLK),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_clk */ \
	MV(CP(USBB1_ULPITLL_STP),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* usbb1_ulpiphy_stp */ \
	MV(CP(USBB1_ULPITLL_DIR),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dir */ \
	MV(CP(USBB1_ULPITLL_NXT),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_nxt */ \
	MV(CP(USBB1_ULPITLL_DAT0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat0 */ \
	MV(CP(USBB1_ULPITLL_DAT1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat1 */ \
	MV(CP(USBB1_ULPITLL_DAT2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat2 */ \
	MV(CP(USBB1_ULPITLL_DAT3),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat3 */ \
	MV(CP(USBB1_ULPITLL_DAT4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat4 */ \
	MV(CP(USBB1_ULPITLL_DAT5),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat5 */ \
	MV(CP(USBB1_ULPITLL_DAT6),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat6 */ \
	MV(CP(USBB1_ULPITLL_DAT7),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_ulpiphy_dat7 */ \
	MV(CP(USBB1_HSIC_DATA),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_strobe */ \
	MV(CP(USBC1_ICUSB_DP),	(M0_SAFE)) /* usbc1_icusb_dp */ \
	MV(CP(USBC1_ICUSB_DM),	(M0_SAFE)) /* usbc1_icusb_dm */ \
	MV(CP(SDMMC1_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat4 */ \
	MV(CP(SDMMC1_DAT5),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat5 */ \
	MV(CP(SDMMC1_DAT6),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat6 */ \
	MV(CP(SDMMC1_DAT7),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc1_dat7 */ \
	MV(CP(ABE_MCBSP2_CLKX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_clkx */ \
	MV(CP(ABE_MCBSP2_DR),	(IEN | OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dr */ \
	MV(CP(ABE_MCBSP2_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp2_dx */ \
	MV(CP(ABE_MCBSP2_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp2_fsx */ \
	MV(CP(ABE_MCBSP1_CLKX),	(M1_SAFE)) /* abe_slimbus1_clock */ \
	MV(CP(ABE_MCBSP1_DR),	(M1_SAFE)) /* abe_slimbus1_data */ \
	MV(CP(ABE_MCBSP1_DX),	(OFF_EN | OFF_OUT_PTD | M0)) /* abe_mcbsp1_dx */ \
	MV(CP(ABE_MCBSP1_FSX),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* abe_mcbsp1_fsx */ \
	MV(CP(ABE_PDM_UL_DATA),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_ul_data */ \
	MV(CP(ABE_PDM_DL_DATA),	(PTD | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_dl_data */ \
	MV(CP(ABE_PDM_FRAME),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_frame */ \
	MV(CP(ABE_PDM_LB_CLK),	(OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_pdm_lb_clk */ \
	MV(CP(ABE_CLKS),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* abe_clks */ \
	MV(CP(ABE_DMIC_CLK1),	(M0_SAFE)) /* abe_dmic_clk1 */ \
	MV(CP(ABE_DMIC_DIN1),	(M0_SAFE)) /* abe_dmic_din1 */ \
	MV(CP(ABE_DMIC_DIN2),	(M0_SAFE)) /* abe_dmic_din2 */ \
	MV(CP(ABE_DMIC_DIN3),	(M0_SAFE)) /* abe_dmic_din3 */ \
	MV(CP(UART2_CTS),	(PTU | IEN | M0)) /* uart2_cts */ \
	MV(CP(UART2_RTS),	(M0)) /* uart2_rts */ \
	MV(CP(UART2_RX),	(IEN | M0)) /* uart2_rx */ \
	MV(CP(UART2_TX),	(M0)) /* uart2_tx */ \
	MV(CP(HDQ_SIO),	(M3_SAFE)) /* gpio_127 */ \
	MV(CP(I2C1_SCL),	(PTU | IEN | M0)) /* i2c1_scl */ \
	MV(CP(I2C1_SDA),	(PTU | IEN | M0)) /* i2c1_sda */ \
	MV(CP(I2C2_SCL),	(PTU | IEN | M0)) /* i2c2_scl */ \
	MV(CP(I2C2_SDA),	(PTU | IEN | M0)) /* i2c2_sda */ \
	MV(CP(I2C3_SCL),	(PTU | IEN | M0)) /* i2c3_scl */ \
	MV(CP(I2C3_SDA),	(PTU | IEN | M0)) /* i2c3_sda */ \
	MV(CP(I2C4_SCL),	(PTU | IEN | M0)) /* i2c4_scl */ \
	MV(CP(I2C4_SDA),	(PTU | IEN | M0)) /* i2c4_sda */ \
	MV(CP(MCSPI1_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_clk */ \
	MV(CP(MCSPI1_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_somi */ \
	MV(CP(MCSPI1_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_simo */ \
	MV(CP(MCSPI1_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi1_cs0 */ \
	MV(CP(MCSPI1_CS1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0_SAFE)) /* mcspi1_cs1 */ \
	MV(CP(MCSPI1_CS2),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_139 */ \
	MV(CP(MCSPI1_CS3),	(M3_SAFE)) /* gpio_140 */ \
	MV(CP(UART3_CTS_RCTX),	(PTU | IEN | M0)) /* uart3_tx */ \
	MV(CP(UART3_RTS_SD),	(M0)) /* uart3_rts_sd */ \
	MV(CP(UART3_RX_IRRX),	(IEN | M0)) /* uart3_rx */ \
	MV(CP(UART3_TX_IRTX),	(M0)) /* uart3_tx */ \
	MV(CP(SDMMC5_CLK),	(PTU | OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc5_clk */ \
	MV(CP(SDMMC5_CMD),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_cmd */ \
	MV(CP(SDMMC5_DAT0),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat0 */ \
	MV(CP(SDMMC5_DAT1),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat1 */ \
	MV(CP(SDMMC5_DAT2),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat2 */ \
	MV(CP(SDMMC5_DAT3),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* sdmmc5_dat3 */ \
	MV(CP(MCSPI4_CLK),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_simo */ \
	MV(CP(MCSPI4_SOMI),	(IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_somi */ \
	MV(CP(MCSPI4_CS0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_cs0 */ \
	MV(CP(UART4_RX),	(IEN | M0)) /* uart4_rx */ \
	MV(CP(UART4_TX),	(M0)) /* uart4_tx */ \
	MV(CP(USBB2_ULPITLL_CLK),	(M3)) /* gpio_157 */ \
	MV(CP(USBB2_ULPITLL_STP),	(M5)) /* dispc2_data23 */ \
	MV(CP(USBB2_ULPITLL_DIR),	(M5)) /* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT),	(M5)) /* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0),	(M5)) /* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1),	(M5)) /* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2),	(M5)) /* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3),	(M5)) /* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4),	(M5)) /* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5),	(M5)) /* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6),	(M5)) /* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7),	(M5)) /* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_169 */ \
	MV(CP(USBB2_HSIC_STROBE),	(OFF_EN | OFF_OUT_PTU | M3)) /* gpio_170 */ \
	MV(CP(UNIPRO_TX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col0 */ \
	MV(CP(UNIPRO_TY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col1 */ \
	MV(CP(UNIPRO_TX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col2 */ \
	MV(CP(UNIPRO_TY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col3 */ \
	MV(CP(UNIPRO_TX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col4 */ \
	MV(CP(UNIPRO_TY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_col5 */ \
	MV(CP(UNIPRO_RX0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row0 */ \
	MV(CP(UNIPRO_RY0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row1 */ \
	MV(CP(UNIPRO_RX1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row2 */ \
	MV(CP(UNIPRO_RY1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row3 */ \
	MV(CP(UNIPRO_RX2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row4 */ \
	MV(CP(UNIPRO_RY2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M1)) /* kpd_row5 */ \
	MV(CP(USBA0_OTG_CE),	(PTU | OFF_EN | OFF_PD | OFF_OUT_PTD | M0)) /* usba0_otg_ce */ \
	MV(CP(USBA0_OTG_DP),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dp */ \
	MV(CP(USBA0_OTG_DM),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usba0_otg_dm */ \
	MV(CP(FREF_CLK1_OUT),	(M0_SAFE)) /* fref_clk1_out */ \
	MV(CP(FREF_CLK2_OUT),	(M0_SAFE)) /* fref_clk2_out */ \
	MV(CP(SYS_NIRQ1),	(PTU | IEN | M0)) /* sys_nirq1 */ \
	MV(CP(SYS_NIRQ2),	(PTU | IEN | M0)) /* sys_nirq2 */ \
	MV(CP(SYS_BOOT0),	(M3_SAFE)) /* gpio_184 */ \
	MV(CP(SYS_BOOT1),	(M3_SAFE)) /* gpio_185 */ \
	MV(CP(SYS_BOOT2),	(M3_SAFE)) /* gpio_186 */ \
	MV(CP(SYS_BOOT3),	(M3_SAFE)) /* gpio_187 */ \
	MV(CP(SYS_BOOT4),	(M3_SAFE)) /* gpio_188 */ \
	MV(CP(SYS_BOOT5),	(M3_SAFE)) /* gpio_189 */ \
	MV(CP(DPM_EMU0),	(M0_SAFE)) /* dpm_emu0 */ \
	MV(CP(DPM_EMU1),	(M0_SAFE)) /* dpm_emu1 */ \
	MV(CP(DPM_EMU2),	(M0_SAFE)) /* dpm_emu2 */ \
	MV(CP(DPM_EMU3),	(M5)) /* dispc2_data10 */ \
	MV(CP(DPM_EMU4),	(M5)) /* dispc2_data9 */ \
	MV(CP(DPM_EMU5),	(M5)) /* dispc2_data16 */ \
	MV(CP(DPM_EMU6),	(M5)) /* dispc2_data17 */ \
	MV(CP(DPM_EMU7),	(M5)) /* dispc2_hsync */ \
	MV(CP(DPM_EMU8),	(M5)) /* dispc2_pclk */ \
	MV(CP(DPM_EMU9),	(M5)) /* dispc2_vsync */ \
	MV(CP(DPM_EMU10),	(M5)) /* dispc2_de */ \
	MV(CP(DPM_EMU11),	(M5)) /* dispc2_data8 */ \
	MV(CP(DPM_EMU12),	(M5)) /* dispc2_data7 */ \
	MV(CP(DPM_EMU13),	(M5)) /* dispc2_data6 */ \
	MV(CP(DPM_EMU14),	(M5)) /* dispc2_data5 */ \
	MV(CP(DPM_EMU15),	(M5)) /* dispc2_data4 */ \
	MV(CP(DPM_EMU16),	(M5)) /* dispc2_data3/dmtimer8_pwm_evt */ \
	MV(CP(DPM_EMU17),	(M5)) /* dispc2_data2 */ \
	MV(CP(DPM_EMU18),	(M5)) /* dispc2_data1 */ \
	MV(CP(DPM_EMU19),	(M5)) /* dispc2_data0 */ \
	MV1(WK(PAD0_SIM_IO),	(M0_SAFE)) /* sim_io */ \
	MV1(WK(PAD1_SIM_CLK),	(M0_SAFE)) /* sim_clk */ \
	MV1(WK(PAD0_SIM_RESET),	(M0_SAFE)) /* sim_reset */ \
	MV1(WK(PAD1_SIM_CD),	(M0_SAFE)) /* sim_cd */ \
	MV1(WK(PAD0_SIM_PWRCTRL),	(M0_SAFE)) /* sim_pwrctrl */ \
	MV1(WK(PAD1_SR_SCL),	(PTU | IEN | M0)) /* sr_scl */ \
	MV1(WK(PAD0_SR_SDA),	(PTU | IEN | M0)) /* sr_sda */ \
	MV1(WK(PAD1_FREF_XTAL_IN),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_SLICER_IN),	(M0_SAFE)) /* fref_slicer_in */ \
	MV1(WK(PAD1_FREF_CLK_IOREQ),	(M0_SAFE)) /* fref_clk_ioreq */ \
	MV1(WK(PAD0_FREF_CLK0_OUT),	(M3_SAFE)) /* sys_drm_msecure */ \
	MV1(WK(PAD1_FREF_CLK3_REQ),	(M7_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_CLK3_OUT),	(M0_SAFE)) /* fref_clk3_out */ \
	MV1(WK(PAD1_FREF_CLK4_REQ),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_CLK4_OUT),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD1_SYS_32K),	(IEN | M0_SAFE)) /* sys_32k */ \
	MV1(WK(PAD0_SYS_NRESPWRON),	(IEN | M0_SAFE)) /* sys_nrespwron */ \
	MV1(WK(PAD1_SYS_NRESWARM),	(IEN | M0_SAFE)) /* sys_nreswarm */ \
	MV1(WK(PAD0_SYS_PWR_REQ),	(M0_SAFE)) /* sys_pwr_req */ \
	MV1(WK(PAD1_SYS_PWRON_RESET),	(M3_SAFE)) /* gpio_wk29 */ \
	MV1(WK(PAD0_SYS_BOOT6),	(M3_SAFE)) /* gpio_wk9 */ \
	MV1(WK(PAD1_SYS_BOOT7),	(M3_SAFE)) /* gpio_wk10 */ \
	//MV1(WK(PAD0_JTAG_NTRST),	(IEN | M0)) /* jtag_ntrst */ \
	MV1(WK(PAD1_JTAG_TCK),	(IEN | M0)) /* jtag_tck */ \
	MV1(WK(PAD0_JTAG_RTCK),	(M0)) /* jtag_rtck */ \
	MV1(WK(PAD1_JTAG_TMS_TMSC),	(IEN | M0)) /* jtag_tms_tmsc */ \
	MV1(WK(PAD0_JTAG_TDI),	(IEN | M0)) /* jtag_tdi */ \
	MV1(WK(PAD1_JTAG_TDO),	(M0)) 		  /* jtag_tdo */

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT_OMAP4();
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

#endif

