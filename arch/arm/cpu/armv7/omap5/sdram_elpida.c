/*
 * Timing and Organization details of the Elpida parts used in OMAP5
 * EVM
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 * Sricharan R <r.sricharan@ti.com>
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

#include <asm/emif.h>
#include <asm/arch/sys_proto.h>

/*
 * This file provides details of the LPDDR2 SDRAM parts used on OMAP5
 * EVM. Since the parts used and geometry are identical for
 * evm for a given OMAP5 revision, this information is kept
 * here instead of being in board directory. However the key functions
 * exported are weakly linked so that they can be over-ridden in the board
 * directory if there is a OMAP5 board in the future that uses a different
 * memory device or geometry.
 *
 * For any new board with different memory devices over-ride one or more
 * of the following functions as per the CONFIG flags you intend to enable:
 * - emif_get_reg_dump()
 * - emif_get_dmm_regs()
 * - emif_get_device_details()
 * - emif_get_device_timings()
 */

#ifdef CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS
const struct emif_regs emif_regs_elpida_532_mhz_2cs = {
	.sdram_config_init		= 0x80800EBA,
	.sdram_config			= 0x808022BA,
	.ref_ctrl			= 0x0000081A,
	.sdram_tim1			= 0x772F6873,
	.sdram_tim2			= 0x304A129A,
	.sdram_tim3			= 0x02F7E45F,
	.read_idle_ctrl			= 0x00050000,
	.zq_config			= 0x000B3215,
	.temp_alert_config		= 0x08000A05,
	.emif_ddr_phy_ctlr_1_init	= 0x0E38200D,
	.emif_ddr_phy_ctlr_1		= 0x0E38200D,
	.emif_ddr_ext_phy_ctrl_1	= 0x04020080,
	.emif_ddr_ext_phy_ctrl_2	= 0x14829052,
	.emif_ddr_ext_phy_ctrl_3	= 0x290520A4,
	.emif_ddr_ext_phy_ctrl_4	= 0x000A4148,
	.emif_ddr_ext_phy_ctrl_5	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_6	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_7	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_8	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_9	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_10	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_11	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_12	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_13	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_14	= 0x80080080,
	.emif_ddr_ext_phy_ctrl_15	= 0x00800800,
	.emif_ddr_ext_phy_ctrl_16	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_17	= 0x00000001,
	.emif_ddr_ext_phy_ctrl_18	= 0x540A8150,
	.emif_ddr_ext_phy_ctrl_19	= 0xA81502A0,
	.emif_ddr_ext_phy_ctrl_20	= 0x002A0540,
	.emif_ddr_ext_phy_ctrl_21	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_22	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_23	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_24	= 0x00000077
};

const struct emif_regs emif_regs_elpida_532_mhz_1cs = {
	.sdram_config_init		= 0x80800EB2,
	.sdram_config			= 0x808022B2,
	.ref_ctrl			= 0x0000081A,
	.sdram_tim1			= 0x772F6873,
	.sdram_tim2			= 0x304A129A,
	.sdram_tim3			= 0x02F7E45F,
	.read_idle_ctrl			= 0x00050000,
	.zq_config			= 0x000B3215,
	.temp_alert_config		= 0x08000A05,
	.emif_ddr_phy_ctlr_1_init	= 0x0E38200D,
	.emif_ddr_phy_ctlr_1		= 0x0E38200D,
	.emif_ddr_ext_phy_ctrl_1	= 0x04020080,
	.emif_ddr_ext_phy_ctrl_2	= 0x14829052,
	.emif_ddr_ext_phy_ctrl_3	= 0x290520A4,
	.emif_ddr_ext_phy_ctrl_4	= 0x000A4148,
	.emif_ddr_ext_phy_ctrl_5	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_6	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_7	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_8	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_9	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_10	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_11	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_12	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_13	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_14	= 0x80080080,
	.emif_ddr_ext_phy_ctrl_15	= 0x00800800,
	.emif_ddr_ext_phy_ctrl_16	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_17	= 0x00000001,
	.emif_ddr_ext_phy_ctrl_18	= 0x540A8150,
	.emif_ddr_ext_phy_ctrl_19	= 0xA81502A0,
	.emif_ddr_ext_phy_ctrl_20	= 0x002A0540,
	.emif_ddr_ext_phy_ctrl_21	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_22	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_23	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_24	= 0x00000077
};

const struct emif_regs emif_regs_elpida_266_mhz_2cs = {
	.sdram_config_init		= 0x80800EBA,
	.sdram_config			= 0x808022BA,
	.ref_ctrl			= 0x0000040D,
	.sdram_tim1			= 0x2A86B419,
	.sdram_tim2			= 0x1025094A,
	.sdram_tim3			= 0x026BA22F,
	.read_idle_ctrl			= 0x00050000,
	.zq_config			= 0x000B3215,
	.temp_alert_config		= 0x08000A05,
	.emif_ddr_phy_ctlr_1_init	= 0x0E382008,
	.emif_ddr_phy_ctlr_1		= 0x0E38200D,
	.emif_ddr_ext_phy_ctrl_1	= 0x04020080,
	.emif_ddr_ext_phy_ctrl_2	= 0x0A414829,
	.emif_ddr_ext_phy_ctrl_3	= 0x14829052,
	.emif_ddr_ext_phy_ctrl_4	= 0x000520A4,
	.emif_ddr_ext_phy_ctrl_5	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_6	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_7	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_8	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_9	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_10	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_11	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_12	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_13	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_14	= 0x80080080,
	.emif_ddr_ext_phy_ctrl_15	= 0x00800800,
	.emif_ddr_ext_phy_ctrl_16	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_17	= 0x00000001,
	.emif_ddr_ext_phy_ctrl_18	= 0x540A8150,
	.emif_ddr_ext_phy_ctrl_19	= 0xA81502A0,
	.emif_ddr_ext_phy_ctrl_20	= 0x002A0540,
	.emif_ddr_ext_phy_ctrl_21	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_22	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_23	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_24	= 0x00000077
};

const struct emif_regs emif_regs_elpida_266_mhz_1cs = {
	.sdram_config_init		= 0x80800EB2,
	.sdram_config			= 0x808022B2,
	.ref_ctrl			= 0x0000040D,
	.sdram_tim1			= 0x2A86B419,
	.sdram_tim2			= 0x1025094A,
	.sdram_tim3			= 0x026BA22F,
	.read_idle_ctrl			= 0x00050000,
	.zq_config			= 0x000B3215,
	.temp_alert_config		= 0x08000A05,
	.emif_ddr_phy_ctlr_1_init	= 0x0E382008,
	.emif_ddr_phy_ctlr_1		= 0x0E38200D,
	.emif_ddr_ext_phy_ctrl_1	= 0x04020080,
	.emif_ddr_ext_phy_ctrl_2	= 0x0A414829,
	.emif_ddr_ext_phy_ctrl_3	= 0x14829052,
	.emif_ddr_ext_phy_ctrl_4	= 0x000520A4,
	.emif_ddr_ext_phy_ctrl_5	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_6	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_7	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_8	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_9	= 0x01004010,
	.emif_ddr_ext_phy_ctrl_10	= 0x00001004,
	.emif_ddr_ext_phy_ctrl_11	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_12	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_13	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_14	= 0x80080080,
	.emif_ddr_ext_phy_ctrl_15	= 0x00800800,
	.emif_ddr_ext_phy_ctrl_16	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_17	= 0x00000001,
	.emif_ddr_ext_phy_ctrl_18	= 0x540A8150,
	.emif_ddr_ext_phy_ctrl_19	= 0xA81502A0,
	.emif_ddr_ext_phy_ctrl_20	= 0x002A0540,
	.emif_ddr_ext_phy_ctrl_21	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_22	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_23	= 0x00000000,
	.emif_ddr_ext_phy_ctrl_24	= 0x00000077
};

const struct dmm_lisa_map_regs lisa_map_4G_x_1_x_2 = {
	.dmm_lisa_map_0 = 0xFF020100,
	.dmm_lisa_map_1 = 0,
	.dmm_lisa_map_2 = 0,
	.dmm_lisa_map_3 = 0x80640300
};

static void emif_get_reg_dump_sdp(u32 emif_nr, const struct emif_regs **regs)
{
	*regs = &emif_regs_elpida_532_mhz_1cs;
}
void emif_get_reg_dump(u32 emif_nr, const struct emif_regs **regs)
	__attribute__((weak, alias("emif_get_reg_dump_sdp")));

static void emif_get_dmm_regs_sdp(const struct dmm_lisa_map_regs
						**dmm_lisa_regs)
{
	*dmm_lisa_regs = &lisa_map_4G_x_1_x_2;
}

void emif_get_dmm_regs(const struct dmm_lisa_map_regs **dmm_lisa_regs)
	__attribute__((weak, alias("emif_get_dmm_regs_sdp")));

#else

static const struct lpddr2_device_details elpida_4G_S4_details = {
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.manufacturer	= LPDDR2_MANUFACTURER_ELPIDA
};

static void emif_get_device_details_sdp(u32 emif_nr,
		struct lpddr2_device_details *cs0_device_details,
		struct lpddr2_device_details *cs1_device_details)
{
	/* EMIF1 & EMIF2 have identical configuration */
	*cs0_device_details = elpida_4G_S4_details;

	/* Nothing is conected on cs1 */
	cs1_device_details = NULL;
}

void emif_get_device_details(u32 emif_nr,
		struct lpddr2_device_details *cs0_device_details,
		struct lpddr2_device_details *cs1_device_details)
	__attribute__((weak, alias("emif_get_device_details_sdp")));

#endif /* CONFIG_SYS_EMIF_PRECALCULATED_TIMING_REGS */

#ifndef CONFIG_SYS_DEFAULT_LPDDR2_TIMINGS
static const struct lpddr2_ac_timings timings_jedec_532_mhz = {
	.max_freq	= 532000000,
	.RL		= 8,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 15,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50,
	.tRTWx2		= 15,
	.tCSTA		= 3
};


static const struct lpddr2_min_tck min_tck_elpida = {
	.tRL		= 3,
	.tRP_AB		= 3,
	.tRCD		= 3,
	.tWR		= 3,
	.tRAS_MIN	= 3,
	.tRRD		= 2,
	.tWTR		= 2,
	.tXP		= 2,
	.tRTP		= 2,
	.tCKE		= 3,
	.tCKESR		= 3,
	.tFAW		= 8,
	.tRTW		= 2
};

static const struct lpddr2_ac_timings *elpida_ac_timings[MAX_NUM_SPEEDBINS] = {
	&timings_jedec_532_mhz
};

static const struct lpddr2_device_timings elpida_4G_S4_timings = {
	.ac_timings	= elpida_ac_timings,
	.min_tck	= &min_tck_elpida,
};

void emif_get_device_timings_sdp(u32 emif_nr,
		const struct lpddr2_device_timings **cs0_device_timings,
		const struct lpddr2_device_timings **cs1_device_timings)
{
	/* Identical devices on EMIF1 & EMIF2 */
	*cs0_device_timings = &elpida_4G_S4_timings;
	*cs1_device_timings = NULL;
}

void emif_get_device_timings(u32 emif_nr,
		const struct lpddr2_device_timings **cs0_device_timings,
		const struct lpddr2_device_timings **cs1_device_timings)
	__attribute__((weak, alias("emif_get_device_timings_sdp")));

#endif /* CONFIG_SYS_DEFAULT_LPDDR2_TIMINGS */
