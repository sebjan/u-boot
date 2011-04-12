/*
 * (C) Copyright 2007
 * Sascha Hauer, Pengutronix
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
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
#include <div64.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>

enum pll_clocks {
	PLL1_CLOCK = 0,
	PLL2_CLOCK,
	PLL3_CLOCK,
	PLL4_CLOCK,
	PLL_CLOCKS,
};

struct mxc_pll_reg *mxc_plls[PLL_CLOCKS] = {
	[PLL1_CLOCK] = (struct mxc_pll_reg *)PLL1_BASE_ADDR,
	[PLL2_CLOCK] = (struct mxc_pll_reg *)PLL2_BASE_ADDR,
	[PLL3_CLOCK] = (struct mxc_pll_reg *)PLL3_BASE_ADDR,
	[PLL4_CLOCK] = (struct mxc_pll_reg *)PLL4_BASE_ADDR,
};

#define AHB_CLK_ROOT    133333333
#define SZ_DEC_1M       1000000
#define PLL_PD_MAX      16      /* Actual pd+1 */
#define PLL_MFI_MAX     15
#define PLL_MFI_MIN     5
#define ARM_DIV_MAX     8
#define IPG_DIV_MAX     4
#define AHB_DIV_MAX     8
#define EMI_DIV_MAX     8
#define NFC_DIV_MAX     8

struct fixed_pll_mfd {
	u32 ref_clk_hz;
	u32 mfd;
};

const struct fixed_pll_mfd fixed_mfd[] = {
	{CONFIG_SYS_MX5_HCLK, 24 * 16},
};

struct pll_param {
	u32 pd;
	u32 mfi;
	u32 mfn;
	u32 mfd;
};

#define PLL_FREQ_MAX(ref_clk)  (4 * (ref_clk) * PLL_MFI_MAX)
#define PLL_FREQ_MIN(ref_clk) \
		((2 * (ref_clk) * (PLL_MFI_MIN - 1)) / PLL_PD_MAX)
#define MAX_DDR_CLK     420000000
#define NFC_CLK_MAX     34000000

struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)MXC_CCM_BASE;

/*
 * Calculate the frequency of this pll.
 */
static u32 decode_pll(struct mxc_pll_reg *pll, u32 infreq)
{
	u32 mfi, mfn, mfd, pd;

	mfn = __raw_readl(&pll->mfn);
	mfd = __raw_readl(&pll->mfd) + 1;
	mfi = __raw_readl(&pll->op);
	pd = (mfi  & 0xF) + 1;
	mfi = (mfi >> 4) & 0xF;
	mfi = (mfi >= 5) ? mfi : 5;

	return ((4 * (infreq / 1000) * (mfi * mfd + mfn)) / (mfd * pd)) * 1000;
}

/*
 * Get mcu main rate
 */
u32 get_mcu_main_clk(void)
{
	u32 reg, freq;

	reg = (__raw_readl(&mxc_ccm->cacrr) & MXC_CCM_CACRR_ARM_PODF_MASK) >>
		MXC_CCM_CACRR_ARM_PODF_OFFSET;
	freq = decode_pll(mxc_plls[PLL1_CLOCK], CONFIG_SYS_MX5_HCLK);
	return freq / (reg + 1);
}

/*
 * Get the rate of peripheral's root clock.
 */
static u32 get_periph_clk(void)
{
	u32 reg;

	reg = __raw_readl(&mxc_ccm->cbcdr);
	if (!(reg & MXC_CCM_CBCDR_PERIPH_CLK_SEL))
		return decode_pll(mxc_plls[PLL2_CLOCK], CONFIG_SYS_MX5_HCLK);
	reg = __raw_readl(&mxc_ccm->cbcmr);
	switch ((reg & MXC_CCM_CBCMR_PERIPH_CLK_SEL_MASK) >>
		MXC_CCM_CBCMR_PERIPH_CLK_SEL_OFFSET) {
	case 0:
		return decode_pll(mxc_plls[PLL1_CLOCK], CONFIG_SYS_MX5_HCLK);
	case 1:
		return decode_pll(mxc_plls[PLL3_CLOCK], CONFIG_SYS_MX5_HCLK);
	default:
		return 0;
	}
	/* NOTREACHED */
}

/*
 * Get the rate of ipg clock.
 */
static u32 get_ipg_clk(void)
{
	u32 ahb_podf, ipg_podf;

	ahb_podf = __raw_readl(&mxc_ccm->cbcdr);
	ipg_podf = (ahb_podf & MXC_CCM_CBCDR_IPG_PODF_MASK) >>
			MXC_CCM_CBCDR_IPG_PODF_OFFSET;
	ahb_podf = (ahb_podf & MXC_CCM_CBCDR_AHB_PODF_MASK) >>
			MXC_CCM_CBCDR_AHB_PODF_OFFSET;
	return get_periph_clk() / ((ahb_podf + 1) * (ipg_podf + 1));
}

/*
 * Get the rate of ipg_per clock.
 */
static u32 get_ipg_per_clk(void)
{
	u32 pred1, pred2, podf;

	if (__raw_readl(&mxc_ccm->cbcmr) & MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL)
		return get_ipg_clk();
	/* Fixme: not handle what about lpm*/
	podf = __raw_readl(&mxc_ccm->cbcdr);
	pred1 = (podf & MXC_CCM_CBCDR_PERCLK_PRED1_MASK) >>
		MXC_CCM_CBCDR_PERCLK_PRED1_OFFSET;
	pred2 = (podf & MXC_CCM_CBCDR_PERCLK_PRED2_MASK) >>
		MXC_CCM_CBCDR_PERCLK_PRED2_OFFSET;
	podf = (podf & MXC_CCM_CBCDR_PERCLK_PODF_MASK) >>
		MXC_CCM_CBCDR_PERCLK_PODF_OFFSET;

	return get_periph_clk() / ((pred1 + 1) * (pred2 + 1) * (podf + 1));
}

/*
 * Get the rate of uart clk.
 */
static u32 get_uart_clk(void)
{
	unsigned int freq, reg, pred, podf;

	reg = __raw_readl(&mxc_ccm->cscmr1);
	switch ((reg & MXC_CCM_CSCMR1_UART_CLK_SEL_MASK) >>
		MXC_CCM_CSCMR1_UART_CLK_SEL_OFFSET) {
	case 0x0:
		freq = decode_pll(mxc_plls[PLL1_CLOCK],
				    CONFIG_SYS_MX5_HCLK);
		break;
	case 0x1:
		freq = decode_pll(mxc_plls[PLL2_CLOCK],
				    CONFIG_SYS_MX5_HCLK);
		break;
	case 0x2:
		freq = decode_pll(mxc_plls[PLL3_CLOCK],
				    CONFIG_SYS_MX5_HCLK);
		break;
	default:
		return 66500000;
	}

	reg = __raw_readl(&mxc_ccm->cscdr1);

	pred = (reg & MXC_CCM_CSCDR1_UART_CLK_PRED_MASK) >>
		MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET;

	podf = (reg & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET;
	freq /= (pred + 1) * (podf + 1);

	return freq;
}

/*
 * This function returns the low power audio clock.
 */
static u32 get_lp_apm(void)
{
	u32 ret_val = 0;
	u32 ccsr = __raw_readl(&mxc_ccm->ccsr);

	if (((ccsr >> 9) & 1) == 0)
		ret_val = CONFIG_SYS_MX5_HCLK;
	else
		ret_val = ((32768 * 1024));

	return ret_val;
}

/*
 * get cspi clock rate.
 */
static u32 get_cspi_clk(void)
{
	u32 ret_val = 0, pdf, pre_pdf, clk_sel;
	u32 cscmr1 = __raw_readl(&mxc_ccm->cscmr1);
	u32 cscdr2 = __raw_readl(&mxc_ccm->cscdr2);

	pre_pdf = (cscdr2 & MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK) \
			>> MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET;
	pdf = (cscdr2 & MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK) \
			>> MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET;
	clk_sel = (cscmr1 & MXC_CCM_CSCMR1_CSPI_CLK_SEL_MASK) \
			>> MXC_CCM_CSCMR1_CSPI_CLK_SEL_OFFSET;

	switch (clk_sel) {
	case 0:
		ret_val = decode_pll(mxc_plls[PLL1_CLOCK],
					CONFIG_SYS_MX5_HCLK) /
					((pre_pdf + 1) * (pdf + 1));
		break;
	case 1:
		ret_val = decode_pll(mxc_plls[PLL2_CLOCK],
					CONFIG_SYS_MX5_HCLK) /
					((pre_pdf + 1) * (pdf + 1));
		break;
	case 2:
		ret_val = decode_pll(mxc_plls[PLL3_CLOCK],
					CONFIG_SYS_MX5_HCLK) /
					((pre_pdf + 1) * (pdf + 1));
		break;
	default:
		ret_val = get_lp_apm() / ((pre_pdf + 1) * (pdf + 1));
		break;
	}

	return ret_val;
}

static u32 get_axi_a_clk(void)
{
	u32 cbcdr =  __raw_readl(&mxc_ccm->cbcdr);
	u32 pdf = (cbcdr & MXC_CCM_CBCDR_AXI_A_PODF_MASK) \
			>> MXC_CCM_CBCDR_AXI_A_PODF_OFFSET;

	return  get_periph_clk() / (pdf + 1);
}

static u32 get_axi_b_clk(void)
{
	u32 cbcdr =  __raw_readl(&mxc_ccm->cbcdr);
	u32 pdf = (cbcdr & MXC_CCM_CBCDR_AXI_B_PODF_MASK) \
			>> MXC_CCM_CBCDR_AXI_B_PODF_OFFSET;

	return  get_periph_clk() / (pdf + 1);
}

static u32 get_ahb_clk(void)
{
	u32 cbcdr = __raw_readl(&mxc_ccm->cbcdr);
	u32 pdf = (cbcdr & MXC_CCM_CBCDR_AHB_PODF_MASK) \
			>> MXC_CCM_CBCDR_AHB_PODF_OFFSET;

	return  get_periph_clk() / (pdf + 1);
}

static u32 get_emi_slow_clk(void)
{
	u32 cbcdr = __raw_readl(&mxc_ccm->cbcdr);
	u32 emi_clk_sel = cbcdr & MXC_CCM_CBCDR_EMI_CLK_SEL;
	u32 pdf = (cbcdr & MXC_CCM_CBCDR_EMI_PODF_MASK) \
			>> MXC_CCM_CBCDR_EMI_PODF_OFFSET;

	if (emi_clk_sel)
		return  get_ahb_clk() / (pdf + 1);

	return  get_periph_clk() / (pdf + 1);
}

static u32 get_nfc_clk(void)
{
	u32 cbcdr = __raw_readl(&mxc_ccm->cbcdr);
	u32 pdf = (cbcdr & MXC_CCM_CBCDR_NFC_PODF_MASK) \
			>> MXC_CCM_CBCDR_NFC_PODF_OFFSET;

	return  get_emi_slow_clk() / (pdf + 1);
}

static u32 get_ddr_clk(void)
{
	u32 ret_val = 0;
	u32 cbcmr = __raw_readl(&mxc_ccm->cbcmr);
	u32 ddr_clk_sel = (cbcmr & MXC_CCM_CBCMR_DDR_CLK_SEL_MASK) \
				>> MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET;
#ifdef CONFIG_MX51
	u32 cbcdr = __raw_readl(&mxc_ccm->cbcdr);
	if (cbcdr & MXC_CCM_CBCDR_DDR_HIFREQ_SEL) {
		u32 ddr_clk_podf = (cbcdr & MXC_CCM_CBCDR_DDR_PODF_MASK) >> \
					MXC_CCM_CBCDR_DDR_PODF_OFFSET;

		ret_val = decode_pll(mxc_plls[PLL1_CLOCK], CONFIG_SYS_MX5_HCLK);
		ret_val /= ddr_clk_podf + 1;

		return ret_val;
	}
#endif
	switch (ddr_clk_sel) {
	case 0:
		ret_val = get_axi_a_clk();
		break;
	case 1:
		ret_val = get_axi_b_clk();
		break;
	case 2:
		ret_val = get_emi_slow_clk();
		break;
	case 3:
		ret_val = get_ahb_clk();
		break;
	default:
		break;
	}

	return ret_val;
}


/*
 * The API of get mxc clockes.
 */
unsigned int mxc_get_clock(enum mxc_clock clk)
{
	switch (clk) {
	case MXC_ARM_CLK:
		return get_mcu_main_clk();
	case MXC_AHB_CLK:
		break;
	case MXC_IPG_CLK:
		return get_ipg_clk();
	case MXC_IPG_PERCLK:
		return get_ipg_per_clk();
	case MXC_UART_CLK:
		return get_uart_clk();
	case MXC_CSPI_CLK:
		return get_cspi_clk();
	case MXC_FEC_CLK:
		return decode_pll(mxc_plls[PLL1_CLOCK],
				    CONFIG_SYS_MX5_HCLK);
	case MXC_DDR_CLK:
		return get_ddr_clk();
	default:
		break;
	}
	return -1;
}

u32 imx_get_uartclk(void)
{
	return get_uart_clk();
}


u32 imx_get_fecclk(void)
{
	return mxc_get_clock(MXC_IPG_CLK);
}

/*
 * Clock config code start here
 */

/* precondition: m>0 and n>0.  Let g=gcd(m,n). */
static int gcd(int m, int n)
{
	int t;
	while (m > 0) {
		if (n > m) {
			t = m;
			m = n;
			n = t;
		} /* swap */
		m -= n;
	}
	return n;
}

/*
 * This is to calculate various parameters based on reference clock and
 * targeted clock based on the equation:
 *      t_clk = 2*ref_freq*(mfi + mfn/(mfd+1))/(pd+1)
 * This calculation is based on a fixed MFD value for simplicity.
 *
 * @param ref       reference clock freq in Hz
 * @param target    targeted clock in Hz
 * @param pll       pll_param structure.
 *
 * @return          0 if successful; non-zero otherwise.
 */
static int calc_pll_params(u32 ref, u32 target, struct pll_param *pll)
{
	u64 pd, mfi = 1, mfn, mfd, t1;
	u32 n_target = target;
	u32 n_ref = ref, i;

	/*
	 * Make sure targeted freq is in the valid range.
	 * Otherwise the following calculation might be wrong!!!
	 */
	if (n_target < PLL_FREQ_MIN(ref) ||
		n_target > PLL_FREQ_MAX(ref)) {
		printf("Targeted peripheral clock should be"
			"within [%d - %d]\n",
			PLL_FREQ_MIN(ref) / SZ_DEC_1M,
			PLL_FREQ_MAX(ref) / SZ_DEC_1M);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(fixed_mfd); i++) {
		if (fixed_mfd[i].ref_clk_hz == ref) {
			mfd = fixed_mfd[i].mfd;
			break;
		}
	}

	if (i == ARRAY_SIZE(fixed_mfd))
		return -1;

	/* Use n_target and n_ref to avoid overflow */
	for (pd = 1; pd <= PLL_PD_MAX; pd++) {
		t1 = n_target * pd;
		do_div(t1, (4 * n_ref));
		mfi = t1;
		if (mfi > PLL_MFI_MAX)
			return -1;
		else if (mfi < 5)
			continue;
		break;
	}
	/* Now got pd and mfi already */
	/*
	mfn = (((n_target * pd) / 4 - n_ref * mfi) * mfd) / n_ref;
	*/
	t1 = n_target * pd;
	do_div(t1, 4);
	t1 -= n_ref * mfi;
	t1 *= mfd;
	do_div(t1, n_ref);
	mfn = t1;
#ifdef CMD_CLOCK_DEBUG
	printf("ref=%d, target=%d, pd=%d," "mfi=%d,mfn=%d, mfd=%d\n",
		ref, n_target, (u32)pd, (u32)mfi, (u32)mfn, (u32)mfd);
#endif
	i = 1;
	if (mfn != 0)
		i = gcd(mfd, mfn);
	pll->pd = (u32)pd;
	pll->mfi = (u32)mfi;
	do_div(mfn, i);
	pll->mfn = (u32)mfn;
	do_div(mfd, i);
	pll->mfd = (u32)mfd;

	return 0;
}

#define calc_div(tgt_clk, src_clk, limit) ({		\
		u32 v = 0;				\
		if (((src_clk) % (tgt_clk)) <= 100)	\
			v = (src_clk) / (tgt_clk);	\
		else					\
			v = ((src_clk) / (tgt_clk)) + 1;\
		if (v > limit)				\
			v = limit;			\
		(v - 1);				\
	})

static u32 calc_per_cbcdr_val(u32 per_clk)
{
	u32 cbcdr = __raw_readl(&mxc_ccm->cbcdr);
	u32 tmp_clk = 0, div = 0, clk_sel = 0;

	cbcdr &= ~MXC_CCM_CBCDR_PERIPH_CLK_SEL;

	/* emi_slow_podf divider */
	tmp_clk = get_emi_slow_clk();
	clk_sel = cbcdr & MXC_CCM_CBCDR_EMI_CLK_SEL;
	if (clk_sel) {
		div = calc_div(tmp_clk, per_clk, 8);
		cbcdr &= ~MXC_CCM_CBCDR_EMI_PODF_MASK;
		cbcdr |= (div << MXC_CCM_CBCDR_EMI_PODF_OFFSET);
	}

	/* axi_b_podf divider */
	tmp_clk = get_axi_b_clk();
	div = calc_div(tmp_clk, per_clk, 8);
	cbcdr &= ~MXC_CCM_CBCDR_AXI_B_PODF_MASK;
	cbcdr |= (div << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET);

	/* axi_b_podf divider */
	tmp_clk = get_axi_a_clk();
	div = calc_div(tmp_clk, per_clk, 8);
	cbcdr &= ~MXC_CCM_CBCDR_AXI_A_PODF_MASK;
	cbcdr |= (div << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET);

	/* ahb podf divider */
	tmp_clk = AHB_CLK_ROOT;
	div = calc_div(tmp_clk, per_clk, 8);
	cbcdr &= ~MXC_CCM_CBCDR_AHB_PODF_MASK;
	cbcdr |= (div << MXC_CCM_CBCDR_AHB_PODF_OFFSET);

	return cbcdr;
}

#define CHANGE_PLL_SETTINGS(pll, pd, fi, fn, fd) \
	{	\
		__raw_writel(0x1232, &pll->ctrl);		\
		__raw_writel(0x2, &pll->config);		\
		__raw_writel((((pd) - 1) << 0) | ((fi) << 4),	\
			&pll->op);				\
		__raw_writel(fn, &(pll->mfn));			\
		__raw_writel((fd) - 1, &pll->mfd);		\
		__raw_writel((((pd) - 1) << 0) | ((fi) << 4),	\
			&pll->hfs_op);				\
		__raw_writel(fn, &pll->hfs_mfn);		\
		__raw_writel((fd) - 1, &pll->hfs_mfd);		\
		__raw_writel(0x1232, &pll->ctrl);		\
		while (!__raw_readl(&pll->ctrl) & 0x1)		\
			;\
	}

static int config_pll_clk(enum pll_clocks index, struct pll_param *pll_param)
{
	u32 ccsr = __raw_readl(&mxc_ccm->ccsr);
	struct mxc_pll_reg *pll = mxc_plls[index];

	switch (index) {
	case PLL1_CLOCK:
		/* Switch ARM to PLL2 clock */
		__raw_writel(ccsr | 0x4, &mxc_ccm->ccsr);
		CHANGE_PLL_SETTINGS(pll, pll_param->pd,
					pll_param->mfi, pll_param->mfn,
					pll_param->mfd);
		/* Switch back */
		__raw_writel(ccsr & ~0x4, &mxc_ccm->ccsr);
		break;
	case PLL2_CLOCK:
		/* Switch to pll2 bypass clock */
		__raw_writel(ccsr | 0x2, &mxc_ccm->ccsr);
		CHANGE_PLL_SETTINGS(pll, pll_param->pd,
					pll_param->mfi, pll_param->mfn,
					pll_param->mfd);
		/* Switch back */
		__raw_writel(ccsr & ~0x2, &mxc_ccm->ccsr);
		break;
	case PLL3_CLOCK:
		/* Switch to pll3 bypass clock */
		__raw_writel(ccsr | 0x1, &mxc_ccm->ccsr);
		CHANGE_PLL_SETTINGS(pll, pll_param->pd,
					pll_param->mfi, pll_param->mfn,
					pll_param->mfd);
		/* Switch back */
		__raw_writel(ccsr & ~0x1, &mxc_ccm->ccsr);
		break;
	case PLL4_CLOCK:
		/* Switch to pll4 bypass clock */
		__raw_writel(ccsr | 0x20, &mxc_ccm->ccsr);
		CHANGE_PLL_SETTINGS(pll, pll_param->pd,
					pll_param->mfi, pll_param->mfn,
					pll_param->mfd);
		/* Switch back */
		__raw_writel(ccsr & ~0x20, &mxc_ccm->ccsr);
		break;
	default:
		return -1;
	}

	return 0;
}

/* Config CPU clock */
static int config_core_clk(u32 ref, u32 freq)
{
	int ret = 0;
	struct pll_param pll_param;

	memset(&pll_param, 0, sizeof(struct pll_param));

	/* The case that periph uses PLL1 is not considered here */
	ret = calc_pll_params(ref, freq, &pll_param);
	if (ret != 0) {
		printf("Error:Can't find pll parameters: %d\n", ret);
		return ret;
	}

	return config_pll_clk(PLL1_CLOCK, &pll_param);
}

static int config_nfc_clk(u32 nfc_clk)
{
	u32 reg;
	u32 parent_rate = get_emi_slow_clk();
	u32 div = parent_rate / nfc_clk;

	if (nfc_clk <= 0)
		return -1;
	if (div == 0)
		div++;
	if (parent_rate / div > NFC_CLK_MAX)
		div++;
	reg = __raw_readl(&mxc_ccm->cbcdr);
	reg &= ~MXC_CCM_CBCDR_NFC_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_NFC_PODF_OFFSET;
	__raw_writel(reg, &mxc_ccm->cbcdr);
	while (__raw_readl(&mxc_ccm->cdhipr) != 0)
		;
	return 0;
}

/* Config main_bus_clock for periphs */
static int config_periph_clk(u32 ref, u32 freq)
{
	int ret = 0;
	struct pll_param pll_param;

	memset(&pll_param, 0, sizeof(struct pll_param));

	if (__raw_readl(&mxc_ccm->cbcdr) & MXC_CCM_CBCDR_PERIPH_CLK_SEL) {
		ret = calc_pll_params(ref, freq, &pll_param);
		if (ret != 0) {
			printf("Error:Can't find pll parameters: %d\n",
				ret);
			return ret;
		}
		switch ((__raw_readl(&mxc_ccm->cbcmr) & \
			MXC_CCM_CBCMR_PERIPH_CLK_SEL_MASK) >> \
			MXC_CCM_CBCMR_PERIPH_CLK_SEL_OFFSET) {
		case 0:
			return config_pll_clk(PLL1_CLOCK, &pll_param);
			break;
		case 1:
			return config_pll_clk(PLL3_CLOCK, &pll_param);
			break;
		default:
			return -1;
		}
	} else {
		u32 old_cbcmr = __raw_readl(&mxc_ccm->cbcmr);
		u32 new_cbcdr = calc_per_cbcdr_val(freq);
		u32 old_nfc = get_nfc_clk();

		/* Switch peripheral to PLL3 */
		__raw_writel(0x00015154, &mxc_ccm->cbcmr);
		__raw_writel(0x02888945, &mxc_ccm->cbcdr);

		/* Make sure change is effective */
		while (__raw_readl(&mxc_ccm->cdhipr) != 0)
			;

		/* Setup PLL2 */
		ret = calc_pll_params(ref, freq, &pll_param);
		if (ret != 0) {
			printf("Error:Can't find pll parameters: %d\n",
				ret);
			return ret;
		}
		config_pll_clk(PLL2_CLOCK, &pll_param);

		/* Switch peripheral back */
		__raw_writel(new_cbcdr, &mxc_ccm->cbcdr);
		__raw_writel(old_cbcmr, &mxc_ccm->cbcmr);

		/* Make sure change is effective */
		while (__raw_readl(&mxc_ccm->cdhipr) != 0)
			;
		/* restore to old NFC clock */
		config_nfc_clk(old_nfc);
	}

	return 0;
}

static int config_ddr_clk(u32 emi_clk)
{
	u32 clk_src;
	s32 shift = 0, clk_sel, div = 1;
	u32 cbcmr = __raw_readl(&mxc_ccm->cbcmr);
	u32 cbcdr = __raw_readl(&mxc_ccm->cbcdr);

	if (emi_clk > MAX_DDR_CLK) {
		printf("Warning:DDR clock should not exceed %d MHz\n",
			MAX_DDR_CLK / SZ_DEC_1M);
		emi_clk = MAX_DDR_CLK;
	}

	clk_src = get_periph_clk();
	/* Find DDR clock input */
	clk_sel = (cbcmr >> 10) & 0x3;
	switch (clk_sel) {
	case 0:
		shift = 16;
		break;
	case 1:
		shift = 19;
		break;
	case 2:
		shift = 22;
		break;
	case 3:
		shift = 10;
		break;
	default:
		return -1;
	}

	if ((clk_src % emi_clk) < 10000000)
		div = clk_src / emi_clk;
	else
		div = (clk_src / emi_clk) + 1;
	if (div > 8)
		div = 8;

	cbcdr = cbcdr & ~(0x7 << shift);
	cbcdr |= ((div - 1) << shift);
	__raw_writel(cbcdr, &mxc_ccm->cbcdr);
	while (__raw_readl(&mxc_ccm->cdhipr) != 0)
		;
	__raw_writel(0x0, &mxc_ccm->ccdr);

	return 0;
}

/*!
 * This function assumes the expected core clock has to be changed by
 * modifying the PLL. This is NOT true always but for most of the times,
 * it is. So it assumes the PLL output freq is the same as the expected
 * core clock (presc=1) unless the core clock is less than PLL_FREQ_MIN.
 * In the latter case, it will try to increase the presc value until
 * (presc*core_clk) is greater than PLL_FREQ_MIN. It then makes call to
 * calc_pll_params() and obtains the values of PD, MFI,MFN, MFD based
 * on the targeted PLL and reference input clock to the PLL. Lastly,
 * it sets the register based on these values along with the dividers.
 * Note 1) There is no value checking for the passed-in divider values
 *         so the caller has to make sure those values are sensible.
 *      2) Also adjust the NFC divider such that the NFC clock doesn't
 *         exceed NFC_CLK_MAX.
 *      3) IPU HSP clock is independent of AHB clock. Even it can go up to
 *         177MHz for higher voltage, this function fixes the max to 133MHz.
 *      4) This function should not have allowed diag_printf() calls since
 *         the serial driver has been stoped. But leave then here to allow
 *         easy debugging by NOT calling the cyg_hal_plf_serial_stop().
 *
 * @param ref   pll input reference clock (24MHz)
 * @param freq  core clock in Hz
 * @param clk   clock type, e.g CPU_CLK, DDR_CLK, etc.
 * @return      0 if successful; non-zero otherwise
 */
int mxc_set_clock(u32 ref, u32 freq, enum mxc_clock clk)
{
	freq *= SZ_DEC_1M;

	switch (clk) {
	case MXC_ARM_CLK:
		if (config_core_clk(ref, freq))
			return -1;
		break;
	case MXC_PERIPH_CLK:
		if (config_periph_clk(ref, freq))
			return -1;
		break;
	case MXC_DDR_CLK:
		if (config_ddr_clk(freq))
			return -1;
		break;
	case MXC_NFC_CLK:
		if (config_nfc_clk(freq))
			return -1;
		break;
	default:
		printf("Warning:Unsupported or invalid clock type\n");
	}

	return 0;
}


/*
 * Dump some core clockes.
 */
int do_mx5_showclocks(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	u32 freq;

	freq = decode_pll(mxc_plls[PLL1_CLOCK], CONFIG_SYS_MX5_HCLK);
	printf("pll1: %dMHz\n", freq / 1000000);
	freq = decode_pll(mxc_plls[PLL2_CLOCK], CONFIG_SYS_MX5_HCLK);
	printf("pll2: %dMHz\n", freq / 1000000);
	freq = decode_pll(mxc_plls[PLL3_CLOCK], CONFIG_SYS_MX5_HCLK);
	printf("pll3: %dMHz\n", freq / 1000000);
	printf("ipg clock     : %dHz\n", mxc_get_clock(MXC_IPG_CLK));
	printf("ipg per clock : %dHz\n", mxc_get_clock(MXC_IPG_PERCLK));
	printf("ddr clock     : %dHz\n", mxc_get_clock(MXC_DDR_CLK));

	return 0;
}

/***************************************************/

U_BOOT_CMD(
	clocks,	CONFIG_SYS_MAXARGS, 1, do_mx5_showclocks,
	"display clocks",
	""
);
