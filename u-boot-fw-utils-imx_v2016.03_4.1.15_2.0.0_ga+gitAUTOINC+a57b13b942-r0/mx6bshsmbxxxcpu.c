/*------------------------------------------------------------------------------
  Copyright 2018 BSH Hausgeraete GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
	 this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
	 notice, this list of conditions and the following disclaimer in the
	 documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
	 contributors may be used to endorse or promote products derived from this
	 software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/


#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>

#ifdef CONFIG_POWER
#include <power/pmic.h>
#include <power/mp541x_pmic.h>
#endif

#include <common.h>
#include <fsl_esdhc.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <i2c.h>
#include <fuse.h>



DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL   (PAD_CTL_PKE | PAD_CTL_PUE | \
						PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
						PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE | \
						PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
						PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
						PAD_CTL_ODE)


/* ######################################### */



#ifdef CONFIG_SYS_I2C_MXC

#define I2C3_DEVICE_INDEX 2

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC and EEPROM */
static struct i2c_pads_info i2c_pad_info_i2c3 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART1_TX_DATA__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_UART1_TX_DATA__GPIO1_IO16 | PC,
		.gp = IMX_GPIO_NR(1, 16),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART1_RX_DATA__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_UART1_RX_DATA__GPIO1_IO17 | PC,
		.gp = IMX_GPIO_NR(1, 17),
	},
};
#endif /* CONFIG_SYS_I2C_MXC */

#ifdef CONFIG_REGULATOR_MP5416
#define POWER_REGULATOR_NAME "MP5416"
#else
#define POWER_REGULATOR_NAME "MP5415"
#endif


#ifdef CONFIG_POWER
#define I2C_PMIC    0
static struct pmic *mp541x;
int power_init_board(void)
{
	int ret;
	unsigned int reg;

	ret = power_mp541x_init(I2C_PMIC);
	if (ret)
		return ret;

	mp541x = pmic_get(POWER_REGULATOR_NAME);
	ret = pmic_probe(mp541x);
	if (ret)
		return ret;

	pmic_reg_read(mp541x, MP541X_ID2, &reg);

	printf("PMIC:  " POWER_REGULATOR_NAME " ID=0x%02x\n", reg);

	return 0;
}
#endif /* CONFIG_POWER */


#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	int is_400M;
	u32 vddarm;
	struct pmic *p = mp541x;

	if (!p) {
			printf("No PMIC found!\n");
			return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
	prep_anatop_bypass();

	/* decrease VDDARM to 1.275V */
	pmic_reg_read(p, MP541X_VSET1, &value);
	value &= ~0x3f;
	value |= MP541X_BUCK_1_3_SETP(12750);
	pmic_reg_write(p, MP541X_VSET1, value);

	is_400M = set_anatop_bypass(1);
	if (is_400M)
		vddarm = MP541X_BUCK_1_3_SETP(10750);
	else
		vddarm = MP541X_BUCK_1_3_SETP(11750);

	pmic_reg_read(p, MP541X_VSET1, &value);
	value &= ~0x3f;
	value |= vddarm;
	pmic_reg_write(p, MP541X_VSET1, value);

	finish_anatop_bypass();
	printf("switch to ldo_bypass mode!\n");
	}
}
#endif


#ifdef CONFIG_FEC_MXC
int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_fec(CONFIG_FEC_ENET_DEV);

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC%d MXC: %s:failed\n", CONFIG_FEC_ENET_DEV, __func__);

	return 0;
}

static int setup_fec(int fec_id)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;
	int ret;

	if (0 == fec_id) {
		/*
		 * Use 50M anatop loopback REF_CLK1 for ENET1,
		 * clear gpr1[13], set gpr1[17]
		 */
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
		ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
		if (ret)
			return ret;

	} else {
		/* clk from phy, set gpr1[14], clear gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
				IOMUX_GPR1_FEC2_CLOCK_MUX2_SEL_MASK);
	}

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	if (CONFIG_FEC_ENET_DEV == 0) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x202);
		phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);
	} else if (CONFIG_FEC_ENET_DEV == 1) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x201);
		phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8110);
	}

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#else /* CONFIG_FEC_MXC */
int board_eth_init(bd_t *bis)
{
	return 0;
}

#define ENET_EN     IMX_GPIO_NR(5, 7)
#define ENET_INT    IMX_GPIO_NR(5, 2)

static iomux_v3_cfg_t const enet_ctrl_pads[] = {
	/* ENET_EN */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* ENET_INT */
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_fec(int fec_id)
{
	imx_iomux_v3_setup_multiple_pads(enet_ctrl_pads, ARRAY_SIZE(enet_ctrl_pads));
	gpio_direction_output(ENET_EN, 1); /* Enable ENET */
	gpio_direction_input(ENET_INT);
	return 0;
}

#endif /* CONFIG_FEC_MXC */

#define PIN_WL_REG_ON     IMX_GPIO_NR(2, 21)
#define PIN_BT_REG_ON     IMX_GPIO_NR(1, 1)

static iomux_v3_cfg_t const wifi_bt_ctrl_pads[] = {
	MX6_PAD_SD1_DATA3__GPIO2_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL), 	/* WL_REG_ON */
	MX6_PAD_GPIO1_IO01__GPIO1_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL), /* BT_REG_ON */
};

static int setup_wifi_bt(void)
{
	imx_iomux_v3_setup_multiple_pads(wifi_bt_ctrl_pads, ARRAY_SIZE(wifi_bt_ctrl_pads));
	/* Set Chip to RESET */
	gpio_direction_output(PIN_WL_REG_ON , 0);
	gpio_direction_output(PIN_BT_REG_ON , 0);

	if (getenv_yesno("wifi_bt_disable") != 1)
	{
		mdelay(20); /* Chip must stay in RESET for at least 10ms */
		gpio_direction_output(PIN_WL_REG_ON , 1);
		gpio_direction_output(PIN_BT_REG_ON , 1);
	}

	return 0;
}


#define VTT_LDO_S3 IMX_GPIO_NR(1, 22)
#define VTT_LDO_S5 IMX_GPIO_NR(1, 18)

static iomux_v3_cfg_t const vtt_ldo_pads[] = {
	/* VTT_S3 */
	MX6_PAD_UART2_CTS_B__GPIO1_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* VTT_S5 */
	MX6_PAD_UART1_CTS_B__GPIO1_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


static void setup_vtt_ldo(void)
{
	imx_iomux_v3_setup_multiple_pads(vtt_ldo_pads, ARRAY_SIZE(vtt_ldo_pads));
	gpio_direction_output(VTT_LDO_S3, 1);
	gpio_direction_output(VTT_LDO_S5, 1);
}


int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_UART4_TX_DATA__UART4_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART4_RX_DATA__UART4_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

iomux_v3_cfg_t nfc_pads[] = {
		MX6_PAD_NAND_CE0_B__RAWNAND_CE0_B      | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_CE1_B__RAWNAND_CE1_B      | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_RE_B__RAWNAND_RE_B        | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_WE_B__RAWNAND_WE_B        | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_WP_B__RAWNAND_WP_B        | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_ALE__RAWNAND_ALE          | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_CLE__RAWNAND_CLE          | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DQS__RAWNAND_DQS          | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_READY_B__RAWNAND_READY_B  | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA00__RAWNAND_DATA00    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA01__RAWNAND_DATA01    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA02__RAWNAND_DATA02    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA03__RAWNAND_DATA03    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA04__RAWNAND_DATA04    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA05__RAWNAND_DATA05    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA06__RAWNAND_DATA06    | MUX_PAD_CTRL(NO_PAD_CTRL),
		MX6_PAD_NAND_DATA07__RAWNAND_DATA07    | MUX_PAD_CTRL(NO_PAD_CTRL),

#if 0
		MX6_PAD_CSI_MCLK__RAWNAND_CE2_B        | MUX_PAD_CTRL(NO_PAD_CTRL), /* N.C. */
		MX6_PAD_CSI_PIXCLK__RAWNAND_CE3_B      | MUX_PAD_CTRL(NO_PAD_CTRL), /* N.C. */
#endif
};


static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(nfc_pads, ARRAY_SIZE(nfc_pads));

	/* disable gpmi and bch clock gating */
	clrbits_le32(&mxc_ccm->CCGR4,
				 MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
				 MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
				 MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
				 MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
				 MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_MASK);

	/* config gpmi and bch clock to 100 MHz */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
					MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
					MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
					MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
					MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
					MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
					MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
				 MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
				 MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
				 MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
				 MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
				 MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_MASK);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}


int board_mmc_get_env_dev(int devno)
{
	return devno;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno;
}

int board_early_init_f(void)
{
	setup_vtt_ldo();
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(I2C3_DEVICE_INDEX, CONFIG_SYS_I2C_SPEED, 0x69, &i2c_pad_info_i2c3);
#endif
	setup_gpmi_nand();

	//setup_fec(CONFIG_FEC_ENET_DEV);
	setup_fec(0); /* 0 = Supply clock from external */

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* NAND */
	{"nand",  MAKE_CFGVAL(0x80, 0x02, 0x00, 0x00)},
	/* 4 bit bus width */
	{"sd1",   MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"sd2",   MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"qspi1", MAKE_CFGVAL(0x10, 0x00, 0x00, 0x00)},
	{NULL,   0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);

#ifdef CONFIG_MFG_UBOOT
	/* Linux kernel is not interested in the boot mode. And if
	 * the bootmode is set via command bmode nobody ever will reset it.
	 * As a result a reset from MFG kernel lets the system again start
	 * in USB mode. By resetting the mode here to "normal" the reset from
	 * MFG lets the board start up "normally" */
	boot_mode_apply(0);
#endif

#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	setenv("board_name", "SMB");
	setenv("board_rev",  "1");
#endif

	setup_wifi_bt();

	return 0;
}



/**
 * detects the i.mx6 ULZ SoC (can only distinguish ULL and ULZ)
 *
 * \return 0: i.mx6 ULL
 * \return 1: i.mx6 ULZ
 */
int is_ulz(void)
{
#define SRC_SBMR2	0x1C
#define SRC_BASE	0x20D8000

	u32 sbmr2;
	sbmr2 = readl(SRC_BASE + SRC_SBMR2);

	/* src_sbmr2 bit 6 is to identify if it is i.MX6ULZ */
	return (sbmr2 & (1 << 6)) ? 1 : 0;
}


int checkboard(void)
{
	if(is_ulz())
	{
		puts("Board: MX6ULZ BSH SMB\n");
	}
	else
	{
		puts("Board: MX6ULL BSH SMB\n");
	}
	return 0;
}

/**
 * checks if a device is secured.
 * A device is secured (for SMM) when HAB is activated
 * \return 0: device is not secured
 *         1: device is secured
 */
int device_secured(void)
{
	int ret;
	int val = 0;

	const int SECURE_DEVICE_BANK = 0;
	const int SECURE_DEVICE_WORD = 6;
	const int SECURE_DEVICE_MASK = 0x02;

	ret = fuse_read(SECURE_DEVICE_BANK, SECURE_DEVICE_WORD, &val);
	if (ret)
	{
		printf("Failed to read secured fuse. This device is considered secure nontheless\n");
		return 1;
	}
	return (val & SECURE_DEVICE_MASK) ? 1 : 0;
}

