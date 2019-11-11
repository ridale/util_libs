/*
* @TAG(OTHER_GPL)
*/

/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2017 NXP
 */

#pragma once

#include <stdint.h>

#define ROM_VERSION_A0		0x800
#define ROM_VERSION_B0		0x83C

#define M4_BOOTROM_BASE_ADDR	0x007E0000

#define SAI1_BASE_ADDR		0x30010000
#define SAI6_BASE_ADDR		0x30030000
#define SAI5_BASE_ADDR		0x30040000
#define SAI4_BASE_ADDR		0x30050000
#define SPBA2_BASE_ADDR		0x300F0000
#define AIPS1_BASE_ADDR		0x301F0000
#define GPIO1_BASE_ADDR		0X30200000
#define GPIO2_BASE_ADDR		0x30210000
#define GPIO3_BASE_ADDR		0x30220000
#define GPIO4_BASE_ADDR		0x30230000
#define GPIO5_BASE_ADDR		0x30240000
#define ANA_TSENSOR_BASE_ADDR	0x30260000
#define ANA_OSC_BASE_ADDR	0x30270000
#define WDOG1_BASE_ADDR		0x30280000
#define WDOG2_BASE_ADDR		0x30290000
#define WDOG3_BASE_ADDR		0x302A0000
#define SDMA2_BASE_ADDR		0x302C0000
#define GPT1_BASE_ADDR		0x302D0000
#define GPT2_BASE_ADDR		0x302E0000
#define GPT3_BASE_ADDR		0x302F0000
#define ROMCP_BASE_ADDR		0x30310000
#define LCDIF_BASE_ADDR		0x30320000
#define IOMUXC_BASE_ADDR	0x30330000
#define IOMUXC_GPR_BASE_ADDR	0x30340000
#define OCOTP_BASE_ADDR		0x30350000
#define ANATOP_BASE_ADDR	0x30360000
#define SNVS_HP_BASE_ADDR	0x30370000
#define CCM_BASE_ADDR		0x30380000
#define SRC_BASE_ADDR		0x30390000
#define GPC_BASE_ADDR		0x303A0000
#define SEMAPHORE1_BASE_ADDR	0x303B0000
#define SEMAPHORE2_BASE_ADDR	0x303C0000
#define RDC_BASE_ADDR		0x303D0000
#define CSU_BASE_ADDR		0x303E0000

#define AIPS2_BASE_ADDR		0x305F0000
#define PWM1_BASE_ADDR		0x30660000
#define PWM2_BASE_ADDR		0x30670000
#define PWM3_BASE_ADDR		0x30680000
#define PWM4_BASE_ADDR		0x30690000
#define SYSCNT_RD_BASE_ADDR	0x306A0000
#define SYSCNT_CMP_BASE_ADDR	0x306B0000
#define SYSCNT_CTRL_BASE_ADDR	0x306C0000
#define GPT6_BASE_ADDR		0x306E0000
#define GPT5_BASE_ADDR		0x306F0000
#define GPT4_BASE_ADDR		0x30700000
#define PERFMON1_BASE_ADDR	0x307C0000
#define PERFMON2_BASE_ADDR	0x307D0000
#define QOSC_BASE_ADDR		0x307F0000

#define SPDIF1_BASE_ADDR	0x30810000
#define ECSPI1_BASE_ADDR	0x30820000
#define ECSPI2_BASE_ADDR	0x30830000
#define ECSPI3_BASE_ADDR	0x30840000
#define UART1_BASE_ADDR		0x30860000
#define UART3_BASE_ADDR		0x30880000
#define UART2_BASE_ADDR		0x30890000
#define SPDIF2_BASE_ADDR	0x308A0000
#define SAI2_BASE_ADDR		0x308B0000
#define SAI3_BASE_ADDR		0x308C0000
#define SPBA1_BASE_ADDR		0x308F0000
#define CAAM_BASE_ADDR		0x30900000
#define AIPS3_BASE_ADDR		0x309F0000
#define MIPI_PHY_BASE_ADDR	0x30A00000
#define MIPI_DSI_BASE_ADDR	0x30A10000
#define I2C1_BASE_ADDR		0x30A20000
#define I2C2_BASE_ADDR		0x30A30000
#define I2C3_BASE_ADDR		0x30A40000
#define I2C4_BASE_ADDR		0x30A50000
#define UART4_BASE_ADDR		0x30A60000
#define MIPI_CSI_BASE_ADDR	0x30A70000
#define MIPI_CSI_PHY1_BASE_ADDR	0x30A80000
#define CSI1_BASE_ADDR		0x30A90000
#define MU_A_BASE_ADDR		0x30AA0000
#define MU_B_BASE_ADDR		0x30AB0000
#define SEMAPHOR_HS_BASE_ADDR	0x30AC0000
#define USDHC1_BASE_ADDR	0x30B40000
#define USDHC2_BASE_ADDR	0x30B50000
#define MIPI_CS2_BASE_ADDR	0x30B60000
#define MIPI_CSI_PHY2_BASE_ADDR	0x30B70000
#define CSI2_BASE_ADDR		0x30B80000
#define QSPI0_BASE_ADDR		0x30BB0000
#define QSPI0_AMBA_BASE		0x08000000
#define SDMA1_BASE_ADDR		0x30BD0000
#define ENET1_BASE_ADDR		0x30BE0000

#define HDMI_CTRL_BASE_ADDR	0x32C00000
#define AIPS4_BASE_ADDR		0x32DF0000
#define DC1_BASE_ADDR		0x32E00000
#define DC2_BASE_ADDR		0x32E10000
#define DC3_BASE_ADDR		0x32E20000
#define HDMI_SEC_BASE_ADDR	0x32E40000
#define TZASC_BASE_ADDR		0x32F80000
#define MTR_BASE_ADDR		0x32FB0000
#define PLATFORM_CTRL_BASE_ADDR	0x32FE0000

#define MXS_APBH_BASE		0x33000000
#define MXS_GPMI_BASE		0x33002000
#define MXS_BCH_BASE		0x33004000

#define USB1_BASE_ADDR		0x38100000
#define USB2_BASE_ADDR		0x38200000
#define USB1_PHY_BASE_ADDR	0x381F0000
#define USB2_PHY_BASE_ADDR	0x382F0000

#define MXS_LCDIF_BASE		LCDIF_BASE_ADDR

#define SRC_IPS_BASE_ADDR	0x30390000
#define SRC_DDRC_RCR_ADDR	0x30391000
#define SRC_DDRC2_RCR_ADDR	0x30391004

#define DDRC_DDR_SS_GPR0	0x3d000000
#define DDRC_IPS_BASE_ADDR(X)	(0x3d400000 + ((X) * 0x2000000))
#define DDR_CSD1_BASE_ADDR	0x40000000

#if !(defined(__KERNEL_STRICT_NAMES) || defined(__ASSEMBLY__))

#define GPR_TZASC_EN		BIT(0)
#define GPR_TZASC_EN_LOCK	BIT(16)

#define SRC_SCR_M4_ENABLE_OFFSET	3
#define SRC_SCR_M4_ENABLE_MASK		BIT(3)
#define SRC_SCR_M4C_NON_SCLR_RST_OFFSET	0
#define SRC_SCR_M4C_NON_SCLR_RST_MASK	BIT(0)
#define SRC_DDR1_ENABLE_MASK		0x8F000000UL
#define SRC_DDR2_ENABLE_MASK		0x8F000000UL
#define SRC_DDR1_RCR_PHY_PWROKIN_N_MASK	BIT(3)
#define SRC_DDR1_RCR_PHY_RESET_MASK	BIT(2)
#define SRC_DDR1_RCR_CORE_RESET_N_MASK	BIT(1)
#define SRC_DDR1_RCR_PRESET_N_MASK	BIT(0)

struct iomuxc_gpr_base_regs {
	uint32_t gpr[47];
};

struct ocotp_regs {
	uint32_t	ctrl;
	uint32_t	ctrl_set;
	uint32_t     ctrl_clr;
	uint32_t	ctrl_tog;
	uint32_t	timing;
	uint32_t     rsvd0[3];
	uint32_t     data;
	uint32_t     rsvd1[3];
	uint32_t     read_ctrl;
	uint32_t     rsvd2[3];
	uint32_t	read_fuse_data;
	uint32_t     rsvd3[3];
	uint32_t	sw_sticky;
	uint32_t     rsvd4[3];
	uint32_t     scs;
	uint32_t     scs_set;
	uint32_t     scs_clr;
	uint32_t     scs_tog;
	uint32_t     crc_addr;
	uint32_t     rsvd5[3];
	uint32_t     crc_value;
	uint32_t     rsvd6[3];
	uint32_t     version;
	uint32_t     rsvd7[0xdb];

	/* fuse banks */
	struct fuse_bank {
		uint32_t	fuse_regs[0x10];
	} bank[0];
};

struct fuse_bank0_regs {
	uint32_t lock;
	uint32_t rsvd0[3];
	uint32_t uid_low;
	uint32_t rsvd1[3];
	uint32_t uid_high;
	uint32_t rsvd2[7];
};

struct fuse_bank1_regs {
	uint32_t tester3;
	uint32_t rsvd0[3];
	uint32_t tester4;
	uint32_t rsvd1[3];
	uint32_t tester5;
	uint32_t rsvd2[3];
	uint32_t cfg0;
	uint32_t rsvd3[3];
};

struct anamix_pll {
	uint32_t audio_pll1_cfg0;
	uint32_t audio_pll1_cfg1;
	uint32_t audio_pll2_cfg0;
	uint32_t audio_pll2_cfg1;
	uint32_t video_pll_cfg0;
	uint32_t video_pll_cfg1;
	uint32_t gpu_pll_cfg0;
	uint32_t gpu_pll_cfg1;
	uint32_t vpu_pll_cfg0;
	uint32_t vpu_pll_cfg1;
	uint32_t arm_pll_cfg0;
	uint32_t arm_pll_cfg1;
	uint32_t sys_pll1_cfg0;
	uint32_t sys_pll1_cfg1;
	uint32_t sys_pll1_cfg2;
	uint32_t sys_pll2_cfg0;
	uint32_t sys_pll2_cfg1;
	uint32_t sys_pll2_cfg2;
	uint32_t sys_pll3_cfg0;
	uint32_t sys_pll3_cfg1;
	uint32_t sys_pll3_cfg2;
	uint32_t video_pll2_cfg0;
	uint32_t video_pll2_cfg1;
	uint32_t video_pll2_cfg2;
	uint32_t dram_pll_cfg0;
	uint32_t dram_pll_cfg1;
	uint32_t dram_pll_cfg2;
	uint32_t digprog;
	uint32_t osc_misc_cfg;
	uint32_t pllout_monitor_cfg;
	uint32_t frac_pllout_div_cfg;
	uint32_t sscg_pllout_div_cfg;
};

struct fuse_bank9_regs {
	uint32_t mac_addr0;
	uint32_t rsvd0[3];
	uint32_t mac_addr1;
	uint32_t rsvd1[11];
};

/* System Reset Controller (SRC) */
struct src {
	uint32_t scr;
	uint32_t a53rcr;
	uint32_t a53rcr1;
	uint32_t m4rcr;
	uint32_t reserved1[4];
	uint32_t usbophy1_rcr;
	uint32_t usbophy2_rcr;
	uint32_t mipiphy_rcr;
	uint32_t pciephy_rcr;
	uint32_t hdmi_rcr;
	uint32_t disp_rcr;
	uint32_t reserved2[2];
	uint32_t gpu_rcr;
	uint32_t vpu_rcr;
	uint32_t pcie2_rcr;
	uint32_t mipiphy1_rcr;
	uint32_t mipiphy2_rcr;
	uint32_t reserved3;
	uint32_t sbmr1;
	uint32_t srsr;
	uint32_t reserved4[2];
	uint32_t sisr;
	uint32_t simr;
	uint32_t sbmr2;
	uint32_t gpr1;
	uint32_t gpr2;
	uint32_t gpr3;
	uint32_t gpr4;
	uint32_t gpr5;
	uint32_t gpr6;
	uint32_t gpr7;
	uint32_t gpr8;
	uint32_t gpr9;
	uint32_t gpr10;
	uint32_t reserved5[985];
	uint32_t ddr1_rcr;
	uint32_t ddr2_rcr;
};

struct gpc_reg {
	uint32_t lpcr_bsc;
	uint32_t lpcr_ad;
	uint32_t lpcr_cpu1;
	uint32_t lpcr_cpu2;
	uint32_t lpcr_cpu3;
	uint32_t slpcr;
	uint32_t mst_cpu_mapping;
	uint32_t mmdc_cpu_mapping;
	uint32_t mlpcr;
	uint32_t pgc_ack_sel;
	uint32_t pgc_ack_sel_m4;
	uint32_t gpc_misc;
	uint32_t imr1_core0;
	uint32_t imr2_core0;
	uint32_t imr3_core0;
	uint32_t imr4_core0;
	uint32_t imr1_core1;
	uint32_t imr2_core1;
	uint32_t imr3_core1;
	uint32_t imr4_core1;
	uint32_t imr1_cpu1;
	uint32_t imr2_cpu1;
	uint32_t imr3_cpu1;
	uint32_t imr4_cpu1;
	uint32_t imr1_cpu3;
	uint32_t imr2_cpu3;
	uint32_t imr3_cpu3;
	uint32_t imr4_cpu3;
	uint32_t isr1_cpu0;
	uint32_t isr2_cpu0;
	uint32_t isr3_cpu0;
	uint32_t isr4_cpu0;
	uint32_t isr1_cpu1;
	uint32_t isr2_cpu1;
	uint32_t isr3_cpu1;
	uint32_t isr4_cpu1;
	uint32_t isr1_cpu2;
	uint32_t isr2_cpu2;
	uint32_t isr3_cpu2;
	uint32_t isr4_cpu2;
	uint32_t isr1_cpu3;
	uint32_t isr2_cpu3;
	uint32_t isr3_cpu3;
	uint32_t isr4_cpu3;
	uint32_t slt0_cfg;
	uint32_t slt1_cfg;
	uint32_t slt2_cfg;
	uint32_t slt3_cfg;
	uint32_t slt4_cfg;
	uint32_t slt5_cfg;
	uint32_t slt6_cfg;
	uint32_t slt7_cfg;
	uint32_t slt8_cfg;
	uint32_t slt9_cfg;
	uint32_t slt10_cfg;
	uint32_t slt11_cfg;
	uint32_t slt12_cfg;
	uint32_t slt13_cfg;
	uint32_t slt14_cfg;
	uint32_t pgc_cpu_0_1_mapping;
	uint32_t cpu_pgc_up_trg;
	uint32_t mix_pgc_up_trg;
	uint32_t pu_pgc_up_trg;
	uint32_t cpu_pgc_dn_trg;
	uint32_t mix_pgc_dn_trg;
	uint32_t pu_pgc_dn_trg;
	uint32_t lpcr_bsc2;
	uint32_t pgc_cpu_2_3_mapping;
	uint32_t lps_cpu0;
	uint32_t lps_cpu1;
	uint32_t lps_cpu2;
	uint32_t lps_cpu3;
	uint32_t gpc_gpr;
	uint32_t gtor;
	uint32_t debug_addr1;
	uint32_t debug_addr2;
	uint32_t cpu_pgc_up_status1;
	uint32_t mix_pgc_up_status0;
	uint32_t mix_pgc_up_status1;
	uint32_t mix_pgc_up_status2;
	uint32_t m4_mix_pgc_up_status0;
	uint32_t m4_mix_pgc_up_status1;
	uint32_t m4_mix_pgc_up_status2;
	uint32_t pu_pgc_up_status0;
	uint32_t pu_pgc_up_status1;
	uint32_t pu_pgc_up_status2;
	uint32_t m4_pu_pgc_up_status0;
	uint32_t m4_pu_pgc_up_status1;
	uint32_t m4_pu_pgc_up_status2;
	uint32_t a53_lp_io_0;
	uint32_t a53_lp_io_1;
	uint32_t a53_lp_io_2;
	uint32_t cpu_pgc_dn_status1;
	uint32_t mix_pgc_dn_status0;
	uint32_t mix_pgc_dn_status1;
	uint32_t mix_pgc_dn_status2;
	uint32_t m4_mix_pgc_dn_status0;
	uint32_t m4_mix_pgc_dn_status1;
	uint32_t m4_mix_pgc_dn_status2;
	uint32_t pu_pgc_dn_status0;
	uint32_t pu_pgc_dn_status1;
	uint32_t pu_pgc_dn_status2;
	uint32_t m4_pu_pgc_dn_status0;
	uint32_t m4_pu_pgc_dn_status1;
	uint32_t m4_pu_pgc_dn_status2;
	uint32_t res[3];
	uint32_t mix_pdn_flg;
	uint32_t pu_pdn_flg;
	uint32_t m4_mix_pdn_flg;
	uint32_t m4_pu_pdn_flg;
	uint32_t imr1_core2;
	uint32_t imr2_core2;
	uint32_t imr3_core2;
	uint32_t imr4_core2;
	uint32_t imr1_core3;
	uint32_t imr2_core3;
	uint32_t imr3_core3;
	uint32_t imr4_core3;
	uint32_t pgc_ack_sel_pu;
	uint32_t pgc_ack_sel_m4_pu;
	uint32_t slt15_cfg;
	uint32_t slt16_cfg;
	uint32_t slt17_cfg;
	uint32_t slt18_cfg;
	uint32_t slt19_cfg;
	uint32_t gpc_pu_pwrhsk;
	uint32_t slt0_cfg_pu;
	uint32_t slt1_cfg_pu;
	uint32_t slt2_cfg_pu;
	uint32_t slt3_cfg_pu;
	uint32_t slt4_cfg_pu;
	uint32_t slt5_cfg_pu;
	uint32_t slt6_cfg_pu;
	uint32_t slt7_cfg_pu;
	uint32_t slt8_cfg_pu;
	uint32_t slt9_cfg_pu;
	uint32_t slt10_cfg_pu;
	uint32_t slt11_cfg_pu;
	uint32_t slt12_cfg_pu;
	uint32_t slt13_cfg_pu;
	uint32_t slt14_cfg_pu;
	uint32_t slt15_cfg_pu;
	uint32_t slt16_cfg_pu;
	uint32_t slt17_cfg_pu;
	uint32_t slt18_cfg_pu;
	uint32_t slt19_cfg_pu;
};

#define WDOG_WDT_MASK	BIT(3)
#define WDOG_WDZST_MASK	BIT(0)
struct wdog_regs {
	uint16_t	wcr;	/* Control */
	uint16_t	wsr;	/* Service */
	uint16_t	wrsr;	/* Reset Status */
	uint16_t	wicr;	/* Interrupt Control */
	uint16_t	wmcr;	/* Miscellaneous Control */
};

struct bootrom_sw_info {
	uint8_t reserved_1;
	uint8_t boot_dev_instance;
	uint8_t boot_dev_type;
	uint8_t reserved_2;
	uint32_t core_freq;
	uint32_t axi_freq;
	uint32_t ddr_freq;
	uint32_t tick_freq;
	uint32_t reserved_3[3];
};

#define ROM_SW_INFO_ADDR_B0	0x00000968
#define ROM_SW_INFO_ADDR_A0	0x000009e8

#define ROM_SW_INFO_ADDR is_soc_rev(CHIP_REV_1_0) ? \
		(struct bootrom_sw_info **)ROM_SW_INFO_ADDR_A0 : \
		(struct bootrom_sw_info **)ROM_SW_INFO_ADDR_B0
#endif
