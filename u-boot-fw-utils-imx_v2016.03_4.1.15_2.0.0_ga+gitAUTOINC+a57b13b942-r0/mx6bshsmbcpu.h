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

#ifndef __MX6BSHSMB_CONFIG_H
#define __MX6BSHSMB_CONFIG_H


#ifndef CONFIG_MX6UL
#define CONFIG_MX6UL
#endif

#ifndef CONFIG_MX6ULL
#define CONFIG_MX6ULL
#endif


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

#define CONFIG_ROM_UNIFIED_SECTIONS

#define CONFIG_SECURE_BOOT

/* Enable boot counter */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_SYS_BOOTCOUNT_SINGLEWORD
#define CONFIG_SYS_BOOTCOUNT_ADDR   0x020CC068  /* Write boot count into SNVS_LPGPR register (reset upon POR) */

#undef CONFIG_MMC
#undef CONFIG_CMD_MMC
#undef CONFIG_GENERIC_MMC
#undef CONFIG_BOUNCE_BUFFER
#undef CONFIG_FSL_ESDHC
#undef CONFIG_FSL_USDHC
#undef CONFIG_SUPPORT_EMMC_BOOT


#ifdef CONFIG_SECURE_BOOT
	#ifndef CONFIG_CSF_SIZE
	#define CONFIG_CSF_SIZE 0x4000
	#endif
#endif

/* #define CONFIG_SYS_DCACHE_OFF */

#define PHYS_SDRAM_SIZE		SZ_128M
#define CONFIG_BOOTARGS_CMA_SIZE	""

#undef CONFIG_LDO_BYPASS_CHECK


#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT


#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART4_BASE /* Console */

/* MMC Configs */

#undef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM 	0
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR


#ifdef CONFIG_FSL_USDHC
	#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR
	/* NAND pin conflicts with usdhc2 */
	#ifdef CONFIG_SYS_USE_NAND
		#define CONFIG_SYS_FSL_USDHC_NUM	1
	#else
		#define CONFIG_SYS_FSL_USDHC_NUM	2
	#endif
#endif

/* I2C configs */
/* #define CONFIG_CMD_I2C */
#ifdef CONFIG_CMD_I2C
	#define CONFIG_SYS_I2C
	#define CONFIG_SYS_I2C_MXC
	#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
	#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
	#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
	#define CONFIG_SYS_I2C_SPEED		100000
#endif


#define CONFIG_MFG_NAND_PARTITION "mtdparts=gpmi-nand:4m(boot),-(ubispace)\0"

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		CONFIG_BOOTARGS_CMA_SIZE \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		CONFIG_MFG_NAND_PARTITION \
		"clk_ignore_unused "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \

/* BSH Feature fuses */

/* GP1 Fuse register address = 0x660
	=> bank = (((0x660 - 0x400)/0x10)/8) = 4
	=> word = (((0x660 - 0x400)/0x10)%8) = 6 */
#define FUSE_REG_GP1 "4 6 "

/* GP2 Fuse register address = 0x670
	=> bank = (((0x670 - 0x400)/0x10)/8) = 4
	=> word = (((0x670 - 0x400)/0x10)%8) = 7 */
#define FUSE_REG_GP2 "4 7 "

#define IMX_GENERATE_32KHZ_CLK_FUSE         FUSE_REG_GP1 " 0" /* GP1, bit 0 */
#define FEATURE_1                           FUSE_REG_GP1 " 1" /* GP1, bit 1 */
/* ... */
#define FEATURE_32                          FUSE_REG_GP2 " 0" /* GP2, bit 0 */
#define FEATURE_33                          FUSE_REG_GP2 " 1" /* GP2, bit 1 */
/* ... */

#if defined(CONFIG_SYS_BOOT_NAND)
#define CONFIG_MSG_INFO(msg) "echo \033[1m=> " msg "\033[m"
#define CONFIG_MSG_ERROR(msg) "echo \033[31m=> " msg "\033[m"
#define CONFIG_EXTRA_ENV_SETTINGS \
	"initrd_high=0xffffffff\0" /* Disable ramdisk relocation */ \
	"fdt_high=0xffffffff\0" /* Disable device tree relocation */ \
	"bootoption=0\0" /* Select Recovery, ELP, EMS */ \
	"bootlimit=5\0" /* altbootcmd is executed if booting a Fit image fails this many times */ \
	"bootlimit_bmode_usb=11\0" /* fallback to USB boot mode after this many boots */ \
	"wifi_bt_disable=0\0" /* Test flag to force Wifi chip OFF */ \
	"ethernet_disable=0\0" /* Test flag to force ethernet PHY OFF */ \
	"loglevel=4\0" /* Linux kernel console log level */ \
	"console=ttymxc3\0" \
	"mtdids=nand0=gpmi-nand\0" \
	"mtdparts=" CONFIG_MFG_NAND_PARTITION \
	"bootcmd=" \
		"run init; " \
		"if test \"x${bootoption}\" = x1; then " \
			"run boot_elp_image; " \
		"elif test \"x${bootoption}\" = x2; then " \
			"run boot_ems_image; " \
		"elif test \"x${bootoption}\" = x3; then " \
			"run boot_end_of_line_test; " \
		"fi; " \
		"run boot_recovery_image; " \
		"reset; " /* Let the boot counter handle this case */ \
		"\0" \
	"altbootcmd=" \
		"if test ${bootcount} -ge ${bootlimit_bmode_usb}; then " \
			CONFIG_MSG_ERROR("Boot loop detected, fallback to USB bootmode") "; " \
			"bmode usb; " /* Allow flashing via USB if we are in a boot loop */ \
		"fi; " \
		"run init; " \
		"if test \"x${bootoption}\" = x1; then " \
			CONFIG_MSG_ERROR("Boot count for ELP exceeded, trying Recovery") "; " \
			"run boot_recovery_image; " \
		"else " \
			CONFIG_MSG_ERROR("Boot count for Recovery exceeded, trying ELP") "; " \
			"run boot_elp_image; " \
		"fi; " \
		"reset; " /* Let the boot counter handle this case */ \
		"\0" \
	"init=" \
		"ubi part ubispace; " \
		"if test $? -ne 0; then " \
			CONFIG_MSG_ERROR("No UBI Volumes found. Something is really foobar") "; " \
			"sleep 1; " \
			"reset; " \
		"fi; " \
		"if hwvariant ULZ; then " \
			"if fuse test "IMX_GENERATE_32KHZ_CLK_FUSE"; then " /* if fuse is set */ \
				"fdt_config=conf@smb-ulz-ratio.dtb; " \
			"else " \
				"fdt_config=conf@smb-ulz.dtb; " \
			"fi; " \
		"else " \
			"if fuse test "IMX_GENERATE_32KHZ_CLK_FUSE"; then " /* if fuse is set */ \
				"fdt_config=conf@smb-ull-ratio.dtb; " \
			"else " \
				"fdt_config=conf@smb-ull.dtb; " \
			"fi; " \
		"fi; " \
		"if test \"x${ethernet_disable}\" = x1; then " /* ethernet_disable is whitelisted, don't pass it directly to the kernel */ \
			"ethernet_disable_flag=1; " \
		"fi; " \
		"setenv bootargs " /* expand bootargs variable */ \
			"console=${console},${baudrate} " \
			"${mtdparts} " \
			"loglevel=${loglevel} " \
			"ethernet_disable=${ethernet_disable_flag:-0}; " \
		"\0" \
	"boot_ems_image=" \
		"run check_boot_unsigned_images; " \
		"if test $? -eq 1; then " \
			CONFIG_MSG_INFO("Booting EMS image") " && " \
			"ubifsmount ubi0:data && " \
			"ubifsload ${loadaddr} fitImage-bsh-image-ems-initramfs-bsh-smb.bin && " \
			"setenv bootargs ${bootargs} quiet && " \
			"bootm ${loadaddr}#${fdt_config}; " \
		"fi; " \
		"\0" \
	"boot_recovery_image=" \
		"setexpr secondary_boot *0x020d8044 '&' 0x40000000; " /* the rom code uses the SRC_GPR10 register to switch between redundant U-Boot copies */ \
		"if test \"x${secondary_boot}\" = x40000000; then " \
			"setenv bootargs ${bootargs} bsh.secondary_boot=1 && " /* the kernel argument is used to decide which U-Boot copy to update first */ \
		"fi; " \
		"volume=recovery; " \
		"tagsize=0x800; " /* Tag size from BSH tagger for FitImages */ \
		"run verified_boot; " \
		"volume=recovery_backup; " /* Try backup volume */ \
		"setenv bootargs ${bootargs} bsh.recovery_backup=1 && " \
		"run verified_boot; " \
		"\0" \
	"boot_elp_image=" \
		"volume=fw_kernel; " \
		"tagsize=0; " \
		"run verified_boot; " \
		"\0" \
	"boot_end_of_line_test=" \
		"setenv bootargs ${bootargs} systemd.unit=bsh-end-of-line-test.target && " \
		"run boot_elp_image; " \
		"\0" \
	"check_boot_unsigned_images=" \
		"fuse test 0 0 14; " /* Boot unsigned images if SRK_LOCK fuse was not burnt yet */ \
		"\0" \
	"verified_boot=" \
		CONFIG_MSG_INFO("Booting ${volume}") " && " \
		"ubi read ${loadaddr} ${volume} && " \
		"get_fit_ivt_offset ivt_offset ${loadaddr} ${tagsize} && " \
		"run verify_loaded_image && " \
		"bootm ${loadaddr}#${fdt_config}; ||" \
		CONFIG_MSG_ERROR("Booting legacy config ...") "; " \
		"bootm ${loadaddr}; " /* Legacy to also boot an old fitImage */ \
		"\0" \
	"verify_loaded_image=" \
		"run check_boot_unsigned_images; " \
		"if test $? -eq 1; then " \
			"hab_auth_img ${loadaddr} ${ivt_offset}; " \
			CONFIG_MSG_INFO("Strict image signature check not fused yet") "; " \
		"else " \
			"hab_auth_img ${loadaddr} ${ivt_offset}; " \
			"test $? -eq 1; " /* hab_auth_img uses the wrong return code in this u-boot version */ \
		"fi; " \
		"\0" \

#else
	#error Only NAND boot supported. Please check CONFIG_SYS_BOOT_NAND !!!
#endif

/* #define CONFIG_BOOTCOMMAND "run bootcmd" */

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#if defined CONFIG_SYS_BOOT_NAND
#define CONFIG_SYS_USE_NAND
#define CONFIG_ENV_IS_IN_NAND
#endif

#define CONFIG_CMD_BMODE

/* NAND stuff */
#ifdef CONFIG_SYS_USE_NAND
	#define CONFIG_CMD_NAND
	#define CONFIG_CMD_NAND_TRIMFFS

	#define CONFIG_NAND_MXS
	#define CONFIG_SYS_MAX_NAND_DEVICE	1
	#define CONFIG_SYS_NAND_BASE		0x40000000
	#define CONFIG_SYS_NAND_5_ADDR_CYCLE
	#define CONFIG_SYS_NAND_ONFI_DETECTION

	/* DMA stuff, needed for GPMI/MXS NAND support */
	#define CONFIG_APBH_DMA
	#define CONFIG_APBH_DMA_BURST
	#define CONFIG_APBH_DMA_BURST8
#endif


#if defined(CONFIG_ENV_IS_IN_NAND)
	#define CONFIG_BSH_BOOT_PARTITION_SIZE    SZ_4M // This value is BSH specific and might change with a different MTD layout. Use 'cat /proc/mtd' to show MTD layout from Linux
	#define CONFIG_BSH_FCB_SIZE    SZ_1M // Flash control block is at the beginning of the boot partition
	#define CONFIG_BSH_FIRMWARE1_END    (CONFIG_BSH_FCB_SIZE + ((CONFIG_BSH_BOOT_PARTITION_SIZE - CONFIG_BSH_FCB_SIZE) / 2)) // The remaining space is divided by two for the two redundant copies of U-Boot
	#define CONFIG_BSH_FIRMWARE2_END    CONFIG_BSH_BOOT_PARTITION_SIZE
	#define CONFIG_BSH_ENV_MAX_BADBLOCKS    3 // even if we have n bad blocks we can still use the environment

	#define CONFIG_ENV_SECT_SIZE    SZ_128K // erase size from NAND chip
	#define CONFIG_ENV_SIZE    CONFIG_ENV_SECT_SIZE
	#define CONFIG_ENV_RANGE    ((CONFIG_BSH_ENV_MAX_BADBLOCKS+1) * CONFIG_ENV_SIZE) // U-Boot uses the next block if one is marked as bad
	#define CONFIG_ENV_OFFSET    (CONFIG_BSH_FIRMWARE1_END - CONFIG_ENV_RANGE)  // store one environment at the end of each U-Boot copy
	#define CONFIG_ENV_OFFSET_REDUND    (CONFIG_BSH_FIRMWARE2_END - CONFIG_ENV_RANGE)

	#define CONFIG_BSH_ENV_WHITELIST "bootoption", "wifi_bt_disable", "ethernet_disable"
#else
	#error "Only NAND supported. Please check CONFIG_ENV_IS_IN_NAND !!!"
#endif


/* USB Configs */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
	#define CONFIG_USB_EHCI
	#define CONFIG_USB_EHCI_MX6
	#define CONFIG_USB_STORAGE
	#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
	#define CONFIG_USB_HOST_ETHER
	#define CONFIG_USB_ETHER_ASIX
	#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
	#define CONFIG_MXC_USB_FLAGS   0
	#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif


#undef CONFIG_POWER
#undef CONFIG_POWER_I2C
#undef CONFIG_POWER_MP541X
#undef CONFIG_POWER_MP541X_I2C_ADDR

#undef CONFIG_LDO_BYPASS_CHECK



/* UBI FS configuration */
#define CONFIG_CMD_UBI			/* UBI commands */
#define CONFIG_CMD_UBIFS		/* UBIFS commands */
#define CONFIG_LZO				/* LZO is needed for UBIFS */
#define CONFIG_RBTREE			/* needed for UBI */
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	0
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define CONFIG_SYS_NAND_USE_FLASH_BBT

#endif
