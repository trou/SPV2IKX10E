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


#ifndef __MX6BSHSMB_CONFIG_MFG_H
#define __MX6BSHSMB_CONFIG_MFG_H

#define CONFIG_MFG_UBOOT

#include "mx6bshsmbcpu.h"

/* additional flags for MFG U-Boot */
#define CONFIG_IOMUX_LPSR


/* restore MFG environment */
#define CONFIG_MFG_NAND_PARTITION "mtdparts=gpmi-nand:4m(boot),-(ubispace)\0"
#if defined(CONFIG_SYS_BOOT_NAND)
#undef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
   "bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \
	"panel=TFT43AB\0" \
	"fdt_addr=0x83000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"console=ttymxc3\0" \
	"bootargs_nand=setenv bootargs console=ttymxc3,115200 ubi.mtd=1 "  \
		"root=ubi0:rootfs rootfstype=ubifs "                 \
		CONFIG_BOOTARGS_CMA_SIZE \
		CONFIG_MFG_NAND_PARTITION \
	"bootargs=console=ttymxc3,115200 ubi.mtd=1 "  \
		"root=ubi0:rootfs rootfstype=ubifs "		     \
		CONFIG_BOOTARGS_CMA_SIZE \
		CONFIG_MFG_NAND_PARTITION \
	"bootcmd=run bootargs_nand; "\
		"nand read ${loadaddr} 0x800000 0x800000;"\
		"nand read ${fdt_addr} 0x1000000 0x100000;"\
		"bootz ${loadaddr} - ${fdt_addr}\0"\
	"mtdids=nand0=gpmi-nand\0"\
	"mtdparts="\
		CONFIG_MFG_NAND_PARTITION

#else
	#error Only NAND boot supported. Please check CONFIG_SYS_BOOT_NAND !!!
#fi
#endif

#endif
