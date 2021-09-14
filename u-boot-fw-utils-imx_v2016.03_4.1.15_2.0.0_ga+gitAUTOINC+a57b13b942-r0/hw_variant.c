/*
 * Copyright 2020
 * BSH Hausgeraete GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>

extern int is_ulz(void);

int do_hw_variant(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{

	if (argc != 2)
		return CMD_RET_USAGE;

    if( strcmp(argv[1], "ULZ") == 0 )
    {
        if(is_ulz())
        {
            /* printf("ULZ\n"); */
            return 0;
        }
        else
        {
            /* printf("ULL\n"); */
            return 1;
        }
    }
    else if( strcmp(argv[1], "ULL") == 0 )
    {
        if(is_ulz())
        {
            /* printf("ULZ\n"); */
            return 1;
        }
        else
        {
            /* printf("ULL\n"); */
            return 0;
        }
    }
    else
    {
        printf("Unknown HW variant : %s\n", argv[1]);
        return 2;
    }
}

U_BOOT_CMD(
	hwvariant, 2, 1, do_hw_variant,
	"display current hw variant",
	"ULL|ULZ"
);
