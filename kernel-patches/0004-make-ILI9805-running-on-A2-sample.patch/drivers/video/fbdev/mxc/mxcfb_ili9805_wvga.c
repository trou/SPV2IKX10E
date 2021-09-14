/*
 * TODO: Add copyright note
 */


#define DEBUG


#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>

#include <linux/byteorder/generic.h>


#include "mipi_dsi.h"

#define MIPI_DSI_MAX_RET_PACK_SIZE				(0x4)


#define ILI9805_DEF_BRIGHT		(4)
#define ILI9805_MAX_BRIGHT		(4)



#define ILI9805_MAX_DPHY_CLK					(500)		// 18.4.4.1 Two Lanes Speed, Table 47



#define ILI9805_EXTCMD_CMD_SET_ENABLE_REG			(0xFF)
#define ILI9805_SETEXTC_PARAMETER1		            (0xFF)
#define ILI9805_SETEXTC_PARAMETER2		            (0x98)
#define ILI9805_SETEXTC_PARAMETER3		            (0x05)
#define ILI9805_EXTCMD_CMD_SET_ENABLE_REG_LEN		(4)

#define ILI9805_EXTCMD_READ_DEVICE_CODE				(0xD3)
#define ILI9805_EXTCMD_READ_DEVICE_CODE_LEN			4

#define ILI9805_ID									(0x980500)
#define ILI9805_ID_MASK								(0xFFFFFF)



// TODO: All of these define need to be removed at some point. Instead these values need to be read from the device tree
#define DISPLAY_POWER_PIN		 56
#define STANDBY_PIN				169
#define RESET_PIN				176



#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static int ili9805_brightness;
static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi);

//VFP=8, VBP=11, HBP=20

static struct fb_videomode truly_lcd_modedb[] = {
	{
	 "TRULY-WVGA", 62, 480, 800, 36443,
	 36, 10,
	 10, 2,
	 2, 1,
	 FB_SYNC_OE_LOW_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch		= 0x0,
	.data_lane_num  = 2,
	.max_phy_clk    = ILI9805_MAX_DPHY_CLK,
	.dpi_fmt		= MIPI_RGB888,
};
void mipid_ili9805_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &truly_lcd_modedb[0];
	*size = ARRAY_SIZE(truly_lcd_modedb);
	*data = &lcd_config;
}


static int mipid_init_gpios(struct mipi_dsi_info *mipi_dsi)
{
	int ret;

	ret = gpio_request(DISPLAY_POWER_PIN, "DISPLAY_POWER");
	CHECK_RETCODE(ret);

	ret = gpio_request(STANDBY_PIN, "STANDBY");
	CHECK_RETCODE(ret);

	ret = gpio_request(RESET_PIN, "RESET");
	CHECK_RETCODE(ret);

	gpio_direction_output(DISPLAY_POWER_PIN, 0);

	msleep(100);

	gpio_direction_output(STANDBY_PIN, 0);
	gpio_direction_output(RESET_PIN, 0);

	msleep(100);		// reset needs to be low for at 100ms

	gpio_set_value(RESET_PIN, 1);
	gpio_set_value(STANDBY_PIN, 1);		// TODO: ILI has no standby pin. What does this connect to???

	msleep(120);		// this is the maximum reset time for all potential conditions

	return ret;
}



// send a DCS (display command set) to the display. The command has no param
#define DCS_CMD(cmd) do  {																	\
		buf[0] = cmd; 																		\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);		\
		CHECK_RETCODE(err);																	\
	} while(0)

// according to the datasheet for the ILI9805 only use these commands for DCS_CMD (4.6.3.2.1.2, page105)
// 0x0 : NOP
// 0x01: RESET
// 0x10: Sleep In
// 0x11: Sleep Out
// 0x12: Partial Mode
// 0x13: Normal Mode
// 0x22: All Pixel Off
// 0x23: All Pixel On
// 0x28: Display Off
// 0x29: Display On
// 0x34: Tearing Effect Line OFF
// 0x38: Idle Mode Off
// 0x39: Idle Mode On

// send a DCS with one parameter to the display.
#define DCS_CMD_PARAM(cmd, param) do {														\
		b[0] = cmd;																			\
		b[1] = param;																		\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_SHORT_WRITE_PARAM, buf, 0);\
		CHECK_RETCODE(err);																	\
	} while(0)
// according to the datasheet for the ILI9805 only use these commands for DCS_CMD_PARAM (4.6.3.2.1.3, page 106)
// 0x26: Gamma Set
// 0x2C: Memory Write
// 0x35: Tearing Effect Line ON
// 0x36: Memory Access Control
// 0x3A: Interface Pixel Format
// 0x3C: Memory Write Continue
// 0x51: Write Display Brightness
// 0x55: Write Content Adaptive Brightness Control
// 0x5E: Write CABC Minimum Brightness

// It must be assumed that for all other types MIPI DSI_GENERIC_LONG_WRITE or MIPI_DSI_DCS_LONG_WRITE must be used

#define DCS_LCMD(cmd) do { \
	b[0] = cmd; \
	b[1] = 0x0; \
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2); \
	CHECK_RETCODE(err); \
} while(0)


#define DCS_LCMD_PARAM(cmd, param) do { \
	b[0] = cmd; \
	b[1] = param; \
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 2); \
	CHECK_RETCODE(err); \
} while(0)


#define DCS_LCMD_PARAM2(cmd, param1, param2) do { \
	b[0] = cmd; \
	b[1] = param1; \
	b[2] = param2; \
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 3); \
	CHECK_RETCODE(err); \
} while(0)

#define DCS_LCMD_PARAM3(cmd, param1, param2, param3) do { \
	b[0] = cmd; \
	b[1] = param1; \
	b[2] = param2; \
	b[3] = param3; \
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 4); \
	CHECK_RETCODE(err); \
} while(0)

#define DCS_LCMD_PARAM4(cmd, param1, param2, param3, param4) do { \
	b[0] = cmd; \
	b[1] = param1; \
	b[2] = param2; \
	b[3] = param3; \
	b[4] = param4; \
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 5); \
	CHECK_RETCODE(err); \
} while(0)

#define DCS_LCMD_PARAM5(cmd, param1, param2, param3, param4, param5) do { \
	b[0] = cmd; \
	b[1] = param1; \
	b[2] = param2; \
	b[3] = param3; \
	b[4] = param4; \
	b[5] = param5; \
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 6); \
	CHECK_RETCODE(err); \
} while(0)


int mipid_ili9805_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u32 buf[DSI_CMD_BUF_MAXSIZE];
	u8 *b = (u8 *) buf;
	int err;

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD setup.\n");

	err = mipid_init_gpios(mipi_dsi);
	CHECK_RETCODE(err);

	b[0] = ILI9805_EXTCMD_CMD_SET_ENABLE_REG;
	b[1] = ILI9805_SETEXTC_PARAMETER1;
	b[2] = ILI9805_SETEXTC_PARAMETER2;
	b[3] = ILI9805_SETEXTC_PARAMETER3;

	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
					buf, ILI9805_EXTCMD_CMD_SET_ENABLE_REG_LEN);
	CHECK_RETCODE(err);
	buf[0] = MIPI_DSI_MAX_RET_PACK_SIZE;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
				buf, 0);
	CHECK_RETCODE(err);
	buf[0] = ILI9805_EXTCMD_READ_DEVICE_CODE;
	err =  mipi_dsi->mipi_dsi_pkt_read(mipi_dsi,
			MIPI_DSI_DCS_READ,
			buf, ILI9805_EXTCMD_READ_DEVICE_CODE_LEN);
	if (!err && (cpu_to_be32(buf[0] & ILI9805_ID_MASK) == ILI9805_ID)) {
		dev_info(&mipi_dsi->pdev->dev,
				"MIPI DSI LCD ID:0x%x.\n", buf[0]);
	} else {
		dev_err(&mipi_dsi->pdev->dev,
			"mipi_dsi_pkt_read err:%d, data:0x%x.\n",
			err, buf[0]);
		dev_info(&mipi_dsi->pdev->dev,
				"MIPI DSI LCD not detected!\n");
		return err;
	}

	DCS_CMD(MIPI_DCS_SOFT_RESET);
	msleep(120);
	DCS_LCMD_PARAM3(ILI9805_EXTCMD_CMD_SET_ENABLE_REG, ILI9805_SETEXTC_PARAMETER1, ILI9805_SETEXTC_PARAMETER2, ILI9805_SETEXTC_PARAMETER3);
	msleep(100);

	// I omit 0xDF: Select SDA and SDO Output Behaviour as I am not using these two
	// I also omit another msleep(10);
	DCS_LCMD_PARAM4(0xFD, 0x0F, 0x10, 0x44, 0x00);
	msleep(100);

	b[0] = 0xf8;
	b[1] = 0x18;
	b[2] = 0x02;
	b[3] = 0x02;
	b[4] = 0x18;
	b[5] = 0x02;
	b[6] = 0x02;
	b[7] = 0x30;
	b[8] = 0x00;
	b[9] = 0x00;
	b[10] = 0x30;
	b[11] = 0x00;
	b[12] = 0x00;
	b[13] = 0x30;
	b[14] = 0x00;
	b[15] = 0x00;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 16);
	CHECK_RETCODE(err);

	DCS_LCMD_PARAM(0xB8, 0x62);			// DBI Type B settings -> we have never used DBI Type B with the ILITEK. That's odd...

	DCS_LCMD_PARAM(0xF1, 0x00);			// Panel Timing Control1 -> there should be more parameters

	DCS_LCMD_PARAM3(0xF2, 0x00, 0x58, 0x40);	// Panel Timing Control2

	DCS_LCMD_PARAM3(0xF3, 0x60, 0x83, 0x04);	// BIAS Control 1

	DCS_LCMD_PARAM3(0xFC, 0x04, 0x0F, 0x01);		// Panel Timing Control 3


//	DCS_LCMD_PARAM2(0xEB, 0x00, 0x0F); 			 // DSI Lanes Control, one 1 lane
	DCS_LCMD_PARAM2(0xEB, 0x08, 0x0F); 			 // DSI Lanes Control, auto detect


	// positive gamma control
	b[0] = 0xe0;
	b[1] = 0x00;
	b[2] = 0x09;
	b[3] = 0x0c;
	b[4] = 0x06;
	b[5] = 0x0b;
	b[6] = 0x14;
	b[7] = 0x0c;
	b[8] = 0x0c;
	b[9] = 0x00;
	b[10] = 0x04;
	b[11] = 0x10;
	b[12] = 0x18;
	b[13] = 0x1f;
	b[14] = 0x1b;
	b[15] = 0x1f;
	b[16] = 0x00;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 17);
	CHECK_RETCODE(err);

	// negative gamma control
	b[0] = 0xe1;
	b[1] = 0x00;
	b[2] = 0x09;
	b[3] = 0x0c;
	b[4] = 0x06;
	b[5] = 0x0b;
	b[6] = 0x14;
	b[7] = 0x0c;
	b[8] = 0x0c;
	b[9] = 0x00;
	b[10] = 0x04;
	b[11] = 0x10;
	b[12] = 0x18;
	b[13] = 0x1f;
	b[14] = 0x1b;
	b[15] = 0x1f;
	b[16] = 0x00;
	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 17);
	CHECK_RETCODE(err);


	DCS_LCMD_PARAM4(0xc1, 0x13, 0x2d, 0x0d, 0x06);		// Power Control 1
	msleep(10);

	DCS_LCMD_PARAM(0xc7, 0xD7);							// VCOM Control 1
	msleep(10);

	DCS_LCMD_PARAM3(0xB1, 0x00, 0x12, 0x13);			// Frame Rate Control 1, should result in 61.7 Hz
	msleep(10);

	DCS_LCMD_PARAM(0xB4, 0x02);							// Display Inversion Control, this should have more parameters
	msleep(10);

	DCS_LCMD_PARAM2(0xBB, 0x14, 0x55);					// Internal GRAM delay settings


	DCS_CMD_PARAM(MIPI_DCS_SET_ADDRESS_MODE, 0x08);		// Memory Access Control, shoudln't be needed


	DCS_CMD_PARAM(MIPI_DCS_SET_PIXEL_FORMAT, 0x77);		// relevant for both DPI and DBI (and thus DSI) format, here he is using 16bit!!!!!


	DCS_LCMD(0x21);										// display inversion ON


	DCS_LCMD_PARAM(0xB0, 0x01);							// Interface Mode Control


	DCS_LCMD_PARAM(0xB6, 0x31);							// Display Function Control, this might be completely wrong for DSI!!! (->DE mode instead of SYNC mode, clock from RGB???)

//	DCS_LCMD_PARAM5(0xB5, 0x00, 0x08, 0x0B, 0x00, 0x14);	// Blanking porch control, VFP=INVALID SETTINGS!!!, VBP=8, HBP = invalid settings!!!, 0x14 ignored
//	DCS_LCMD_PARAM5(0xB5, 0x0A, 0x02, 0x14, 0x0A, 0x0A);	// Blanking porch control, VFP=8, VBP=11, HBP=20
//	msleep(10);
//
//	DCS_LCMD_PARAM(0xC8, 0xD5);							// Backlight Control 1
//	msleep(10);
//
//	DCS_LCMD_PARAM(0xCF, 0x00);							// Backlight Control 8
//	msleep(10);
//
//	DCS_LCMD_PARAM(0x53, 0x00);						// Write CTRL Display -> Backlight Control Block off, Display Dimming off, Backlight Control Off
//	msleep(10);
//
//	DCS_LCMD_PARAM(0x51, 0x00);						// Brightness Value = 0
//	msleep(10);
//
//	DCS_LCMD_PARAM(0x55, 0x00);						// CABC off
//	msleep(10);

	DCS_LCMD_PARAM(0xDF, 0x23);					// SDI/SDO mode


	DCS_LCMD_PARAM2(0xB9, 0x40, 0x00);			// swap RGB BGR


	// these two calls are not made at all by the init sequence

	DCS_CMD(MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(120);										// datasheet explicitly says we only need 5ms

	DCS_CMD(MIPI_DCS_SET_DISPLAY_ON);
	msleep(50);

	// we omit Tearing Line ON as we are not using the Tearing Linel


//	/* set LCD resolution as 480RGBx800, DPI interface,
//	 * display operation mode: RGB data bypass GRAM mode.
//	 */
//	buf[0] = HX8369_CMD_SETDISP | (HX8369_CMD_SETDISP_1_HALT << 8) |
//			(HX8369_CMD_SETDISP_2_RES_MODE << 16) |
//			(HX8369_CMD_SETDISP_3_BP << 24);
//	buf[1] = HX8369_CMD_SETDISP_4_FP | (HX8369_CMD_SETDISP_5_SAP << 8) |
//			 (HX8369_CMD_SETDISP_6_GENON << 16) |
//			 (HX8369_CMD_SETDISP_7_GENOFF << 24);
//	buf[2] = HX8369_CMD_SETDISP_8_RTN | (HX8369_CMD_SETDISP_9_TEI << 8) |
//			 (HX8369_CMD_SETDISP_10_TEP_UP << 16) |
//			 (HX8369_CMD_SETDISP_11_TEP_LOW << 24);
//	buf[3] = HX8369_CMD_SETDISP_12_BP_PE |
//			(HX8369_CMD_SETDISP_13_FP_PE << 8) |
//			 (HX8369_CMD_SETDISP_14_RTN_PE << 16) |
//			 (HX8369_CMD_SETDISP_15_GON << 24);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
//						buf, HX8369_CMD_SETDISP_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set display waveform cycle */
//	buf[0] = HX8369_CMD_SETCYC | (HX8369_CMD_SETCYC_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETCYC_PARAM_2;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
//						buf, HX8369_CMD_SETCYC_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set GIP timing output control */
//	buf[0] = HX8369_CMD_SETGIP | (HX8369_CMD_SETGIP_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETGIP_PARAM_2;
//	buf[2] = HX8369_CMD_SETGIP_PARAM_3;
//	buf[3] = HX8369_CMD_SETGIP_PARAM_4;
//	buf[4] = HX8369_CMD_SETGIP_PARAM_5;
//	buf[5] = HX8369_CMD_SETGIP_PARAM_6;
//	buf[6] = HX8369_CMD_SETGIP_PARAM_7;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
//				HX8369_CMD_SETGIP_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set power: standby, DC etc. */
//	buf[0] = HX8369_CMD_SETPOWER | (HX8369_CMD_SETPOWER_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETPOWER_PARAM_2;
//	buf[2] = HX8369_CMD_SETPOWER_PARAM_3;
//	buf[3] = HX8369_CMD_SETPOWER_PARAM_4;
//	buf[4] = HX8369_CMD_SETPOWER_PARAM_5;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
//				HX8369_CMD_SETPOWER_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set VCOM voltage. */
//	buf[0] = HX8369_CMD_SETVCOM | (HX8369_CMD_SETVCOM_PARAM_1 << 8);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
//				HX8369_CMD_SETVCOM_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set Panel: BGR/RGB or Inversion. */
//	buf[0] = HX8369_CMD_SETPANEL | (HX8369_CMD_SETPANEL_PARAM_1 << 8);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,
//		MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM, buf, 0);
//	CHECK_RETCODE(err);
//
//	/* Set gamma curve related setting */
//	buf[0] = HX8369_CMD_SETGAMMA | (HX8369_CMD_SETGAMMA_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETGAMMA_PARAM_2;
//	buf[2] = HX8369_CMD_SETGAMMA_PARAM_3;
//	buf[3] = HX8369_CMD_SETGAMMA_PARAM_4;
//	buf[4] = HX8369_CMD_SETGAMMA_PARAM_5;
//	buf[5] = HX8369_CMD_SETGAMMA_PARAM_6;
//	buf[6] = HX8369_CMD_SETGAMMA_PARAM_7;
//	buf[7] = HX8369_CMD_SETGAMMA_PARAM_8;
//	buf[8] = HX8369_CMD_SETGAMMA_PARAM_9;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
//				HX8369_CMD_SETGAMMA_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set MIPI: DPHYCMD & DSICMD, data lane number */
//	buf[0] = HX8369_CMD_SETMIPI | (HX8369_CMD_SETMIPI_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETMIPI_PARAM_2;
//	buf[2] = HX8369_CMD_SETMIPI_PARAM_3;
//	if (lcd_config.data_lane_num == HX8369_ONE_DATA_LANE)
//		buf[2] |= HX8369_CMD_SETMIPI_ONELANE;
//	else
//		buf[2] |= HX8369_CMD_SETMIPI_TWOLANE;
//	buf[3] = HX8369_CMD_SETMIPI_PARAM_4;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE, buf,
//				HX8369_CMD_SETMIPI_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set pixel format:24bpp */
//	buf[0] = HX8369_CMD_SETPIXEL_FMT;
//	switch (lcd_config.dpi_fmt) {
//	case MIPI_RGB565_PACKED:
//	case MIPI_RGB565_LOOSELY:
//	case MIPI_RGB565_CONFIG3:
//		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_16BPP << 8);
//		break;
//
//	case MIPI_RGB666_LOOSELY:
//	case MIPI_RGB666_PACKED:
//		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_18BPP << 8);
//		break;
//
//	case MIPI_RGB888:
//		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_24BPP << 8);
//		break;
//
//	default:
//		buf[0] |= (HX8369_CMD_SETPIXEL_FMT_24BPP << 8);
//		break;
//	}
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
//			buf, 0);
//	CHECK_RETCODE(err);
//
//	/* Set column address: 0~479 */
//	buf[0] = HX8369_CMD_SETCLUMN_ADDR |
//		(HX8369_CMD_SETCLUMN_ADDR_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETCLUMN_ADDR_PARAM_2;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
//				buf, HX8369_CMD_SETCLUMN_ADDR_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set page address: 0~799 */
//	buf[0] = HX8369_CMD_SETPAGE_ADDR |
//		(HX8369_CMD_SETPAGE_ADDR_PARAM_1 << 8);
//	buf[1] = HX8369_CMD_SETPAGE_ADDR_PARAM_2;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_LONG_WRITE,
//					buf, HX8369_CMD_SETPAGE_ADDR_LEN);
//	CHECK_RETCODE(err);
//
//	/* Set display brightness related */
//	buf[0] = HX8369_CMD_WRT_DISP_BRIGHT |
//			(HX8369_CMD_WRT_DISP_BRIGHT_PARAM_1 << 8);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
//		buf, 0);
//	CHECK_RETCODE(err);
//
//	buf[0] = HX8369_CMD_WRT_CABC_CTRL |
//		(HX8369_CMD_WRT_CABC_CTRL_PARAM_1 << 8);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
//		buf, 0);
//	CHECK_RETCODE(err);
//
//	buf[0] = HX8369_CMD_WRT_CTRL_DISP |
//		(HX8369_CMD_WRT_CTRL_DISP_PARAM_1 << 8);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
//		buf, 0);
//	CHECK_RETCODE(err);
//
//	/* exit sleep mode and set display on */
//	buf[0] = MIPI_DCS_EXIT_SLEEP_MODE;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM,
//		buf, 0);
//	CHECK_RETCODE(err);
//	/* To allow time for the supply voltages
//	 * and clock circuits to stabilize.
//	 */
//	msleep(5);
//	buf[0] = MIPI_DCS_SET_DISPLAY_ON;
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM,
//		buf, 0);
//	CHECK_RETCODE(err);

	err = mipid_init_backlight(mipi_dsi);
	return err;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
//	int err;
//	u32 buf;
	int brightness = bl->props.brightness;
//	struct mipi_dsi_info *mipi_dsi = bl_get_data(bl);
//
//	if (bl->props.power != FB_BLANK_UNBLANK ||
//	    bl->props.fb_blank != FB_BLANK_UNBLANK)
//		brightness = 0;
//
//	buf = HX8369_CMD_WRT_DISP_BRIGHT |
//			((brightness & HX8369BL_MAX_BRIGHT) << 8);
//	err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi, MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,
//		&buf, 0);
//	CHECK_RETCODE(err);
//
//	hx8369bl_brightness = brightness & HX8369BL_MAX_BRIGHT;

	dev_dbg(&bl->dev, "mipid backlight bringtness:%d.\n", brightness);
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return ili9805_brightness;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};

static int mipid_init_backlight(struct mipi_dsi_info *mipi_dsi)
{
	struct backlight_properties props;
	struct backlight_device	*bl;

	if (mipi_dsi->bl) {
		pr_debug("mipid backlight already init!\n");
		return 0;
	}
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = ILI9805_MAX_BRIGHT;
	props.type = BACKLIGHT_RAW;
	bl = backlight_device_register("mipid-bl", &mipi_dsi->pdev->dev,
		mipi_dsi, &mipid_lcd_bl_ops, &props);
	if (IS_ERR(bl)) {
		pr_err("error %ld on backlight register\n", PTR_ERR(bl));
		return PTR_ERR(bl);
	}
	mipi_dsi->bl = bl;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.brightness = ILI9805_DEF_BRIGHT;

	mipid_bl_update_status(bl);
	return 0;
}
