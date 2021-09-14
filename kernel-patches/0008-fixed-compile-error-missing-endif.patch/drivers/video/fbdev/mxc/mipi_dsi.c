/*
 * Copyright (C) 2011-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


#define DEBUG

#define USE_DEBUGFS


#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/ipu.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mipi_dsi.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <video/mipi_display.h>




#ifdef USE_DEBUGFS
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#endif

#include "mipi_dsi.h"

#define DISPDRV_MIPI			"mipi_dsi"
#define ROUND_UP(x)			((x)+1)
#define NS2PS_RATIO			(1000)
#define NUMBER_OF_CHUNKS		(0x8)
#define NULL_PKT_SIZE			(0x8)
#define PHY_BTA_MAXTIME			(0xd00)
#define PHY_LP2HS_MAXTIME		(0x40)
#define PHY_HS2LP_MAXTIME		(0x40)
#define	PHY_STOP_WAIT_TIME		(0x20)
#define	DSI_CLKMGR_CFG_CLK_DIV		(0x107)
#define DSI_GEN_PLD_DATA_BUF_ENTRY	(0x10)
#define	MIPI_MUX_CTRL(v)		(((v) & 0x3) << 4)
#define	MIPI_LCD_SLEEP_MODE_DELAY	(120)
#define	MIPI_DSI_REG_RW_TIMEOUT		(20)
#define	MIPI_DSI_PHY_TIMEOUT		(10)

static struct mipi_dsi_match_lcd mipi_dsi_lcd_db[] = {
#ifdef CONFIG_FB_MXC_TRULY_WVGA_SYNC_PANEL
	{
	 "TRULY-WVGA",
	 {mipid_hx8369_get_lcd_videomode, mipid_hx8369_lcd_setup}
	},
#ifdef CONFIG_FB_MXC_BSH_D5_PANEL
	{
	 "BSH-D5",
	 {mipid_ili9805_get_lcd_videomode, mipid_ili9805_lcd_setup}
	},
#endif
	{
	"", {NULL, NULL}
	}
};

struct _mipi_dsi_phy_pll_clk {
	u32		max_phy_clk;
	u32		config;
};


//struct mipi_dsi_host_settings_entry {
//	const char *name[];
//};
//
//
//// this is for registers, that are to be modified
//struct mipi_dsi_host_settings {
//
//};
//


#ifdef USE_DEBUGFS

static int filevalue;
static struct mipi_dsi_info *dsi;

//static u32 readbuf[DSI_CMD_BUF_MAXSIZE];
//static int readbufsize;

static u8 next_reg_to_read;
static u8 next_reg_size;

static inline void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
	bool cmd_mode);

static int mipi_dsi_enable(struct mxc_dispdrv_handle *disp,
			   struct fb_info *fbi);
static void mipi_dsi_disable(struct mxc_dispdrv_handle *disp,
			   struct fb_info *fbi);


//static ssize_t short_write_writer(struct file *fp, char __user *user_buffer, size_t count, loff_t *position)
//{
//	int err;
//	mipi_dsi_set_mode(dsi, 1);
//	u32 buf;
//	if(count > 1)
//		return -EINVAL;
//
//	// copy data from userspace
//	simple_write_to_buffer(&buf, sizeof(buf), position, user_buffer, count);
//
//	err = dsi->mipi_dsi_pkt_write(dsi, MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
//
//	mipi_dsi_set_mode(dsi, 0);
//
//	return err < 0 ? err : count;
//}
//
//
//static ssize_t short_write_param_writer(struct file *fp, char __user *user_buffer, size_t count, loff_t *position)
//{
//	int err;
//	mipi_dsi_set_mode(dsi, 1);
//	u32 buf;
//	u8 *b = &buf;
//	if(count != 2)
//		return -EINVAL;
//
//	// copy data from userspace
//	simple_write_to_buffer(&buf, sizeof(buf), position, user_buffer, count);
//
//	err = dsi->mipi_dsi_pkt_write(dsi, MIPI_DSI_DCS_SHORT_WRITE_PARAM, buf, 0);
//
//	mipi_dsi_set_mode(dsi, 0);
//
//	return err < 0 ? err : count;
//}
//
//
//static ssize_t long_write_writer(struct file *fp, char __user *user_buffer, size_t count, loff_t *position)
//{
//	int err;
//	mipi_dsi_set_mode(dsi, 1);
//	u32 buf[(count+3)/4];
//	u8 *b = &buf;
//	if(count < 1)
//		return -EINVAL;
//
//	// copy data from userspace
//	simple_write_to_buffer(buf, sizeof(buf), position, user_buffer, count);
//
//	err = dsi->mipi_dsi_pkt_write(dsi, MIPI_DSI_DCS_LONG_WRITE, buf, 0);
//
//	mipi_dsi_set_mode(dsi, 0);
//
//	return err < 0 ? err : count;
//}
//
//
//static ssize_t dcs_read_writer(struct file *fp, char __user *user_buffer, size_t count, loff_t *position)
//{
//	int err;
//	mipi_dsi_set_mode(dsi, 1);
//	u32 buf;
//	u8 *b = &buf;
//
//	if(count != 2)
//		return -EINVAL;
//
//	// copy data from userspace
//	simple_write_to_buffer(&buf, sizeof(buf), position, user_buffer, count);
//
//	u8 cnt = b[1];
//
//	if(cnt > sizeof(readbuf))
//		return -EINVAL;
//
//	readbuf[0] = b[0];
//	readbufsize = cnt;
//
//	err =  dsi->mipi_dsi_pkt_read(dsi, MIPI_DSI_DCS_READ, &readbuf, cnt);
//
//	mipi_dsi_set_mode(dsi, 0);
//
//	return err < 0 ? err : count;
//}
//
//
//
//static ssize_t dcs_read_reader(struct file *file,	/* file descriptor */
//		char __user *user_buf,	/* user buffer */
//		size_t len,		/* length of buffer */
//		loff_t *offset)		/* offset in the file */
//{
//	/* simple_read_from_buffer() is helper for copy to user space
//		   It copies up to @2 (num) bytes from kernel buffer @4 (kbuf) at offset
//		   @3 (offset) into the user space address starting at @1 (user_buf).
//		   @5 (len) is max size of user buffer
//		 */
//		return simple_read_from_buffer(user_buf, readbufsize, offset, readbuf, len);
//}
//
//
//
//static const struct file_operations fops_short_write = {
//		.write = short_write_writer,
//};
//
//static const struct file_operations fops_short_write_param = {
//		.write = short_write_param_writer,
//};
//
//static const struct file_operations fops_long_write = {
//		.write = long_write_writer,
//};
//
//static const struct file_operations fops_dcs_read = {
//		.write = dcs_read_writer,
//		.read  = dcs_read_reader,
//};
//
//#define DEBUGFS_USER_DATA(reg, read_param_cnt, write_param_cnt) ((reg) | ((read_param_cnt) << 8) | ((write_param_cnt) << 16))
//#define DEBUGFS_REG(data)(((size_t) data) & 0xFF)
//#define DEBUGFS_READCNT(data)((((size_t) data) >> 8) & 0x00FF)

static u8 dcs_package_for_write_reg(u8 reg)
{
	switch(reg)
	{
	case MIPI_DCS_NOP:
	case MIPI_DCS_SOFT_RESET:
	case MIPI_DCS_ENTER_SLEEP_MODE:
	case MIPI_DCS_EXIT_SLEEP_MODE:
	case MIPI_DCS_ENTER_PARTIAL_MODE:
	case MIPI_DCS_ENTER_NORMAL_MODE:
	case MIPI_DCS_ALL_PIXEL_OFF:
	case MIPI_DCS_ALL_PIXEL_ON:
	case MIPI_DCS_SET_DISPLAY_OFF:
	case MIPI_DCS_SET_DISPLAY_ON:
	case MIPI_DCS_SET_TEAR_OFF:
	case MIPI_DCS_EXIT_IDLE_MODE:
	case MIPI_DCS_ENTER_IDLE_MODE:
		return MIPI_DSI_DCS_SHORT_WRITE;

	case MIPI_DCS_SET_GAMMA_CURVE:
/*	case MIPI_DCS_WRITE_MEMORY_START: write memory start may also use short package, but its better to use long packet */
	case MIPI_DCS_SET_TEAR_ON:
	case MIPI_DCS_SET_ADDRESS_MODE:
	case MIPI_DCS_SET_PIXEL_FORMAT:
/*	case MIPI_DCS_WRITE_MEMORY_CONTINUE: write memory continue may also use short package, but its better to use long packet */
	case 0x53:		/* write control display (only ILI9805 specific??) */
	case 0x55:		/* write CABC (only ILI9805 specific??) */
	case 0x5E:		/* write CABC min (only ILI9805 specific??) */
		return MIPI_DSI_DCS_SHORT_WRITE_PARAM;
	}

	/* default value: generic long write */
	return MIPI_DSI_GENERIC_LONG_WRITE;
}


static int _dcs_write(u8 *param, size_t param_cnt)
{
	int err;

	mipi_dsi_set_mode(dsi, true);	/* command mode */

	err = dsi->mipi_dsi_pkt_write(dsi, dcs_package_for_write_reg(param[0]), (u32 *) param, param_cnt);

	mipi_dsi_set_mode(dsi, false);	/* pixel transmission mode */

	return err;
}


static ssize_t dcs_write(struct file *fp, const char __user *user_buffer, size_t count, loff_t *position)
{
	char buf[80];
	u8 params[20];
	size_t size;
	size_t param_cnt;

	int i;
	int res;

	size = min(sizeof(buf) - 1, count);
	if (copy_from_user(buf, user_buffer, size))
		return -EFAULT;

	/* convert spaces and newline to '\0' */

	for(i = 0; i != size; i++)
		if(buf[i] == ' ' || buf[i] == '\n')
			buf[i] = '\0';

	i = 0;
	res = 0;

	param_cnt = 0;

	do
	{
		res = kstrtou8(buf + i, 0, &params[param_cnt]);
		if(res == 0)
		{
			param_cnt++;
			i += strlen(buf + i) + 1;
		}
	} while(i < size && res == 0 && param_cnt < ARRAY_SIZE(params));

	if(res)		/* not all conversions successful ? */
		return -EFAULT;

	res = _dcs_write(params, param_cnt);

	if(res == param_cnt)
		return count;		/* on success, claim we got the whole input */

	return -EFAULT;
}

static int _dcs_read(u8 reg, u8 *buf, size_t buffersize)
{
	int err;

	mipi_dsi_set_mode(dsi, true);		/* command mode */

	err =  dsi->mipi_dsi_pkt_read(dsi, MIPI_DSI_DCS_READ, (u32 *) buf, buffersize);

	mipi_dsi_set_mode(dsi, false);	/* pixel transmission mode */

	return err;
}

static ssize_t dcs_read(struct file *file,	/* file descriptor */
		char __user *user_buf,	/* user buffer */
		size_t len,		/* length of buffer */
		loff_t *offset)		/* offset in the file */
{
	u8 buf[next_reg_size];
	int err;

	memset(buf, 0, next_reg_size);
	buf[0] = next_reg_to_read;

	err = _dcs_read(next_reg_to_read, buf, next_reg_size);

	if(err == 0)
	{
		u8 outbuf[next_reg_size*5];
		int i;
		u8 temp;
		u8 *p = outbuf;
		for(i = 0; i != next_reg_size; i++)
		{
			*p++ = '0';
			*p++ = 'x';

			// high nibble
			temp = buf[i] / 16;
			if(temp >= 10)
				*p++ = 'A' + temp - 10;
			else
				*p++ = '0' + temp;

			// low nibble
			temp = buf[i] & 0xF;

			if(temp >= 10)
				*p++ = 'A' + temp - 10;
			else
				*p++ = '0' + temp;

			*p++ = ' ';
		}
		p--;			// go back to last space
		*p = '\0';

	/* simple_read_from_buffer() is helper for copy to user space
		   It copies up to @2 (num) bytes from kernel buffer @4 (kbuf) at offset
		   @3 (offset) into the user space address starting at @1 (user_buf).
		   @5 (len) is max size of user buffer
		 */
		return simple_read_from_buffer(user_buf, len, offset, outbuf, sizeof(outbuf));
	}

	return -EINVAL;
}

static int simple_input_check(const char __user *user_buffer)
{
	int val;
	if(user_buffer)
	{
		val = user_buffer[0] - '0';
		return val;
	}
	else
	{
		return -EINVAL;
	}
}


static ssize_t host_func_display_gpios(struct file *fp, const char __user *user_buffer, size_t count, loff_t *position)
{
	int val;
	(void) fp;
	(void) position;

	/* we accept values of '0' and '1', and we ignore everything beyond the first character.
	 * We don't care about proper string termination */

	val = simple_input_check(user_buffer);
	if(val == 0)
	{
		mipid_deinit_gpios(dsi);
		return count;	/* let's say we accept all input data */
	}
	else if(val == 1)
	{
		mipid_init_gpios(dsi);
		return count;	/* let's say we accept all input data */
	}

	return -EINVAL;
}

static ssize_t host_func_dsi_power(struct file *fp, const char __user *user_buffer, size_t count, loff_t *position)
{
	int val;
	struct mxc_dispdrv_handle *handle = dsi->disp_mipi;
	(void) fp;
	(void) position;

	/* we accept values of '0' and '1', and we ignore everything beyond the first character.
	 * We don't care about proper string termination */

	val = simple_input_check(user_buffer);
	if(val == 0)
	{
		mipi_dsi_disable(handle, NULL);
		return count;	/* let's say we accept all input data */
	}
	else if(val == 1)
	{
		mipi_dsi_enable(handle, NULL);
		return count;	/* let's say we accept all input data */
	}
	return -EINVAL;
}



//static const struct file_operations fops_dcs = {
//		.write = dcs_write,
//		.read  = dcs_read,
//};

static const struct file_operations fops_dcs_read = {
		.read = dcs_read,
};

static const struct file_operations fops_dcs_write = {
		.write = dcs_write,
};


static const struct file_operations fops_display_gpios = {
		.write = host_func_display_gpios,
};

static const struct file_operations fops_dsi_power = {
		.write = host_func_dsi_power,
};


//struct dentry *debugfs_create_dcs(const char *name, umode_t mode,
//				 struct dentry *parent, void *data)
//{
//	/* if there are no write bits set, make read only */
//	if (!(mode & S_IWUGO))
//		return debugfs_create_file(name, mode, parent, data, &fops_dcs_ro);
//	/* if there are no read bits set, make write only */
//	if (!(mode & S_IRUGO))
//		return debugfs_create_file(name, mode, parent, data, &fops_dcs_wo);
//
//	return debugfs_create_file(name, mode, parent, data, &fops_dcs);
//}


static void create_host_reg_entries(struct dentry *parent, size_t base)
{
	umode_t mode = 0600;
	umode_t readmode = 0400;


	struct dentry *dir;

	dir = debugfs_create_dir("host_regs", parent);
	if(dir) {
		debugfs_create_x32("version", readmode, dir, (u32 *) (base + MIPI_DSI_VERSION));
		debugfs_create_x32("pwr_up", mode, dir, (u32 *) (base + MIPI_DSI_PWR_UP));
		debugfs_create_x32("clkmgr_cfg", mode, dir, (u32 *) (base + MIPI_DSI_CLKMGR_CFG));
		debugfs_create_x32("dpi_cfg", mode, dir, (u32 *)(base + MIPI_DSI_DPI_CFG));
		debugfs_create_x32("dbi_cfg", mode, dir, (u32 *)(base + MIPI_DSI_DBI_CFG));
		debugfs_create_x32("dbis_cmdsize", mode, dir, (u32 *)(base + MIPI_DSI_DBIS_CMDSIZE));
		debugfs_create_x32("pckhdl_cfg", mode, dir, (u32 *)(base + MIPI_DSI_PCKHDL_CFG));
		debugfs_create_x32("vid_mode_cfg", mode, dir, (u32 *)(base + MIPI_DSI_VID_MODE_CFG));
		debugfs_create_x32("vid_pkt_cfg", mode, dir, (u32 *)(base + MIPI_DSI_VID_PKT_CFG));
		debugfs_create_x32("cmd_mode_cfg", mode, dir, (u32 *)(base + MIPI_DSI_CMD_MODE_CFG));
		debugfs_create_x32("tmr_line_cfg", mode, dir, (u32 *)(base + MIPI_DSI_TMR_LINE_CFG));
		debugfs_create_x32("vtiming_cfg", mode, dir, (u32 *)(base + MIPI_DSI_VTIMING_CFG));
		debugfs_create_x32("phy_tmr_cfg", mode, dir, (u32 *)(base + MIPI_DSI_PHY_TMR_CFG));
		debugfs_create_x32("gen_hdr", mode, dir, (u32 *)(base + MIPI_DSI_GEN_HDR));
		debugfs_create_x32("gen_pld_data", mode, dir, (u32 *)(base + MIPI_DSI_GEN_PLD_DATA));
		debugfs_create_x32("cmd_pkt_status", mode, dir, (u32 *)(base + MIPI_DSI_CMD_PKT_STATUS));
		debugfs_create_x32("to_cnt_cfg", mode, dir, (u32 *)(base + MIPI_DSI_TO_CNT_CFG));
		debugfs_create_x32("error_st0", mode, dir, (u32 *)(base + MIPI_DSI_ERROR_ST0));
		debugfs_create_x32("error_st1", mode, dir, (u32 *)(base + MIPI_DSI_ERROR_ST1));
		debugfs_create_x32("error_msk0", mode, dir, (u32 *)(base + MIPI_DSI_ERROR_MSK0));
		debugfs_create_x32("error_msk1", mode, dir, (u32 *)(base + MIPI_DSI_ERROR_MSK1));
		debugfs_create_x32("phy_rstz", mode, dir, (u32 *)(base + MIPI_DSI_PHY_RSTZ));
		debugfs_create_x32("phy_if_cfg", mode, dir, (u32 *)(base + MIPI_DSI_PHY_IF_CFG));
		debugfs_create_x32("phy_if_ctrl", mode, dir, (u32 *)(base + MIPI_DSI_PHY_IF_CTRL));
		debugfs_create_x32("phy_status", mode, dir, (u32 *)(base + MIPI_DSI_PHY_STATUS));
		debugfs_create_x32("phy_tst_ctrl0", mode, dir, (u32 *)(base + MIPI_DSI_PHY_TST_CTRL0));
		debugfs_create_x32("phy_tst_ctrl1", mode, dir, (u32 *)(base + MIPI_DSI_PHY_TST_CTRL1));
	}
}

static void create_host_func_entries(struct dentry *parent)
{
	umode_t writeonly = 0200;

	struct dentry *dir;

	dir = debugfs_create_dir("host_funcs", parent);
	if(dir) {
		debugfs_create_file("display_gpios",   writeonly, dir, NULL, &fops_display_gpios);
		debugfs_create_file("dsi_power",       writeonly, dir, NULL, &fops_dsi_power);

	}
}


static void create_debugfs_entries(struct mipi_dsi_info *mipi_dsi)
{
	// TODO: we never clean up. That's why we (can) keep this on the stack
	struct dentry *dir;
	dir = debugfs_create_dir(mipi_dsi->disp_mipi->drv->name, NULL);
	if(dir) {
		umode_t mode = 0600;
		umode_t writeonly = 0200;
		size_t base = (size_t) mipi_dsi->mmio_base;

		dsi = mipi_dsi;

		// TODO: we ignore the return values which means that we have no way of cleaning up those entries
		create_host_reg_entries(dir, base);

		create_host_func_entries(dir);



//
//		debugfs_create_file("short_write", writeonly, dir, &filevalue, &fops_short_write);
//		debugfs_create_file("short_write_param", writeonly, dir, &filevalue, &fops_short_write_param);
//		debugfs_create_file("long_write", writeonly, dir, &filevalue, &fops_long_write);
//

		debugfs_create_file("dcs_write", writeonly, dir, &filevalue, &fops_dcs_write);
		debugfs_create_file("dcs_read", mode, dir, &filevalue, &fops_dcs_read);

		debugfs_create_x8("next_dcs_read_reg", mode, dir, &next_reg_to_read);
		debugfs_create_x8("next_dcs_read_buf", mode, dir, &next_reg_size);

	}
}
#else
static void create_debugfs_entries(struct mipi_dsi_info *mipi_dsi)
{
	(VOID) mipi_dsi;		// no warning
}
#endif


/* configure data for DPHY PLL 27M reference clk out */
static const struct _mipi_dsi_phy_pll_clk mipi_dsi_phy_pll_clk_table[] = {
	{1000, 0x74}, /*  950-1000MHz	*/
	{950,  0x54}, /*  900-950Mhz	*/
	{900,  0x34}, /*  850-900Mhz	*/
	{850,  0x14}, /*  800-850MHz	*/
	{800,  0x32}, /*  750-800MHz	*/
	{750,  0x12}, /*  700-750Mhz	*/
	{700,  0x30}, /*  650-700Mhz	*/
	{650,  0x10}, /*  600-650MHz	*/
	{600,  0x2e}, /*  550-600MHz	*/
	{550,  0x0e}, /*  500-550Mhz	*/
	{500,  0x2c}, /*  450-500Mhz	*/
	{450,  0x0c}, /*  400-450MHz	*/
	{400,  0x4a}, /*  360-400MHz	*/
	{360,  0x2a}, /*  330-360Mhz	*/
	{330,  0x48}, /*  300-330Mhz	*/
	{300,  0x28}, /*  270-300MHz	*/
	{270,  0x08}, /*  250-270MHz	*/
	{250,  0x46}, /*  240-250Mhz	*/
	{240,  0x26}, /*  210-240Mhz	*/
	{210,  0x06}, /*  200-210MHz	*/
	{200,  0x44}, /*  180-200MHz	*/
	{180,  0x24}, /*  160-180MHz	*/
	{160,  0x04}, /*  150-160MHz	*/
};

static int valid_mode(int pixel_fmt)
{
	return ((pixel_fmt == IPU_PIX_FMT_RGB24)  ||
			(pixel_fmt == IPU_PIX_FMT_BGR24)  ||
			(pixel_fmt == IPU_PIX_FMT_RGB666) ||
			(pixel_fmt == IPU_PIX_FMT_RGB565) ||
			(pixel_fmt == IPU_PIX_FMT_BGR666) ||
			(pixel_fmt == IPU_PIX_FMT_RGB332));
}

static inline void mipi_dsi_read_register(struct mipi_dsi_info *mipi_dsi,
				u32 reg, u32 *val)
{
	*val = ioread32(mipi_dsi->mmio_base + reg);
	dev_dbg(&mipi_dsi->pdev->dev, "read_reg:0x%02x, val:0x%08x.\n",
			reg, *val);
}

static inline void mipi_dsi_write_register(struct mipi_dsi_info *mipi_dsi,
				u32 reg, u32 val)
{
	iowrite32(val, mipi_dsi->mmio_base + reg);
	dev_dbg(&mipi_dsi->pdev->dev, "\t\twrite_reg:0x%02x, val:0x%08x.\n",
			reg, val);
}

static int mipi_dsi_pkt_write(struct mipi_dsi_info *mipi_dsi,
				u8 data_type, const u32 *buf, int len)
{
	u32 val;
	u32 status = 0;
	int write_len = len;
	uint32_t	timeout = 0;

	if (len) {
		/* generic long write command */
		while (len / DSI_GEN_PLD_DATA_BUF_SIZE) {
			mipi_dsi_write_register(mipi_dsi,
				MIPI_DSI_GEN_PLD_DATA, *buf);
			buf++;
			len -= DSI_GEN_PLD_DATA_BUF_SIZE;
			mipi_dsi_read_register(mipi_dsi,
				MIPI_DSI_CMD_PKT_STATUS, &status);
			while ((status & DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) ==
					 DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) {
				msleep(1);
				timeout++;
				if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
					return -EIO;
				mipi_dsi_read_register(mipi_dsi,
					MIPI_DSI_CMD_PKT_STATUS, &status);
			}
		}
		/* write the remainder bytes */
		if (len > 0) {
			while ((status & DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) ==
					 DSI_CMD_PKT_STATUS_GEN_PLD_W_FULL) {
				msleep(1);
				timeout++;
				if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
					return -EIO;
				mipi_dsi_read_register(mipi_dsi,
					MIPI_DSI_CMD_PKT_STATUS, &status);
			}
			mipi_dsi_write_register(mipi_dsi,
				MIPI_DSI_GEN_PLD_DATA, *buf);
		}

		val = data_type | ((write_len & DSI_GEN_HDR_DATA_MASK)
			<< DSI_GEN_HDR_DATA_SHIFT);
	} else {
		/* generic short write command */
		val = data_type | ((*buf & DSI_GEN_HDR_DATA_MASK)
			<< DSI_GEN_HDR_DATA_SHIFT);
	}

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS, &status);
	while ((status & DSI_CMD_PKT_STATUS_GEN_CMD_FULL) ==
			 DSI_CMD_PKT_STATUS_GEN_CMD_FULL) {
		msleep(1);
		timeout++;
		if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
			return -EIO;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
				&status);
	}
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_GEN_HDR, val);

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS, &status);
	while (!((status & DSI_CMD_PKT_STATUS_GEN_CMD_EMPTY) ==
			 DSI_CMD_PKT_STATUS_GEN_CMD_EMPTY) ||
			!((status & DSI_CMD_PKT_STATUS_GEN_PLD_W_EMPTY) ==
			DSI_CMD_PKT_STATUS_GEN_PLD_W_EMPTY)) {
		msleep(1);
		timeout++;
		if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
			return -EIO;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
				&status);
	}

	return 0;
}

static int mipi_dsi_pkt_read(struct mipi_dsi_info *mipi_dsi,
				u8 data_type, u32 *buf, int len)
{
	u32		val;
	int		read_len = 0;
	uint32_t	timeout = 0;

	if (!len) {
		mipi_dbg("%s, len = 0 invalid error!\n", __func__);
		return -EINVAL;
	}

	val = data_type | ((*buf & DSI_GEN_HDR_DATA_MASK)
		<< DSI_GEN_HDR_DATA_SHIFT);
	memset(buf, 0, len);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_GEN_HDR, val);

	/* wait for cmd to sent out */
	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS, &val);
	while ((val & DSI_CMD_PKT_STATUS_GEN_RD_CMD_BUSY) !=
			 DSI_CMD_PKT_STATUS_GEN_RD_CMD_BUSY) {
		msleep(1);
		timeout++;
		if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
			return -EIO;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
			&val);
	}
	/* wait for entire response stroed in FIFO */
	while ((val & DSI_CMD_PKT_STATUS_GEN_RD_CMD_BUSY) ==
			 DSI_CMD_PKT_STATUS_GEN_RD_CMD_BUSY) {
		msleep(1);
		timeout++;
		if (timeout == MIPI_DSI_REG_RW_TIMEOUT)
			return -EIO;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
			&val);
	}

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS, &val);
	while (!(val & DSI_CMD_PKT_STATUS_GEN_PLD_R_EMPTY)) {
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_GEN_PLD_DATA, buf);
		read_len += DSI_GEN_PLD_DATA_BUF_SIZE;
		buf++;
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_PKT_STATUS,
			&val);
		if (read_len == (DSI_GEN_PLD_DATA_BUF_ENTRY *
					DSI_GEN_PLD_DATA_BUF_SIZE))
			break;
	}

	if ((len <= read_len) &&
		((len + DSI_GEN_PLD_DATA_BUF_SIZE) >= read_len))
		return 0;
	else {
		dev_err(&mipi_dsi->pdev->dev,
			"actually read_len:%d != len:%d.\n", read_len, len);
		return -ERANGE;
	}
}

static int mipi_dsi_dcs_cmd(struct mipi_dsi_info *mipi_dsi,
				u8 cmd, const u32 *param, int num)
{
	int err = 0;
	u32 buf[DSI_CMD_BUF_MAXSIZE];

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	switch (cmd) {
	case MIPI_DCS_EXIT_SLEEP_MODE:
	case MIPI_DCS_ENTER_SLEEP_MODE:
	case MIPI_DCS_SET_DISPLAY_ON:
	case MIPI_DCS_SET_DISPLAY_OFF:
		buf[0] = cmd;
		err = mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
		break;

	default:
	dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command:0x%x Not supported!\n", cmd);
		break;
	}

	return err;
}

static void mipi_dsi_dphy_init(struct mipi_dsi_info *mipi_dsi,
						u32 cmd, u32 data)
{
	u32 val;
	u32 timeout = 0;

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CTRL,
			DSI_PHY_IF_CTRL_RESET);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_POWERUP);

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 0);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL1,
		(0x10000 | cmd));
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 2);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 0);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL1, (0 | data));
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 2);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TST_CTRL0, 0);
	val = DSI_PHY_RSTZ_EN_CLK | DSI_PHY_RSTZ_DISABLE_RST |
			DSI_PHY_RSTZ_DISABLE_SHUTDOWN;
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_RSTZ, val);

	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_PHY_STATUS, &val);
	while ((val & DSI_PHY_STATUS_LOCK) != DSI_PHY_STATUS_LOCK) {
		msleep(1);
		timeout++;
		if (timeout == MIPI_DSI_PHY_TIMEOUT) {
			dev_err(&mipi_dsi->pdev->dev,
				"Error: phy lock timeout!\n");
			break;
		}
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_PHY_STATUS, &val);
	}
	timeout = 0;
	while ((val & DSI_PHY_STATUS_STOPSTATE_CLK_LANE) !=
			DSI_PHY_STATUS_STOPSTATE_CLK_LANE) {
		msleep(1);
		timeout++;
		if (timeout == MIPI_DSI_PHY_TIMEOUT) {
			dev_err(&mipi_dsi->pdev->dev,
				"Error: phy lock lane timeout!\n");
			break;
		}
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_PHY_STATUS, &val);
	}
}

static void mipi_dsi_enable_controller(struct mipi_dsi_info *mipi_dsi,
				bool init)
{
	u32		val = 0;
	u32		lane_byte_clk_period;
	struct  fb_videomode *mode = mipi_dsi->mode;
	struct  mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	if (init) {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_RESET);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_RSTZ,
			DSI_PHY_RSTZ_RST);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CLKMGR_CFG,
			DSI_CLKMGR_CFG_CLK_DIV);

		if (!(mode->sync & FB_SYNC_VERT_HIGH_ACT))
			val = DSI_DPI_CFG_VSYNC_ACT_LOW;
		if (!(mode->sync & FB_SYNC_HOR_HIGH_ACT))
			val |= DSI_DPI_CFG_HSYNC_ACT_LOW;
		if ((mode->sync & FB_SYNC_OE_LOW_ACT))
			val |= DSI_DPI_CFG_DATAEN_ACT_LOW;
		if (MIPI_RGB666_LOOSELY == lcd_config->dpi_fmt)
			val |= DSI_DPI_CFG_EN18LOOSELY;
		val |= (lcd_config->dpi_fmt & DSI_DPI_CFG_COLORCODE_MASK)
				<< DSI_DPI_CFG_COLORCODE_SHIFT;
		val |= (lcd_config->virtual_ch & DSI_DPI_CFG_VID_MASK)
				<< DSI_DPI_CFG_VID_SHIFT;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_DPI_CFG, val);

		val = DSI_PCKHDL_CFG_EN_BTA |
				DSI_PCKHDL_CFG_EN_ECC_RX |
				DSI_PCKHDL_CFG_EN_CRC_RX;

		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PCKHDL_CFG, val);

		val = (mode->xres & DSI_VID_PKT_CFG_VID_PKT_SZ_MASK)
				<< DSI_VID_PKT_CFG_VID_PKT_SZ_SHIFT;
		val |= (NUMBER_OF_CHUNKS & DSI_VID_PKT_CFG_NUM_CHUNKS_MASK)
				<< DSI_VID_PKT_CFG_NUM_CHUNKS_SHIFT;
		val |= (NULL_PKT_SIZE & DSI_VID_PKT_CFG_NULL_PKT_SZ_MASK)
				<< DSI_VID_PKT_CFG_NULL_PKT_SZ_SHIFT;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_PKT_CFG, val);

		/* enable LP mode when TX DCS cmd and enable DSI command mode */
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG,
				MIPI_DSI_CMD_MODE_CFG_EN_LOWPOWER);

		 /* mipi lane byte clk period in ns unit */
		lane_byte_clk_period = NS2PS_RATIO /
				(lcd_config->max_phy_clk / BITS_PER_BYTE);
		val  = ROUND_UP(mode->hsync_len * mode->pixclock /
				NS2PS_RATIO / lane_byte_clk_period)
				<< DSI_TME_LINE_CFG_HSA_TIME_SHIFT;
		val |= ROUND_UP(mode->left_margin * mode->pixclock /
				NS2PS_RATIO / lane_byte_clk_period)
				<< DSI_TME_LINE_CFG_HBP_TIME_SHIFT;
		val |= ROUND_UP((mode->left_margin + mode->right_margin +
				mode->hsync_len + mode->xres) * mode->pixclock
				/ NS2PS_RATIO / lane_byte_clk_period)
				<< DSI_TME_LINE_CFG_HLINE_TIME_SHIFT;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_TMR_LINE_CFG, val);

		val = ((mode->vsync_len & DSI_VTIMING_CFG_VSA_LINES_MASK)
					<< DSI_VTIMING_CFG_VSA_LINES_SHIFT);
		val |= ((mode->upper_margin & DSI_VTIMING_CFG_VBP_LINES_MASK)
				<< DSI_VTIMING_CFG_VBP_LINES_SHIFT);
		val |= ((mode->lower_margin & DSI_VTIMING_CFG_VFP_LINES_MASK)
				<< DSI_VTIMING_CFG_VFP_LINES_SHIFT);
		val |= ((mode->yres & DSI_VTIMING_CFG_V_ACT_LINES_MASK)
				<< DSI_VTIMING_CFG_V_ACT_LINES_SHIFT);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VTIMING_CFG, val);

		val = ((PHY_BTA_MAXTIME & DSI_PHY_TMR_CFG_BTA_TIME_MASK)
				<< DSI_PHY_TMR_CFG_BTA_TIME_SHIFT);
		val |= ((PHY_LP2HS_MAXTIME & DSI_PHY_TMR_CFG_LP2HS_TIME_MASK)
				<< DSI_PHY_TMR_CFG_LP2HS_TIME_SHIFT);
		val |= ((PHY_HS2LP_MAXTIME & DSI_PHY_TMR_CFG_HS2LP_TIME_MASK)
				<< DSI_PHY_TMR_CFG_HS2LP_TIME_SHIFT);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_TMR_CFG, val);

		val = (((lcd_config->data_lane_num - 1) &
			DSI_PHY_IF_CFG_N_LANES_MASK)
			<< DSI_PHY_IF_CFG_N_LANES_SHIFT);
		val |= ((PHY_STOP_WAIT_TIME & DSI_PHY_IF_CFG_WAIT_TIME_MASK)
				<< DSI_PHY_IF_CFG_WAIT_TIME_SHIFT);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CFG, val);

		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_ST0, &val);
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_ST1, &val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_ERROR_MSK0, 0);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_ERROR_MSK1, 0);

		mipi_dsi_dphy_init(mipi_dsi, DSI_PHY_CLK_INIT_COMMAND,
					mipi_dsi->dphy_pll_config);
	} else {
		mipi_dsi_dphy_init(mipi_dsi, DSI_PHY_CLK_INIT_COMMAND,
					mipi_dsi->dphy_pll_config);
	}
}

static void mipi_dsi_disable_controller(struct mipi_dsi_info *mipi_dsi)
{
	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CTRL,
			DSI_PHY_IF_CTRL_RESET);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP, DSI_PWRUP_RESET);
	mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_RSTZ, DSI_PHY_RSTZ_RST);
}

static irqreturn_t mipi_dsi_irq_handler(int irq, void *data)
{
	u32		mask0;
	u32		mask1;
	u32		status0;
	u32		status1;
	struct mipi_dsi_info *mipi_dsi;

	mipi_dsi = (struct mipi_dsi_info *)data;
	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_ST0,  &status0);
	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_ST1,  &status1);
	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_MSK0, &mask0);
	mipi_dsi_read_register(mipi_dsi, MIPI_DSI_ERROR_MSK1, &mask1);

	if ((status0 & (~mask0)) || (status1 & (~mask1))) {
		dev_err(&mipi_dsi->pdev->dev,
		"mipi_dsi IRQ status0:0x%x, status1:0x%x!\n",
		status0, status1);
	}

	return IRQ_HANDLED;
}

static inline void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
	bool cmd_mode)
{
	u32	val;

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	if (cmd_mode) {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_RESET);
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, &val);
		val |= MIPI_DSI_CMD_MODE_CFG_EN_CMD_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_MODE_CFG, 0);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_POWERUP);
	} else {
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_RESET);
		 /* Disable Command mode when transferring video data */
		mipi_dsi_read_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, &val);
		val &= ~MIPI_DSI_CMD_MODE_CFG_EN_CMD_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_CMD_MODE_CFG, val);
		val = DSI_VID_MODE_CFG_EN | DSI_VID_MODE_CFG_EN_BURSTMODE  |
				DSI_VID_MODE_CFG_EN_LP_MODE;
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_VID_MODE_CFG, val);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PWR_UP,
			DSI_PWRUP_POWERUP);
		mipi_dsi_write_register(mipi_dsi, MIPI_DSI_PHY_IF_CTRL,
				DSI_PHY_IF_CTRL_TX_REQ_CLK_HS);
	}
}

static int mipi_dsi_power_on(struct mxc_dispdrv_handle *disp)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	if (!mipi_dsi->dsi_power_on) {
		clk_prepare_enable(mipi_dsi->dphy_clk);
		clk_prepare_enable(mipi_dsi->cfg_clk);
		mipi_dsi_enable_controller(mipi_dsi, false);
		mipi_dsi_set_mode(mipi_dsi, false);
		/* host send pclk/hsync/vsync for two frames before sleep-out */
		msleep((1000/mipi_dsi->mode->refresh + 1) << 1);
		mipi_dsi_set_mode(mipi_dsi, true);
		err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE,
			NULL, 0);
		if (err) {
			dev_err(&mipi_dsi->pdev->dev,
				"MIPI DSI DCS Command sleep-in error!\n");
		}
		msleep(MIPI_LCD_SLEEP_MODE_DELAY);
		mipi_dsi_set_mode(mipi_dsi, false);
		mipi_dsi->dsi_power_on = 1;
	}

	return 0;
}

void mipi_dsi_power_off(struct mxc_dispdrv_handle *disp)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	if (mipi_dsi->dsi_power_on) {
		mipi_dsi_set_mode(mipi_dsi, true);
		err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_ENTER_SLEEP_MODE,
			NULL, 0);
		if (err) {
			dev_err(&mipi_dsi->pdev->dev,
				"MIPI DSI DCS Command display on error!\n");
		}
		/* To allow time for the supply voltages
		 * and clock circuits to stabilize.
		 */
		msleep(5);
		/* video stream timing on */
		mipi_dsi_set_mode(mipi_dsi, false);
		msleep(MIPI_LCD_SLEEP_MODE_DELAY);

		mipi_dsi_set_mode(mipi_dsi, true);
		mipi_dsi_disable_controller(mipi_dsi);
		mipi_dsi->dsi_power_on = 0;
		clk_disable_unprepare(mipi_dsi->dphy_clk);
		clk_disable_unprepare(mipi_dsi->cfg_clk);
	}
}

static int mipi_dsi_lcd_init(struct mipi_dsi_info *mipi_dsi,
	struct mxc_dispdrv_setting *setting)
{
	int		err;
	int		size;
	int		i;
	struct  fb_videomode *mipi_lcd_modedb;
	struct  fb_videomode mode;
	struct  device		 *dev = &mipi_dsi->pdev->dev;

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	for (i = 0; i < ARRAY_SIZE(mipi_dsi_lcd_db); i++) {
		if (!strcmp(mipi_dsi->lcd_panel,
			mipi_dsi_lcd_db[i].lcd_panel)) {
			mipi_dsi->lcd_callback =
				&mipi_dsi_lcd_db[i].lcd_callback;
			break;
		}
	}
	if (i == ARRAY_SIZE(mipi_dsi_lcd_db)) {
		dev_err(dev, "failed to find supported lcd panel.\n");
		return -EINVAL;
	}
	/* get the videomode in the order: cmdline->platform data->driver */
	mipi_dsi->lcd_callback->get_mipi_lcd_videomode(&mipi_lcd_modedb, &size,
					&mipi_dsi->lcd_config);
	err = fb_find_mode(&setting->fbi->var, setting->fbi,
				setting->dft_mode_str,
				mipi_lcd_modedb, size, NULL,
				setting->default_bpp);
	if (err != 1)
		fb_videomode_to_var(&setting->fbi->var, mipi_lcd_modedb);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < size; i++) {
		fb_var_to_videomode(&mode, &setting->fbi->var);
		if (fb_mode_is_equal(&mode, mipi_lcd_modedb + i)) {
			err = fb_add_videomode(mipi_lcd_modedb + i,
					&setting->fbi->modelist);
			 /* Note: only support fb mode from driver */
			mipi_dsi->mode = mipi_lcd_modedb + i;
			break;
		}
	}
	if ((err < 0) || (size == i)) {
		dev_err(dev, "failed to add videomode.\n");
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(mipi_dsi_phy_pll_clk_table); i++) {
		if (mipi_dsi_phy_pll_clk_table[i].max_phy_clk <
				mipi_dsi->lcd_config->max_phy_clk)
			break;
	}
	if ((i == ARRAY_SIZE(mipi_dsi_phy_pll_clk_table)) ||
		(mipi_dsi->lcd_config->max_phy_clk >
			mipi_dsi_phy_pll_clk_table[0].max_phy_clk)) {
		dev_err(dev, "failed to find data in"
				"mipi_dsi_phy_pll_clk_table.\n");
		return -EINVAL;
	}
	mipi_dsi->dphy_pll_config = mipi_dsi_phy_pll_clk_table[--i].config;
	dev_dbg(dev, "dphy_pll_config:0x%x.\n", mipi_dsi->dphy_pll_config);

	return 0;
}

static int mipi_dsi_enable(struct mxc_dispdrv_handle *disp,
			   struct fb_info *fbi)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	if (!mipi_dsi->lcd_inited) {
		err = clk_prepare_enable(mipi_dsi->dphy_clk);
		err |= clk_prepare_enable(mipi_dsi->cfg_clk);
		if (err)
			dev_err(&mipi_dsi->pdev->dev,
				"clk enable error:%d!\n", err);
		mipi_dsi_enable_controller(mipi_dsi, true);
		err = mipi_dsi->lcd_callback->mipi_lcd_setup(
			mipi_dsi);
		if (err < 0) {
			dev_err(&mipi_dsi->pdev->dev,
				"failed to init mipi lcd.");
			clk_disable_unprepare(mipi_dsi->dphy_clk);
			clk_disable_unprepare(mipi_dsi->cfg_clk);
			return err;
		}
		mipi_dsi_set_mode(mipi_dsi, false);
		mipi_dsi->dsi_power_on = 1;
		mipi_dsi->lcd_inited = 1;
	}
	mipi_dsi_power_on(mipi_dsi->disp_mipi);

	return 0;
}

static void mipi_dsi_disable(struct mxc_dispdrv_handle *disp,
			    struct fb_info *fbi)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	mipi_dsi_power_off(mipi_dsi->disp_mipi);
}

static int mipi_dsi_disp_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);
	struct device *dev = &mipi_dsi->pdev->dev;
	int ret = 0;

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	if (!valid_mode(setting->if_fmt)) {
		dev_warn(dev, "Input pixel format not valid"
			"use default RGB24\n");
		setting->if_fmt = IPU_PIX_FMT_RGB24;
	}

	ret = ipu_di_to_crtc(dev, mipi_dsi->dev_id,
			     mipi_dsi->disp_id, &setting->crtc);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_lcd_init(mipi_dsi, setting);
	if (ret) {
		dev_err(dev, "failed to init mipi dsi lcd\n");
		return ret;
	}

	dev_dbg(dev, "MIPI DSI dispdrv inited!\n");
	return ret;
}

static void mipi_dsi_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct mipi_dsi_info    *mipi_dsi;
	mipi_dsi = mxc_dispdrv_getdata(disp);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	mipi_dsi_power_off(mipi_dsi->disp_mipi);
	if (mipi_dsi->bl)
		backlight_device_unregister(mipi_dsi->bl);
}

static int mipi_dsi_setup(struct mxc_dispdrv_handle *disp,
			  struct fb_info *fbi)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	int xres_virtual = fbi->var.xres_virtual;
	int yres_virtual = fbi->var.yres_virtual;
	int xoffset = fbi->var.xoffset;
	int yoffset = fbi->var.yoffset;
	int pixclock = fbi->var.pixclock;

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	dump_stack();

	if (!mipi_dsi->mode)
		return 0;

	/* set the mode back to var in case userspace changes it */
	fb_videomode_to_var(&fbi->var, mipi_dsi->mode);

	/* restore some var entries cached */
	fbi->var.xres_virtual = xres_virtual;
	fbi->var.yres_virtual = yres_virtual;
	fbi->var.xoffset = xoffset;
	fbi->var.yoffset = yoffset;
	fbi->var.pixclock = pixclock;
	return 0;
}

static struct mxc_dispdrv_driver mipi_dsi_drv = {
	.name	= DISPDRV_MIPI,
	.init	= mipi_dsi_disp_init,
	.deinit	= mipi_dsi_disp_deinit,
	.enable	= mipi_dsi_enable,
	.disable = mipi_dsi_disable,
	.setup	= mipi_dsi_setup,
};

static int imx6q_mipi_dsi_get_mux(int dev_id, int disp_id)
{
	if (dev_id > 1 || disp_id > 1)
		return -EINVAL;

	return (dev_id << 5) | (disp_id << 4);
}

static struct mipi_dsi_bus_mux imx6q_mipi_dsi_mux[] = {
	{
		.reg = IOMUXC_GPR3,
		.mask = IMX6Q_GPR3_MIPI_MUX_CTL_MASK,
		.get_mux = imx6q_mipi_dsi_get_mux,
	},
};

static int imx6dl_mipi_dsi_get_mux(int dev_id, int disp_id)
{
	if (dev_id > 1 || disp_id > 1)
		return -EINVAL;

	/* MIPI DSI source is LCDIF */
	if (dev_id)
		disp_id = 0;

	return (dev_id << 5) | (disp_id << 4);
}

static struct mipi_dsi_bus_mux imx6dl_mipi_dsi_mux[] = {
	{
		.reg = IOMUXC_GPR3,
		.mask = IMX6Q_GPR3_MIPI_MUX_CTL_MASK,
		.get_mux = imx6dl_mipi_dsi_get_mux,
	},
};

static const struct of_device_id imx_mipi_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx6q-mipi-dsi", .data = imx6q_mipi_dsi_mux, },
	{ .compatible = "fsl,imx6dl-mipi-dsi", .data = imx6dl_mipi_dsi_mux, },
	{ }
};
MODULE_DEVICE_TABLE(of, imx_mipi_dsi_dt_ids);

/**
 * This function is called by the driver framework to initialize the MIPI DSI
 * device.
 *
 * @param	pdev	The device structure for the MIPI DSI passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int mipi_dsi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(of_match_ptr(imx_mipi_dsi_dt_ids),
					&pdev->dev);

	struct mipi_dsi_info *mipi_dsi;
	struct resource *res;
	u32 dev_id, disp_id;
	const char *lcd_panel;
	int mux;
	int ret = 0;

	dev_dbg(&pdev->dev, "%s\n", __FUNCTION__);

	mipi_dsi = devm_kzalloc(&pdev->dev, sizeof(*mipi_dsi), GFP_KERNEL);
	if (!mipi_dsi)
		return -ENOMEM;

	ret = of_property_read_string(np, "lcd_panel", &lcd_panel);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property lcd_panel\n");
		return ret;
	}

	ret = of_property_read_u32(np, "dev_id", &dev_id);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property dev_id\n");
		return ret;
	}
	ret = of_property_read_u32(np, "disp_id", &disp_id);
	if (ret) {
		dev_err(&pdev->dev, "failed to read of property disp_id\n");
		return ret;
	}
	mipi_dsi->dev_id = dev_id;
	mipi_dsi->disp_id = disp_id;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform resource 0\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start,
				resource_size(res), pdev->name))
		return -EBUSY;

	mipi_dsi->mmio_base = devm_ioremap(&pdev->dev, res->start,
				   resource_size(res));
	if (!mipi_dsi->mmio_base)
		return -EBUSY;

	mipi_dsi->irq = platform_get_irq(pdev, 0);
	if (mipi_dsi->irq < 0) {
		dev_err(&pdev->dev, "failed get device irq\n");
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, mipi_dsi->irq,
				mipi_dsi_irq_handler,
				0, "mipi_dsi", mipi_dsi);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return ret;
	}

	mipi_dsi->dphy_clk = devm_clk_get(&pdev->dev, "mipi_pllref_clk");
	if (IS_ERR(mipi_dsi->dphy_clk)) {
		dev_err(&pdev->dev, "failed to get dphy pll_ref_clk\n");
		return PTR_ERR(mipi_dsi->dphy_clk);
	}

	mipi_dsi->cfg_clk = devm_clk_get(&pdev->dev, "mipi_cfg_clk");
	if (IS_ERR(mipi_dsi->cfg_clk)) {
		dev_err(&pdev->dev, "failed to get cfg_clk\n");
		return PTR_ERR(mipi_dsi->cfg_clk);
	}

	mipi_dsi->disp_power_on = devm_regulator_get(&pdev->dev,
							"disp-power-on");
	if (!IS_ERR(mipi_dsi->disp_power_on)) {
		ret = regulator_enable(mipi_dsi->disp_power_on);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable display "
				"power regulator, err=%d\n", ret);
			return ret;
		}
	} else {
		mipi_dsi->disp_power_on = NULL;
	}

	ret = device_reset(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to reset: %d\n", ret);
		goto dev_reset_fail;
	}

	if (of_id)
		mipi_dsi->bus_mux = of_id->data;

	mipi_dsi->regmap = syscon_regmap_lookup_by_phandle(np, "gpr");
	if (IS_ERR(mipi_dsi->regmap)) {
		dev_err(&pdev->dev, "failed to get parent regmap\n");
		ret = PTR_ERR(mipi_dsi->regmap);
		goto get_parent_regmap_fail;
	}

	mux = mipi_dsi->bus_mux->get_mux(dev_id, disp_id);
	if (mux >= 0)
		regmap_update_bits(mipi_dsi->regmap, mipi_dsi->bus_mux->reg,
				   mipi_dsi->bus_mux->mask, (unsigned int)mux);
	else
		dev_warn(&pdev->dev, "invalid dev_id or disp_id muxing\n");

	mipi_dsi->lcd_panel = kstrdup(lcd_panel, GFP_KERNEL);
	if (!mipi_dsi->lcd_panel) {
		dev_err(&pdev->dev, "failed to allocate lcd panel name\n");
		ret = -ENOMEM;
		goto kstrdup_fail;
	}

	mipi_dsi->pdev = pdev;
	mipi_dsi->disp_mipi = mxc_dispdrv_register(&mipi_dsi_drv);
	if (IS_ERR(mipi_dsi->disp_mipi)) {
		dev_err(&pdev->dev, "mxc_dispdrv_register error\n");
		ret = PTR_ERR(mipi_dsi->disp_mipi);
		goto dispdrv_reg_fail;
	}

	mipi_dsi->mipi_dsi_pkt_read  = mipi_dsi_pkt_read;
	mipi_dsi->mipi_dsi_pkt_write = mipi_dsi_pkt_write;
	mipi_dsi->mipi_dsi_dcs_cmd   = mipi_dsi_dcs_cmd;

	mxc_dispdrv_setdata(mipi_dsi->disp_mipi, mipi_dsi);
	dev_set_drvdata(&pdev->dev, mipi_dsi);

	create_debugfs_entries(mipi_dsi);

	dev_info(&pdev->dev, "i.MX MIPI DSI driver probed\n");
	return ret;

dispdrv_reg_fail:
	kfree(mipi_dsi->lcd_panel);
kstrdup_fail:
get_parent_regmap_fail:
dev_reset_fail:
	if (mipi_dsi->disp_power_on)
		regulator_disable(mipi_dsi->disp_power_on);
	return ret;
}

static void mipi_dsi_shutdown(struct platform_device *pdev)
{
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	mipi_dsi_power_off(mipi_dsi->disp_mipi);
}

static int mipi_dsi_remove(struct platform_device *pdev)
{
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	dev_dbg(&mipi_dsi->pdev->dev, "%s\n", __FUNCTION__);

	mxc_dispdrv_puthandle(mipi_dsi->disp_mipi);
	mxc_dispdrv_unregister(mipi_dsi->disp_mipi);

	if (mipi_dsi->disp_power_on)
		regulator_disable(mipi_dsi->disp_power_on);

	kfree(mipi_dsi->lcd_panel);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static struct platform_driver mipi_dsi_driver = {
	.driver = {
		   .of_match_table = imx_mipi_dsi_dt_ids,
		   .name = "mxc_mipi_dsi",
	},
	.probe = mipi_dsi_probe,
	.remove = mipi_dsi_remove,
	.shutdown = mipi_dsi_shutdown,
};

static int __init mipi_dsi_init(void)
{
	int err;

	err = platform_driver_register(&mipi_dsi_driver);
	if (err) {
		pr_err("mipi_dsi_driver register failed\n");
		return -ENODEV;
	}
	pr_info("MIPI DSI driver module loaded\n");
	return 0;
}

static void __exit mipi_dsi_cleanup(void)
{
	platform_driver_unregister(&mipi_dsi_driver);
}

module_init(mipi_dsi_init);
module_exit(mipi_dsi_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX MIPI DSI driver");
MODULE_LICENSE("GPL");
