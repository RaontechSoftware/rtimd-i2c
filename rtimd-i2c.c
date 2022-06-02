
/*
 * File name: rtimd-i2c.c
 *
 * Description : RAONTECH Micro Display I2C driver.
 *
 * Copyright (C) (2017, RAONTECH)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "rtimd-i2c.h"
#include "rtimd-i2c_internal.h"
#include "rtimd_dev_config.h"

/* Define when boot-done is checked using BUSY GPIO pin. */
#define _BOOT_CHECK_USING_BUSY_GPIO_PIN


#ifndef UPPERCASE
#define UPPERCASE(c) (((c) >= 'a' && (c) <= 'z') ? ((c) - 0x20) : (c))
#endif

#define SYSFS_BURST_DATA_BUF_SIZE		1024

#define SYSFS_BWR_DATA_OFFSET		11
#define SYSFS_BRD_WDATA_OFFSET		16

static struct platform_device *rtimd_device;
static struct class *rtimd_class;
static int rtimd_major;

RTIMD_CB_T *rtimd_cb;

static RTIMD_SINGLE_READ_REG_T srd_param;

static RTIMD_BURST_READ_REG_T brd_param;
static uint8_t sysfs_brd_wdata[SYSFS_BURST_DATA_BUF_SIZE];
static uint8_t sysfs_brd_rdata[SYSFS_BURST_DATA_BUF_SIZE];

static uint8_t sysfs_bwr_data[SYSFS_BURST_DATA_BUF_SIZE];

/* Forward functions */
static int rtimd_probe(struct platform_device *pdev);
static int rtimd_remove(struct platform_device *pdev);

typedef struct {
	U16_T h_size;
	U16_T v_size;	
} RDC200A_RESOLUTION_INFO_T;

static const RDC200A_RESOLUTION_INFO_T rdc200a_input_res[] = {
	{ 640 ,  480},// 0x00 : VGA          ( 640 x  480), 
	{ 854 ,  480},// 0x01 : WVGA         ( 854 x  480), 
	{ 960 ,  540},// 0x02 : QFHD         ( 960 x  540), 
	{1024 ,  600},// 0x03 : WSVGA        (1024 x  600), 
	{1024 ,  768},// 0x04 : XGA          (1024 x  768), 
	{1280 ,  720},// 0x05 : HD           (1280 x  720), 
	{1280 ,  768},// 0x06 : WXGA_S       (1280 x  768), 
	{1280 ,  800},// 0x07 : WXGA         (1280 x  800), 
	{1024 , 1024},// 0x08 : 1K1K         (1024 x 1024), 
	{1080 , 1080},// 0x09 : 1080R        (1080 x 1080), 
	{1280 ,  960},// 0x0a : MLO          (1280 x  960), 
	{1280 , 1024},// 0x0b : SXGA         (1280 x 1024), 
	{1600 ,  900},// 0x0c : 1600X900     (1600 x  900), 
	{1440 , 1080},// 0x0d : 1440X1080    (1440 x 1080),
	{1280 , 1280},// 0x0e : 1280R        (1280 x 1280), 
	{2560 ,  720},// 0x0f : DUALHD       (2560 x  720), 
	{1600 , 1200},// 0x10 : UXGA_S       (1600 x 1200), 
	{2560 ,  768},// 0x11 : DUALWXGA_S   (2560 x  768), 
	{1600 , 1280},// 0x12 : UXGA         (1600 x 1280), 
	{2560 ,  800},// 0x13 : DUALWXGA     (2560 x  800), 
	{1440 , 1440},// 0x14 : 1440R        (1440 x 1440), 
	{1920 , 1080},// 0x15 : FHD          (1920 x 1080), 
	{2048 , 1024},// 0x16 : 2K1K         (2048 x 1024), 
	{2560 , 1440},// 0x17 : WQHD         (2560 x 1440), 
	{2560 , 1600},// 0x18 : WQXGA        (2560 x 1600), 
	{1080 , 3840},// 0x19 : RFU0         (1080 x 3840), 
	{2048 , 2048},// 0x1a : 2K2K         (2048 x 2048), 
	{1280 ,  728},// 0x1b : 1280X728     (1280 x  728), 
	{1024 , 4096},// 0x1c : RFU2         (1024 x 4096), 
	{1024 , 4096},// 0x1d : RFU3         (1024 x 4096), 
	{1024 , 4096},// 0x1e : RFU4         (1024 x 4096), 
	{1024 , 4096},// 0x1f : RFU5         (1024 x 4096), 
	{ 480 ,  640},// 0x20 : PVGA         ( 480 x  640)
	{ 480 ,  854},// 0x21 : PWVGA        ( 480 x  854)
	{ 540 ,  960},// 0x22 : PQFHD        ( 540 x  960)
	{ 600 , 1024},// 0x23 : PWSVGA       ( 600 x 1024)
	{ 768 , 1024},// 0x24 : PXGA         ( 768 x 1024)
	{ 720 , 1280},// 0x25 : PHD          ( 720 x 1280)
	{ 768 , 1280},// 0x26 : PWXGA_S      ( 768 x 1280)
	{ 800 , 1280},// 0x27 : PWXGA        ( 800 x 1280)
	{1024 , 1024},// 0x28 : RFU6         (1024 x 1024)
	{1080 , 1080},// 0x29 : RFU7         (1080 x 1080)
	{ 960 , 1280},// 0x2a : PMLO         ( 960 x 1280)
	{1024 , 1280},// 0x2b : PSXGA        (1024 x 1280)
	{ 900 , 1600},// 0x2c : 900X1600    ( 900 x 1600)
	{1080 , 1440},// 0x2d : 1080X1440   (1080 x 1440)
	{1280 , 1280},// 0x2e : RFU8         (1280 x 1280)
	{ 720 , 2560},// 0x2f : PDUALHD      ( 720 x 2560)
	{1200 , 1600},// 0x30 : PUXGA_S      (1200 x 1600)
	{ 768 , 2560},// 0x31 : PDUALWXGA_S  ( 768 x 2560)
	{1200 , 1600},// 0x32 : PUXGA        (1200 x 1600)
	{ 800 , 2560},// 0x33 : PDUALWXGA    ( 800 x 2560)
	{1440 , 1440},// 0x34 : RFU9         (1440 x 1440)
	{1080 , 1920},// 0x35 : PFHD         (1080 x 1920)
	{1024 , 2048},// 0x36 : P2K1K        (1024 x 2048)
	{1440 , 2560},// 0x37 : PWQHD        (1440 x 2560)
	{1600 , 2560},// 0x38 : PWQXGA       (1600 x 2560)
	{1080 , 3840},// 0x39 : PDUALFHD     (1080 x 3840)
	{2048 , 2048},// 0x3a : RFUA         (2048 x 2048)
	{1024 , 4096},// 0x3b : RFUB         (1024 x 4096)
	{1024 , 4096},// 0x3c : RFUC         (1024 x 4096)
	{1024 , 4096},// 0x3d : RFUD         (1024 x 4096)
	{1024 , 4096},// 0x3e : RFUE         (1024 x 4096)
	{0, 0}        // 0x3f : ETC (TX_INPUT_H_SIZE_IDX, TX_INPUT_V_SIZE_IDX)
};

#ifdef CONFIG_COMPAT
static void __user *u64_to_uptr(u64 value)
{
	if (in_compat_syscall())
		return compat_ptr(value);
	else
		return (void __user *)(unsigned long)value;
}

static u64 uptr_to_u64(void __user *ptr)
{
	if (in_compat_syscall())
		return ptr_to_compat(ptr);
	else
		return (u64)(unsigned long)ptr;
}
#else
static inline void __user *u64_to_uptr(u64 value)
{
	return (void __user *)(unsigned long)value;
}

static inline u64 uptr_to_u64(void __user *ptr)
{
	return (u64)(unsigned long)ptr;
}
#endif /* CONFIG_COMPAT */

static int rtimd_get_i2c_adapter(uint8_t slave_addr, int bus_num)
{
	int ret = 0;

	if (bus_num != rtimd_cb->bus_num) {
		//RMDDBG("Bus number (%d => %d) prev adap(0x%p)\n", rtimd_cb->bus_num, bus_num, rtimd_cb->adap);
	
		/* Close the previous bus if opened. */
		if (rtimd_cb->adap)
			i2c_put_adapter(rtimd_cb->adap);

		rtimd_cb->adap = i2c_get_adapter(bus_num);
		if (rtimd_cb->adap)
			rtimd_cb->bus_num = bus_num; /* Set new bus number */
		else {
			rtimd_cb->bus_num = -1;
			RMDERR("I2C device not found.\n");
			ret =  -ENODEV;
		}
	}

	return ret;
}

static int rtimd_i2c_burst_read(RTIMD_BURST_READ_REG_T *br, uint8_t *wbuf,
						uint8_t *rbuf)
{
	int ret = 0;
	struct i2c_msg msgs[2] = {
		{
			.addr = br->slave_addr,
			.flags = 0,
			.buf = wbuf,
			.len = br->wsize
		},
		{
			.addr = br->slave_addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = br->rsize
		}
	};

	//RMDDBG("bus(%d) slave_addr(0x%02X) msgs[0].buf[0](0x%02X)\n",
	//		br->bus_num, br->slave_addr, msgs[0].buf[0]);

	ret = i2c_transfer(rtimd_cb->adap, msgs, 2);

	/* If everything went ok, return #bytes transmitted, else error code. */	
	if (ret == 2)
		ret = br->rsize;

	return ret;
}

static int rtimd_i2c_burst_write(RTIMD_BURST_WRITE_REG_T *bw, uint8_t *wbuf)
{
	int ret = 0;
	struct i2c_msg msgs = {
		.addr = bw->slave_addr,
		.flags = 0,
		.buf = wbuf,
		.len = bw->wsize
	};

	ret = i2c_transfer(rtimd_cb->adap, &msgs, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */	
	if (ret == 1)
		ret = msgs.len;

	return ret;
}

static int rtimd_i2c_single_write(RTIMD_SINGLE_WRITE_REG_T *sw)
{
	uint8_t wbuf[3]; /* max reg size is 2. max data size is 1 */
	int ret = 0;
	struct i2c_msg msgs = {
		.addr = sw->slave_addr,
		.flags = 0,
		.buf = wbuf,
		.len = sw->reg_size + 1/*data*/
	};

	switch (sw->reg_size) {
	case 1:
		wbuf[0] = sw->reg_addr & 0xFF;
		wbuf[1] = sw->data;
		break;
	
	case 2:
		wbuf[0] = sw->reg_addr >> 8;
		wbuf[1] = sw->reg_addr & 0xFF;
		wbuf[2] = sw->data;
		break;
	
	default:
		RMDERR("Invalid register size\n");
		return -EINVAL;
	}

	//RMDDBG("sw: bus_num(%d) saddr(0x%02X) regaddr(0x%04X) data(0x%02X)\n",
	//	sw->bus_num, sw->slave_addr, sw->reg_addr, sw->data);

	ret = i2c_transfer(rtimd_cb->adap, &msgs, 1);

	/*
	 * If everything went ok (i.e. 1 msg transmitted), return #bytes
	 * transmitted, else error code.
	 */
	if (ret == 1)
		ret = msgs.len;

	return ret;
}

static int rtimd_i2c_single_read(RTIMD_SINGLE_READ_REG_T *sr, uint8_t *rbuf)
{
	uint8_t wbuf[2]; /* max reg size is 2. */
	int ret = 0;
	struct i2c_msg msgs[2] = {
		{
			.addr = sr->slave_addr,
			.flags = 0,
			.buf = wbuf,
			.len = sr->reg_size
		},
		{
			.addr = sr->slave_addr,
			.flags = I2C_M_RD,
			.buf = rbuf,
			.len = 1
		}
	};

	switch (sr->reg_size) {
	case 1:
		wbuf[0] = sr->reg_addr & 0xFF;
		break;
	
	case 2:
		wbuf[0] = sr->reg_addr >> 8;
		wbuf[1] = sr->reg_addr & 0xFF;
		break;
	
	default:
		RMDERR("Invalid register size\n");
		return -EINVAL;
	}

	//RMDDBG("adap(0x%p) bus_num(%d) addr(0x%02X) reg_addr(0x%04X) reg_size(%u) wbuf[0](0x%02X)\n",
	//	rtimd_cb->adap, sr->bus_num, msgs[0].addr, sr->reg_addr, sr->reg_size, wbuf[0]);

	ret = i2c_transfer(rtimd_cb->adap, msgs, 2);

	if (ret == 2) /* If everything went ok, return #bytes transmitted, else error code. */
		ret = msgs[1].len;
	else {		
		RMDERR("i2c(%d) 0x%0X read failed! ret(%d)\n", rtimd_cb->bus_num, sr->reg_addr, ret);
	}

	return ret;	
}

static int RDC_REG_SET(uint reg, uint val)
{
	int ret;
	RTIMD_SINGLE_WRITE_REG_T sw;
	RTIMD_RDC_CFG_INFO_T *rdccfg
		= &rtimd_rdc_cfg_table[rtimd_cb->selected_rdc_idx];

	sw.reg_addr = reg;
	sw.bus_num = rdccfg->i2c_bus_num;
	sw.slave_addr = rdccfg->i2c_addr;
	sw.data = val;
#if defined(CFG_MDC_RDC200)
	sw.reg_size = 1;
#elif defined(CFG_MDC_RDC200A)
	sw.reg_size = 2;
#else
	#error "Code not present"
#endif
	
	ret = rtimd_i2c_single_write(&sw);

	return ret;
}

static int RDC_REG_GET(uint reg)
{
	int ret;
	RTIMD_SINGLE_READ_REG_T sr;
	uint8_t rbuf[1];
	RTIMD_RDC_CFG_INFO_T *rdccfg
		= &rtimd_rdc_cfg_table[rtimd_cb->selected_rdc_idx];

	sr.reg_addr = reg;
	sr.bus_num = rdccfg->i2c_bus_num;
	sr.slave_addr = rdccfg->i2c_addr;
#if defined(CFG_MDC_RDC200)
	sr.reg_size = 1;
#elif defined(CFG_MDC_RDC200A)
	sr.reg_size = 2;
#else
	#error "Code not present"
#endif
	
	ret = rtimd_i2c_single_read(&sr, rbuf);
	if (ret > 0)
		return rbuf[0];
	else			
		return 0xFF;
}

static int rdc200a_drive_panel(BOOL_T on)
{
	U8_T rd0015;

	rd0015 = RDC_REG_GET(0x0015);

	if (on == TRUE)
		rd0015 |= ((rd0015 & 0x03) << 2);
	else
		rd0015 &= 0xF3;

	RDC_REG_SET(0x0015, rd0015);

	return 0;
}

static void rdc200a_dcdc_disable(void)
{
	U8_T reg_0x0019;
	
	reg_0x0019 = RDC_REG_GET(0x0019);
	if (reg_0x0019 & 0x02)
		reg_0x0019 |= 0x01; // Low active power down
	else
		reg_0x0019 &= ~0x01; // High active power down

	RDC_REG_SET(0x0019, reg_0x0019); // dcdc disable
		
	mdelay(1);
}

static void rdc200a_dcdc_enable(void)
{
	U8_T reg_0x0019;

	reg_0x0019 = RDC_REG_GET(0x0019);
	if (reg_0x0019 & 0x02)
		reg_0x0019 &= ~0x01; // Low active power down
	else
		reg_0x0019 |= 0x01; // High active power down

	/*
	 <0> : DCDC power down manual control
   		'0' : DCDC active ( GPIO outputs low )
		'1' : DCDC power down ( GPIO outputs high ) <default>
	*/
	RDC_REG_SET(0x0019, reg_0x0019); // dcdc disable	
	
	mdelay(1);
}

static int rdc200a_poweron_panel(BOOL_T on)
{
	if (on) {
		rdc200a_drive_panel(TRUE);
		rdc200a_dcdc_enable();
	}
	else {
		rdc200a_dcdc_disable();
		rdc200a_drive_panel(FALSE);
	}

	return 0;
}

static void rdc200a_reset_mipi_interface(void)
{
	/* MIPI Power Down */
	RDC_REG_SET(0x0010, 0x01);
	RDC_REG_SET(0x0241, 0x00);
	RDC_REG_SET(0x0011, 0xC0);

	/* MIPI Power ON */
	RDC_REG_SET(0x0010, 0x00);

	/* MIPI RESET */
	RDC_REG_SET(0x0012, 0x10);
	RDC_REG_SET(0x0013, 0x7F);
	RDC_REG_SET(0x0241, 0x10);
	mdelay(1);

	RDC_REG_SET(0x0012, 0x11);

	RDC_REG_SET(0x0012, 0x1F);
	RDC_REG_SET(0x0013, 0xFF);
}

static int rdc200a_get_video_coord_info(RTIMD_VIDEO_COORD_INFO_T *vcd)
{
    vcd->hsw = (uint16_t)RDC_REG_GET(0x0293);
    vcd->hbp = (uint16_t)RDC_REG_GET(0x0294);
    vcd->hfp = (uint16_t)RDC_REG_GET(0x0292);
    vcd->ha  = (((uint16_t)RDC_REG_GET(0x0295) << 8)
				| (uint16_t)RDC_REG_GET(0x0296));
    
    vcd->vsw = (uint16_t)RDC_REG_GET(0x0299);
    vcd->vbp = (uint16_t)RDC_REG_GET(0x029A);
    vcd->vfp = (((uint16_t)RDC_REG_GET(0x0297) << 8)
				| (uint16_t)RDC_REG_GET(0x0298));

    vcd->va =  (((uint16_t)RDC_REG_GET(0x029B) << 8)
				| (uint16_t)RDC_REG_GET(0x029C));

#if 1
    RMDDBG("HSW = %d HBP = %d HFP = %d HACTIVE = %d \n", vcd->hsw, vcd->hbp,  vcd->hfp, vcd->ha);
    RMDDBG("VSW = %d VBP = %d VFP = %d VACTIVE = %d \n", vcd->vsw, vcd->vbp,  vcd->vfp, vcd->va);
#endif

	return 0;
}

static int rdc200a_check_video_coord(void)
{
	uint8_t input_size_idx;
	uint h_input_size, v_input_size;
	RTIMD_VIDEO_COORD_INFO_T vcd;
	int ret = 0;
	uint success_cnt = 0;
	uint loop = 20;

	input_size_idx = RDC_REG_GET(0x0201) >> 2;
	h_input_size = rdc200a_input_res[input_size_idx].h_size;
	v_input_size = rdc200a_input_res[input_size_idx].v_size;

	do {
		rdc200a_get_video_coord_info(&vcd);

		if((h_input_size == vcd.ha) && (v_input_size == vcd.va)) {
			ret = 0;
			if(++success_cnt > 2)
				break;
		}
		else
			ret = -1;

		mdelay(5);
	} while (loop--);

	return ret;
}

static int rdc200a_check_boot_done(unsigned gpio)
{
	int i, ret = 0;
	int flag_boot_done;

	for (i = 0; i <= 100; i++) {
		flag_boot_done = gpio_get_value(gpio);
		if (flag_boot_done == 0)
			break;

		mdelay(10);
		RMDDBG("Boot Wait Count = %d  Time  = %d \n", i, i * 10);
	}

	if (i >= 100)
		ret = -1; //Power On fail
    
    return ret;
}

int rtimd_control_display_on(E_VC_CTRL_DISP_CH_T ctrl_dch, bool on)
{
	int ret = 0;
	UINT_T saved_rdc_idx, i = 0;
	const RTIMD_DISPLAY_CH_INFO_T *dch;
	UINT_T chk_dch = (UINT_T)ctrl_dch;

	mutex_lock(&rtimd_cb->access_lock);

	/* Save rdc idx to restore. */
	saved_rdc_idx = rtimd_cb->selected_rdc_idx;

	do {
		if (chk_dch & 0x1) {
			dch = &rtimd_dev_info_table.chan[i];
			if (dch->activated) {
				rtimd_cb->selected_rdc_idx = dch->rdc_idx;

				ret = rtimd_get_i2c_adapter(dch->rdc_i2c_addr,
											dch->rdc_i2c_bus_num);
				if (ret)
					break;

				if (on) {
					ret = rdc200a_check_video_coord();
					if (ret) {
						RMDERR("DCH %u: Failed to display on (H,V is NOT target)\n", i);
						break;
					}
				}

				rdc200a_poweron_panel(on);

#if (_CFG_NUM_ATTACHED_RDC == 1)
				/* To avoid I2C communication twice for 1 RDC. */
				break;
#endif
			}
			else
				RMDERR("DCH %u: Inactivated channel\n", i);
		}

		i++;
		chk_dch >>= 1;
	} while (chk_dch);

	rtimd_cb->selected_rdc_idx = saved_rdc_idx;

	mutex_unlock(&rtimd_cb->access_lock);

	return ret;
}
EXPORT_SYMBOL(rtimd_control_display_on);

int rtimd_reset_mipi_interface(E_VC_CTRL_DISP_CH_T ctrl_dch)
{
	int ret = 0;
	UINT_T saved_rdc_idx, i = 0;
	const RTIMD_DISPLAY_CH_INFO_T *dch;
	UINT_T chk_dch = (UINT_T)ctrl_dch;

	mutex_lock(&rtimd_cb->access_lock);

	/* Save rdc idx to restore. */
	saved_rdc_idx = rtimd_cb->selected_rdc_idx;

	do {
		if (chk_dch & 0x1) {
			dch = &rtimd_dev_info_table.chan[i];
			if (dch->activated) {
				rtimd_cb->selected_rdc_idx = dch->rdc_idx;

				//RMDDBG("rdc_idx(%u) rdc_i2c_bus_num(%d)\n", dch->rdc_idx, dch->rdc_i2c_bus_num);

				ret = rtimd_get_i2c_adapter(dch->rdc_i2c_addr,
											dch->rdc_i2c_bus_num);
				if (ret)
					break;

				rdc200a_reset_mipi_interface();

#if (_CFG_NUM_ATTACHED_RDC == 1)
				/* To avoid I2C communication twice for 1 RDC. */
				break;
#endif
			}
			else
				RMDERR("DCH %u: Inactivated channel\n", i);
		}

		i++;
		chk_dch >>= 1;
	} while (chk_dch);

	rtimd_cb->selected_rdc_idx = saved_rdc_idx;

	mutex_unlock(&rtimd_cb->access_lock);

	return ret;
}
EXPORT_SYMBOL(rtimd_reset_mipi_interface);

int rtimd_check_firmware_boot_done(E_VC_CTRL_DISP_CH_T ctrl_dch,
								unsigned gpio[MAX_NUM_VC_DCH])
{
	int ret = 0;

#ifdef _BOOT_CHECK_USING_BUSY_GPIO_PIN
	UINT_T saved_rdc_idx, i = 0;
	const RTIMD_DISPLAY_CH_INFO_T *dch;
	UINT_T chk_dch = (UINT_T)ctrl_dch;

	mutex_lock(&rtimd_cb->access_lock);

	/* Save rdc idx to restore. */
	saved_rdc_idx = rtimd_cb->selected_rdc_idx;

	do {
		if (chk_dch & 0x1) {
			dch = &rtimd_dev_info_table.chan[i];
			if (dch->activated) {
				rtimd_cb->selected_rdc_idx = dch->rdc_idx;

				ret = rdc200a_check_boot_done(gpio[i]);
				if (ret) {
					RMDERR("DCH %u: Failed to power on\n", i);
					break;
				}

#if (_CFG_NUM_ATTACHED_RDC == 1)
				/* To avoid I2C communication twice for 1 RDC. */
				break;
#endif
			}
			else
				RMDERR("DCH %u: Inactivated channel\n", i);
		}

		i++;
		chk_dch >>= 1;
	} while (chk_dch);

	rtimd_cb->selected_rdc_idx = saved_rdc_idx;

	mutex_unlock(&rtimd_cb->access_lock);
#else

	mdelay(400);
#endif

	return ret;
}
EXPORT_SYMBOL(rtimd_check_firmware_boot_done);

/* NOTE: The gpio numbers are pseudo-number for use in the examples. */
#define GPIO_RDC200A_LDO_1_2V    14
#define GPIO_RDC200A_LDO_1_8V    18
#define GPIO_RDC200A_LDO_3_3V    19
#define GPIO_RDC200A_RESETB      20
#define GPIO_BOOT_DONE_FROM_RDC200A_GPIOB3_0	21 // RDC 0
#define GPIO_BOOT_DONE_FROM_RDC200A_GPIOB3_1	22 // RDC 0 for 1 RDC, RDC 1 for 2 RDC

/*
 NOTE:
 1. The host must have disabled MIPI Tx before calling this function.

 2. The host should act as below after calling this function.
    a. example_rdc200a_resume();
    b. rtimd_reset_mipi_interface(VC_CTRL_DCH_ALL);
    c. Enable MIPI Tx.
    d. rtimd_control_display_on(VC_CTRL_DCH_ALL, TRUE);
*/
int example_rdc200a_resume(void)
{
	int ret = 0;
#ifdef _BOOT_CHECK_USING_BUSY_GPIO_PIN
	unsigned gpio[MAX_NUM_VC_DCH];
#endif

	gpio_set_value(GPIO_RDC200A_RESETB, 1);
	mdelay(100);

#ifdef _BOOT_CHECK_USING_BUSY_GPIO_PIN
	gpio[VC_DCH_0] = GPIO_BOOT_DONE_FROM_RDC200A_GPIOB3_0;
	gpio[VC_DCH_1] = GPIO_BOOT_DONE_FROM_RDC200A_GPIOB3_1;
	ret = rtimd_check_firmware_boot_done(VC_CTRL_DCH_ALL, gpio);
#else
	mdelay(400);
#endif

	return ret;
}

int example_rdc200a_suspend(void)
{
	int ret;

	ret = rtimd_control_display_on(VC_CTRL_DCH_ALL, FALSE);
	mdelay(10);

	gpio_set_value(GPIO_RDC200A_RESETB, 0);
	mdelay(10);

	return ret;
}

int example_rdc200a_power_off(void)
{
	int ret;

	ret = rtimd_control_display_on(VC_CTRL_DCH_ALL, FALSE);
	mdelay(10);

	gpio_set_value(GPIO_RDC200A_RESETB, 0);
	mdelay(10);

	gpio_set_value(GPIO_RDC200A_LDO_3_3V, 0);
	gpio_set_value(GPIO_RDC200A_LDO_1_8V, 0);
	gpio_set_value(GPIO_RDC200A_LDO_1_2V, 0);

	return 0;
}

int example_rdc200a_power_on(void)
{
	int ret = 0;
#ifdef _BOOT_CHECK_USING_BUSY_GPIO_PIN
	unsigned gpio[MAX_NUM_VC_DCH];
#endif

	gpio_set_value(GPIO_RDC200A_LDO_1_2V, 1);
	gpio_set_value(GPIO_RDC200A_LDO_1_8V, 1);
	gpio_set_value(GPIO_RDC200A_LDO_3_3V, 1);
	mdelay(1);

	gpio_set_value(GPIO_RDC200A_RESETB, 1);
	mdelay(100);

#ifdef _BOOT_CHECK_USING_BUSY_GPIO_PIN
	gpio[VC_DCH_0] = GPIO_BOOT_DONE_FROM_RDC200A_GPIOB3_0;
	gpio[VC_DCH_1] = GPIO_BOOT_DONE_FROM_RDC200A_GPIOB3_1;
	ret = rtimd_check_firmware_boot_done(VC_CTRL_DCH_ALL, gpio);
#else
	mdelay(400);
#endif

    return ret;
}

static int ioctl_burst_write(unsigned long arg)
{
	int ret = 0;
	RTIMD_BURST_WRITE_REG_T bw;
	RTIMD_BURST_WRITE_REG_T __user *argp = (RTIMD_BURST_WRITE_REG_T __user *)arg;

	if (copy_from_user(&bw, argp, sizeof(RTIMD_BURST_WRITE_REG_T))) {
		RMDERR("copy_from_user() failed.\n");
		return -EFAULT;
	}

	if (copy_from_user((void *)rtimd_cb->write_buf,
						u64_to_uptr(bw.wbuf_addr),
						(unsigned long)bw.wsize)) {
		RMDERR("copy_from_user() failed.\n");
		return -EFAULT;
	}

	mutex_lock(&rtimd_cb->access_lock);

	ret = rtimd_get_i2c_adapter(bw.slave_addr, bw.bus_num);
	if (ret)
		goto out;

	ret = rtimd_i2c_burst_write(&bw, rtimd_cb->write_buf);

out:
	mutex_unlock(&rtimd_cb->access_lock);

	return ret;
}

static int ioctl_single_write(unsigned long arg)
{
	int ret = 0;
	RTIMD_SINGLE_WRITE_REG_T sw;
	RTIMD_SINGLE_WRITE_REG_T __user *argp = (RTIMD_SINGLE_WRITE_REG_T __user *)arg;

	if (copy_from_user(&sw, argp, sizeof(RTIMD_SINGLE_WRITE_REG_T))) {
		RMDERR("copy_from_user() failed.\n");
		return -EFAULT;
	}

	mutex_lock(&rtimd_cb->access_lock);

	ret = rtimd_get_i2c_adapter(sw.slave_addr, sw.bus_num);
	if (ret)
		goto out;

	ret = rtimd_i2c_single_write(&sw);

out:
	mutex_unlock(&rtimd_cb->access_lock);

	return ret;
}
//__copy_to_user((u8 __user *)(uintptr_t)u_tmp->rx_buf, rx_buf, u_tmp->len))

static int ioctl_burst_read(unsigned long arg)
{
	int ret;
	RTIMD_BURST_READ_REG_T br;
	RTIMD_BURST_READ_REG_T __user *argp = (RTIMD_BURST_READ_REG_T __user *)arg;

	if (copy_from_user(&br, argp, sizeof(RTIMD_BURST_READ_REG_T))) {
		RMDERR("copy_from_user() failed.\n");
		return -EFAULT;
	}

	if (br.rsize > MAX_RTIMD_REG_DATA_SIZE) {
		RMDERR("Invalid count to be read register\n");
		return -EINVAL;	
	}

	if (copy_from_user(rtimd_cb->write_buf, u64_to_uptr(br.wbuf_addr), br.wsize)) {
		RMDERR("copy_from_user() failed.\n");
		return -EFAULT;
	}

	mutex_lock(&rtimd_cb->access_lock);

	ret = rtimd_get_i2c_adapter(br.slave_addr, br.bus_num);
	if (ret)
		goto out;	

	ret = rtimd_i2c_burst_read(&br,
							rtimd_cb->write_buf,
							rtimd_cb->read_buf);
	if (ret > 0) {
		ret = copy_to_user(u64_to_uptr(br.rbuf_addr),
				rtimd_cb->read_buf, br.rsize) ? -EFAULT : ret;
	}

out:
	mutex_unlock(&rtimd_cb->access_lock);

	return ret;
}

static int ioctl_single_read(unsigned long arg)
{
	int ret;
	uint8_t sbuf; /* Single reade buffer */
	RTIMD_SINGLE_READ_REG_T sr;
	RTIMD_SINGLE_READ_REG_T __user *argp = (RTIMD_SINGLE_READ_REG_T __user *)arg;

	if (copy_from_user(&sr, argp, sizeof(RTIMD_SINGLE_READ_REG_T))) {
		RMDERR("copy_from_user() failed.\n");
		return -EFAULT;
	}

	mutex_lock(&rtimd_cb->access_lock);

	ret = rtimd_get_i2c_adapter(sr.slave_addr, sr.bus_num);
	if (ret)
		goto out;	

	ret = rtimd_i2c_single_read(&sr, &sbuf);
	if (ret > 0) {
		ret = copy_to_user(u64_to_uptr(sr.rbuf_addr),
				&sbuf, 1) ? -EFAULT : ret;
	}

out:
	mutex_unlock(&rtimd_cb->access_lock);

	return ret;
}

static long rtimd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case IOCTL_RTIMD_SINGLE_READ:
		ret = ioctl_single_read(arg);
		break;

	case IOCTL_RTIMD_BURST_READ:
		ret = ioctl_burst_read(arg);
		break;

	case IOCTL_RTIMD_SINGLE_WRITE:
		ret = ioctl_single_write(arg);
		break;

	case IOCTL_RTIMD_BURST_WRITE:
		ret = ioctl_burst_write(arg);
		break;

	default:
		RMDERR("Invalid ioctl command\n");
		ret = ENOIOCTLCMD;
		break;
	}

//	RMDDBG("ret: %d\n", ret);

	return ret;
}

#ifdef CONFIG_COMPAT
static long compat_rtimd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return rtimd_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif 

/* 
 * If successful, function returns the number of bytes actually written.
 * NOTE: For the single write mode, count vlaue don't care!
 */
static ssize_t rtimd_write(struct file *file, const char __user *buf,
							size_t count, loff_t *offset)
{
	RMDERR("Unsupport!\n");
	return 0;
}

static ssize_t rtimd_read(struct file *file, char __user *buf, size_t count,
						loff_t *offset)
{
	RMDERR("Unsupport!\n");
	return 0;
}

static int rtimd_release(struct inode *inode, struct file *file)
{
	if (rtimd_cb->read_buf) {
		kfree(rtimd_cb->read_buf);
		rtimd_cb->read_buf = NULL;
	}

	if (rtimd_cb->write_buf) {
		kfree(rtimd_cb->write_buf);
		rtimd_cb->write_buf = NULL;
	}

	atomic_set(&rtimd_cb->open_flag, 0);

	RMDDBG("Device closed\n");

	return 0;
}

static int rtimd_open(struct inode *inode, struct file *file)
{
	/* Check if the device is already opened ? */
	if (atomic_cmpxchg(&rtimd_cb->open_flag, 0, 1)) {
		RMDERR("%s is already opened\n", RTI_MD_DEV_NAME);
		return -EBUSY;
	}

	rtimd_cb->read_buf = kmalloc(MAX_RTIMD_REG_DATA_SIZE, GFP_KERNEL);
	if (!rtimd_cb->read_buf) {
		RMDERR("Fail to allocate a read buffer\n");
		return -ENOMEM;
	}

	rtimd_cb->write_buf = kmalloc(MAX_RTIMD_REG_DATA_SIZE, GFP_KERNEL);
	if (!rtimd_cb->write_buf) {
		RMDERR("Fail to allocate a write buffer\n");
		kfree(rtimd_cb->read_buf);
		rtimd_cb->read_buf = NULL;
		return -ENOMEM;
	}

	RMDDBG("Device opened\n");

	return 0;
}

static int rtimd_pm_suspend(struct device *dev)
{
	RMDDBG("\n");

	return 0;
}

static int rtimd_pm_resume(struct device *dev)
{
	int ret = 0;

	RMDDBG("\n");

	return ret;
}

static const struct file_operations rtimd_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= rtimd_read,
	.write		= rtimd_write,
	.unlocked_ioctl	= rtimd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = compat_rtimd_ioctl, /* 32-bit entry */
#endif	
	.open		= rtimd_open,
	.release	= rtimd_release,
};

static const struct dev_pm_ops rtimd_dev_pm_ops = {
		.suspend = rtimd_pm_suspend,
		.resume = rtimd_pm_resume,
};

static struct platform_driver rtimd_driver = {
		.probe = rtimd_probe,
		.remove = rtimd_remove,
		.driver = {
				.name = RTI_MD_DEV_NAME,
				.owner = THIS_MODULE,
				.pm = &rtimd_dev_pm_ops,
		}
};

static void hex_string_to_digit(uint8_t *out, const char *in, int len)
{
	int i, t;
	uint8_t hn, ln;
	char msb_ch, lsb_ch;

	//RMDDBG("in bytes(%d): [%s]\n", len, in);

	for (t = 0, i = 0; i < len; i+=2, t++) {
		msb_ch = UPPERCASE(in[i]);
		lsb_ch = UPPERCASE(in[i + 1]);
	
		hn = (msb_ch > '9') ? (msb_ch - 'A' + 10) : (msb_ch - '0');
		ln = (lsb_ch > '9') ? (lsb_ch - 'A' + 10) : (lsb_ch - '0');
		//RMDDBG("hn(%01X) ln(%01X)\n", hn, ln);

		out[t] = ((hn&0xF) << 4) | ln;
	}

	//RMDDBG("out: %02X%02X%02X%02X%02X%02X\n", out[0], out[1], out[2], out[3], out[4], out[5]);
}

/**
 * Set parameters to read a byte from register.
 *
 * ex) echo 06 44 005E 01 > /sys/devices/platform/rtimd-i2c/rtimd_srd_param
 *
 * Parameters: BNR SA RA RS
 * BNR: Bus Number (2 hex)
 * SA: Slave Address (2 hex)
 * RA: Register address of RDC or RDP (4 hex)
 * RS: Register size of RDC or RDP (2 hex)
 */
static ssize_t store_rtimd_srd_set_param(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int bus_num, slave_addr, reg_addr, reg_size;
	RTIMD_SINGLE_READ_REG_T *param = &srd_param;

	//RMDDBG("%s\n", buf);

	sscanf(buf, "%X %X %X %X", &bus_num, &slave_addr, &reg_addr, &reg_size);

	param->bus_num = (uint8_t)bus_num;
	param->slave_addr = (uint8_t)slave_addr;
	param->reg_addr = (uint32_t)reg_addr;
	param->reg_size = (uint8_t)reg_size;

	return count;
}

/**
 * Get a byte from register using the saved parameters
 * in show_rtimd_srd().
 *
 * ex) cat /sys/devices/platform/rtimd-i2c/rtimd_sreg
 */
static ssize_t show_rtimd_srd(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t sbuf; /* Single reade buffer */
	ssize_t count;
	RTIMD_SINGLE_READ_REG_T *param = &srd_param;

	//RMDDBG("Param: %hhu 0x%02X 0x%02X %hhu\n",
	//	param->bus_num, param->slave_addr, param->reg_addr, param->reg_size);

	ret = rtimd_get_i2c_adapter(param->slave_addr, param->bus_num);
	if (ret)
		return 0;

	ret = rtimd_i2c_single_read(param, &sbuf);
	if (ret > 0)
		count = sprintf(buf, "00 %02X\n", sbuf);
	else
		count = sprintf(buf, "FF\n");

	return count;
}

/**
 * Write a byte to register.
 *
 * ex) echo 06 44 005E 01 0x7A > /sys/devices/platform/rtimd-i2c/rtimd_sreg
 * Parameters: BNR SA RA RS VAL
 * BNR: Bus Number (2 hex)
 * SA: Slave Address (2 hex)
 * RA: Register address of RDC or RDP (4 hex)
 * RS: Register size of RDC or RDP (2 hex)
 * VAL: Register value to be written (2 hex)
 */
static ssize_t store_rtimd_swr(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int bus_num, slave_addr, reg_addr, reg_size, data;
	static RTIMD_SINGLE_WRITE_REG_T swr_param;

	//RMDDBG("%s\n", buf);

	sscanf(buf, "%X %X %X %X %X", &bus_num, &slave_addr,
								&reg_addr, &reg_size, &data);

	swr_param.bus_num = (uint8_t)bus_num;
	swr_param.slave_addr = (uint8_t)slave_addr;
	swr_param.reg_addr = (uint8_t)reg_addr;
	swr_param.reg_size = (uint8_t)reg_size;
	swr_param.data = (uint8_t)data;

	ret = rtimd_get_i2c_adapter((uint8_t)slave_addr, bus_num);
	if (ret)
		return 0;

	rtimd_i2c_single_write(&swr_param);

	return count;
}

/**
 * Set parameters to read the multiple bytes from register.
 *
 * ex) echo 06 44 0001 0005 5E > /sys/devices/platform/rtimd-i2c/rtimd_brd_param
 * Parameters: BNR SA WSIZE RSIZE WDATA
 * 	BNR: Bus Number (2 hex)
 *	SA: Slave Address (2 hex)
 * 	WSIZE: Number of bytes to write to the device before READ command in
 *		   I2C protocol. (4 hex)
 * RSIZE: Number of bytes to be read from the device (4 hex)
 * WDATA: Data to write to the device before READ command in I2C protocol.
 */
static ssize_t store_rtimd_brd_set_param(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int bus_num, slave_addr, wsize, rsize;
	const char *buf_ptr = buf;
	RTIMD_BURST_READ_REG_T *param = &brd_param;

	//RMDDBG("%s\n", buf);

	sscanf(buf, "%X %X %X %X", &bus_num, &slave_addr, &wsize, &rsize);
	buf_ptr += SYSFS_BRD_WDATA_OFFSET;

	if (wsize > 512) {
		RMDERR("Exceed the number of write bytes.\n");
		return -EINVAL;
	}

	if (rsize > 512) {
		RMDERR("Exceed the number of read bytes.\n");
		return -EINVAL;
	}

	/* 1hex: 2bytes asscii */
	hex_string_to_digit(sysfs_brd_wdata, buf_ptr, wsize<<1);

	param->bus_num = (uint8_t)bus_num;
	param->slave_addr = (uint8_t)slave_addr;
	param->wsize = (uint16_t)wsize;
	param->rsize = (uint16_t)rsize;

	return count;
}

/**
 * Get the multiple bytes from register using the saved parameters
 * in store_rtimd_brd_set_param().
 *
 * ex) cat /sys/devices/platform/rtimd-i2c/rtimd_breg
 */ 
static ssize_t show_rtimd_brd(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	int ret, i;
	ssize_t count;
	char nibble, hex_ch, *buf_ptr = buf;
	RTIMD_BURST_READ_REG_T *param = &brd_param;

	//RMDDBG("Param: %hhu 0x%02X 0x%04X 0x%04X\n",
	//		param->bus_num, param->slave_addr,
	//		param->wsize, param->rsize);

	ret = rtimd_get_i2c_adapter(param->slave_addr, param->bus_num);
	if (ret)
		return 0;

	ret = rtimd_i2c_burst_read(param, sysfs_brd_wdata, sysfs_brd_rdata);
	if (ret > 0) {
#if 0
		RMDDBG("sysfs_brd_rdata(%d): %02X%02X%02X%02X%02X%02X\n",
			param->rsize,
			sysfs_brd_rdata[0], sysfs_brd_rdata[1], sysfs_brd_rdata[2],
			sysfs_brd_rdata[3], sysfs_brd_rdata[4], sysfs_brd_rdata[5]);
#endif

		*buf_ptr++ = '0'; /* Success */
		*buf_ptr++ = '0';
		*buf_ptr++ = ' ';

		/* rdata */
		for (i = 0; i < param->rsize; i++) {
			nibble = (sysfs_brd_rdata[i] & 0xF0) >> 4;
			hex_ch = (nibble > 9) ? (nibble - 0xA + 'A') : (nibble + '0');
			*buf_ptr++ = hex_ch;

			nibble = sysfs_brd_rdata[i] & 0x0F;
			hex_ch = (nibble > 9) ? (nibble - 0xA + 'A') : (nibble + '0');
			*buf_ptr++ = hex_ch;
		}

		*buf_ptr++ = '\n';

		/*
		 * Returns the number of bytes stored in buffer,
		 * not counting the terminating null character.
		 */
		*buf_ptr = '\0';
		count = (ssize_t)(buf_ptr - buf);
	}
	else
		count = sprintf(buf, "FF\n");

	return count;
}

/**
 * Write the mutiple bytes to register.
 *
 * ex) echo 06 44 0006 5E123456789A > /sys/devices/platform/rtimd-i2c/rtimd_breg
 * Parameters: BNR SA WSIZE WDATA
 * 	BNR: Bus Number
 * 	SA: Slave Address
 * 	WSIZE: Number of bytes to write to the device (4 ..)
 * 	WDATA: Data to be written
 */
static ssize_t store_rtimd_bwr(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int bus_num;
	unsigned int slave_addr, wsize;
	RTIMD_BURST_WRITE_REG_T bwr_param;

	//RMDDBG("%s\n", buf);

	sscanf(buf, "%X %X %X", &bus_num, &slave_addr, &wsize);
	bwr_param.bus_num = (uint8_t)bus_num;
	bwr_param.slave_addr = (uint8_t)slave_addr;
	bwr_param.wsize = (uint16_t)wsize;

	if (wsize > 512) {
		RMDERR("Exceed the write bytes.\n");
		return -EINVAL;
	}

	/* 1hex: 2bytes asscii */
	hex_string_to_digit(sysfs_bwr_data, &buf[SYSFS_BWR_DATA_OFFSET], wsize<<1);

	ret = rtimd_get_i2c_adapter((uint8_t)slave_addr, bus_num);
	if (ret)
		return 0;

	rtimd_i2c_burst_write(&bwr_param, sysfs_bwr_data);

	return count;
}

static DEVICE_ATTR(rtimd_srd_param, S_IWGRP|S_IWUSR, NULL, store_rtimd_srd_set_param);
static DEVICE_ATTR(rtimd_sreg, S_IWUSR|S_IRUGO, show_rtimd_srd, store_rtimd_swr);
static DEVICE_ATTR(rtimd_brd_param, S_IWGRP|S_IWUSR, NULL, store_rtimd_brd_set_param);
static DEVICE_ATTR(rtimd_breg, S_IWUSR|S_IRUGO, show_rtimd_brd, store_rtimd_bwr);

static int rtimd_probe(struct platform_device *pdev)
{
	int ret;

	RMDDBG("Enter\n");

	/* register the driver */
	rtimd_major = register_chrdev(0, RTI_MD_DEV_NAME, &rtimd_fops);
	if (rtimd_major < 0) {
		RMDERR("register_chrdev() failed\n");
		return -EINVAL;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_rtimd_srd_param);
	ret |= device_create_file(&pdev->dev, &dev_attr_rtimd_sreg);
	ret |= device_create_file(&pdev->dev, &dev_attr_rtimd_brd_param);
	ret |= device_create_file(&pdev->dev, &dev_attr_rtimd_breg);
	if (ret) {
		RMDERR("Unable to create sysfs entries\n");

		/* un-register driver */
		unregister_chrdev(rtimd_major, RTI_MD_DEV_NAME);
		return ret;
	}

	mutex_init(&rtimd_cb->access_lock);

	rtimd_init_rdc_configuration();

	RMDDBG("End\n");

	return 0;
}

static int rtimd_remove(struct platform_device *pdev)
{
	RMDDBG("\n");

	device_remove_file(&pdev->dev, &dev_attr_rtimd_srd_param);
	device_remove_file(&pdev->dev, &dev_attr_rtimd_sreg);
	device_remove_file(&pdev->dev, &dev_attr_rtimd_brd_param);
	device_remove_file(&pdev->dev, &dev_attr_rtimd_breg);

	/* un-register driver */
	unregister_chrdev(rtimd_major, RTI_MD_DEV_NAME);

	return 0;
}

static int __init rtimd_dev_init(void)
{
	int retval = 0;
	int ret = 0;
	struct device *dev = NULL;

	RMDDBG("\n");

	rtimd_cb = kzalloc(sizeof(RTIMD_CB_T), GFP_KERNEL);
	if (!rtimd_cb)
		return -ENOMEM;

	ret = platform_driver_register(&rtimd_driver);
	if (ret != 0) {
		RMDERR("platform_driver_register failed.\n");
		kfree(rtimd_cb);
		return ret;
	}

	rtimd_device = platform_device_alloc(RTI_MD_DEV_NAME, -1);
	if (!rtimd_device) {
		RMDERR("platform_device_alloc() failed.\n");
		kfree(rtimd_cb);
		platform_driver_unregister(&rtimd_driver);
		return -ENOMEM;
	}

	/* add device */
	ret = platform_device_add(rtimd_device);
	if (ret) {
		RMDERR("platform_device_add() failed.\n");
		retval = ret;
		goto out;
	}

	/* create the node of device */
	rtimd_class = class_create(THIS_MODULE, RTI_MD_DEV_NAME);
	if (IS_ERR(rtimd_class)) {
		RMDERR("class_create() failed.\n");
		retval = PTR_ERR(rtimd_class);
		goto out;
	}

	/* create the logical device */
	dev = device_create(rtimd_class, NULL,
			MKDEV(rtimd_major, 0), NULL,
			RTI_MD_DEV_NAME);
	if (IS_ERR(dev)) {
		RMDERR("device_create() failed.\n");
		retval = PTR_ERR(dev);
		goto out;
	}

	rtimd_cb->bus_num = -1; /* Set default bus number as invalid */

	return 0;

out:
	platform_device_put(rtimd_device);
	platform_driver_unregister(&rtimd_driver);

	if (rtimd_cb) {
		kfree(rtimd_cb);
		rtimd_cb = NULL;
	}

	return retval;
}

static void __exit rtimd_dev_exit(void)
{
	RMDDBG("\n");

	device_destroy(rtimd_class, MKDEV(rtimd_major, 0));

	class_destroy(rtimd_class);

	platform_device_unregister(rtimd_device);

	platform_driver_unregister(&rtimd_driver);

	if (rtimd_cb->read_buf)
		kfree(rtimd_cb->read_buf);

	if (rtimd_cb->write_buf)
		kfree(rtimd_cb->write_buf);

	if (rtimd_cb->adap)
		i2c_put_adapter(rtimd_cb->adap);

	if (rtimd_cb) {
		kfree(rtimd_cb);
		rtimd_cb = NULL;
	}
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RAONTECH Inc.");
MODULE_DESCRIPTION("RAONTECH Micro Display I2C Driver");

module_init(rtimd_dev_init);
module_exit(rtimd_dev_exit);


