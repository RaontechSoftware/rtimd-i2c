/*
 * File name: rtimd-i2c.h
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

#ifndef __RTIMD_I2C_H__
#define __RTIMD_I2C_H__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/list.h> 
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#define RMDERR(fmt, args...) \
	pr_err("RTI_MD: %s(): " fmt, __func__, ## args)

// pr_debug was not output msg......
#define RMDDBG(fmt, args...) \
	pr_err("RTI_MD: %s(): " fmt, __func__, ## args)


/* device file name */
#define RTI_MD_DEV_NAME		"rtimd-i2c"

#ifndef FALSE
	#define FALSE		0
#endif

#ifndef TRUE
	#define TRUE		1
#endif

#define MAX_RTIMD_REG_DATA_SIZE		PAGE_SIZE /* 4KB Kernel page size */


#define VC_RDC_INDEX_0			0
#define VC_RDC_INDEX_1			1
#define MAX_NUM_VC_RDC_INDEX	2

#define VC_DCH_0		0 /* Display channel 0 */
#define VC_DCH_1		1 /* Display channel 1 */
#define MAX_NUM_VC_DCH 	2

#define VC_ACT_MASK_DCH_0		1
#define VC_ACT_MASK_DCH_1		2
#define VC_ACT_MASK_DCH_ALL		3

/**
 * @brief Display channel bit types to control.
 */
typedef enum {
	/** Control for display channel 0 only */
	VC_CTRL_DCH_0 = (1 << VC_DCH_0),

	/** Control for display channel 1 only */
	VC_CTRL_DCH_1 = (1 << VC_DCH_1),

	/** Control for display channel all */
	VC_CTRL_DCH_ALL = (1 << VC_DCH_0) | (1 << VC_DCH_1)
} E_VC_CTRL_DISP_CH_T;

/*
 RDC200A INPUT VCD Monitoring
 */
typedef struct {
    uint16_t hsw;
    uint16_t hbp;
    uint16_t hfp;
    uint16_t vsw;

    uint16_t vbp;
    uint16_t vfp;
    uint16_t ha;
    uint16_t va;
} RTIMD_VIDEO_COORD_INFO_T;


/* RDC Control Block */
typedef struct {
	atomic_t open_flag; /* to open only once */

	uint8_t	*read_buf;
	uint8_t	*write_buf;

	int bus_num;
	struct i2c_adapter *adap;

	struct mutex access_lock;
	uint selected_rdc_idx;
} RTIMD_CB_T;


/*
 * NOTE: Force align to 64-bit to compat 32-bit application.
 */
typedef struct {
	const char dch_name[32];
	uint16_t activated; // if false, the display ch is not uesd or not configured.
	uint16_t rdc_idx; /* 0 or 1 */

	int32_t rdc_i2c_bus_num;
	uint8_t rdc_i2c_addr;
	uint8_t panel_i2c_addr;
	uint8_t panel_port_num; /* 0 or 1 */
	uint8_t reserved;
} IOCTL_RTIMD_DISPLAY_CH_INFO_T;

typedef struct {
	const char rdc_name[16];
	const char panel_name[16];

	uint8_t num_attached_rdc;
	uint8_t num_attached_panel;
	uint8_t active_dch_mask;
	uint8_t reserved;

	/* The configured information of channel. */
	IOCTL_RTIMD_DISPLAY_CH_INFO_T chan[MAX_NUM_VC_DCH];
} IOCTL_RTIMD_DEVICE_CFG_INFO_T;

typedef struct {
	uint32_t reg_addr;
	uint8_t bus_num;
	uint8_t slave_addr;
	uint8_t reg_size; /* RDC200A: 2 */
	uint8_t data;
} RTIMD_SINGLE_WRITE_REG_T;

typedef struct {
	uint64_t wbuf_addr;

	uint8_t bus_num;
	uint8_t slave_addr;
	uint16_t wsize;
	uint32_t pad;
} RTIMD_BURST_WRITE_REG_T;

typedef struct {
	uint64_t rbuf_addr;

	uint32_t reg_addr;
	uint8_t bus_num;
	uint8_t slave_addr;
	uint8_t reg_size;
	uint8_t pad;
} RTIMD_SINGLE_READ_REG_T;

typedef struct {
	uint64_t wbuf_addr;
	uint64_t rbuf_addr;

	uint16_t wsize;
	uint16_t rsize;
	uint8_t bus_num;
	uint8_t slave_addr;
	uint16_t pad;
} RTIMD_BURST_READ_REG_T;

#define RTIMD_IOC_MAGIC	'R'

#define IOCTL_RTIMD_SINGLE_READ		_IOWR(RTIMD_IOC_MAGIC, 1, RTIMD_SINGLE_READ_REG_T)
#define IOCTL_RTIMD_BURST_READ		_IOWR(RTIMD_IOC_MAGIC, 2, RTIMD_BURST_READ_REG_T)
#define IOCTL_RTIMD_SINGLE_WRITE	_IOWR(RTIMD_IOC_MAGIC, 3, RTIMD_SINGLE_WRITE_REG_T)
#define IOCTL_RTIMD_BURST_WRITE		_IOWR(RTIMD_IOC_MAGIC, 4, RTIMD_BURST_WRITE_REG_T)
#define IOCTL_RTIMD_GET_DEVICE_CFG_INFO\
	_IOWR(RTIMD_IOC_MAGIC, 5, IOCTL_RTIMD_DEVICE_CFG_INFO_T)


int rtimd_control_display_on(E_VC_CTRL_DISP_CH_T ctrl_dch, bool on);

int rtimd_reset_mipi_interface(E_VC_CTRL_DISP_CH_T ctrl_dch);

int rtimd_check_firmware_boot_done(E_VC_CTRL_DISP_CH_T ctrl_dch,
								unsigned gpio[MAX_NUM_VC_DCH]);

#endif /* __RTIMD_I2C_H__ */

