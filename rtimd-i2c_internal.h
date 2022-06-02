
#ifndef __RTIMD_I2C_INTERNAL_H__
#define __RTIMD_I2C_INTERNAL_H__

#include <linux/types.h>

typedef signed char			S8_T;
typedef unsigned char		U8_T;
typedef signed short		S16_T;
typedef unsigned short		U16_T;
typedef signed int			S32_T;
typedef unsigned int		U32_T;
typedef int                 BOOL_T;
typedef unsigned int		UINT_T;

typedef struct {
	const char *dch_name;
	BOOL_T activated; // if false, the display ch is not uesd or not configured.
	//E_VC_RDC_INDEX_T rdc_idx;
	UINT_T rdc_idx;
	int rdc_i2c_bus_num;
	U8_T rdc_i2c_addr;
	U8_T panel_i2c_addr;

	U8_T panel_port_num; /* 0 or 1 */
} RTIMD_DISPLAY_CH_INFO_T;

typedef struct {
	const char *rdc_name;
	const char *panel_name;

	U8_T num_attached_rdc;
	U8_T num_attached_panel;
	U8_T active_dch_mask;

	/* The configured information of channel. */
	RTIMD_DISPLAY_CH_INFO_T chan[MAX_NUM_VC_DCH];
} RTIMD_DEVICE_INFO_T;

typedef struct {
	BOOL_T activated;
	U8_T i2c_addr;
	int i2c_bus_num;
	UINT_T attached_panel_mask;

	UINT_T attached_dch_mask;
	const RTIMD_DISPLAY_CH_INFO_T *dch[MAX_NUM_VC_DCH];
} RTIMD_RDC_CFG_INFO_T;

extern RTIMD_RDC_CFG_INFO_T rtimd_rdc_cfg_table[MAX_NUM_VC_RDC_INDEX];
extern const RTIMD_DEVICE_INFO_T rtimd_dev_info_table;

void rtimd_init_rdc_configuration(void);

#endif /* __RTIMD_I2C_INTERNAL_H__ */


