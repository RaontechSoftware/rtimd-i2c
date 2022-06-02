
#include "rtimd-i2c.h"
#include "rtimd-i2c_internal.h"
#include "rtimd_dev_config.h"

RTIMD_RDC_CFG_INFO_T rtimd_rdc_cfg_table[MAX_NUM_VC_RDC_INDEX];

/*
 NOTE:
  1. RDC 0 is connected as default.
  2. DCH 0 operates on RDC 0.
*/
const RTIMD_DEVICE_INFO_T rtimd_dev_info_table =
{
	/* RDC Name */
#if defined(CFG_MDC_RDC200)
	"RDC200",
#elif defined(CFG_MDC_RDC200A)
	"RDC200A",
#else
	#error "Code not present"
#endif

	/* Panel Name */
#if defined(CFG_PANEL_RDP700Q)
	"RDP700Q",
#elif defined(CFG_PANEL_RDP551F)
	"RDP551F",
#elif defined(CFG_PANEL_RDP502H)	
	"RDP502H",
#elif defined(CFG_PANEL_RDP370F)
	"RDP370F",
#else
	#error "Code not present"
#endif

	_CFG_NUM_ATTACHED_RDC, /* The numner of attached RDC */
	NUM_VC_ATTACHED_PANEL, /* The numner of attached Panel */

#if defined(_CFG_ACTIVATE_DISP_CH_0) && defined(_CFG_ACTIVATE_DISP_CH_1)
	VC_ACT_MASK_DCH_ALL,
#elif defined(_CFG_ACTIVATE_DISP_CH_0) && !defined(_CFG_ACTIVATE_DISP_CH_1)
	VC_ACT_MASK_DCH_0,
#elif !defined(_CFG_ACTIVATE_DISP_CH_0) && defined(_CFG_ACTIVATE_DISP_CH_1)
	VC_ACT_MASK_DCH_1,
#else
	#error "Code not present"
#endif
	
	/* Channel informations */
	{
		/* Channel 0 */
		{
			"DCH 0",
#ifdef _CFG_ACTIVATE_DISP_CH_0
			TRUE, /* Flag whether the channel is activated */
#else
			FALSE, /* Flag whether the channel is activated */
#endif
			VC_RDC_INDEX_0, /* RDC index on the specified display channel */
			_CFG_I2C_BUS_NUM_DISP_CH_0, /* I2C Bus number */
			_CFG_RDC_I2C_ADDR_DISP_CH_0, /* RDC I2C address */
			_CFG_PANEL_I2C_ADDR, /* Panel I2C address */
			_CFG_PANEL_PORT_NUM_DISP_CH_0 /* Panel port number */
		},

		/* Channel 1 */
		{
			"DCH 1",
#ifdef _CFG_ACTIVATE_DISP_CH_1
			TRUE, /* Flag whether the channel is activated */
#else
			FALSE, /* Flag whether the channel is activated */
#endif
#if (_CFG_NUM_ATTACHED_RDC == 2)
			VC_RDC_INDEX_1, /* RDC index on the specified display channel */
#else
			VC_RDC_INDEX_0,
#endif
			_CFG_I2C_BUS_NUM_DISP_CH_1, /* I2C Bus number */
			_CFG_RDC_I2C_ADDR_DISP_CH_1, /* RDC I2C address */
			_CFG_PANEL_I2C_ADDR, /* Panel I2C address */
			_CFG_PANEL_PORT_NUM_DISP_CH_1 /* Panel port number */
		}
	}
};

void rtimd_init_rdc_configuration(void)
{
	uint i, k;
	const RTIMD_DISPLAY_CH_INFO_T *dch;

	memset(rtimd_rdc_cfg_table, 0x0, sizeof(rtimd_rdc_cfg_table));

	for (i = 0; i < MAX_NUM_VC_RDC_INDEX; i++) {
		for (k = 0; k < MAX_NUM_VC_DCH; k++) {
			dch = &rtimd_dev_info_table.chan[k];
			if (dch->activated) {
				if (i == (uint)dch->rdc_idx) {
					rtimd_rdc_cfg_table[i].activated = TRUE;

					rtimd_rdc_cfg_table[i].i2c_addr = dch->rdc_i2c_addr;
					rtimd_rdc_cfg_table[i].i2c_bus_num = dch->rdc_i2c_bus_num;
					rtimd_rdc_cfg_table[i].attached_panel_mask
						|= (1 << dch->panel_port_num);

					rtimd_rdc_cfg_table[i].attached_dch_mask
						|= (1 << k);
					rtimd_rdc_cfg_table[i].dch[k] = dch;
				}
			}
		}
	}
}


