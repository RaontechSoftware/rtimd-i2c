
#ifndef __RTI_VC_CONFIG_H__
#define __RTI_VC_CONFIG_H__


/**
 * Defines the number of RDC controllers attached to the development system.
 */
#define _CFG_NUM_ATTACHED_RDC			2 // 1 or 2

/**
 * Definition statement to decide whether or not to activate display channel 0.
 * Declare this definition to activate the channel, otherwise comment out.
 */
#define _CFG_ACTIVATE_DISP_CH_0

/**
 * Definition statement to decide whether or not to activate display channel 1.
 * Declare this definition to activate the channel, otherwise comment out.
 */
#define _CFG_ACTIVATE_DISP_CH_1

/* I2C slave address of Panel. (7 bits) */
#define _CFG_PANEL_I2C_ADDR	    0x18

/**
 * Configure for display channel 0 to activate.
 */
/* Defines the I2C slave address of the RDC on the display channel 0 (7 bits) */
#define _CFG_RDC_I2C_ADDR_DISP_CH_0			0x4A

/* Defines the port number(index) of panel on the display channel 0. */
#define _CFG_PANEL_PORT_NUM_DISP_CH_0		0 // 0 or 1

/* I2C bus number(port) on display channel 0 */
#define _CFG_I2C_BUS_NUM_DISP_CH_0		3//0

/**
 * Configure for display channel 1 to activate.
 */
/* Defines the I2C slave address of the RDC on the display channel 1 (7 bits) */
#define _CFG_RDC_I2C_ADDR_DISP_CH_1			0x4A //0x4C

/* Defines the port number(index) of panel on the display channel 1. */
#define _CFG_PANEL_PORT_NUM_DISP_CH_1		0 // 0 or 1

/* I2C bus number(port) on display channel 1 */
#define _CFG_I2C_BUS_NUM_DISP_CH_1		14
	

#if defined(_CFG_ACTIVATE_DISP_CH_0) && defined(_CFG_ACTIVATE_DISP_CH_1)
	#define NUM_VC_NUM_DISPLAY_CH			2
	#define NUM_VC_ATTACHED_PANEL			2
#else
	#define NUM_VC_NUM_DISPLAY_CH			1
	#define NUM_VC_ATTACHED_PANEL			1
#endif


#if ((_CFG_NUM_ATTACHED_RDC != 1) && (_CFG_NUM_ATTACHED_RDC != 2))
	#error "Invalid _CFG_NUM_ATTACHED_RDC configuration!"
#endif

#if !defined(_CFG_ACTIVATE_DISP_CH_0) && !defined(_CFG_ACTIVATE_DISP_CH_1)
	#error "One of _CFG_ACTIVATE_DISP_CH_0 and _CFG_ACTIVATE_DISP_CH_1 must be defined!."
#endif

#if (_CFG_NUM_ATTACHED_RDC == 1)
	#if defined(_CFG_ACTIVATE_DISP_CH_0) && defined(_CFG_ACTIVATE_DISP_CH_1)
		#if (_CFG_RDC_I2C_ADDR_DISP_CH_0 != _CFG_RDC_I2C_ADDR_DISP_CH_1)
			#error "Must be the same address of RDC"
		#endif

		#if (_CFG_I2C_BUS_NUM_DISP_CH_0 != _CFG_I2C_BUS_NUM_DISP_CH_1)
			#error "Must be the same bus number"
		#endif
	#endif

	#if (NUM_VC_ATTACHED_PANEL == 2) 
		#if (_CFG_PANEL_PORT_NUM_DISP_CH_0 == _CFG_PANEL_PORT_NUM_DISP_CH_1)
			#error "For 1 RDC and 2 panels, the two panel port numbers must be different!"
		#endif
	#endif
	
#endif

#ifdef _CFG_ACTIVATE_DISP_CH_0
	#if (_CFG_RDC_I2C_ADDR_DISP_CH_0 > 0x7F)
		#error "[CH 0] Invalid RDC I2C address! Must be 7bits address"
	#endif

	#if ((_CFG_PANEL_PORT_NUM_DISP_CH_0 != 0) && (_CFG_PANEL_PORT_NUM_DISP_CH_0 != 1))
		#error "[CH 0] Invalid panel port!"
	#endif
#endif

#ifdef _CFG_ACTIVATE_DISP_CH_1
	#if (_CFG_RDC_I2C_ADDR_DISP_CH_1 > 0x7F)
		#error "[CH 1] Invalid RDC I2C address! Must be 7bits address"
	#endif

	#if ((_CFG_PANEL_PORT_NUM_DISP_CH_1 != 0) && (_CFG_PANEL_PORT_NUM_DISP_CH_1 != 1))
		#error "[CH 1] Invalid panel port!"
	#endif
#endif

#endif /* __RTI_VC_CONFIG_H__ */


