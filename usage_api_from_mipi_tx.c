
1. Makefile
ccflags-y += -Idrivers/misc/rtimd-i2c

2. Power-On RDC200A
 a. Supply power to the RDC200A
 b. rtimd_check_firmware_boot_done(VC_CTRL_DCH_ALL, gpio);

 - See example function: example_rdc200a_power_on()
 	
3. When an interrupt to reset MIPI TX occurs
 #include "rtimd-i2c.h"

 a. Disable MIPI TX port.
 b. delay 10ms
 c. rtimd_reset_mipi_interface(VC_CTRL_DCH_ALL); // Reset MIPI RX.
 d. Enable MIPI TX port.
 e. rtimd_control_display_on(VC_CTRL_DCH_ALL, TRUE); // Panel On.

3. NOTE
 If you get an i2c adapter using a device tree instead of a bus number,
 you need to modify the rtimd_get_i2c_adapter() function.
