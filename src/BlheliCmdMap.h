#pragma once

// source: https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt

enum dshot_cmd_t {
	DIGITAL_CMD_MOTOR_STOP,							// Currently not implemented
	DIGITAL_CMD_BEEP1, 								// Wait at least length of beep (380ms) before next command
	DIGITAL_CMD_BEEP2, 								// Wait at least length of beep (380ms) before next command
	DIGITAL_CMD_BEEP3, 								// Wait at least length of beep (400ms) before next command
	DIGITAL_CMD_BEEP4, 								// Wait at least length of beep (400ms) before next command
	DIGITAL_CMD_BEEP5, 								// Wait at least length of beep (400ms) before next command
	DIGITAL_CMD_ESC_INFO,  							// Wait at least 12ms before next command
	DIGITAL_CMD_SPIN_DIRECTION_1, 					// Currently not implemented
	DIGITAL_CMD_SPIN_DIRECTION_2, 					// Need 6x, no wait required
	DIGITAL_CMD_3D_MODE_OFF, 						// Need 6x, no wait required
	DIGITAL_CMD_3D_MODE_ON,  						// Need 6x, no wait required
	DIGITAL_CMD_SETTINGS_REQUEST,  					// Currently not implemented
	DIGITAL_CMD_SAVE_SETTINGS,  					// Need 6x, wait at least 12ms before next command
	DIGITAL_CMD_SPIN_DIRECTION_NORMAL = 20, 	    // Need 6x, no wait required
	DIGITAL_CMD_SPIN_DIRECTION_REVERSED, 			// Need 6x, no wait required
	DIGITAL_CMD_LED0_ON, 							// No wait required
	DIGITAL_CMD_LED1_ON, 							// No wait required
	DIGITAL_CMD_LED2_ON, 							// No wait required
	DIGITAL_CMD_LED3_ON, 							// No wait required
	DIGITAL_CMD_LED0_OFF, 							// No wait required
	DIGITAL_CMD_LED1_OFF, 							// No wait required
	DIGITAL_CMD_LED2_OFF, 							// No wait required
	DIGITAL_CMD_LED3_OFF, 							// No wait required
	DSHOT_CMD_MAX = 47
};
