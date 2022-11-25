#pragma once

// Source:	https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt
// Date:	04.07.2021

enum dshot_cmd_t
{
    DSHOT_CMD_MOTOR_STOP,              // Currently not implemented - STOP Motors
    DSHOT_CMD_BEEP1,                   // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP2,                   // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP3,                   // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP4,                   // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP5,                   // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_ESC_INFO,                // Currently not implemented
    DSHOT_CMD_SPIN_DIRECTION_1,        // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_2,        // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_OFF,             // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_ON,              // Need 6x, no wait required
    DSHOT_CMD_SETTINGS_REQUEST,        // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,           // Need 6x, wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_NORMAL,   // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_REVERSED, // Need 6x, no wait required
    DSHOT_CMD_LED0_ON,                 // Currently not implemented
    DSHOT_CMD_LED1_ON,                 // Currently not implemented
    DSHOT_CMD_LED2_ON,                 // Currently not implemented
    DSHOT_CMD_LED3_ON,                 // Currently not implemented
    DSHOT_CMD_LED0_OFF,                // Currently not implemented
    DSHOT_CMD_LED1_OFF,                // Currently not implemented
    DSHOT_CMD_LED2_OFF,                // Currently not implemented
    DSHOT_CMD_LED3_OFF,                // Currently not implemented
    DSHOT_CMD_MAX = 47
};
