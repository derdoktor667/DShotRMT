#pragma once

#include <Arduino.h>
#include "DShotRMT.h" // Include the full class definition

// Helper to quick print DShot result codes
inline void printDShotResult(dshot_result_t &result, Stream &output = Serial)
{
    output.printf("Status: %s - %s", result.success ? "SUCCESS" : "FAILED", _get_result_code_str(result.result_code));

    // Print telemetry data if available
    if (result.success && (result.erpm > 0 || result.motor_rpm > 0))
    {
        output.printf(" | eRPM: %u, Motor RPM: %u", result.erpm, result.motor_rpm);
    }

    output.println();
}

// Helper to print DShot signal info
inline void printDShotInfo(const DShotRMT &dshot_rmt, Stream &output = Serial)
{
    output.println("\n === DShot Signal Info === ");

    uint16_t dshot_mode_val = 0;
    switch (dshot_rmt.getMode())
    {
    case DSHOT150:
        dshot_mode_val = 150;
        break;
    case DSHOT300:
        dshot_mode_val = 300;
        break;
    case DSHOT600:
        dshot_mode_val = 600;
        break;
    case DSHOT1200:
        dshot_mode_val = 1200;
        break;
    }

    output.printf("Current Mode: DSHOT%d\n", dshot_mode_val);
    output.printf("Bidirectional: %s\n", dshot_rmt.isBidirectional() ? "YES" : "NO");
    output.printf("Current Packet: ");

    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
    {
        output.print((dshot_rmt.getEncodedFrameValue() >> i) & 1);
    }

    output.printf("\nCurrent Value: %u\n", dshot_rmt.getThrottleValue());
}

// Helper to print CPU info
inline void printCpuInfo(Stream &output = Serial)
{
    output.println("\n ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}
