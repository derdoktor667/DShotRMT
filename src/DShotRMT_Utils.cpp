#include "DShotRMT_Utils.h"
#include "DShotRMT.h" // Include DShotRMT.h for DShotRMT class definition

void printDShotInfo(const DShotRMT &dshot_rmt, Stream &output)
{
    output.println("\n === DShot Signal Info === ");
    output.printf("Current Mode: DSHOT%d\n", dshot_rmt.getMode() == dshot_mode_t::DSHOT150 ? 150 : 
                                             dshot_rmt.getMode() == dshot_mode_t::DSHOT300 ? 300 :
                                             dshot_rmt.getMode() == dshot_mode_t::DSHOT600 ? 600 : 
                                             dshot_rmt.getMode() == dshot_mode_t::DSHOT1200 ? 1200 : 0);
    output.printf("Bidirectional: %s\n", dshot_rmt.isBidirectional() ? "YES" : "NO");
    output.printf("Current Packet: ");

    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
    {
        output.print((dshot_rmt.getEncodedFrameValue() >> i) & 1);
    }

    output.printf("\nCurrent Value: %u\n", dshot_rmt.getThrottleValue());
}

void printCpuInfo(Stream &output)
{
    output.println("\n ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}
