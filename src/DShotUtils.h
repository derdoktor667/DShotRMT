#pragma once

#include <string.h>

typedef struct {
    uint8_t mPeriodShift : 3;
    uint16_t mPeriodBase : 9;
    uint8_t mChecksum : 4;
} dshotTelemetryFrameT;

static const int8_t sQuintToNibbleMap[0x1F] = 
{
    // This row of thirty -1s ensures that all values not explicitly set are -1
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    [0x09] = 0x09,
    [0x0A] = 0x0A,
    [0x0B] = 0x0B,
    [0x0D] = 0x0D,
    [0x0E] = 0x0E,
    [0x0F] = 0x0F,
    [0x12] = 0x02,
    [0x13] = 0x03,
    [0x15] = 0x05,
    [0x16] = 0x06,
    [0x17] = 0x07,
    [0x19] = 0x00,
    [0x1A] = 0x08,
    [0x1B] = 0x01,
    [0x1D] = 0x04,
    [0x1E] = 0x0c,
};

static constexpr uint8_t cCrcMask = 0x0F;
static constexpr unsigned cCrcBitLen = 4;
static constexpr uint16_t cPeriodMask = 0x1FF;
static constexpr unsigned cPeriodBitLen = 9;
static constexpr uint8_t cShiftMask = 0x07;
static constexpr unsigned cShiftBitLen = 3;

constexpr unsigned decodeERpmData(uint16_t rawData)
{
    // Mask off the period, isolate the shift, then shift by that
    return (rawData & cPeriodMask) << (cShiftMask & (rawData >> cPeriodBitLen));
}

const uint32_t decodeGcr(uint32_t aGcrVal)
{
    static constexpr unsigned cBitsInNibble = 4;
    static constexpr unsigned cNibbleMask = 0x0F;
    static constexpr unsigned cBitsInQuint = 5;
    static constexpr unsigned cQuintMask = 0x1F;

    uint32_t output = 0;

    for (int nibblesLeft = 3; nibblesLeft >= 0; nibblesLeft--)
    {
        int8_t decodedNibble = sQuintToNibbleMap[(aGcrVal >> (cBitsInQuint * nibblesLeft)) & cQuintMask];
        if (decodedNibble >= 0)
        {
            output |=  decodedNibble;
        }
        else
        {
            output = 0;
            printf("Invalid set of bits: 0x%x\n", (aGcrVal & cQuintMask));
            break;
        }

	    if (nibblesLeft > 0)
	    {
            output <<= cBitsInNibble;
	    }
    }

    return output;
}

constexpr bool verifyCrc(uint16_t eRpmFrame, uint8_t crc)
{
    return crc == ((eRpmFrame ^ (eRpmFrame >> 4) ^ (eRpmFrame >> 8)) & 0x0f);
}

const int getRpmFromFrame(uint32_t rawFrame)
{
    int ret = -1;
    rawFrame ^= rawFrame >> 1;

    uint32_t eRpmFrame = decodeGcr(rawFrame);
    uint8_t crc = eRpmFrame & cCrcMask;
    eRpmFrame >>= cCrcBitLen;

    if (verifyCrc(eRpmFrame, crc))
    {
        ret = (int) decodeERpmData(eRpmFrame);
    }

    return ret;
}
