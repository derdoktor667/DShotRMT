#include <gtest/gtest.h>

#include "DShotUtils.h"

TEST(GcrTest, GcrDecode) {
    EXPECT_EQ(0x2D18, decodeGcr(0x9377A));
    EXPECT_EQ(0x82C6, decodeGcr(0xD4BD6));
    EXPECT_EQ(0x5A5A, decodeGcr(0xAAAAA));
}

TEST(CrcTest, CrcTest) {
    EXPECT_EQ(true, verifyCrc(0x82C, 0x6));
    EXPECT_EQ(true, verifyCrc(0x5A5, 0xA));
    EXPECT_EQ(false, verifyCrc(0x5A5, 0x5));
}

TEST(DecodeERpmDataTest, DecodeERpmData) {
    EXPECT_EQ(704, decodeERpmData(0x82C));
}

TEST(DecodeERpmFrameTest, GetRpmFromFrame) {
    EXPECT_EQ(1684, getRpmFromFrame(0xCCCCC));
}
