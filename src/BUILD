cc_library(
    name = "DShotRmt",
    srcs = ["DShotRMT.cpp"],
    hdrs = ["DShotRMT.h", "BlheliCmdMap.h"],
)

cc_library(
    name = "DShotUtils",
    hdrs = ["DShotUtils.h"],
)

cc_test(
    name = "DShotUtilsTests",
    size = "small",
    srcs = ["DShotUtilsTests.cpp"],
    deps = ["DShotUtils", "@com_google_googletest//:gtest_main"],
)
