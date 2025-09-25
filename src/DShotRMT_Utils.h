#pragma once

#include <Arduino.h>
#include "DShotRMT.h" // Include DShotRMT.h for DShotRMT class definition

/**
 * @brief Utility functions for printing DShot and CPU information
 */

/**
 * @brief Prints detailed DShot signal information for a given DShotRMT instance.
 * @param dshot_rmt The DShotRMT instance to get information from.
 * @param output The output stream (e.g., Serial) to print to. Defaults to Serial.
 */
void printDShotInfo(const DShotRMT &dshot_rmt, Stream &output = Serial);

/**
 * @brief Prints detailed CPU information.
 * @param output The output stream (e.g., Serial) to print to. Defaults to Serial.
 */
void printCpuInfo(Stream &output = Serial);
