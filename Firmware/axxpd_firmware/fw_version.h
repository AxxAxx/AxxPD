/**
 * @file    fw_version.h
 * @brief   Compile-time firmware version string for the bootloader-visible
 *          app header (Core/Src/app_header.c).
 *
 * Keep this in sync with the fw_version_major/minor/patch fields in cli.cpp
 * (which build the runtime string for *IDN?/help). Bump both when cutting a
 * new build. The bootloader never compares versions — any valid AxxPD image
 * can be flashed, including downgrades.
 */
#ifndef FW_VERSION_H
#define FW_VERSION_H

#define FW_VERSION "1.0.0"

#endif /* FW_VERSION_H */
