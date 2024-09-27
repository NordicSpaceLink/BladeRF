/*
 * @file libbladeRF.h
 *
 * @brief bladeRF library
 *
 * Copyright (C) 2013-2017 Nuand LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

namespace NordicSpaceLink.BladeRF;

public enum Backend
{
    Any,
    Linux,
    Libusb,
    Cypress,
    Dummy = 100,
}


public enum GainMode
{
    Default,
    Manual,
    FastAttackAGC,
    SlowAttackAGC,
    HybridAGC,
}

public enum Format
{
    SC16_Q11,
    SC16_Q11_META,
    PACKET_META,
    SC8_Q7,
    SC8_Q7_META,
}

public enum DevSpeed
{
    Unknown,
    High,
    Super
}

public enum LogLevel
{
    Verbose,
    Debug,
    Info,
    Warning,
    Error,
    Critical,
    Silent
}

public enum RxMux
{
    MUX_INVALID = -1,
    MUX_BASEBAND = 0x0,
    MUX_12BIT_COUNTER = 0x1,
    MUX_32BIT_COUNTER = 0x2,
    MUX_DIGITAL_LOOPBACK = 0x4,
}

public enum Loopback
{
    NONE = 0,
    FIRMWARE,
    BB_TXLPF_RXVGA2,
    BB_TXVGA1_RXVGA2,
    BB_TXLPF_RXLPF,
    BB_TXVGA1_RXLPF,
    RF_LNA1,
    RF_LNA2,
    RF_LNA3,
    RFIC_BIST,
}

public enum FPGASource
{
    Unknown = 0,
    Flash = 1,
    Host = 2
}

public enum FPGASize
{
    FPGA_UNKNOWN = 0,
    FPGA_40KLE = 40,
    FPGA_115KLE = 115,
    FPGA_A4 = 49,
    FPGA_A5 = 77,
    FPGA_A9 = 301
}