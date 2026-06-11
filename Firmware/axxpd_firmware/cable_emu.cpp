// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

// Cable e-marker emulation — SOP' Discover_Identity response.
//
// Emulates an EPR-capable passive cable:
//   - 50V / 5A rating
//   - PD 3.0+ revision
//   - USB 2.0 speed
//   - No VCONN required
//
// Response VDO layout (5 data objects = 20 bytes + 2 byte header = 22 bytes):
//   DO[0] VDM Header:       0xFF008041 | (svdm_ver << 11)
//   DO[1] ID Header VDO:    0x18600483  (Passive Cable, USB-C plug, VID 0x0483)
//   DO[2] Cert Stat VDO:    0x00000000
//   DO[3] Product VDO:      0x00000000
//   DO[4] Passive Cable VDO: 0x116A2640  (50V 5A EPR)

#include "cable_emu.h"
#include <cstring>

static uint8_t cable_msgid = 0;

void cable_emu_reset(void) { cable_msgid = 0; }

uint16_t cable_emu_build_response(const uint8_t* rx, uint16_t rx_size,
                                   uint8_t* tx, uint16_t tx_buf_size) {
    // Need at least header (2) + VDM header (4) = 6 bytes, and output >= 22
    if (rx_size < 6 || tx_buf_size < 22) return 0;

    // Check PD header: message_type must be 0x0F (Vendor_Defined)
    uint8_t msg_type = rx[0] & 0x1F;
    if (msg_type != 0x0F) return 0;

    // Parse VDM header (bytes 2..5, little-endian)
    uint32_t vdm = static_cast<uint32_t>(rx[2])
                 | (static_cast<uint32_t>(rx[3]) << 8)
                 | (static_cast<uint32_t>(rx[4]) << 16)
                 | (static_cast<uint32_t>(rx[5]) << 24);

    // Validate: SVID=0xFF00, structured VDM, cmd=Discover Identity (1), cmd_type=REQ (0)
    if ((vdm >> 16) != 0xFF00) return 0;       // wrong SVID
    if (((vdm >> 15) & 1) != 1) return 0;      // not structured
    if ((vdm & 0x1F) != 1) return 0;           // not Discover Identity
    if (((vdm >> 6) & 3) != 0) return 0;       // not REQ

    // Extract SVDM version from request to mirror it in response
    uint8_t svdm_ver = static_cast<uint8_t>((vdm >> 13) & 0x3);

    // --- Build response (22 bytes) ---

    // PD Header: NDO=5, CablePlug=1, Rev=PD3.0, MsgType=0x0F (Vendor_Defined)
    // Bits: [15:13]=NDO=5 -> 101, [12]=ext=0, [11:9]=msgid,
    //       [8]=CablePlug=1 (bit 8 in SOP' headers), [7:6]=rev=01(PD3.0),
    //       [5]=reserved=0 for SOP', [4:0]=0x0F
    uint16_t hdr = 0x518F | (static_cast<uint16_t>(cable_msgid & 7) << 9);
    cable_msgid = (cable_msgid + 1) & 7;
    tx[0] = hdr & 0xFF;
    tx[1] = hdr >> 8;

    // DO[0] VDM Header: SVID=0xFF00, structured=1, ACK(cmd_type=01), cmd=1
    // Match requester's SVDM version in bits[14:13]
    uint32_t vdm_resp = 0xFF008041 | (static_cast<uint32_t>(svdm_ver & 0x3) << 13);
    memcpy(&tx[2], &vdm_resp, 4);

    // DO[1] ID Header VDO (SOP', PD 3.1): 0x18600483
    //   [31]    USB host capable = 0
    //   [30]    USB device capable = 0
    //   [29:27] Product Type (Cable Plug) = 011 (Passive Cable)
    //   [26]    modal operation = 0
    //   [25:23] Product Type (DFP) = 000 (zero for cable plug)
    //   [22:21] connector type = 11 (USB-C plug)
    //   [20:16] reserved = 0
    //   [15:0]  USB VID = 0x0483 (STMicroelectronics)
    uint32_t idh = 0x18600483;
    memcpy(&tx[6], &idh, 4);

    // DO[2] Cert Stat VDO
    uint32_t cert = 0x00000000;
    memcpy(&tx[10], &cert, 4);

    // DO[3] Product VDO
    uint32_t prod = 0x00000000;
    memcpy(&tx[14], &prod, 4);

    // DO[4] Passive Cable VDO: 50V/5A EPR capable, PD3.0+, USB 2.0
    // Encoding: 0x116A2640
    //   [31:29] VDO ver = 001 (1.3)
    //   [28:27] connector = 10 (USB-C)
    //   [26]    EPR mode capable = 1
    //   [25:24] latency = 00 (<10ns)
    //   [23:21] = 001 (reserved/latency)
    //   [20:19] VCONN power = 01 (1W)
    //   [18]    VCONN req = 0 (no)
    //   [17:16] VBUS through = 10
    //   [15:13] SOP'' present = 000 (no)
    //   [12:11] USB type = 01 (USB 2.0)
    //   [10:9]  max VBUS voltage = 11 (50V)
    //   [8:5]   current = 010 (5A) + EPR bits
    //   [4:0]   USB speed = 00000 (USB 2.0)
    uint32_t cable_vdo = 0x116A2640;
    memcpy(&tx[18], &cable_vdo, 4);

    return 22;
}
