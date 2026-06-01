// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#pragma once
// AppDPM — minimal DPM subclass for EPR sink.
// Overrides get_epr_watts() to declare 140W EPR capability.
// All other logic (PDO matching, RDO building) uses pdsink defaults.

#include "pd/dpm.h"

class AppDPM : public pd::DPM {
public:
    using pd::DPM::DPM;  // Inherit Port& constructor

    uint32_t get_epr_watts() override { return 140; }
};
