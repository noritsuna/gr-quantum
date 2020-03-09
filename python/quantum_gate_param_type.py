"""
/* -*- Python -*- */
/*
 * Copyright (c) 2006-2019 SIProp.org Noritsuna Imamura(noritsuna@siprop.org)
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
"""
from enum import IntEnum

class quantum_gate_param_type(IntEnum):
    GATE_TYPE = 1
    FREQ = 2
    I_AMP = 3
    Q_AMP = 4
    I_BW = 5
    Q_BW = 6
    PROC_TIME = 7
    CTRL_DC_MODE = 8
    QUBIT_ID = 9
    WAIT_TIME = 10
