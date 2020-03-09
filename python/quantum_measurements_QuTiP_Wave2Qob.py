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
from __future__ import division
from __future__ import unicode_literals

from gnuradio import gr
from gnuradio import blocks
import pmt

import matplotlib.pyplot as plt
import numpy as np
from qutip import *
import numpy as np
import scipy.sparse as sp
import scipy.linalg as la
import qutip.settings as settings
import sys, math
import threading


class quantum_measurements_QuTiP_Wave2Qob(gr.basic_block):


    def __init__(self, qubit_num):
        gr.basic_block.__init__(self,
            "quantum_measurements_QuTiP_Wave2Qob",
            gr.py_io_signature(0, 0, np.array([np.float64])),
            gr.py_io_signature(0, 0, np.array([np.float64])))
        pass

