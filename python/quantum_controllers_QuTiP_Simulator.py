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
import sys, math
from enum import IntEnum
from time import sleep

try:
    from qutip import *
except ImportError:
    sys.stderr.write('quantum_controllers_QuTiP_Simulator.py required QuTiP.\n')
    sys.exit(1)

import numpy as np
import scipy.sparse as sp
import scipy.linalg as la
import qutip.settings as settings

from quantum import quantum_gate_param_type
from quantum import quantum_gate_type
from quantum import quantum_qubit_param_type
from quantum import quantum_qubit_RO_state_type


class quantum_qubit_state_struct:
    def __init__(self):
        self.state = quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.END
        self.angle = 0.0
        self.circuit = 0
        self.qubit_pole = 0

class quantum_controllers_QuTiP_Simulator(gr.basic_block):
    _qubit_num = 0
    _qubit_noise_rate = 0.0
    _scale_rate = 0.0

    _qubit_stat_map = {}

    _qbuit_proc_time_ns = 2.0

    _qubit_circuit = None
    _qcircuit_index_cnt = 0

    def __init__(self, qubit_num, qubit_noise_rate, scale_rate):
        gr.basic_block.__init__(self,
            "quantum_controllers_QuTiP_Simulator",
            gr.py_io_signature(0, 0, np.array([np.float64])),
            gr.py_io_signature(0, 0, np.array([np.float64])))

        self._qubit_num = qubit_num
        self._qubit_noise_rate = qubit_noise_rate
        self._scale_rate = scale_rate

        self._qubit_circuit = QubitCircuit(qubit_num, reverse_states=False)

        for id in range(qubit_num):
            self._qubit_stat_map[id+1] = quantum_qubit_state_struct()
        

        self.message_port_register_in(pmt.intern('analyzed_data'))
        self.message_port_register_in(pmt.intern('in'))
        self.set_msg_handler(pmt.intern('analyzed_data'), self.msg_handler_analyzed_data_in)
        self.set_msg_handler(pmt.intern('in'), self.msg_handler_cmd_in)
        self.message_port_register_out(pmt.intern('simulated_data'))
        self.message_port_register_out(pmt.intern('out'))

    def get_qbuit_proc_time(self, qbuit_proc_time):
        return qbuit_proc_time/1000000000 * self._scale_rate


    def serve_forever(self):
        while(True) :
            for qubit_id in self._qubit_stat_map.keys():
                RO_STATE = self._qubit_stat_map[qubit_id]

                if(RO_STATE.state == quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.END) :
                    sleep(self.get_qbuit_proc_time(0.5))
                    continue
                elif(RO_STATE.state == quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.START) :
                    print("in START loop")
                    sleep(self.get_qbuit_proc_time(self._qbuit_proc_time_ns))

                    SIM_msg = pmt.make_dict()
                    SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ID), pmt.from_float(qubit_id))
                    SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ANGLE), pmt.from_float(float(RO_STATE.angle)))
                    SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.STATE), pmt.from_float(quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.PROC))
                    self.message_port_pub(pmt.intern('simulated_data'), SIM_msg)

                    sleep(self.get_qbuit_proc_time(self._qbuit_proc_time_ns))
                    RO_STATE.state = quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.END
                    self._qubit_stat_map[qubit_id] = RO_STATE
                    print("in START send qubit_id=" + str(qubit_id) + " angle=" + str(RO_STATE.angle) + " state=" + str(RO_STATE.state))
                    continue
                elif(RO_STATE == quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.PROC) :
                    print("in PROC loop")
                    SIM_msg = pmt.make_dict()
                    SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ID), pmt.from_float(qubit_id))
                    SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ANGLE), pmt.from_float(float(RO_STATE.angle)))
                    SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.STATE), pmt.from_float(quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.END))
                    self.message_port_pub(pmt.intern('simulated_data'), SIM_msg)
                    RO_STATE.state = quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.END
                    self._qubit_stat_map[qubit_id] = RO_STATE
                    print("in PROC send qubit_id=" + str(qubit_id) + " angle=" + str(RO_STATE.angle) + " state=" + str(RO_STATE.state))
                    continue

    def stop(self):
        return True

    def msg_handler_analyzed_data_in(self, msg):
        print("in msg_handler_analyzed_data_in")
        self.lock()
        gate_params = pmt.vector_ref(msg, 0)
        gate_type_PMT = pmt.dict_ref(gate_params, pmt.from_float(quantum_gate_param_type.quantum_gate_param_type.GATE_TYPE), pmt.PMT_NIL)
        if(pmt.eq(gate_type_PMT, pmt.PMT_NIL )):
            return
        
        gate_type = pmt.to_float(gate_type_PMT)
        print("gate_params.gate_type=" + str(gate_type))

        qubit_id_PMT = pmt.dict_ref(gate_params, pmt.from_float(quantum_gate_param_type.quantum_gate_param_type.QUBIT_ID), pmt.PMT_NIL)
        if(pmt.eq(qubit_id_PMT, pmt.PMT_NIL )):
            return
        qubit_id = pmt.to_float(qubit_id_PMT)
        print("gate_params.qubit_id=" + str(qubit_id))

        if(gate_type == quantum_gate_type.quantum_gate_type.X):
            print("in msg_handler_analyzed_data_in X gate")
            #回路を作る
            RO_STATE = self._qubit_stat_map[qubit_id]
            if(float(RO_STATE.angle) == 0.0) :
                RO_STATE.angle = 180.0
            else :
                RO_STATE.angle = 0.0
            self._qubit_stat_map[qubit_id] = RO_STATE
        elif(gate_type == quantum_gate_type.quantum_gate_type.RO):
            print("in msg_handler_analyzed_data_in RO")
            #回路を実行する
            RO_STATE = self._qubit_stat_map[qubit_id]
            SIM_msg = pmt.make_dict()
            SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ID), pmt.from_float(qubit_id))
            SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ANGLE), pmt.from_float(float(RO_STATE.angle)))
            SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.STATE), pmt.from_float(quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.START))
            self.message_port_pub(pmt.intern('simulated_data'), SIM_msg)
            RO_STATE.state = quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.START
            self._qubit_stat_map[qubit_id] = RO_STATE
        self.unlock()


    def msg_handler_analyzed_data_in_circuit(self, msg):
        print("in msg_handler_analyzed_data_in")
        self.lock()
        gate_params = pmt.vector_ref(msg, 0)
        gate_type_PMT = pmt.dict_ref(gate_params, pmt.from_float(quantum_gate_param_type.quantum_gate_param_type.GATE_TYPE), pmt.PMT_NIL)
        if(pmt.eq(gate_type_PMT, pmt.PMT_NIL )):
            return
        
        gate_type = pmt.to_float(gate_type_PMT)
        print("gate_params.gate_type=" + str(gate_type))

        qubit_id_PMT = pmt.dict_ref(gate_params, pmt.from_float(quantum_gate_param_type.quantum_gate_param_type.QUBIT_ID), pmt.PMT_NIL)
        if(pmt.eq(qubit_id_PMT, pmt.PMT_NIL )):
            return
        qubit_id = pmt.to_float(qubit_id_PMT)
        print("gate_params.qubit_id=" + str(qubit_id))

        if(gate_type == quantum_gate_type.quantum_gate_type.X):
            print("in msg_handler_analyzed_data_in X gate")
            self._qubit_circuit.add_gate("RX", targets=qubit_id, arg_value=pi, index=this._qcircuit_index_cnt)

        elif(gate_type == quantum_gate_type.quantum_gate_type.Y):
            print("in msg_handler_analyzed_data_in Y gate")
            self._qubit_circuit.add_gate("RY", targets=qubit_id, arg_value=pi, index=this._qcircuit_index_cnt)

        elif(gate_type == quantum_gate_type.quantum_gate_type.Z):
            print("in msg_handler_analyzed_data_in Z gate")
            self._qubit_circuit.add_gate("RZ", targets=qubit_id, arg_value=pi, index=this._qcircuit_index_cnt)

        elif(gate_type == quantum_gate_type.quantum_gate_type.H):
            print("in msg_handler_analyzed_data_in H gate")
            self._qubit_circuit.add_gate("hadamard_transform", targets=qubit_id, index=this._qcircuit_index_cnt)

        elif(gate_type == quantum_gate_type.quantum_gate_type.S):
            print("in msg_handler_analyzed_data_in S gate")
            self._qubit_circuit.add_gate("RZ", targets=qubit_id, arg_value=pi/2, index=this._qcircuit_index_cnt)

        elif(gate_type == quantum_gate_type.quantum_gate_type.T):
            print("in msg_handler_analyzed_data_in T gate")
            self._qubit_circuit.add_gate("RZ", targets=qubit_id, arg_value=pi/4, index=this._qcircuit_index_cnt)

        elif(gate_type == quantum_gate_type.quantum_gate_type.INIT):
            print("in msg_handler_analyzed_data_in INIT gate")
            pass

        elif(gate_type == quantum_gate_type.quantum_gate_type.CNOT):
            print("in msg_handler_analyzed_data_in CNOT gate")
            self._qubit_circuit.add_gate("CNOT", controls=[0], targets=[1])

        elif(gate_type == quantum_gate_type.quantum_gate_type.JUNC):
            print("in msg_handler_analyzed_data_in JUNC gate")
            pass

        elif(gate_type == quantum_gate_type.quantum_gate_type.RO):
            print("in msg_handler_analyzed_data_in RO")
            #回路を実行する
            RO_STATE = self._qubit_stat_map[qubit_id]
            SIM_msg = pmt.make_dict()
            SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ID), pmt.from_float(qubit_id))
            SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ANGLE), pmt.from_float(float(RO_STATE.angle)))
            SIM_msg = pmt.dict_add(SIM_msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.STATE), pmt.from_float(quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.START))
            self.message_port_pub(pmt.intern('simulated_data'), SIM_msg)
            RO_STATE.state = quantum_qubit_RO_state_type.quantum_qubit_RO_state_type.START
            self._qubit_stat_map[qubit_id] = RO_STATE
        self.unlock()


    def msg_handler_cmd_in(self, msg):
#https://wiki.gnuradio.org/index.php/Guided_Tutorial_Programming_Topics
#https://wiki.gnuradio.org/index.php/Polymorphic_Types_(PMTs)
        self.message_port_pub(pmt.intern('out'), msg)
3\