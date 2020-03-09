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

from quantum import quantum_gate_param_type
from quantum import quantum_gate_type
from quantum import quantum_qubit_param_type
from quantum import quantum_qubit_RO_state_type


class quantum_viewer_QuTiP_Bloch(gr.basic_block):

    _plot_ptr = None
    _is_start_plot_thread = False
    _block_view = None
    _qubit_id = 0


    _gate = None
    _flag_x = False
    _flag_y = False
    _flag_z = False
    _flag_draw = False

    def __init__(self, qubit_id):
        gr.basic_block.__init__(self,
            "quantum_viewer_QuTiP_Bloch",
            gr.py_io_signature(0, 0, np.array([np.float64])),
            gr.py_io_signature(0, 0, np.array([np.float64])))
        self.message_port_register_in(pmt.intern('result'))
        self.set_msg_handler(pmt.intern('result'), self.msg_handler_QOBJ_in)
        self._qubit_id = qubit_id

    def serve_forever(self):
        if not self._is_start_plot_thread:
            self._block_view = Bloch()

            e_ops = [sigmax(), sigmay(), sigmaz()]
            Z = sigmaz()
            self._block_view.add_vectors(expect(Z.unit(), e_ops))


            self._block_view.make_sphere()
            self._plot_ptr = plt
            self._plot_ptr.show()
            self._is_start_plot_thread = True

    def angle_converter(qubit_angle):
        pi = 3.141592653589793
        q_1 = qubit_angle / pi
        q = 1 / (q_1*32.0)
        return q

    def msg_handler_QOBJ_in(self, msg):
        print("in msg_handler_QOBJ_in")
        self.lock()
        qubit_id_PMT = pmt.dict_ref(msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ID), pmt.PMT_NIL)
        if(pmt.eq(qubit_id_PMT, pmt.PMT_NIL)):
            return
        qubit_id = pmt.to_float(qubit_id_PMT)
        print("quantum_qubit_param_type.qubit_id=" + str(qubit_id))
        if(this._qubit_id != qubit_id) :
            return

        qubit_angle_PMT = pmt.dict_ref(msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.ANGLE), pmt.PMT_NIL)
        if(pmt.eq(qubit_angle_PMT, pmt.PMT_NIL)):
            return
        qubit_angle = pmt.to_float(qubit_angle_PMT)
        print("quantum_qubit_param_type.qubit_angle=" + str(qubit_angle))

        qubit_pole_PMT = pmt.dict_ref(msg, pmt.from_float(quantum_qubit_param_type.quantum_qubit_param_type.POLE), pmt.PMT_NIL)
        if(pmt.eq(qubit_pole_PMT, pmt.PMT_NIL)):
            return
        qubit_pole = pmt.to_float(qubit_pole_PMT)
        print("quantum_qubit_param_type.qubit_pole=" + str(qubit_pole))

        e_ops = [sigmax(), sigmay(), sigmaz()]
        if(qubit_pole == 1.0) :
            Z = sigmaz()
        else :
            Z = -sigmaz()

        if(qubit_angle == 0.0) :
            Q = Z
        elif(qubit_angle > 0.0) :
            Q = sigmax() * self.angle_converter(qubit_angle)
            Q = Q + Z
        elif() :
            Q = sigmax() * self.angle_converter(-qubit_angle)
            Q = Q + (-Z)

#        self._block_view.clear()
        self._block_view.add_vectors(expect(Q.unit(), e_ops))
        self._block_view.make_sphere()
        self._block_view.show()



    def msg_handler_QOBJ_in_demo(self, msg):
#https://wiki.gnuradio.org/index.php/Guided_Tutorial_Programming_Topics
#https://wiki.gnuradio.org/index.php/Polymorphic_Types_(PMTs)


        if(str(msg) == "TrueFalseFalse"):
            if(self._flag_x == True):
                return True

            if(self._gate == None):
                self._gate = sigmax()
            else:
                self._gate = self._gate + sigmax()
            self._flag_x = True
            self._flag_draw = True
        elif (str(msg) == "TrueTrueFalse"):
            if(self._flag_y == True):
                return True

            if(self._gate == None):
                self._gate = sigmay()
            else:
                self._gate = self._gate + sigmay()
            self._flag_y = True
            self._flag_draw = True
        elif (str(msg) == "TrueTrueTrue"):
            if(self._flag_z == True):
                return True

            if(self._gate == None):
                self._gate = sigmaz()
            else:
                self._gate = self._gate + sigmaz()
            self._flag_z = True
            self._flag_draw = True

        if(self._flag_draw == False):
            return True

        e_ops = [sigmax(), sigmay(), sigmaz()]
        self._block_view.clear()
        self._block_view.add_vectors(expect(self._gate.unit(), e_ops))
        self._block_view.make_sphere()
        self._block_view.show()
        self._flag_draw = False


    def stop(self):
        if self._is_start_plot_thread:
            self._plot_ptr.close()
            self._is_start_plot_thread = False
        return True


