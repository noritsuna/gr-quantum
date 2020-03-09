# Copyright 2011 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Include python install macros
########################################################################
include(GrPython)
if(NOT PYTHONINTERP_FOUND)
    return()
endif()

########################################################################
# Install python sources
########################################################################
GR_PYTHON_INSTALL(
    FILES
    __init__.py
    quantum_qubit_param_type.py
    quantum_qubit_RO_state_type.py
    quantum_gate_type.py
    quantum_gate_param_type.py
    quantum_controllers_QuTiP_Simulator.py
    quantum_viewer_QuTiP_Bloch.py
    quantum_measurements_QuTiP_Wave2Qob.py
    DESTINATION ${GR_PYTHON_DIR}/quantum
)

########################################################################
# Handle the unit tests
########################################################################
include(GrTest)

set(GR_TEST_TARGET_DEPS gnuradio-quantum)
set(GR_TEST_PYTHON_DIRS ${CMAKE_BINARY_DIR}/swig)
GR_ADD_TEST(qa_quantum_controllers_Coprocessor ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_controllers_Coprocessor.py)
GR_ADD_TEST(qa_quantum_controllers_Initializer ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_controllers_Initializer.py)
GR_ADD_TEST(qa_quantum_controllers_readout ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_controllers_readout.py)
GR_ADD_TEST(qa_quantum_controllers_Simulator ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_controllers_Simulator.py)
GR_ADD_TEST(qa_quantum_controllers_sync ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_controllers_sync.py)
GR_ADD_TEST(qa_quantum_controllers_wait ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_controllers_wait.py)
GR_ADD_TEST(qa_quantum_gates_CNOT ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_CNOT.py)
GR_ADD_TEST(qa_quantum_gates_H ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_H.py)
GR_ADD_TEST(qa_quantum_gates_junction ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_junction.py)
GR_ADD_TEST(qa_quantum_gates_S ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_S.py)
GR_ADD_TEST(qa_quantum_gates_T ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_T.py)
GR_ADD_TEST(qa_quantum_gates_X ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_X.py)
GR_ADD_TEST(qa_quantum_gates_Y ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_Y.py)
GR_ADD_TEST(qa_quantum_gates_Z ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_gates_Z.py)
GR_ADD_TEST(qa_quantum_measurements_Detector ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_quantum_measurements_Detector.py)
