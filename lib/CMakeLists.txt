# Copyright 2011,2012,2016 Free Software Foundation, Inc.
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
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX
include_directories(${OSC_INCLUDE_DIRS} ${VOLK_INCLUDE_DIRS})
link_directories(${OSC_LIBRARY_DIRS} ${VOLK_LIBRARIES})

list(APPEND quantum_sources
  controlled_sig_source.cc
  gate.cc
  qubit.cc
  gates_X_impl.cc
  gates_Y_impl.cc
  gates_Z_impl.cc
  gates_H_impl.cc
  gates_T_impl.cc
  gates_S_impl.cc
  gates_CNOT_impl.cc
  gates_junction_impl.cc
  controllers_Coprocessor_impl.cc
  controllers_Analyzer_impl.cc
  controllers_Generator_impl.cc
  controllers_Initializer_impl.cc
  controllers_readout_impl.cc
  controllers_wait_impl.cc
  controllers_sync_impl.cc
  controllers_gatesParams_impl.cc
  controllers_callback_message_impl.cc
  measurements_Detector_impl.cc
  API_OpenQL_Source_impl.cc
  API_OpenQL_Sink_impl.cc
)


set(quantum_sources "${quantum_sources}" PARENT_SCOPE)
if(NOT quantum_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT quantum_sources)

set(GR_REQUIRED_COMPONENTS RUNTIME BLOCKS PMT FILTER DIGITAL ANALOG FFT)
add_library(gnuradio-quantum SHARED ${quantum_sources})

target_include_directories(
    gnuradio-quantum
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
    ${GNURADIO_ALL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OSC_INCLUDE_DIRS}
)
target_link_libraries(
    gnuradio-quantum
    ${Boost_LIBRARIES}
    ${OSC_LIBRARIES}
    ${VOLK_LIBRARIES}
    ${GNURADIO_RUNTIME_LIBRARIES}
    ${GNURADIO_ALL_LIBRARIES}
    gnuradio-runtime
    gnuradio-blocks
    gnuradio-filter
    gnuradio-fft
    gnuradio-analog
    gnuradio-digital
    gnuradio-pmt
)


set_target_properties(gnuradio-quantum PROPERTIES DEFINE_SYMBOL "gnuradio_quantum_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-quantum PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
#GR_LIBRARY_FOO(gnuradio-quantum)
GR_LIBRARY_FOO(gnuradio-quantum RUNTIME_COMPONENT "quantum_runtime" DEVEL_COMPONENT "quantum_devel")

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")


########################################################################
# Build and register unit test
########################################################################
include(GrTest)

# If your unit tests require special include paths, add them here
#include_directories()
# List all files that contain Boost.UTF unit tests here
list(APPEND test_quantum_sources
)
# Anything we need to link to for the unit tests go here
list(APPEND GR_TEST_TARGET_DEPS gnuradio-quantum)

if(NOT test_quantum_sources)
    MESSAGE(STATUS "No C++ unit tests... skipping")
    return()
endif(NOT test_quantum_sources)

foreach(qa_file ${test_quantum_sources})
    GR_ADD_CPP_TEST("quantum_${qa_file}"
        ${CMAKE_CURRENT_SOURCE_DIR}/${qa_file}
    )
endforeach(qa_file)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

