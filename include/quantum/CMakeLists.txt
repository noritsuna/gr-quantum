# Copyright 2011,2012 Free Software Foundation, Inc.
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
# Install public header files
########################################################################
install(FILES
    api.h
    gates_X.h
    gates_Y.h
    gates_Z.h
    gates_H.h
    gates_T.h
    gates_S.h
    gates_CNOT.h
    gates_junction.h
    controllers_Coprocessor.h
    controllers_Analyzer.h
    controllers_Generator.h
    controllers_Initializer.h
    controllers_gatesParams.h
    controllers_readout.h
    controllers_wait.h
    controllers_sync.h
    controllers_callback_message.h
    measurements_Detector.h
    API_OpenQL_Source.h
    API_OpenQL_Sink.h
    DESTINATION include/quantum
)
