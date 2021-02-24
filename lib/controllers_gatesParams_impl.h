/* -*- c++ -*- */
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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_GATESPARAMS_IMPL_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_GATESPARAMS_IMPL_H

#include <gnuradio/top_block.h>
#include "gate.h"
#include "gates_junction_impl.h"
#include <quantum/controllers_gatesParams.h>
#include <gnuradio/logger.h>

namespace gr {
  namespace quantum {

    class controllers_gatesParams_impl : public controllers_gatesParams
    {
    private:
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_in;
      
      double d_qubit_id;
      gate::wave_type_t d_wave_type;
      gate *INIT;
      gate *RO;
      gate *X_gate;
      gate *Y_gate;
      gate *Z_gate;
      gate *H_gate;
      gate *T_gate;
      gate *S_gate;
      gate *JUNC;
      gate *CNOT;

      pmt::pmt_t d_gate_msg;
      pmt::pmt_t d_qubit_msg;
      pmt::pmt_t d_send_msg;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

    public:
      controllers_gatesParams_impl(double qubit_id, std::string wave_type);
      ~controllers_gatesParams_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void set_qubit_id(double qubit_id);
      double qubit_id();
      void set_wave_type(std::string wave_type);
      gate::wave_type_t wave_type();

      void set_INIT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_RO_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);

      void set_X_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_Y_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_Z_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_H_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_T_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_S_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type);
      void set_CNOT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type, bool ctrl_dc_mode);
      void set_JUNC_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type, bool ctrl_dc_mode);

      void handle_cmd_msg(pmt::pmt_t msg);
    };
  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_GATESPARAMS_IMPL_H */
