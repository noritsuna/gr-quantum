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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_GENERATOR_IMPL_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_GENERATOR_IMPL_H

#include "gate.h"
#include "qubit.h"
#include <quantum/controllers_Generator.h>
#include <map> 
#include <queue> 
#include <vector> 
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/fft_vfc.h>
#include <boost/timer/timer.hpp>
#include <gnuradio/logger.h>
#include "controlled_sig_source.h"
#include <quantum/controllers_callback_message.h>

namespace gr {
  namespace quantum {

    class controllers_Generator_impl : public controllers_Generator
    {
    private:

      double d_qubit_num;
      double d_samps_per_us;
      double d_time_scale_rate;
      bool d_SYNC_port;
      const pmt::pmt_t d_port_in;
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_SIM_in;
      const pmt::pmt_t d_port_SYNC_in;

      bool d_is_nanosec_mode;
      boost::mutex sleep_mutex;
      boost::condition_variable sleep_condition;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

      boost::timer::cpu_timer d_proc_rate_timer;

      struct RO_wave_synthesizer_t {
        analog::sig_source_f::sptr readout_I_src_base;
        analog::sig_source_f::sptr readout_Q_src_base;
        analog::sig_source_f::sptr readout_I_src_bw;
        analog::sig_source_f::sptr readout_Q_src_bw;
        blocks::add_ff::sptr readout_I_src_add;
        blocks::add_ff::sptr readout_Q_src_add;
        blocks::float_to_complex::sptr readout_IQ_mixer;
      };

      controllers_callback_message::sptr callback_message_port;

#define QUANTUM_QUBID_ID int
      std::map<QUANTUM_QUBID_ID, RO_wave_synthesizer_t> d_RO_wave_synthesizers;
      std::map<QUANTUM_QUBID_ID, gate::gate_param_t> d_RO_gate_params;
            

      void set_params_to_list(int qubit_id, gate::gate_type_t type, pmt::pmt_t gate);
      void copy_gate_param_t(gate::gate_param_t* copyto, gate::gate_param_t* orig);
      void proccessing_wait(long wait_time_ns);
      bool start_waveform(int qubit_id, gate::gate_param_t gate_params);
      bool change_waveform(int qubit_id, gate::gate_param_t gate_params, float angle);
      bool stop_waveform(int qubit_id, gate::gate_param_t gate_params);
      double deg2rad(double degree);


    public:
      controllers_Generator_impl(double qubit_num, double samples_per_sec, double time_scale_rate, bool show_SYNC_port=true);
      ~controllers_Generator_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void set_qubit_num(double qubit_num);
      double qubit_num();

      void set_sample_rate(double rate);
      double sample_rate();

      void set_time_scale_rate(double time_scale_rate);
      double time_scale_rate();

      void set_SYNC_port(bool is_use);
      bool SYNC_port() const;

      void handle_cmd_msg(pmt::pmt_t msg);
      void handle_simurated_msg(pmt::pmt_t msg);
      void handle_SYNC_msg(pmt::pmt_t msg);

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_GENERATOR_IMPL_H */
