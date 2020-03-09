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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_COPROSESSOR_IMPL_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_COPROSESSOR_IMPL_H

#include <gnuradio/top_block.h>
#include "gate.h"
#include <quantum/controllers_Coprocessor.h>
#include <quantum/controllers_callback_message.h>
#include <gnuradio/logger.h>
#include <boost/timer/timer.hpp>

namespace gr {
  namespace quantum {

    class controllers_Coprocessor_impl : public controllers_Coprocessor
    {
    private:
      double d_qubit_id;
      double d_samps_per_tick, d_samps_per_us;
      double d_time_scale_rate;
      bool d_SYNC_port;
      bool d_is_use_gates_params_block;
      bool d_first_INIT;
      bool d_is_nanosec_mode;
      double d_SYNC_clock_time;
      double d_gates_proc_total_time;
      boost::timer::cpu_timer d_proc_timer;

      const pmt::pmt_t d_port_in;
      const pmt::pmt_t d_port_SYNC_in;
      const pmt::pmt_t d_port_feedback_in;
      const pmt::pmt_t d_port_in_intern;
      const pmt::pmt_t d_port_SYNC_in_intern;
      const pmt::pmt_t d_port_feedback_in_intern;
      const pmt::pmt_t d_port_out;
      pmt::pmt_t INIT;
      pmt::pmt_t RO;
      pmt::pmt_t X_gate;
      pmt::pmt_t Y_gate;
      pmt::pmt_t Z_gate;
      pmt::pmt_t H_gate;
      pmt::pmt_t T_gate;
      pmt::pmt_t S_gate;
      pmt::pmt_t CNOT;
      pmt::pmt_t JUNC;
      pmt::pmt_t WAIT;
      analog::sig_source_f::sptr gate_I_src_base;
      analog::sig_source_f::sptr gate_Q_src_base;
      analog::sig_source_f::sptr gate_I_src_bw;
      analog::sig_source_f::sptr gate_Q_src_bw;
      blocks::add_ff::sptr gate_I_src_add;
      blocks::add_ff::sptr gate_Q_src_add;
      blocks::float_to_complex::sptr gate_IQ_mixer;
      analog::sig_source_f::sptr ctrl_I_src_base;
      analog::sig_source_f::sptr ctrl_Q_src_base;
      analog::sig_source_f::sptr ctrl_I_src_bw;
      analog::sig_source_f::sptr ctrl_Q_src_bw;
      blocks::add_ff::sptr ctrl_I_src_add;
      blocks::add_ff::sptr ctrl_Q_src_add;
      blocks::float_to_complex::sptr ctrl_IQ_mixer;
      analog::sig_source_f::sptr readout_I_src_base;
      analog::sig_source_f::sptr readout_Q_src_base;
      analog::sig_source_f::sptr readout_I_src_bw;
      analog::sig_source_f::sptr readout_Q_src_bw;
      blocks::add_ff::sptr readout_I_src_add;
      blocks::add_ff::sptr readout_Q_src_add;
      blocks::float_to_complex::sptr readout_IQ_mixer;
      controllers_callback_message::sptr callback_message_port;

      boost::mutex sleep_mutex;
      boost::condition_variable sleep_condition;
      gr::thread::mutex d_cmd_mutex;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;



      void proccessing_wait(long wait_time_ns);
      long calc_wait_time(long wait_time);
      bool send_waveform(pmt::pmt_t cmd_gate);
      bool exec_interpreter(pmt::pmt_t msg);
      bool is_use_gates_params_block();
      void set_is_use_gates_params_block(bool flag);
      void set_gates_by_block(pmt::pmt_t gates_params, pmt::pmt_t send_msg);

    public:
      controllers_Coprocessor_impl(double qubit_id, double samples_per_sec, double time_scale_rate, bool show_SYNC_port=true);
      ~controllers_Coprocessor_impl();

//      // Overloading gr::block::start to reset timer
//      bool start();

      void set_qubit_id(double qubit_id);
      double qubit_id();

      void set_sample_rate(double rate);
      double sample_rate();

      void set_time_scale_rate(double time_scale_rate);
      double time_scale_rate();


      void set_SYNC_port(bool is_use);
      bool SYNC_port() const;

      void set_first_INIT(bool first_INIT);
      bool first_INIT();

      void handle_cmd_msg(pmt::pmt_t msg);
      void handle_SYNC_msg(pmt::pmt_t msg);
      void handle_FEEDBACK_msg(pmt::pmt_t msg);
/*TODO
      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
*/
    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLER_COPROSESSORS_IMPL_H */
