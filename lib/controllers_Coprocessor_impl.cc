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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "controllers_Coprocessor_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/blocks/null_sink.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    controllers_Coprocessor::sptr
    controllers_Coprocessor::make(double qubit_id, double samples_per_sec, double time_scale_rate, bool show_SYNC_port)
    {
      return gnuradio::get_initial_sptr
        (new controllers_Coprocessor_impl(qubit_id, samples_per_sec, time_scale_rate, show_SYNC_port));
    }

    controllers_Coprocessor_impl::controllers_Coprocessor_impl(double qubit_id, double samples_per_sec, double time_scale_rate, bool show_SYNC_port)
      : hier_block2("controllers_Coprocessor",
                      io_signature::make(0, 0, 0),
                      io_signature::make(3, 3, sizeof(gr_complex))),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in")),
      d_port_SYNC_in(pmt::mp("SYNC_CLK_in")),
      d_port_feedback_in(pmt::mp("feedback_in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Coprocessor");

      set_qubit_id(qubit_id);
      set_sample_rate(samples_per_sec);
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      set_time_scale_rate(time_scale_rate);
      d_is_nanosec_mode = true;
#else
      set_time_scale_rate(time_scale_rate/1000);
      d_is_nanosec_mode = false;
#endif
      set_SYNC_port(show_SYNC_port);
      set_is_use_gates_params_block(false);
      set_first_INIT(false);
      d_SYNC_clock_time = 0.0;
      d_gates_proc_total_time = 0.0;
      d_proc_timer.start();

      gate_I_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      gate_Q_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      gate_I_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      gate_Q_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      ctrl_I_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      ctrl_Q_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      ctrl_I_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      ctrl_Q_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      readout_I_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      readout_Q_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      readout_I_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);
      readout_Q_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_CONST_WAVE, 1, 0.0, 0.0, 0.0);

      gate_I_src_add = blocks::add_ff::make(1);
      gate_Q_src_add = blocks::add_ff::make(1);
      ctrl_I_src_add = blocks::add_ff::make(1);
      ctrl_Q_src_add = blocks::add_ff::make(1);
      readout_I_src_add = blocks::add_ff::make(1);
      readout_Q_src_add = blocks::add_ff::make(1);

      gate_IQ_mixer = blocks::float_to_complex::make(1);
      ctrl_IQ_mixer = blocks::float_to_complex::make(1);
      readout_IQ_mixer = blocks::float_to_complex::make(1);

      callback_message_port = controllers_callback_message::make();

      connect(gate_I_src_base, 0, gate_I_src_add, 0);
      connect(gate_I_src_bw, 0, gate_I_src_add, 1);
      connect(gate_Q_src_base, 0, gate_Q_src_add, 0);
      connect(gate_Q_src_bw, 0, gate_Q_src_add, 1);
      connect(ctrl_I_src_base, 0, ctrl_I_src_add, 0);
      connect(ctrl_I_src_bw, 0, ctrl_I_src_add, 1);
      connect(ctrl_Q_src_base, 0, ctrl_Q_src_add, 0);
      connect(ctrl_Q_src_bw, 0, ctrl_Q_src_add, 1);
      connect(readout_I_src_base, 0, readout_I_src_add, 0);
      connect(readout_I_src_bw, 0, readout_I_src_add, 1);
      connect(readout_Q_src_base, 0, readout_Q_src_add, 0);
      connect(readout_Q_src_bw, 0, readout_Q_src_add, 1);

      connect(gate_I_src_add, 0, gate_IQ_mixer, 0);
      connect(gate_Q_src_add, 0, gate_IQ_mixer, 1);
      connect(ctrl_I_src_add, 0, ctrl_IQ_mixer, 0);
      connect(ctrl_Q_src_add, 0, ctrl_IQ_mixer, 1);
      connect(readout_I_src_add, 0, readout_IQ_mixer, 0);
      connect(readout_Q_src_add, 0, readout_IQ_mixer, 1);

      connect(gate_IQ_mixer, 0, self(), 0);
      connect(ctrl_IQ_mixer, 0, self(), 1);
      connect(readout_IQ_mixer, 0, self(), 2);


      message_port_register_hier_in(d_port_in);
      message_port_register_hier_in(d_port_SYNC_in);
      message_port_register_hier_in(d_port_feedback_in);

      callback_message_port->set_msg_handler(d_port_in, boost::bind(&controllers_Coprocessor_impl::handle_cmd_msg, this, _1));
      callback_message_port->set_msg_handler(d_port_SYNC_in, boost::bind(&controllers_Coprocessor_impl::handle_SYNC_msg, this, _1));
      callback_message_port->set_msg_handler(d_port_feedback_in, boost::bind(&controllers_Coprocessor_impl::handle_FEEDBACK_msg, this, _1));

      msg_connect(self(), d_port_in, callback_message_port, d_port_in);
      msg_connect(self(), d_port_SYNC_in, callback_message_port, d_port_SYNC_in);
      msg_connect(self(), d_port_feedback_in, callback_message_port, d_port_feedback_in);

      message_port_register_hier_out(d_port_out);
      callback_message_port->message_port_register_out(d_port_out);
      msg_connect(callback_message_port, d_port_out, self(), d_port_out);

    }

    controllers_Coprocessor_impl::~controllers_Coprocessor_impl()
    {
    }

    bool
    controllers_Coprocessor_impl::send_waveform(pmt::pmt_t cmd_gate)
    {
      double freq;
      double freq_min;
      double I_amp;
      double Q_amp;
      double I_bw;
      double Q_bw;
      double proc_time;

      bool CTRL_dc_mode;

      analog::sig_source_f::sptr I_wave_src_base;
      analog::sig_source_f::sptr Q_wave_src_base;
      analog::sig_source_f::sptr I_wave_src_bw;
      analog::sig_source_f::sptr Q_wave_src_bw;


      freq = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::FREQ), pmt::PMT_NIL));
      I_amp = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::I_AMP), pmt::PMT_NIL));
      Q_amp = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::Q_AMP), pmt::PMT_NIL));
      I_bw = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::I_BW), pmt::PMT_NIL));
      Q_bw = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::Q_BW), pmt::PMT_NIL));
      proc_time = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::PROC_TIME), pmt::PMT_NIL));

      CTRL_dc_mode = pmt::to_double(pmt::dict_ref(cmd_gate, pmt::from_float(gate::CTRL_DC_MODE), pmt::PMT_NIL));

      pmt::pmt_t type_value = pmt::dict_ref(cmd_gate, pmt::from_float(gate::GATE_TYPE), pmt::PMT_NIL);
      int type_int = (int)(pmt::to_float(type_value));
      switch(type_int) {
        case gate::X:
        case gate::Y:
        case gate::Z:
        case gate::T:
        case gate::S:
        case gate::H:
        case gate::INIT:
          I_wave_src_base = gate_I_src_base;
          Q_wave_src_base = gate_Q_src_base;
          I_wave_src_bw = gate_I_src_bw;
          Q_wave_src_bw = gate_Q_src_bw;
          break;
        case gate::RO:
          I_wave_src_base = readout_I_src_base;
          Q_wave_src_base = readout_Q_src_base;
          I_wave_src_bw = readout_I_src_bw;
          Q_wave_src_bw = readout_Q_src_bw;
          break;
        case gate::JUNC:
        case gate::CNOT:
          I_wave_src_base = ctrl_I_src_base;
          Q_wave_src_base = ctrl_Q_src_base;
          I_wave_src_bw = ctrl_I_src_bw;
          Q_wave_src_bw = ctrl_Q_src_bw;
          break;
        default:
          break;
        }

      if(CTRL_dc_mode)
      {
        freq = 0.0;
        I_bw = 0.0;
        Q_bw = 0.0;
        I_wave_src_base->set_waveform(analog::GR_CONST_WAVE);
        Q_wave_src_base->set_waveform(analog::GR_CONST_WAVE);
        I_wave_src_bw->set_waveform(analog::GR_CONST_WAVE);
        Q_wave_src_bw->set_waveform(analog::GR_CONST_WAVE);
      } else {
        I_wave_src_base->set_waveform(analog::GR_SIN_WAVE);
        Q_wave_src_base->set_waveform(analog::GR_SIN_WAVE);
        I_wave_src_bw->set_waveform(analog::GR_SIN_WAVE);
        Q_wave_src_bw->set_waveform(analog::GR_SIN_WAVE);
        I_bw = I_bw/2;
        Q_bw = Q_bw/2;
        freq_min = freq - I_bw;
        if(freq_min < 0) freq_min = 0; 
        I_wave_src_base->set_frequency(freq_min);
        freq_min = freq - Q_bw;
        if(freq_min < 0) freq_min = 0; 
        Q_wave_src_base->set_frequency(freq_min);
        I_wave_src_bw->set_frequency(freq+I_bw);
        Q_wave_src_bw->set_frequency(freq+Q_bw);
      }

      I_wave_src_base->set_amplitude(I_amp);
      Q_wave_src_base->set_amplitude(Q_amp);
      I_wave_src_bw->set_amplitude(I_amp);
      Q_wave_src_bw->set_amplitude(Q_amp);

      d_proc_timer.stop();
      proccessing_wait((long)(proc_time * time_scale_rate()));
      d_proc_timer.start();
      d_gates_proc_total_time += proc_time;

      I_wave_src_base->set_waveform(analog::GR_CONST_WAVE);
      Q_wave_src_base->set_waveform(analog::GR_CONST_WAVE);
      I_wave_src_bw->set_waveform(analog::GR_CONST_WAVE);
      Q_wave_src_bw->set_waveform(analog::GR_CONST_WAVE);
      I_wave_src_base->set_amplitude(0.0);
      Q_wave_src_base->set_amplitude(0.0);
      I_wave_src_bw->set_amplitude(0.0);
      Q_wave_src_bw->set_amplitude(0.0);

      return true;
    }
    
    long
    controllers_Coprocessor_impl::calc_wait_time(long wait_time)
    {
      long scaled_gates_proc_total_time = d_gates_proc_total_time * time_scale_rate();
      long real_wait_time = wait_time;
      long real_proc_total_time;
      long diff_real_proc_total_time;
      long diff_SYNC_clock_time;   
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      auto cur_proc_time = boost::chrono::nanoseconds(d_proc_timer.elapsed().wall);
#else
      auto cur_proc_time = boost::chrono::microseconds(d_proc_timer.elapsed().wall);
#endif
      if(!d_is_nanosec_mode) {
         real_proc_total_time = cur_proc_time.count() / 1000;
         wait_time /= 1000;
      } else {
         real_proc_total_time = cur_proc_time.count();
      }
      
      diff_real_proc_total_time = real_proc_total_time - scaled_gates_proc_total_time;
      real_wait_time -= diff_real_proc_total_time;

      if(SYNC_port() && d_SYNC_clock_time == 0.0) {
        diff_SYNC_clock_time = d_SYNC_clock_time - real_proc_total_time;
        real_wait_time += diff_SYNC_clock_time;
      }

      return real_wait_time;
    }

    void
    controllers_Coprocessor_impl::set_is_use_gates_params_block(bool flag)
    {
      d_is_use_gates_params_block = flag;
    }

    bool
    controllers_Coprocessor_impl::is_use_gates_params_block()
    {
      return d_is_use_gates_params_block;
    }

    void
    controllers_Coprocessor_impl::set_gates_by_block(pmt::pmt_t gates_params, pmt::pmt_t send_msg)
    {
      int cnt = 0;
      INIT = pmt::dict_ref(gates_params, pmt::from_float(gate::INIT), pmt::PMT_NIL);
      RO = pmt::dict_ref(gates_params, pmt::from_float(gate::RO), pmt::PMT_NIL);
      X_gate = pmt::dict_ref(gates_params, pmt::from_float(gate::X), pmt::PMT_NIL);
      Y_gate = pmt::dict_ref(gates_params, pmt::from_float(gate::Y), pmt::PMT_NIL);
      Z_gate = pmt::dict_ref(gates_params, pmt::from_float(gate::Z), pmt::PMT_NIL);
      H_gate = pmt::dict_ref(gates_params, pmt::from_float(gate::H), pmt::PMT_NIL);
      T_gate = pmt::dict_ref(gates_params, pmt::from_float(gate::T), pmt::PMT_NIL);
      S_gate = pmt::dict_ref(gates_params, pmt::from_float(gate::S), pmt::PMT_NIL);
      CNOT = pmt::dict_ref(gates_params, pmt::from_float(gate::CNOT), pmt::PMT_NIL);
      JUNC = pmt::dict_ref(gates_params, pmt::from_float(gate::JUNC), pmt::PMT_NIL);
      WAIT = pmt::dict_ref(gates_params, pmt::from_float(gate::WAIT), pmt::PMT_NIL);

      int qubitid = pmt::to_double(pmt::dict_ref(INIT, pmt::from_float(gate::QUBIT_ID), pmt::PMT_NIL));
      if(qubitid == 0) {
        INIT = pmt::dict_add(INIT, pmt::from_float(gate::QUBIT_ID), pmt::from_double(qubit_id()));
      } else {
        set_qubit_id(qubitid);
      }

      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), INIT);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), RO);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), X_gate);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), Y_gate);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), Z_gate);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), H_gate);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), T_gate);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), S_gate);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), CNOT);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), JUNC);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(cnt++), WAIT);

      set_is_use_gates_params_block(true);

    }

    bool
    controllers_Coprocessor_impl::exec_interpreter(pmt::pmt_t msg)
    {
      int msg_size = pmt::length(msg);
      long wait_time;
      long real_wait_time;

      //Skip Init
      for(int i = 0; i <= msg_size; i++ ) {
        pmt::pmt_t gate = pmt::dict_ref(msg, pmt::from_float(i), pmt::PMT_NIL);
        if(pmt::eq(gate,pmt::PMT_NIL)) {
          continue;
        }
        pmt::pmt_t type_value = pmt::dict_ref(gate, pmt::from_float(gate::GATE_TYPE), pmt::PMT_NIL);
        int type_int = (int)(pmt::to_float(type_value));


        switch(type_int) {
          case gate::X:
            if(!is_use_gates_params_block()) {
              X_gate = gate;
            }
            send_waveform(X_gate);
            break;
          case gate::Y:
            if(!is_use_gates_params_block()) {
              Y_gate = gate;
            }
            send_waveform(Y_gate);
            break;
          case gate::Z:
            if(!is_use_gates_params_block()) {
              Z_gate = gate;
            }
            send_waveform(Z_gate);
            break;
          case gate::T:
            if(!is_use_gates_params_block()) {
              T_gate = gate;
            }
            send_waveform(T_gate);
            break;
          case gate::S:
            if(!is_use_gates_params_block()) {
              S_gate = gate;
            }
            send_waveform(S_gate);
            break;
          case gate::H:
            if(!is_use_gates_params_block()) {
              H_gate = gate;
            }
            send_waveform(H_gate);
            break;
          case gate::INIT:
            if(!is_use_gates_params_block()) {
              if(first_INIT()) {
                set_first_INIT(false);
                break;
              }
              INIT = gate;
            }
            send_waveform(INIT);
            break;
          case gate::RO:
            if(!is_use_gates_params_block()) {
              RO = gate;
            }
            send_waveform(RO);
            break;
          case gate::CNOT:
            if(!is_use_gates_params_block()) {
              CNOT = gate;
            }
            send_waveform(CNOT);
            break;
          case gate::JUNC:
            if(!is_use_gates_params_block()) {
              JUNC = gate;
            }
            send_waveform(JUNC);
            break;
          case gate::WAIT:
            if(!is_use_gates_params_block()) {
              WAIT = gate;
            }
            wait_time = (long)pmt::to_double(pmt::dict_ref(WAIT, pmt::from_float(gate::WAIT_TIME), pmt::PMT_NIL));
            real_wait_time = (long)calc_wait_time(wait_time * time_scale_rate());
            proccessing_wait(real_wait_time);
            d_gates_proc_total_time += (real_wait_time/time_scale_rate());
            break;
          default:
            break;
        }
      }

      return true;
    }

    void
    controllers_Coprocessor_impl::handle_cmd_msg(pmt::pmt_t msg)
    {
      boost::lock_guard<gr::thread::mutex> lock(d_cmd_mutex);

      pmt::pmt_t msg_chk = pmt::dict_ref(msg, pmt::from_float(gate::NONE), pmt::PMT_NIL);
      if(pmt::eq(msg_chk, pmt::PMT_NIL)) {
        if(!is_use_gates_params_block()) {
          callback_message_port->message_port_pub(d_port_out, msg);
        }
        exec_interpreter(msg);
      } else {
        pmt::pmt_t send_msg = pmt::make_dict();
        set_gates_by_block(msg_chk, send_msg);
        callback_message_port->message_port_pub(d_port_out, send_msg);
      }

    }

    void
    controllers_Coprocessor_impl::handle_SYNC_msg(pmt::pmt_t msg)
    {
       double cur_time = pmt::to_double(msg);
       if(cur_time < d_SYNC_clock_time) return;
       d_SYNC_clock_time = cur_time;
    }

    void
    controllers_Coprocessor_impl::handle_FEEDBACK_msg(pmt::pmt_t msg)
    {
//TODO not impl
    }


    void
    controllers_Coprocessor_impl::set_qubit_id(double qubit_id)
    {
      d_qubit_id = qubit_id;
    }
    double
    controllers_Coprocessor_impl::qubit_id()
    {
      return d_qubit_id;
    }

    void
    controllers_Coprocessor_impl::set_sample_rate(double rate)
    {
      d_samps_per_us = rate/1e6;
    }

    double
    controllers_Coprocessor_impl::sample_rate()
    {
      return d_samps_per_us * 1e6;
    }

    void
    controllers_Coprocessor_impl::set_time_scale_rate(double rate)
    {
      if(rate == 0.0)
        d_time_scale_rate = 1.0;
      else
        d_time_scale_rate = rate;
    }

    double
    controllers_Coprocessor_impl::time_scale_rate()
    {
      return d_time_scale_rate;
    }

    void
    controllers_Coprocessor_impl::set_SYNC_port(bool is_use) {
        d_SYNC_port = !is_use;
    }

    bool
    controllers_Coprocessor_impl::SYNC_port() const{
      return d_SYNC_port;
    }

    void
    controllers_Coprocessor_impl::set_first_INIT(bool first_INIT) {
        d_first_INIT = first_INIT;
    }
    bool
    controllers_Coprocessor_impl::first_INIT() {
      return d_first_INIT;
    }


    void
    controllers_Coprocessor_impl::proccessing_wait(long wait_time_ns)
    {

#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      boost::posix_time::time_duration sleep_time = boost::posix_time::nanoseconds(wait_time_ns);
#else      
      boost::posix_time::time_duration sleep_time = boost::posix_time::microseconds(wait_time_ns);
#endif
      boost::system_time st = boost::get_system_time() + sleep_time;
      boost::unique_lock<boost::mutex> lk(sleep_mutex);
      while(sleep_condition.timed_wait(lk, st));
    }
  } /* namespace quantum */
} /* namespace gr */
