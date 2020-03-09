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

#include "gates_junction_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    gates_junction::sptr
    gates_junction::make(bool DC_mode,
                  double frequency,
                  double I_amplitude,
                  double Q_amplitude,
                  double I_bandwidth,
                  double Q_bandwidth,
                  double processing_time,
                  double samples_per_sec)

    {
      return gnuradio::get_initial_sptr
        (new gates_junction_impl(DC_mode,
                          frequency,
                          I_amplitude,
                          Q_amplitude,
                          I_bandwidth,
                          Q_bandwidth,
                          processing_time,
                          samples_per_sec));
    }

    gates_junction_impl::gates_junction_impl(bool DC_mode,
                                 double frequency,
                                 double I_amplitude,
                                 double Q_amplitude,
                                 double I_bandwidth,
                                 double Q_bandwidth,
                                 double processing_time, 
                                 double samples_per_second)
      : block("gates_junction",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in")),
      d_CTRL_port(pmt::mp("CTRL_port"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Junction Gate");

      JUNC = new gate(gate::JUNC, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_second);
      JUNC->set_ctrl_dc_mode(DC_mode);

      message_port_register_out(d_port_out);
      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&gates_junction_impl::handle_cmd_msg, this, _1));
      message_port_register_in(d_CTRL_port);
      set_msg_handler(d_CTRL_port, boost::bind(&gates_junction_impl::handle_CTRL_msg, this, _1));

      is_recv_CTRL = false;
    }

    gates_junction_impl::~gates_junction_impl()
    {
      delete JUNC;
    }

    bool
    gates_junction_impl::start()
    {
      return block::start();
    }

    void
    gates_junction_impl::handle_cmd_msg(pmt::pmt_t msg)
    {
      while(is_recv_CTRL == false) {
        boost::posix_time::time_duration sleep_time = boost::posix_time::microseconds(1000);
        boost::system_time st = boost::get_system_time() + sleep_time;
        boost::unique_lock<boost::mutex> lk(sleep_mutex);
        while(sleep_condition.timed_wait(lk, st));
      }
      msg = pmt::dict_add(msg, pmt::from_float(pmt::length(msg)+1), JUNC->get_parameters());
      message_port_pub(d_port_out, msg);
      is_recv_CTRL = false;
      delete JUNC;
    }

    void
    gates_junction_impl::handle_CTRL_msg(pmt::pmt_t msg)
    {
      int CNOT_potision = pmt::length(msg);

      if(JUNC->frequency() == 0.0f && !JUNC->ctrl_dc_mode()) {
        pmt::pmt_t CNOT_gates_params = pmt::dict_ref(msg, pmt::from_float(CNOT_potision), pmt::PMT_NIL);
        double freq = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::FREQ), pmt::PMT_NIL));
        double I_amp = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::I_AMP), pmt::PMT_NIL)) / QUANTUM_JUNC_AMP_RATE;
        double Q_amp = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::Q_AMP), pmt::PMT_NIL)) / QUANTUM_JUNC_AMP_RATE;
        double I_bw = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::I_BW), pmt::PMT_NIL));
        double Q_bw = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::Q_BW), pmt::PMT_NIL));
        double  proc_time = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::PROC_TIME), pmt::PMT_NIL));
        int qubit_ID = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::QUBIT_ID), pmt::PMT_NIL));

        bool CTRL_dc_mode = pmt::to_double(pmt::dict_ref(CNOT_gates_params, pmt::from_float(gate::CTRL_DC_MODE), pmt::PMT_NIL));

        JUNC = new gate(gate::JUNC, freq, I_amp, Q_amp, I_bw, Q_bw, proc_time, qubit_ID, CTRL_dc_mode);
      }

      is_recv_CTRL = true;
    }

  } /* namespace quantum */
} /* namespace gr */
