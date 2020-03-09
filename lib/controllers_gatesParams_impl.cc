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

#include "controllers_gatesParams_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <limits>

namespace gr {
  namespace quantum {

    controllers_gatesParams::sptr
    controllers_gatesParams::make(double qubit_id)
    {
      return gnuradio::get_initial_sptr
        (new controllers_gatesParams_impl(qubit_id));
    }

    controllers_gatesParams_impl::controllers_gatesParams_impl(double qubit_id)
      : block("controllers_gatesParams",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Gate Params");
      set_qubit_id(qubit_id);
      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&controllers_gatesParams_impl::handle_cmd_msg, this, _1));

      message_port_register_out(d_port_out);

    }

    controllers_gatesParams_impl::~controllers_gatesParams_impl()
    {
      delete INIT;
      delete RO;
      delete X_gate;
      delete Y_gate;
      delete Z_gate;
      delete H_gate;
      delete T_gate;
      delete S_gate;
      delete JUNC;
      delete CNOT;
    }


    bool
    controllers_gatesParams_impl::start()
    {
      d_send_msg = pmt::make_dict();
      d_gate_msg = pmt::make_dict();
      d_qubit_msg = pmt::make_dict();

      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::INIT), INIT->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::X), X_gate->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::Y), Y_gate->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::Z), Z_gate->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::H), H_gate->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::T), T_gate->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::S), S_gate->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::RO), RO->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::CNOT), CNOT->get_parameters());
      d_gate_msg = pmt::dict_add(d_gate_msg, pmt::from_float(gate::JUNC), JUNC->get_parameters());
      
      d_send_msg = pmt::dict_add(d_send_msg, pmt::from_float(gate::NONE), d_gate_msg);

      message_port_pub(d_port_out, d_send_msg);
      return block::start();
    }

    void
    controllers_gatesParams_impl::set_qubit_id(double qubit_id)
    {
      d_qubit_id = qubit_id;
    }
    double
    controllers_gatesParams_impl::qubit_id()
    {
      return d_qubit_id;
    }

    void
    controllers_gatesParams_impl::handle_cmd_msg(pmt::pmt_t msg)
    {
      message_port_pub(d_port_out, msg);
    }


    void
    controllers_gatesParams_impl::set_INIT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      INIT = new gate(gate::INIT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id());
    }
    void
    controllers_gatesParams_impl::set_RO_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      RO = new gate(gate::RO, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }

    void
    controllers_gatesParams_impl::set_X_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      X_gate = new gate(gate::X, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }
    void
    controllers_gatesParams_impl::set_Y_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      Y_gate = new gate(gate::Y, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }
    void
    controllers_gatesParams_impl::set_Z_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      Z_gate = new gate(gate::Z, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }
    void
    controllers_gatesParams_impl::set_H_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      H_gate = new gate(gate::H, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }
    void
    controllers_gatesParams_impl::set_T_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      T_gate = new gate(gate::T, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }
    void
    controllers_gatesParams_impl::set_S_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) {
      S_gate = new gate(gate::S, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
    }
    void
    controllers_gatesParams_impl::set_CNOT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, bool ctrl_dc_mode) {
      CNOT = new gate(gate::CNOT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id(), ctrl_dc_mode);
    }
    void
    controllers_gatesParams_impl::set_JUNC_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, bool ctrl_dc_mode) {
      JUNC = new gate(gate::JUNC, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id(), ctrl_dc_mode);
    }

  } /* namespace quantum */
} /* namespace gr */
