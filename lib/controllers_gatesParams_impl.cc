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
#include <boost/thread/thread.hpp>
#include <cstring>
#include <limits>

namespace gr {
  namespace quantum {

    controllers_gatesParams::sptr
    controllers_gatesParams::make(double qubit_id, std::string wave_type)
    {
      return gnuradio::get_initial_sptr
        (new controllers_gatesParams_impl(qubit_id, wave_type));
    }

    controllers_gatesParams_impl::controllers_gatesParams_impl(double qubit_id, std::string wave_type)
      : block("controllers_gatesParams",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Gate Params");
      set_qubit_id(qubit_id);
      set_wave_type(wave_type);
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
    controllers_gatesParams_impl::set_wave_type(std::string wave_type)
    {
      d_wave_type = gate::convert_wave_type(wave_type);
    }
    gate::wave_type_t
    controllers_gatesParams_impl::wave_type()
    {
      return d_wave_type;
    }


    void
    controllers_gatesParams_impl::handle_cmd_msg(pmt::pmt_t msg)
    {
      gate* _gate;

      pmt::pmt_t type_value = pmt::dict_ref(msg, pmt::from_float(gate::GATE_TYPE), pmt::PMT_NIL);
      int type_int = (int)(pmt::to_float(type_value));

      pmt::pmt_t id_value = pmt::dict_ref(msg, pmt::from_float(gate::QUBIT_ID), pmt::PMT_NIL);
      int id_int = (int)(pmt::to_float(id_value));

      switch(type_int) {
        case gate::INIT:
          _gate = INIT;
          break;
        case gate::X:
          _gate = X_gate;
          break;
        case gate::Y:
          _gate = Y_gate;
          break;
        case gate::Z:
          _gate = Z_gate;
          break;
        case gate::H:
          _gate = H_gate;
          break;
        case gate::T:
          _gate = T_gate;
          break;
        case gate::S:
          _gate = S_gate;
          break;
        case gate::RO:
          _gate = RO;
          break;
        case gate::CNOT:
          _gate = CNOT;
          break;
        case gate::JUNC:
          _gate = JUNC;
          break;
        default:
          _gate = INIT;
          break;
      }
      pmt::pmt_t send_msg = pmt::make_dict();
      _gate->set_qubit_ID(id_int);
      send_msg = pmt::dict_add(send_msg, pmt::from_float(1), _gate->get_parameters());
      message_port_pub(d_port_out, send_msg);

    }


    void
    controllers_gatesParams_impl::set_INIT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        INIT = new gate(gate::INIT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id());
        break;
      case gate::ARRAY:
        INIT = new gate(gate::INIT, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec, qubit_id());
        break;
      default:
        INIT = new gate(gate::INIT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id());
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_RO_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        RO = new gate(gate::RO, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        RO = new gate(gate::RO, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        RO = new gate(gate::RO, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }

    void
    controllers_gatesParams_impl::set_X_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        X_gate = new gate(gate::X, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        X_gate = new gate(gate::X, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        X_gate = new gate(gate::X, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_Y_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        Y_gate = new gate(gate::Y, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        Y_gate = new gate(gate::Y, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        Y_gate = new gate(gate::Y, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_Z_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        Z_gate = new gate(gate::Z, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        Z_gate = new gate(gate::Z, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        Z_gate = new gate(gate::Z, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_H_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        H_gate = new gate(gate::H, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        H_gate = new gate(gate::H, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        H_gate = new gate(gate::H, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_T_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        T_gate = new gate(gate::T, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        T_gate = new gate(gate::T, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        T_gate = new gate(gate::T, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_S_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        S_gate = new gate(gate::S, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      case gate::ARRAY:
        S_gate = new gate(gate::S, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec);
        break;
      default:
        S_gate = new gate(gate::S, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_CNOT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type, bool ctrl_dc_mode) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        CNOT = new gate(gate::CNOT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id(), ctrl_dc_mode);
        break;
      case gate::ARRAY:
        CNOT = new gate(gate::CNOT, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec, qubit_id(), ctrl_dc_mode);
        break;
      default:
        CNOT = new gate(gate::CNOT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id(), ctrl_dc_mode);
        break;
      }
    }
    void
    controllers_gatesParams_impl::set_JUNC_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, const char* wave_file_path, std::string wave_file_type,  bool ctrl_dc_mode) {
      switch (wave_type())
      {
      case gate::FREQ_COMV:
        JUNC = new gate(gate::JUNC, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id(), ctrl_dc_mode);
        break;
      case gate::ARRAY:
        JUNC = new gate(gate::JUNC, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_sec, qubit_id(), ctrl_dc_mode);
        break;
      default:
        JUNC = new gate(gate::JUNC, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_sec, qubit_id(), ctrl_dc_mode);
        break;
      }
    }

  } /* namespace quantum */
} /* namespace gr */
