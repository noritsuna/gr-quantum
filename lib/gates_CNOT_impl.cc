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

#include "gates_CNOT_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    gates_CNOT::sptr
    gates_CNOT::make(std::string wave_type,
                  bool DC_mode,
                  double frequency,
                  double I_amplitude,
                  double Q_amplitude,
                  double I_bandwidth,
                  double Q_bandwidth,
                  double processing_time,
                  double samples_per_sec,
                  const char* wave_file_path,
                  std::string wave_file_type)
    {
      return gnuradio::get_initial_sptr
        (new gates_CNOT_impl(wave_type,
                          DC_mode,
                          frequency,
                          I_amplitude,
                          Q_amplitude,
                          I_bandwidth,
                          Q_bandwidth,
                          processing_time,
                          samples_per_sec,
                          wave_file_path,
                          wave_file_type));
    }

    gates_CNOT_impl::gates_CNOT_impl(std::string wave_type,
                                 bool DC_mode,
                                 double frequency,
                                 double I_amplitude,
                                 double Q_amplitude,
                                 double I_bandwidth,
                                 double Q_bandwidth,
                                 double processing_time, 
                                 double samples_per_second,
                                 const char* wave_file_path,
                                 std::string wave_file_type)
      : block("gates_CNOT",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_CTRL(pmt::mp("CTRL_port")),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "CNOT Gate");

      switch (gate::convert_wave_type(wave_type))
      {
      case gate::FREQ_COMV:
        CNOT = new gate(gate::CNOT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_second);
        break;
      case gate::ARRAY:
        CNOT = new gate(gate::CNOT, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_second);
        break;
      default:
        CNOT = new gate(gate::CNOT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_second);
        break;
      }
      CNOT->set_ctrl_dc_mode(DC_mode);

      message_port_register_out(d_port_CTRL);
      message_port_register_out(d_port_out);
      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&gates_CNOT_impl::handle_cmd_msg, this, _1));
    }

    gates_CNOT_impl::~gates_CNOT_impl()
    {
       delete CNOT;
    }

    bool
    gates_CNOT_impl::start()
    {
      return block::start();
    }

    void
    gates_CNOT_impl::handle_cmd_msg(pmt::pmt_t msg)
    {

      pmt::pmt_t INIT_gates_params = pmt::dict_ref(msg, pmt::from_float(1), pmt::PMT_NIL);
      int qubit_id = pmt::to_double(pmt::dict_ref(INIT_gates_params, pmt::from_float(gate::QUBIT_ID), pmt::PMT_NIL));
      CNOT->set_qubit_ID(qubit_id);

      msg = pmt::dict_add(msg, pmt::from_float(pmt::length(msg)+1), CNOT->get_parameters());
      message_port_pub(d_port_out, msg);
      message_port_pub(d_port_CTRL, CNOT->get_parameters());
      
    }

  } /* namespace quantum */
} /* namespace gr */
