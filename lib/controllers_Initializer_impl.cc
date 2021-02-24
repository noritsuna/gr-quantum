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

#include "controllers_Initializer_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    controllers_Initializer::sptr
    controllers_Initializer::make(
                  double qubit_id,
                  std::string wave_type,
                  double frequency,
                  double I_amplitude,
                  double Q_amplitude,
                  double I_bandwidth,
                  double Q_bandwidth,
                  double processing_time,
                  double samples_per_sec,
                  const char* wave_file_path,
                  std::string wave_file_type,
                  bool isFeedbackMode)
    {
      return gnuradio::get_initial_sptr
        (new controllers_Initializer_impl(
                          qubit_id,
                          wave_type,
                          frequency,
                          I_amplitude,
                          Q_amplitude,
                          I_bandwidth,
                          Q_bandwidth,
                          processing_time,
                          samples_per_sec,
                          wave_file_path,
                          wave_file_type,
                          isFeedbackMode));
    }

    controllers_Initializer_impl::controllers_Initializer_impl(
                                 double qubit_id,
                                 std::string wave_type,
                                 double frequency,
                                 double I_amplitude,
                                 double Q_amplitude,
                                 double I_bandwidth,
                                 double Q_bandwidth,
                                 double processing_time, 
                                 double samples_per_second,
                                 const char* wave_file_path,
                                 std::string wave_file_type,
                                 bool isFeedbackMode)
      : block("controllers_Initializer",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in")),
      d_port_fb(pmt::mp("feedback"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Initializer");

      set_feedback_mode(isFeedbackMode);
      set_qubit_id(qubit_id);

      INIT = new gate(gate::INIT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_second, qubit_id);
      switch (gate::convert_wave_type(wave_type))
      {
      case gate::FREQ_COMV:
        INIT = new gate(gate::INIT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_second, qubit_id);
        break;
      case gate::ARRAY:
        INIT = new gate(gate::INIT, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_second, qubit_id);
        break;
      default:
        INIT = new gate(gate::INIT, frequency, I_amplitude, Q_amplitude, I_bandwidth, Q_bandwidth, processing_time, samples_per_second, qubit_id);
        break;
      }

      message_port_register_out(d_port_out);
      message_port_register_in(d_port_in);
      message_port_register_in(d_port_fb);
      set_msg_handler(d_port_in, boost::bind(&controllers_Initializer_impl::handle_cmd_msg, this, _1));
      set_msg_handler(d_port_fb, boost::bind(&controllers_Initializer_impl::handle_fb_msg, this, _1));

    }

    controllers_Initializer_impl::~controllers_Initializer_impl()
    {
      delete INIT;
    }

    bool
    controllers_Initializer_impl::start()
    {
      d_start = boost::get_system_time();
      d_total_samples = 0;

      if(!is_feedback_mode()) {
	    pmt::pmt_t stack = pmt::make_dict();
	    stack = pmt::dict_add(stack, pmt::from_float(1), INIT->get_parameters());
	    message_port_pub(d_port_out, stack);
      }

      return block::start();
    }

    void
    controllers_Initializer_impl::handle_cmd_msg(pmt::pmt_t msg)
    {
      if(is_feedback_mode()) {
        msg = pmt::dict_add(msg, pmt::from_float(pmt::length(msg)+1), INIT->get_parameters());
        message_port_pub(d_port_out, msg);
      }
    }


    void
    controllers_Initializer_impl::handle_fb_msg(pmt::pmt_t msg)
    {
    }

    bool
    controllers_Initializer_impl::check_topology(int ninputs, int noutputs)
    {

      return true;
    }



    void
    controllers_Initializer_impl::set_feedback_mode(bool mode) {
      d_feedback_mode = mode;
    }

    bool
    controllers_Initializer_impl::is_feedback_mode() const{
      return d_feedback_mode;
    }


    void
    controllers_Initializer_impl::set_qubit_id(double qubit_id)
    {
      d_qubit_id = qubit_id;
    }
    double
    controllers_Initializer_impl::qubit_id() const
    {
      return d_qubit_id;
    }

  } /* namespace quantum */
} /* namespace gr */
