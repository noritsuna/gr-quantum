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

#include "gates_waveform_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    gates_waveform::sptr
    gates_waveform::make(
                  double samples_per_sec,
                  const char* wave_file_path,
                  std::string wave_file_type)
    {
      return gnuradio::get_initial_sptr
        (new gates_waveform_impl(
                          samples_per_sec,
                          wave_file_path,
                          wave_file_type));
    }

    gates_waveform_impl::gates_waveform_impl(
                                 double samples_per_second,
                                 const char* wave_file_path,
                                 std::string wave_file_type)
      : block("gates_waveform",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Waveform Gate");

      d_gate = new gate(gate::WAVEFORM, wave_file_path, gate::convert_wave_file_type(wave_file_type), samples_per_second);

      message_port_register_out(d_port_out);
      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&gates_waveform_impl::handle_cmd_msg, this, _1));
    }

    gates_waveform_impl::~gates_waveform_impl()
    {
       delete d_gate;
    }

    bool
    gates_waveform_impl::start()
    {
      return block::start();
    }

    void
    gates_waveform_impl::handle_cmd_msg(pmt::pmt_t msg)
    {

      msg = pmt::dict_add(msg, pmt::from_float(pmt::length(msg)+1), d_gate->get_parameters());
      message_port_pub(d_port_out, msg);

    }

  } /* namespace quantum */
} /* namespace gr */
