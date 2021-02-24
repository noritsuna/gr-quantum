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

#ifndef INCLUDED_GR_QUANTUM_GATES_WAVEFORM_IMPL_H
#define INCLUDED_GR_QUANTUM_GATES_WAVEFORM_IMPL_H

#include <gnuradio/logger.h>
#include "gate.h"
#include <quantum/gates_waveform.h>
#include <gnuradio/logger.h>

namespace gr {
  namespace quantum {

    class gates_waveform_impl : public gates_waveform
    {
    private:
      gate *d_gate;
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_in;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

    public:
      gates_waveform_impl(
                   double samples_per_sec,
                   const char* wave_file_path,
                   std::string wave_file_type);
      ~gates_waveform_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void handle_cmd_msg(pmt::pmt_t msg);

    };
  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_GATES_WAVEFORM_IMPL_H */
