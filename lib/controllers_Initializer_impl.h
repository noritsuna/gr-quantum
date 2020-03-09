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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_INITIALIZER_IMPL_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_INITIALIZER_IMPL_H

#include "gate.h"
#include <quantum/controllers_Initializer.h>

namespace gr {
  namespace quantum {

    class controllers_Initializer_impl : public controllers_Initializer
    {
    private:
      double d_qubit_id;
      boost::system_time d_start;
      uint64_t d_total_samples;

      bool d_feedback_mode;

      gate *INIT;
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_in;
      const pmt::pmt_t d_port_fb;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

    public:
      controllers_Initializer_impl(
                   double qubit_id,
                   double frequency,
                   double I_amplitude,
                   double Q_amplitude,
                   double I_bandwidth,
                   double Q_bandwidth,
                   double processing_time,
                   double samples_per_sec,
                   bool isFeedbackMode=false);
      ~controllers_Initializer_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void set_feedback_mode(bool mode);
      bool is_feedback_mode() const;

      void set_qubit_id(double qubit_id);
      double qubit_id() const;

      void handle_cmd_msg(pmt::pmt_t msg);
      void handle_fb_msg(pmt::pmt_t msg);

      bool check_topology(int ninputs, int noutputs);

    };
  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_INITIALIZER_IMPL_H */
