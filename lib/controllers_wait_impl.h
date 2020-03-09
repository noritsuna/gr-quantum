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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_WAIT_IMPL_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_WAIT_IMPL_H

#include <quantum/controllers_wait.h>
#include "gate.h"
#include <gnuradio/logger.h>

namespace gr {
  namespace quantum {

    class controllers_wait_impl : public controllers_wait
    {
    private:
      double d_wait_time_ns;

      gate *WAIT;
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_in;

    public:
      controllers_wait_impl(
                   double wait_time);
      ~controllers_wait_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void set_wait_time_ns(double wait_time_ns);
      double wait_time() const;

      void handle_cmd_msg(pmt::pmt_t msg);

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_WAIT_IMPL_H */
