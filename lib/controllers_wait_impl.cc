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

#include "controllers_wait_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    controllers_wait::sptr
    controllers_wait::make(
                  double wait_time)

    {
      return gnuradio::get_initial_sptr
        (new controllers_wait_impl(
                          wait_time));
    }

    controllers_wait_impl::controllers_wait_impl(
                                 double wait_time)
      : block("controllers_wait",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Wait");
      set_wait_time_ns(wait_time);
      WAIT = new gate(gate::WAIT, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, false, wait_time);

      message_port_register_out(d_port_out);
      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&controllers_wait_impl::handle_cmd_msg, this, _1));
    }

    controllers_wait_impl::~controllers_wait_impl()
    {
      delete WAIT;
    }

    bool
    controllers_wait_impl::start()
    {
      return block::start();
    }

    void
    controllers_wait_impl::handle_cmd_msg(pmt::pmt_t msg)
    {

      msg = pmt::dict_add(msg, pmt::from_float(pmt::length(msg)+1), WAIT->get_parameters());
      message_port_pub(d_port_out, msg);
    }



    void
    controllers_wait_impl::set_wait_time_ns(double wait_time_ns)
    {
      d_wait_time_ns = wait_time_ns;
    }
    double
    controllers_wait_impl::wait_time() const
    {
      return d_wait_time_ns;
    }

  } /* namespace quantum */
} /* namespace gr */
