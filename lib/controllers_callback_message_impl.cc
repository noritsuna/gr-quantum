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

#include "controllers_callback_message_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
  namespace quantum {

    controllers_callback_message::sptr
    controllers_callback_message::make()
    {
      return gnuradio::get_initial_sptr
        (new controllers_callback_message_impl());
    }

    controllers_callback_message_impl::controllers_callback_message_impl()
      : block("controllers_callback_message",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in")),
      d_port_SYNC_in(pmt::mp("SYNC_CLK_in")),
      d_port_SIM_in(pmt::mp("simulated_data")),
      d_port_feedback_in(pmt::mp("feedback_in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "controllers_callback_message_impl");

//      message_port_register_out(d_port_out);
      message_port_register_in(d_port_in);
      message_port_register_in(d_port_SYNC_in);
      message_port_register_in(d_port_SIM_in);
      message_port_register_in(d_port_feedback_in);
    }

    controllers_callback_message_impl::~controllers_callback_message_impl()
    {
    }
  } /* namespace quantum */
} /* namespace gr */
