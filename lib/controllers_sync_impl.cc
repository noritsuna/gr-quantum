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

#include "controllers_sync_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <limits>

namespace gr {
  namespace quantum {

    controllers_sync::sptr
    controllers_sync::make(
                           )

    {
      return gnuradio::get_initial_sptr
        (new controllers_sync_impl(
                                   ));
    }

    controllers_sync_impl::controllers_sync_impl(
                                                 )
      : sync_block("controllers_sync",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_SYNC_out(pmt::mp("SYNC_CLK_out"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Sync");

      d_send_time = 0;
      d_proc_timer.start();

      message_port_register_out(d_SYNC_out);
    }

    controllers_sync_impl::~controllers_sync_impl()
    {
    }

    bool
    controllers_sync_impl::start()
    {
      return block::start();
    }

    int
    controllers_sync_impl::work(int noutput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
    {

      double cur_time_ns;
      long wait_time = WAIT_TIME_NS;
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      auto cur_proc_time = boost::chrono::nanoseconds(d_proc_timer.elapsed().wall);
      cur_time_ns = cur_proc_time.count();
#else
      auto cur_proc_time = boost::chrono::microseconds(d_proc_timer.elapsed().wall);
      cur_time_ns = cur_proc_time.count() / 1000;
      wait_time /= 1000;
#endif
      if(wait_time+d_send_time > cur_time_ns) {
        return noutput_items;
      }
      d_send_time = cur_time_ns;

      message_port_pub(d_SYNC_out, pmt::from_double(cur_time_ns));
      return noutput_items;
    }

  } /* namespace quantum */
} /* namespace gr */
