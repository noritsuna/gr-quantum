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

#include "API_OpenQL_Sink_impl.h"
#include "qubit.h"
#include <boost/array.hpp>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace gr {
  namespace quantum {

    API_OpenQL_Sink::sptr
    API_OpenQL_Sink::make(const std::string& host, int port, int qubit_bitnum, int qreg_num)
    {
      return gnuradio::get_initial_sptr
        (new API_OpenQL_Sink_impl(host, port, qubit_bitnum, qreg_num));
    }

    API_OpenQL_Sink_impl::API_OpenQL_Sink_impl(const std::string& host, int port, int qubit_bitnum, int qreg_num)
      : block("API_OpenQL_Sink",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0)),
      d_port_in(pmt::mp("result"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "API_OpenQL_Source");

      make_socket(host, port);
    
      set_qubit_bitnum(qubit_bitnum);
      set_qubit_reg_num(qreg_num);

      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&API_OpenQL_Sink_impl::handle_cmd_msg, this, _1));

    }

    API_OpenQL_Sink_impl::~API_OpenQL_Sink_impl()
    {
    }

    void 
    API_OpenQL_Sink_impl::set_qubit_bitnum(int qubit_bitnum)
    {
        d_qubit_bitnum = qubit_bitnum;
    }
    int 
    API_OpenQL_Sink_impl::qubit_bitnum()
    {
        return d_qubit_bitnum;
    }
    void 
    API_OpenQL_Sink_impl::set_qubit_reg_num(int qreg_num)
    {
        d_qubit_reg_num = qreg_num;
    }
    int 
    API_OpenQL_Sink_impl::qubit_reg_num()
    {
        return d_qubit_reg_num;
    }

    void
    API_OpenQL_Sink_impl::handle_cmd_msg(pmt::pmt_t msg)
    {

      pmt::pmt_t qubit_id_PMT = pmt::dict_ref(msg, pmt::from_float(qubit::ID), pmt::PMT_NIL);
      if(pmt::eq(qubit_id_PMT, pmt::PMT_NIL)) return;
      int qubit_id = pmt::to_float(qubit_id_PMT);

      pmt::pmt_t qubit_angle_PMT = pmt::dict_ref(msg, pmt::from_float(qubit::ANGLE), pmt::PMT_NIL);
      if(pmt::eq(qubit_angle_PMT, pmt::PMT_NIL)) return;
      float qubit_angle = pmt::to_float(qubit_angle_PMT);

      pmt::pmt_t qubit_pole_PMT = pmt::dict_ref(msg, pmt::from_float(qubit::POLE), pmt::PMT_NIL);
      if(pmt::eq(qubit_pole_PMT, pmt::PMT_NIL)) return;
      bool qubit_pole = pmt::to_float(qubit_pole_PMT);

      //true = bit1, false = bit0
      bool qubit_bit_bool = false;
      if(!qubit_pole) {
        qubit_bit_bool = !qubit_bit_bool;
      }
      if(qubit_angle < 0.0) {
        qubit_bit_bool = !qubit_bit_bool;
      }

      int qubit_num_id = 0;
      int qubit_reg_id = 0;
      if(qubit_bitnum()*qubit_reg_num() == 1) {
        qubit_num_id = 0;
        qubit_reg_id = 0;
      } else {
        qubit_num_id = (qubit_id/qubit_reg_num());
        qubit_reg_id = qubit_id%qubit_reg_num();
      }

      char buffer[4096];
      osc::OutboundPacketStream pack(buffer, 4096);
      UdpTransmitSocket socket(IpEndpointName(d_host.c_str(), d_port));
      
      pack << osc::BeginBundleImmediate
          << osc::BeginMessage( "/qubit" ) 
              << (int)qubit_num_id << (int)qubit_reg_id << (int)qubit_bit_bool << osc::EndMessage
          << osc::EndBundle;
      
      socket.Send(pack.Data(), pack.Size());

    }

    void
    API_OpenQL_Sink_impl::make_socket(const std::string& host, int port)
    {
      d_host = host;
      d_port = port;
    }

    void
    API_OpenQL_Sink_impl::connect()
    {
    }

    void
    API_OpenQL_Sink_impl::disconnect()
    {
    }

  } /* namespace quantum */
} /* namespace gr */
