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

#ifndef INCLUDED_GR_QUANTUM_API_OPENQL_SINK_IMPL_H
#define INCLUDED_GR_QUANTUM_API_OPENQL_SINK_IMPL_H

#include <quantum/API_OpenQL_Sink.h>
#include <gnuradio/logger.h>
#include <gnuradio/thread/thread.h>
#include <boost/array.hpp>
#include <boost/format.hpp>

#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <oscpack/osc/MessageMappingOscPacketListener.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <oscpack/ip/IpEndpointName.h>
#include <oscpack/ip/UdpSocket.h>


namespace gr {
  namespace quantum {

    class API_OpenQL_Sink_impl : public API_OpenQL_Sink
    {
    private:
      std::string d_host;
      int d_port;

      int d_qubit_bitnum;
      int d_qubit_reg_num;

      const pmt::pmt_t d_port_in;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

    public:
      API_OpenQL_Sink_impl(const std::string& host, int port, int qubit_bitnum, int qreg_num);
      ~API_OpenQL_Sink_impl();


      void make_socket(const std::string& host, int port);
      void connect();
      void disconnect();

      void set_qubit_bitnum(int qubit_bitnum);
      int qubit_bitnum();
      void set_qubit_reg_num(int qreg_num);
      int qubit_reg_num();

      void handle_cmd_msg(pmt::pmt_t msg);
    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_API_OPENQL_SINK_IMPL_H */
