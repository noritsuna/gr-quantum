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

#ifndef INCLUDED_GR_QUANTUM_API_OPENQL_SOURCE_IMPL_H
#define INCLUDED_GR_QUANTUM_API_OPENQL_SOURCE_IMPL_H

#include <quantum/API_OpenQL_Source.h>
#include <gnuradio/logger.h>
#include <gnuradio/thread/thread.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/bind.hpp>
#include <set>

#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <oscpack/osc/MessageMappingOscPacketListener.h>
#include <oscpack/osc/OscOutboundPacketStream.h>
#include <oscpack/ip/IpEndpointName.h>
#include <oscpack/ip/UdpSocket.h>

namespace gr {
  namespace quantum {

    class API_OpenQL_Source_impl : public API_OpenQL_Source, public osc::OscPacketListener
    {
    private:

      UdpListeningReceiveSocket* d_socket;

      int d_qubit_bitnum;
      int d_qubit_reg_num;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

      gr::thread::thread d_udp_thread;
      gr::thread::mutex d_udp_mutex;

    protected:
      virtual void ProcessMessage( const osc::ReceivedMessage& m, 
          const IpEndpointName& remoteEndpoint );

    public:
      API_OpenQL_Source_impl(int port, int qubit_bitnum, int qreg_num);
      ~API_OpenQL_Source_impl();


      void listen(int port);

      void set_qubit_bitnum(int qubit_bitnum);
      int qubit_bitnum();
      void set_qubit_reg_num(int qreg_num);
      int qubit_reg_num();

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_API_OPENQL_SOURCE_IMPL_H */
