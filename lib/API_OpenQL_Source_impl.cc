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

#include "API_OpenQL_Source_impl.h"
#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace gr {
  namespace quantum {

    API_OpenQL_Source::sptr
    API_OpenQL_Source::make(int port, int qubit_bitnum, int qreg_num)
    {
      return gnuradio::get_initial_sptr
        (new API_OpenQL_Source_impl(port, qubit_bitnum, qreg_num));
    }

    API_OpenQL_Source_impl::API_OpenQL_Source_impl(int port, int qubit_bitnum, int qreg_num)
      : block("API_OpenQL_Source",
                      io_signature::make(0, 0, 0),
                      io_signature::make(0, 0, 0))
    {
      configure_default_loggers(d_logger, d_debug_logger, "API_OpenQL_Source");

      d_udp_thread =
        gr::thread::thread(boost::bind(&API_OpenQL_Source_impl::listen, this, port));

      set_qubit_bitnum(qubit_bitnum);
      set_qubit_reg_num(qreg_num);
      int qubit_num = qubit_bitnum*qreg_num;
      d_qubit_bitnum = qubit_bitnum;
      d_qubit_reg_num = qreg_num;

      if(qubit_num == 1) {
        message_port_register_out(pmt::mp("out"));
      } else {
        for(int i = 0; i < qubit_bitnum*qreg_num; i++) {
            std::string port_str;
            port_str = "out" + std::to_string(i);
            message_port_register_out(pmt::mp(port_str));
        }
      }
      
    }

    API_OpenQL_Source_impl::~API_OpenQL_Source_impl()
    {
      d_udp_thread.join();
      delete d_socket;
    }

    void
    API_OpenQL_Source_impl::listen(int port)
    {
        d_socket = new UdpListeningReceiveSocket(IpEndpointName( IpEndpointName::ANY_ADDRESS, port ), this);
        d_socket->RunUntilSigInt();
    }

    void 
    API_OpenQL_Source_impl::set_qubit_bitnum(int qubit_bitnum)
    {
        d_qubit_bitnum = qubit_bitnum;
    }
    int 
    API_OpenQL_Source_impl::qubit_bitnum()
    {
        return d_qubit_bitnum;
    }
    void 
    API_OpenQL_Source_impl::set_qubit_reg_num(int qreg_num)
    {
        d_qubit_reg_num = qreg_num;
    }
    int 
    API_OpenQL_Source_impl::qubit_reg_num()
    {
        return d_qubit_reg_num;
    }

    void
    API_OpenQL_Source_impl::ProcessMessage( const osc::ReceivedMessage& m, 
        const IpEndpointName& remoteEndpoint )
    {
        boost::lock_guard<gr::thread::mutex> lock(d_udp_mutex);
        gate::gate_type_t gate_type = gate::NONE;
        int qubit_num_id = 0;
        int qubit_reg_id = 0;
        int qubit_id = 0;
        int CNOT_cnt = 0;
        std::vector<int> CNOT_junc_point_list;

        try {
            if( std::strcmp( m.AddressPattern(), "/X" ) == 0 ) {
                gate_type = gate::X;
            } else if( std::strcmp( m.AddressPattern(), "/Y" ) == 0 ) {
                gate_type = gate::Y;
            } else if( std::strcmp( m.AddressPattern(), "/Z" ) == 0 ) {
                gate_type = gate::Z;
            } else if( std::strcmp( m.AddressPattern(), "/CX" ) == 0 ) {
                gate_type = gate::CX;
            } else if( std::strcmp( m.AddressPattern(), "/CY" ) == 0 ) {
                gate_type = gate::CY;
            } else if( std::strcmp( m.AddressPattern(), "/CZ" ) == 0 ) {
                gate_type = gate::CZ;
            } else if( std::strcmp( m.AddressPattern(), "/Mz" ) == 0 ) {
                gate_type = gate::MZ;
            } else if( std::strcmp( m.AddressPattern(), "/H" ) == 0 ) {
                gate_type = gate::H;
            } else if( std::strcmp( m.AddressPattern(), "/S" ) == 0 ) {
                gate_type = gate::S;
            } else if( std::strcmp( m.AddressPattern(), "/T" ) == 0 ) {
                gate_type = gate::T;
            } else if( std::strcmp( m.AddressPattern(), "/Sdg" ) == 0 ) {
                gate_type = gate::Sdg;
            } else if( std::strcmp( m.AddressPattern(), "/Tdg" ) == 0 ) {
                gate_type = gate::Tdg;
            } else if( std::strcmp( m.AddressPattern(), "/CNOT" ) == 0 ) {
                gate_type = gate::CNOT;
            } else if( std::strcmp( m.AddressPattern(), "/InitZero" ) == 0 ) {
                gate_type = gate::INIT;
            } else if( std::strcmp( m.AddressPattern(), "/measure" ) == 0 ) {
                gate_type = gate::RO;
            } else {
                std::cout << m.AddressPattern() << "\n";
                return;
            }
            osc::ReceivedMessage::const_iterator args = m.ArgumentsBegin();
            qubit_num_id = (args++)->AsInt32();
            qubit_reg_id = (args++)->AsInt32();            
            if(gate_type == gate::CNOT) {
              int q_num = 0;
              int q_reg = 0;
              while(args != m.ArgumentsEnd()) {
                q_num = (args++)->AsInt32();
                q_reg = (args++)->AsInt32();
                CNOT_junc_point_list[CNOT_cnt++] = ((q_num)*qubit_reg_num()) + q_reg;
              }
            }
            
            std::string port_str;
            if(qubit_bitnum()*qubit_reg_num() == 1) {
              port_str = "out";
              qubit_id = 1;
            } else {
              int qubit_port_num = ((qubit_num_id)*qubit_reg_num()) + qubit_reg_id;
              port_str = "out" + std::to_string(qubit_port_num);
              qubit_id = qubit_port_num + 1;
            }
            pmt::pmt_t qubit_ret_msg = pmt::make_dict();
            qubit_ret_msg = pmt::dict_add(qubit_ret_msg, pmt::from_float(gate::GATE_TYPE), pmt::from_float(gate_type));
            qubit_ret_msg = pmt::dict_add(qubit_ret_msg, pmt::from_float(gate::QUBIT_ID), pmt::from_float(qubit_id));
            message_port_pub(pmt::mp(port_str), qubit_ret_msg);

            for(int i = 0; i < CNOT_cnt; i++) {
              pmt::pmt_t junc_ret_msg = pmt::make_dict();
              junc_ret_msg = pmt::dict_add(junc_ret_msg, pmt::from_float(gate::GATE_TYPE), pmt::from_float(gate::JUNC));
              junc_ret_msg = pmt::dict_add(qubit_ret_msg, pmt::from_float(gate::QUBIT_ID), pmt::from_float(qubit_id));
              port_str = "out" + std::to_string(CNOT_junc_point_list[i]);
              message_port_pub(pmt::mp(port_str), junc_ret_msg);
            }

        } catch( osc::Exception& e ) {
            std::cout << "error while parsing message: "
                << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
  } /* namespace quantum */
} /* namespace gr */
