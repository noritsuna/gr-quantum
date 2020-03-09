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

#ifndef INCLUDED_GR_QUANTUM_API_OPENQL_SOURCE_H
#define INCLUDED_GR_QUANTUM_API_OPENQL_SOURCE_H

#include <quantum/api.h>
#include <gnuradio/block.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/add_blk.h>

namespace gr {
  namespace quantum {

    /*!
     * \brief Open QL Source for OpenQL compiler's message.
     *
     * \ingroup API_blk
     *
     * \details
     * output: one messages
     *
     */
    class QUANTUM_API API_OpenQL_Source : virtual public block
    {
    public:
      typedef boost::shared_ptr<API_OpenQL_Source> sptr;

      static sptr make(int port, int qubit_bitnum, int qreg_num);

      /*! \brief Change the connection to a new destination
      *
      * \param host         The name or IP address of the receiving host; use
      *                     NULL or None to break the connection without closing
      * \param port         Destination port to connect to on receiving host
      *
      * Calls disconnect() to terminate any current connection first.
      */
      virtual void listen(int port) = 0;

      virtual void set_qubit_bitnum(int qubit_bitnum) = 0;
      virtual int qubit_bitnum() = 0;
      virtual void set_qubit_reg_num(int qreg_num) = 0;
      virtual int qubit_reg_num() = 0;

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_COPROSESSOR_H */
