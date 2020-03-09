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

#ifndef INCLUDED_GR_QUANTUM_QUBIT_H
#define INCLUDED_GR_QUANTUM_QUBIT_H

#include <gnuradio/api.h>
#include <gnuradio/block.h>
#include <gnuradio/runtime_types.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/condition_variable.hpp>
#include <quantum/api.h>
#include <queue> 

namespace gr {
  namespace quantum {

    /*!
     * \brief QUBIT class
     *
     * \ingroup SIMULATOR_blk
     *
     * \details
     *
     */
    class qubit
    {
    private:

    public:
    
      enum qubit_param_type_t {
        ID = 1,
        STATE,
        ANGLE,
        POLE
      };
      enum qubit_RO_state_type_t {
        NONE = 0,
        START,
        PROC,
        END
      };
    
    
      qubit();
      ~qubit();

      
    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_QUBIT_H */
