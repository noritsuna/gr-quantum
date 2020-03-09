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

#ifndef INCLUDED_GR_QUANTUM_GATES_CNOT_H
#define INCLUDED_GR_QUANTUM_GATES_CNOT_H

#include <quantum/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace quantum {

    /*!
     * \brief CNOT gate
     *
     * \ingroup gates_blk
     *
     * \details
     * input: one messages; output: two messages
     *
     */
    class QUANTUM_API gates_CNOT : virtual public block
    {
    public:
      typedef boost::shared_ptr<gates_CNOT> sptr;

      static sptr make(bool DC_mode,
                       double frequency,
                       double I_amplitude,
                       double Q_amplitude,
                       double I_bandwidth,
                       double Q_bandwidth,
                       double processing_time,
                       double samples_per_sec);

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_GATES_CNOT_H */