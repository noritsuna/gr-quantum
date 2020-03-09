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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_GENERATOR_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_GENERATOR_H

#include <quantum/api.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <gnuradio/blocks/add_blk.h>

namespace gr {
  namespace quantum {

    /*!
     * \brief Quantum Simulator for OpenQL compiler's binary.
     *
     * \ingroup controller_blk
     *
     * \details
     * input: one message; output: one streams of complex
     *
     */
    class QUANTUM_API controllers_Generator : virtual public hier_block2
    {
    public:
      typedef boost::shared_ptr<controllers_Generator> sptr;

      static sptr make(double qubit_num, double samples_per_sec, double time_scale_rate, bool show_SYNC_port=true);

      //! Sets using SYNC in/out.
      virtual void set_SYNC_port(bool is_use) = 0;

      //! Get using SYNC in/out.
      virtual bool SYNC_port() const = 0;

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_GENERATOR_H */
