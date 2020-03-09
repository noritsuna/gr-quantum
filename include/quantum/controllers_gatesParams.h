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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_GATESPARAMS_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_GATESPARAMS_H

#include <quantum/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace quantum {

    /*!
     * \brief Quantum Gates Prameters for OpenQL compiler's binary.
     *
     * \ingroup controller_blk
     *
     * \details
     * input: one messages; output: one message
     *
     */
    class QUANTUM_API controllers_gatesParams : virtual public block
    {
    public:
      typedef boost::shared_ptr<controllers_gatesParams> sptr;

      static sptr make(double qubit_id);

      virtual void set_INIT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_RO_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_X_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_Y_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_Z_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_H_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_T_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_S_gate_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec) = 0;

      virtual void set_CNOT_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, bool ctrl_dc_mode = false) = 0;

      virtual void set_JUNC_parameters(double frequency, double I_amplitude, double Q_amplitude, double I_bandwidth, double Q_bandwidth, double processing_time, double samples_per_sec, bool ctrl_dc_mode = false) = 0;

    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_GATESPARAMS_H */
