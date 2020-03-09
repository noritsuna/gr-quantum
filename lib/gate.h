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

#ifndef INCLUDED_GR_QUANTUM_GATE_H
#define INCLUDED_GR_QUANTUM_GATE_H

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
     * \brief gate class
     *
     * \ingroup gates_blk
     *
     * \details
     *
     */
    class gate
    {
    public:

      enum param_type_t {
        GATE_TYPE = 1,
        FREQ,
        I_AMP,
        Q_AMP,
        I_BW,
        Q_BW,
        PROC_TIME,
        CTRL_DC_MODE,
        QUBIT_ID,
        WAIT_TIME
      };
      
      enum gate_type_t {
        NONE = 0,
        INIT,
        X,
        Y,
        Z,
        H,
        S,
        T,
        CNOT,
        RO,
        WAIT,
        JUNC,
        JUNC_LIST,
        SIM_DATA
      };
      
      struct gate_param_t {
        gate_type_t type;
        double freq;
        double I_amp;
        double Q_amp;
        double I_bw;
        double Q_bw;
        double proc_time;

        bool DC_mode;
        int CTRL_num;
      };
    
      struct CTRL_junction_relation_t {
        int qubit_ID;
        int number;
      };

    private:
      gate_type_t d_gate_type;
      double d_freq;
      double d_I_amp, d_Q_amp;
      double d_I_bw, d_Q_bw;
      double d_proc_time_ns;
      double d_samples_per_sec;
      bool d_ctrl_dc_mode;
      int d_qubit_ID;
      int d_wait_time;
      pmt::pmt_t d_gate_params_dict;
      pmt::pmt_t d_JUNC_list_PMT;

    public:        
      gate(gate_type_t gate_type,
           double frequency,
           double I_amplitude,
           double Q_amplitude,
           double I_bandwidth,
           double Q_bandwidth,
           double processing_time,
           double samples_per_sec,
           int qubit_ID = 0,
           bool ctrl_dc_mode = false,
           int wait_time = 0);
      gate(gate_type_t gate_type,
           int qubit_ID);
      gate(gate_type_t gate_type,
           int qubit_ID,
           std::queue<CTRL_junction_relation_t> junc_list);
      ~gate();

      //! Sets frequecy..
      void set_frequency(double freq);
      
      //! Get frequecy..
      double frequency();

      //! Sets I amplitude..
      void set_I_amplitude(double amp);

      //! Get I amplitude..
      double I_amplitude();

      //! Sets Q amplitude..
      void set_Q_amplitude(double amp);

      //! Get Q amplitude..
      double Q_amplitude();

      //! Sets I bandwidth..
      void set_I_bandwidth(double bw);

      //! Get I bandwidth..
      double I_bandwidth();

      //! Sets Q bandwidth..
      void set_Q_bandwidth(double bw);

      //! Get Q bandwidth..
      double Q_bandwidth();

      void set_processing_time_ns(double proc_time_ns);
      double processing_time();

      void set_sample_rate(double rate);
      double sample_rate() const;

      void set_qubit_ID(int qubit_ID);
      int qubit_ID();

      void set_ctrl_dc_mode(bool ctrl_dc_mode);
      bool ctrl_dc_mode();

      void set_wait_time(int wait_time);
      bool wait_time();

      void set_gate_type(gate_type_t gate_type);
      gate_type_t gate_type();

      void set_JUNC_list(std::queue<CTRL_junction_relation_t> junc_list);
      pmt::pmt_t JUNC_list_PMT();

      pmt::pmt_t get_parameters();
      
    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_GATE_H */
