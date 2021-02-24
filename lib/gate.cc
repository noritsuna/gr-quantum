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

#include "gate.h"
#include <cstring>
#include <limits>

namespace gr {
  namespace quantum {

    gate::gate(  gate_type_t gate_type,
                                 double frequency,
                                 double I_amplitude,
                                 double Q_amplitude,
                                 double I_bandwidth,
                                 double Q_bandwidth,
                                 double processing_time,
                                 double samples_per_sec,
                                 int qubit_ID, 
                                 bool ctrl_dc_mode,
                                 int wait_time)
    {
      d_gate_params_dict = pmt::make_dict();
      d_JUNC_list_PMT = pmt::make_dict();
      set_gate_type(gate_type);
      set_wave_type(FREQ_COMV);
      set_frequency(frequency);
      set_I_amplitude(I_amplitude);
      set_Q_amplitude(Q_amplitude);
      set_I_bandwidth(I_bandwidth);
      set_Q_bandwidth(Q_bandwidth);
      set_processing_time_ns(processing_time);
      set_sample_rate(samples_per_sec);
      set_qubit_ID(qubit_ID);
      set_ctrl_dc_mode(ctrl_dc_mode);
      set_wait_time(wait_time);
    }
    gate::gate(  gate_type_t gate_type,
                                 const char* wave_file_path,
                                 wave_file_type_t wave_file_type,           
                                 double samples_per_sec,
                                 int qubit_ID, 
                                 int wait_time)
    {
      d_gate_params_dict = pmt::make_dict();
      d_JUNC_list_PMT = pmt::make_dict();
      set_gate_type(gate_type);
      set_wave_type(ARRAY);
      set_sample_rate(samples_per_sec);
      set_wave_file_path(wave_file_path);
      set_wave_file_type(wave_file_type);
      set_qubit_ID(qubit_ID);
      set_wait_time(wait_time);
    }
    gate::gate(gate_type_t gate_type,
           int qubit_ID)
    {
      d_gate_params_dict = pmt::make_dict();
      d_JUNC_list_PMT = pmt::make_dict();
      set_gate_type(gate_type);
      set_qubit_ID(qubit_ID);
    }
    gate::gate(gate_type_t gate_type,
           int qubit_ID,
           std::queue<CTRL_junction_relation_t> junc_list)
    {
      d_gate_params_dict = pmt::make_dict();
      d_JUNC_list_PMT = pmt::make_dict();
      set_gate_type(gate_type);
      set_qubit_ID(qubit_ID);
    }

    gate::~gate()
    {
    }

    void
    gate::set_gate_type(gate_type_t gate_type)
    {
      d_gate_type = gate_type;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::GATE_TYPE), pmt::from_double(gate_type));
    }

    gate::gate_type_t
    gate::gate_type() {
      return d_gate_type;
    }

    void
    gate::set_wave_type(wave_type_t wave_type)
    {
      d_wave_type = wave_type;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::WAVE_TYPE), pmt::from_double(wave_type));
    }

    gate::wave_type_t
    gate::wave_type() {
      return d_wave_type;
    }

    void
    gate::set_wave_file_path(const char* wave_file_path)
    {
      d_wave_file_path = wave_file_path;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::WAVE_FILE_PATH), pmt::string_to_symbol(wave_file_path));
    }
    const char*
    gate::wave_file_path()
    {
      return d_wave_file_path;
    }

    void
    gate::set_wave_file_type(wave_file_type_t wave_file_type)
    {
      d_wave_file_type = wave_file_type;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::WAVE_FILE_TYPE), pmt::from_double(wave_file_type));
    }
    gate::wave_file_type_t
    gate::wave_file_type()
    {
      return d_wave_file_type;
    }

    void
    gate::set_frequency(double freq)
    {
      d_freq = freq;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::FREQ), pmt::from_double(freq));
    }
    double
    gate::frequency() {
      return d_freq;
    }

    void
    gate::set_I_amplitude(double amp)
    {
      d_I_amp = amp;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::I_AMP), pmt::from_double(amp));
    }
    double
    gate::I_amplitude()
    {
      return d_I_amp;
    }

    void
    gate::set_Q_amplitude(double amp)
    {
      d_Q_amp = amp;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::Q_AMP), pmt::from_double(amp));
    }
    double
    gate::Q_amplitude()
    {
      return d_Q_amp;
    }

    void
    gate::set_I_bandwidth(double bw)
    {
      d_I_bw = bw;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::I_BW), pmt::from_double(bw));
    }
    double
    gate::I_bandwidth()
    {
      return d_I_bw;
    }

    void
    gate::set_Q_bandwidth(double bw)
    {
      d_Q_bw = bw;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::Q_BW), pmt::from_double(bw));
    }
    double
    gate::Q_bandwidth()
    {
      return d_Q_bw;
    }


    void
    gate::set_processing_time_ns(double proc_time_ns)
    {
      d_proc_time_ns = proc_time_ns;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::PROC_TIME), pmt::from_double(proc_time_ns));
    }
    double
    gate::processing_time()
    {
      return d_proc_time_ns;
    }


    void
    gate::set_sample_rate(double rate)
    {
      d_samples_per_sec = rate;
    }

    double
    gate::sample_rate() const
    {
      return d_samples_per_sec;
    }

    void
    gate::set_qubit_ID(int qubit_ID)
    {
      d_qubit_ID = qubit_ID;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::QUBIT_ID), pmt::from_double(qubit_ID));
    }
    int
    gate::qubit_ID()
    {
      return d_qubit_ID;
    }

    void
    gate::set_ctrl_dc_mode(bool ctrl_dc_mode)
    {
      d_ctrl_dc_mode = ctrl_dc_mode;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::CTRL_DC_MODE), pmt::from_double(ctrl_dc_mode));
    }
    bool
    gate::ctrl_dc_mode()
    {
      return d_ctrl_dc_mode;
    }

    void
    gate::set_wait_time(int wait_time)
    {
      d_wait_time = wait_time;
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::WAIT_TIME), pmt::from_double(wait_time));
    }
    bool
    gate::wait_time()
    {
      return d_wait_time;
    }


    void
    gate::set_JUNC_list(std::queue<CTRL_junction_relation_t> junc_list)
    {
      int cnt = 0;
      while (!junc_list.empty()) {
        CTRL_junction_relation_t junc_params = junc_list.front();
        d_JUNC_list_PMT = pmt::dict_add(d_JUNC_list_PMT, pmt::from_float(cnt++), pmt::from_double(junc_params.qubit_ID));
        junc_list.pop();
      }
      d_gate_params_dict = pmt::dict_add(d_gate_params_dict, pmt::from_float(gate::JUNC_LIST), d_JUNC_list_PMT);
    }
    pmt::pmt_t
    gate::JUNC_list_PMT()
    {
      return d_JUNC_list_PMT;
    }

    pmt::pmt_t
    gate::get_parameters()
    {
      return d_gate_params_dict;
    }



  } /* namespace quantum */
} /* namespace gr */
