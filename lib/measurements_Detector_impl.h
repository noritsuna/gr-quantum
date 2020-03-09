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

#ifndef INCLUDED_GR_QUANTUM_MEASUREMENTS_DETECTOR_IMPL_H
#define INCLUDED_GR_QUANTUM_MEASUREMENTS_DETECTOR_IMPL_H

#include "gate.h"
#include <quantum/measurements_Detector.h>
#include <map> 
#include <queue> 
#include <vector> 
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/fir_filter.h>
#include <boost/timer/timer.hpp>
#include <gnuradio/logger.h>
#include "controlled_sig_source.h"


#include <iostream>
#include <fstream>

namespace gr {
  namespace quantum {

    class measurements_Detector_impl : public measurements_Detector
    {
    private:

const char *fileName = "RO_wave_data.csv";
std::ofstream ofs;

#define QUANTUM_QUBID_ID int
#define QUANTUM_PREV_CNT int
#define QUANTUM_PHASE_CNT int

      double d_qubit_num;
      double d_samps_per_us;
      double d_FFT_size;
      double d_time_scale_rate;
      double d_freq_threshold_rate;
      double d_amp_threshold_rate; 
      double d_bw_threshold_rate;
      double d_proc_time_threshold_rate;
      bool d_qubit_pole;

      const pmt::pmt_t d_port_in;
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_out_feedback;

      bool d_is_nanosec_mode;

      gr_complex* d_RO_wave_buf;      
      float* d_RO_vector_I_data;
      float* d_RO_vector_Q_data;

      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

      enum proc_status_type_t {
        CHECKING = 0,
        processing_START,
        proccessing_PHASE,
        proccessing_END,
        searching_PHASE,
        FIRST_PHASE,
        END_PHASE
      };
      enum wave_type_t {
        NONE = 0,
        I_wave,
        Q_wave
      };
      
      struct wave_result_t {
        float qubit_angle;
        bool is_detected_RO;
      };
      struct RO_wave_state_machine_t {
        int qubit_ID;

        proc_status_type_t wave_status;
        boost::timer::cpu_timer proc_timer;
        wave_type_t detected_wave_type;
        double* buffer_data;

        proc_status_type_t phase_status;
        float prev_phase_data;
        float prev_phase_data_abs;
        float diff_phase_adv;

        int check_cycle_cnt;
        int check_cycle_len;
        int check_phase_rate;
        int line_cycle_cnt;
        int total_diff_cycle_cnt;

        bool is_first_riseup_wave;
        bool is_first_check;

        bool is_detected_phase;
        float detected_phase_data;
      };

      std::map<QUANTUM_QUBID_ID, RO_wave_state_machine_t> d_RO_wave_state_machine;
      std::map<QUANTUM_QUBID_ID, gate::gate_param_t> d_RO_gate_params;
      std::map<QUANTUM_QUBID_ID, filter::kernel::fir_filter_ccc*> d_RO_filter;

      bool detecte_changing_phase(float* buf_vec, int total_buf_vec_size, int qubit_ID, RO_wave_state_machine_t* state_machine, wave_result_t* wave_ret);
      int get_ZERO_wave_point(gr_complex* RO_wave, int len, gate::gate_param_t RO_gate, RO_wave_state_machine_t* state_machine);
      void analyze_RO_waves(wave_result_t* wave_ret, int qubit_id, gr_complex* RO_wave, int len);
      void set_params_to_list(int qubit_id, gate::gate_type_t type, pmt::pmt_t gate);
      void calc_changing_phase(RO_wave_state_machine_t* state_machine, wave_result_t* wave_ret);
      void reset_phase_state(RO_wave_state_machine_t* state_machine);
      void reset_wave_state(RO_wave_state_machine_t* state_machine);
      void reset_state_with_END(RO_wave_state_machine_t* state_machine);
      void reset_state(RO_wave_state_machine_t* state_machine);
      bool check_proc_time(float mag_rate, gate::gate_param_t RO_gate, RO_wave_state_machine_t* state_machine);
      void copy_gate_param_t(gate::gate_param_t* copyto, gate::gate_param_t* orig);
      void copy_wave_result_t(wave_result_t* copyto, wave_result_t* orig);
      void convert_waves2IQmags(gr_complex* RO_wave);
      
      gr_complex* RO_wave_buf();
      float* RO_vector_I_data();
      float* RO_vector_Q_data();


    public:
      measurements_Detector_impl(double qubit_num, double samples_per_sec, double time_scale_rate, double freq_threshold_rate, double amp_threshold_rate, double bw_threshold_rate, double proc_time_threshold_rate, bool qubit_pole);
      ~measurements_Detector_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void set_qubit_num(double qubit_num);
      double qubit_num();

      void set_FFT_size(double fft_size);
      double FFT_size();

      void set_sample_rate(double rate);
      double sample_rate();

      void set_time_scale_rate(double time_scale_rate);
      double time_scale_rate();

      void set_freq_threshold_rate(double freq_threshold_rate);
      double freq_threshold_rate();

      void set_amp_threshold_rate(double amp_threshold_rate);
      double amp_threshold_rate();

      void set_bw_threshold_rate(double bw_threshold_rate);
      double bw_threshold_rate();

      void set_proc_time_threshold_rate(double proc_time_threshold_rate);
      double proc_time_threshold_rate();

      void set_qubit_pole(bool qubit_pole);
      bool qubit_pole();

      void handle_cmd_msg(pmt::pmt_t msg);
      void handle_SYNC_msg(pmt::pmt_t msg);

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_MEASUREMENTS_DETECTOR_IMPL_H */
