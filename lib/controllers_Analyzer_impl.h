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

#ifndef INCLUDED_GR_QUANTUM_CONTROLLERS_ANALYZER_IMPL_H
#define INCLUDED_GR_QUANTUM_CONTROLLERS_ANALYZER_IMPL_H

#include "gate.h"
#include "qubit.h"
#include <quantum/controllers_Analyzer.h>
#include <map> 
#include <queue> 
#include <vector> 
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/fft_vfc.h>
#include <boost/timer/timer.hpp>
#include <gnuradio/logger.h>
#include "controlled_sig_source.h"

namespace gr {
  namespace quantum {

    class controllers_Analyzer_impl : public controllers_Analyzer
    {
    private:

      double d_qubit_num;
      double d_samps_per_us;
      double d_time_scale_rate;
      double d_freq_threshold_rate;
      double d_amp_threshold_rate; 
      double d_bw_threshold_rate;
      double d_proc_time_threshold_rate;
      bool d_SYNC_port;
      const pmt::pmt_t d_port_in;
      const pmt::pmt_t d_port_out;
      const pmt::pmt_t d_port_ANALY_out;
      const pmt::pmt_t d_port_sync_in;

      bool d_is_nanosec_mode;

      fft::fft_real_fwd* d_fft;
      gr_complex* d_gate_ffted_I_data;
      gr_complex* d_gate_ffted_Q_data;
      gr_complex* d_CTRL_ffted_I_data;
      gr_complex* d_CTRL_ffted_Q_data;
      gr_complex* d_RO_ffted_I_data;
      gr_complex* d_RO_ffted_Q_data;
      double d_fft_freq_step_Hz;
      float* d_gate_I_wave;
      float* d_gate_Q_wave;
      float* d_CTRL_I_wave;
      float* d_CTRL_Q_wave;
      float* d_RO_I_wave;
      float* d_RO_Q_wave;
      float* d_gate_mag_I_data;
      float* d_gate_mag_Q_data;
      float* d_CTRL_mag_I_data;
      float* d_CTRL_mag_Q_data;
      float* d_RO_mag_I_data;
      float* d_RO_mag_Q_data;


#define QUANTUM_QUBID_ID int
#define QUANTUM_COUNTER int
#define QUANTUM_INIT_COUNTER int

      controlled_sig_source<gr_complex>* d_signale_maker;
      gr::logger_ptr d_logger;
      gr::logger_ptr d_debug_logger;

      boost::timer::cpu_timer d_proc_rate_timer;

      enum proc_status_type_t {
        NOP = 0,
        proccessing_BW,
        END_BW
      };
      
      enum port_type_t {
        GATE = 0,
        CTRL,
        RO
      };

      struct qubit_proc_state_t {
        proc_status_type_t proc_status;
        boost::timer::cpu_timer proc_timer;
        gate::gate_type_t processing_gate_type;
        QUANTUM_QUBID_ID JUNC_num;
        QUANTUM_INIT_COUNTER INIT_num;
      };

      struct qubit_state_machine_t {
        qubit_proc_state_t gate_proc_status;
        qubit_proc_state_t CTRL_proc_status;
        qubit_proc_state_t RO_proc_status;
        qubit_proc_state_t JUNC_proc_status;
        qubit_proc_state_t INIT_proc_status;

        int JUNC_counter;
        int INIT_counter;
        int INIT_stacked_num;
        std::map<gate::gate_type_t, gate::gate_param_t> gates_params_list;
        gate::gate_param_t RO_params;
        gate::gate_param_t CTRL_params;
        std::map<QUANTUM_QUBID_ID, gate::gate_param_t> JUNC_params_list;
        std::map<QUANTUM_INIT_COUNTER, gate::gate_param_t> INIT_params_list;

        std::queue<gate::gate_param_t> stacked_qubits;
      };

      struct wave_result_t {
        gate::gate_type_t detected_gate_type;
        QUANTUM_QUBID_ID detected_JUNC_qubit_id;
        QUANTUM_INIT_COUNTER detected_INIT_num;

        bool is_detected_gate;
        bool is_detected_CTRL;
        bool is_detected_RO;
        bool is_detected_INIT;
        bool is_detected_JUNC;
      };
      struct RO_wave_state_machine_t {
        int qubit_ID;
        int wave_cur_num;
        long proc_time_start;
        long proc_time_cur;
        long proc_time_end;
        proc_status_type_t status;
        gate::CTRL_junction_relation_t CTRL_relation;
      };

      std::map<QUANTUM_QUBID_ID, qubit_state_machine_t> d_qubits_state_machine;
      std::map<QUANTUM_QUBID_ID, RO_wave_state_machine_t> d_RO_wave_state_machine;
      std::map<QUANTUM_QUBID_ID, std::queue<gate::CTRL_junction_relation_t>> d_CTRL_junction_relation_list;
            


      void set_params_to_list(int qubit_id, gate::gate_type_t type, pmt::pmt_t msg);
      void analyze_qubit_waves(wave_result_t* wave_ret, int qubit_id, gr_complex* gate_wave, gr_complex* CTRL_wave, gr_complex* RO_wave);
      double volt2db(double amp);
      double mag2amp(double mag);
      bool check_I_wave_null(gate::gate_param_t gate);
      bool check_Q_wave_null(gate::gate_param_t gate);
      QUANTUM_QUBID_ID analyze_JUNC_by_mag(float* mag_I_data, float* mag_Q_data,std::map<QUANTUM_QUBID_ID, gate::gate_param_t> params_list);
      QUANTUM_INIT_COUNTER analyze_INIT_by_mag(float* mag_I_data, float* mag_Q_data,std::map<QUANTUM_INIT_COUNTER, gate::gate_param_t> params_list);
      gate::gate_type_t analyze_gate_type_by_mag(float* mag_I_data, float* mag_Q_data, std::map<gate::gate_type_t, gate::gate_param_t> params_list);
      bool analyze_params_by_mag(float* mag_I_data, float* mag_Q_data, gate::gate_param_t params);
      bool check_BW_proc_time(qubit_proc_state_t* proc_state, gate::gate_param_t gate, int state_type, proc_status_type_t exec_flag);
      bool check_wave_range(float* mag_data, double target_freq, double target_amp, double target_bw, bool DC_mode);
      void copy_gate_param_t(gate::gate_param_t* copyto, gate::gate_param_t* orig);
      void copy_wave_result_t(wave_result_t* copyto, wave_result_t* orig);
      void convert_waves2IQmags(gr_complex* gate_wave, gr_complex* CTRL_wave, gr_complex* RO_wave);
      gr_complex* gate_ffted_I_data();
      gr_complex* gate_ffted_Q_data();
      gr_complex* CTRL_ffted_I_data();
      gr_complex* CTRL_ffted_Q_data();
      gr_complex* RO_ffted_I_data();
      gr_complex* RO_ffted_Q_data();
      float* gate_I_wave();
      float* gate_Q_wave();
      float* CTRL_I_wave();
      float* CTRL_Q_wave();
      float* RO_I_wave();
      float* RO_Q_wave();
      float* gate_mag_I_data();
      float* gate_mag_Q_data();
      float* CTRL_mag_I_data();
      float* CTRL_mag_Q_data();
      float* RO_mag_I_data();
      float* RO_mag_Q_data();

    public:
      controllers_Analyzer_impl(double qubit_num, double samples_per_sec, double time_scale_rate, double freq_threshold_rate, double amp_threshold_rate, double bw_threshold_rate, double proc_time_threshold_rate, bool show_SYNC_port=true);
      ~controllers_Analyzer_impl();

      // Overloading gr::block::start to reset timer
      bool start();

      void set_qubit_num(double qubit_num);
      double qubit_num();

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

      void set_SYNC_port(bool is_use);
      bool SYNC_port() const;

      void handle_cmd_msg(pmt::pmt_t msg);
      void handle_SYNC_msg(pmt::pmt_t msg);

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
    };

  } /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLERS_ANALYZER_IMPL_H */
