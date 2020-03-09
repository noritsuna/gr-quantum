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

#include "measurements_Detector_impl.h"
#include "qubit.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>
#include <limits>
#include <volk/volk.h>
#include <cstdlib>
#include <cmath>

static const int GR_QUANTUM_CDET_FFT_SIZE = 1024;
static const float GR_QUANTUM_CDET_MAG_AMP_RATE = 5.0f;
static const float GR_QUANTUM_CDET_AMP_ZERO_RATE = 0.000001f;
static const float GR_QUANTUM_CDET_AMP_ZERO_CHECK_NUM = 5;
static const float GR_QUANTUM_CDET_PHASE_THRESHOLD_RATE = 0.1f;
static const float GR_QUANTUM_CDET_FIR_DELAY_RATE = 0.1f;
static const float GR_QUANTUM_CDET_LINE_CYCLE_NUM = 4.0f;
static const float GR_QUANTUM_CDET_DIFF_CYCLE_RATE = 0.1f;

namespace gr {
  namespace quantum {

    measurements_Detector::sptr
    measurements_Detector::make(double qubit_num, double samples_per_sec, double time_scale_rate, double freq_threshold_rate, double amp_threshold_rate, double bw_threshold_rate, double proc_time_threshold_rate, bool qubit_pole)
    {
      return gnuradio::get_initial_sptr
        (new measurements_Detector_impl(qubit_num, samples_per_sec, time_scale_rate, freq_threshold_rate, amp_threshold_rate, bw_threshold_rate, proc_time_threshold_rate, qubit_pole));
    }

    measurements_Detector_impl::measurements_Detector_impl(double qubit_num, double samples_per_sec, double time_scale_rate, double freq_threshold_rate, double amp_threshold_rate, double bw_threshold_rate, double proc_time_threshold_rate, bool qubit_pole)
      : sync_block("measurements_Detector",
                      io_signature::make(1, -1, sizeof(gr_complex)),
                      io_signature::make(0, 0, 0)),
      d_port_in(pmt::mp("in")),
      d_port_out_feedback(pmt::mp("feedback"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Detector");
      set_qubit_num(qubit_num);
      set_FFT_size(GR_QUANTUM_CDET_FFT_SIZE);
      set_sample_rate(samples_per_sec);
      set_freq_threshold_rate(freq_threshold_rate);
      set_amp_threshold_rate(amp_threshold_rate);
      set_bw_threshold_rate(bw_threshold_rate);
      set_proc_time_threshold_rate(proc_time_threshold_rate);
      set_qubit_pole(qubit_pole);
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      d_is_nanosec_mode = true;
      set_time_scale_rate(time_scale_rate);
#else
      d_is_nanosec_mode = false;
      set_time_scale_rate(time_scale_rate/1000);
#endif

      d_RO_wave_buf = new gr_complex[GR_QUANTUM_CDET_FFT_SIZE];
      d_RO_vector_I_data = new float[GR_QUANTUM_CDET_FFT_SIZE];
      d_RO_vector_Q_data = new float[GR_QUANTUM_CDET_FFT_SIZE];

      for(int i = 0; i < (int)qubit_num; i++) {
        int q_id = i+1;
        RO_wave_state_machine_t* machine = &(d_RO_wave_state_machine[q_id]);
        d_RO_filter[q_id] = NULL;
        reset_state(machine);
      }


      if(((int)qubit_num) == 1) {
        message_port_register_out(pmt::mp("result"));
      } else {
        for(int i = 0; i < (int)qubit_num; i++) {
          std::string port_name = "result" + std::to_string(i);
          message_port_register_out(pmt::mp(port_name));
        }
      }
      message_port_register_in(d_port_in);
      set_msg_handler(d_port_in, boost::bind(&measurements_Detector_impl::handle_cmd_msg, this, _1));
      message_port_register_out(d_port_out_feedback);
    }

    measurements_Detector_impl::~measurements_Detector_impl()
    {
      delete d_RO_wave_buf;
      delete d_RO_vector_I_data;
      delete d_RO_vector_Q_data;
      for(int i = 0; i < qubit_num(); i++) {
        int q_id = i+1;
        RO_wave_state_machine_t* machine = &(d_RO_wave_state_machine[q_id]);
        delete &(d_RO_filter[q_id]);
      }
    }

    bool
    measurements_Detector_impl::start()
    {
      return block::start();
    }


    void
    measurements_Detector_impl::set_qubit_num(double qubit_num)
    {
      d_qubit_num = qubit_num;
    }
    double
    measurements_Detector_impl::qubit_num()
    {
      return d_qubit_num;
    }

    void
    measurements_Detector_impl::set_FFT_size(double FFT_size)
    {
      d_FFT_size = FFT_size;
    }
    double
    measurements_Detector_impl::FFT_size()
    {
      return d_FFT_size;
    }

    void
    measurements_Detector_impl::set_sample_rate(double rate)
    {
      //changing the sample rate performs a reset of state params
      d_samps_per_us = rate/1e6;
    }

    double
    measurements_Detector_impl::sample_rate()
    {
      return d_samps_per_us * 1e6;
    }

    void
    measurements_Detector_impl::set_time_scale_rate(double rate)
    {
      d_time_scale_rate = rate;
    }

    double
    measurements_Detector_impl::time_scale_rate()
    {
      return d_time_scale_rate;
    }

    void
    measurements_Detector_impl::set_freq_threshold_rate(double freq_threshold_rate)
    {
      d_freq_threshold_rate = freq_threshold_rate/100.0;
    }
    double
    measurements_Detector_impl::freq_threshold_rate()
    {
      return d_freq_threshold_rate;
    }

    void
    measurements_Detector_impl::set_amp_threshold_rate(double amp_threshold_rate)
    {
      d_amp_threshold_rate = amp_threshold_rate/100.0;
    }
    double
    measurements_Detector_impl::amp_threshold_rate()
    {
      return d_amp_threshold_rate;
    }

    void
    measurements_Detector_impl::set_bw_threshold_rate(double bw_threshold_rate)
    {
      d_bw_threshold_rate = bw_threshold_rate/100.0;
    }
    double
    measurements_Detector_impl::bw_threshold_rate()
    {
      return d_bw_threshold_rate;
    }

    void
    measurements_Detector_impl::set_proc_time_threshold_rate(double proc_time_threshold_rate)
    {
      d_proc_time_threshold_rate= proc_time_threshold_rate/100.0;
    }
    double
    measurements_Detector_impl::proc_time_threshold_rate()
    {
      return d_proc_time_threshold_rate;
    }

    void
    measurements_Detector_impl::set_qubit_pole(bool qubit_pole)
    {
      d_qubit_pole= qubit_pole;
    }
    bool
    measurements_Detector_impl::qubit_pole()
    {
      return d_qubit_pole;
    }

    
    gr_complex* 
    measurements_Detector_impl::RO_wave_buf()
    {
      return &d_RO_wave_buf[0];
    }
    float* 
    measurements_Detector_impl::RO_vector_I_data()
    {
      return &d_RO_vector_I_data[0];
    }
    float* 
    measurements_Detector_impl::RO_vector_Q_data()
    {
      return &d_RO_vector_Q_data[0];
    }    

    void
    measurements_Detector_impl::copy_gate_param_t(gate::gate_param_t* copyto, gate::gate_param_t* orig)
    {
      copyto->type = orig->type;
      copyto->freq = orig->freq;
      copyto->I_amp = orig->I_amp;
      copyto->Q_amp = orig->Q_amp;
      copyto->I_bw = orig->I_bw;
      copyto->Q_bw = orig->Q_bw;
      copyto->proc_time = orig->proc_time;
      copyto->DC_mode = orig->DC_mode;
    }
    void
    measurements_Detector_impl::set_params_to_list(int qubit_id, gate::gate_type_t type, pmt::pmt_t gate)
    {
      pmt::pmt_t gate_type = pmt::dict_ref(gate, pmt::from_float(gate::GATE_TYPE), pmt::PMT_NIL);
      if(pmt::eq(gate_type, pmt::PMT_NIL)) return;
      int gate_type_int = (int)pmt::to_float(gate_type);
      if(gate_type_int == type) {
        gate::gate_param_t* gate_tmp = &(d_RO_gate_params[qubit_id]);

        gate_tmp->type = type;
        gate_tmp->freq= pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::FREQ), pmt::PMT_NIL));
        gate_tmp->I_amp = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::I_AMP), pmt::PMT_NIL));
        gate_tmp->Q_amp = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::Q_AMP), pmt::PMT_NIL));
        gate_tmp->I_bw = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::I_BW), pmt::PMT_NIL));
        gate_tmp->Q_bw = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::Q_BW), pmt::PMT_NIL));
        gate_tmp->proc_time = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::PROC_TIME), pmt::PMT_NIL));

        gate_tmp->DC_mode = (bool)(pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::CTRL_DC_MODE), pmt::PMT_NIL)));

        std::vector<gr_complex> _tap;
        filter::kernel::fir_filter_ccc* _filter; 
        double base_freq = gate_tmp->freq;
        double low_freq = 0.0;
        double high_freq = 0.0;
        double trans_bw = 0.0;
        low_freq = base_freq - 1;
        if(low_freq < 1) {
          low_freq = 1;
          high_freq = 3;
        } else {
          high_freq = base_freq + 1;
        }
        trans_bw = 2.0;

        _tap = filter::firdes::complex_band_pass(1.0, sample_rate(), low_freq, high_freq, trans_bw);
        _filter = new filter::kernel::fir_filter_ccc(1, _tap);
        d_RO_filter[qubit_id] = _filter;

      }
    }

    void
    measurements_Detector_impl::handle_cmd_msg(pmt::pmt_t msg)
    {
      int gate_len = pmt::length(msg);
      int qubit_id = 0;
      GR_LOG_DEBUG(d_debug_logger, "set_params_to_list INIT");

      for(int i = 0; i < gate_len; i++) {
        pmt::pmt_t gate = pmt::dict_ref(msg, pmt::from_float(i), pmt::PMT_NIL);
        if(pmt::eq(gate, pmt::PMT_NIL)) {
          continue;
        }
        pmt::pmt_t type_value = pmt::dict_ref(gate, pmt::from_float(gate::GATE_TYPE), pmt::PMT_NIL);
        int type_int = (int)(pmt::to_float(type_value));
        if(type_int != gate::INIT) {
          continue;
        }

        qubit_id = (int)pmt::to_float(pmt::dict_ref(gate, pmt::from_float(gate::QUBIT_ID), pmt::PMT_NIL));
        break;
      }

      for(int i = 0; i <= gate_len; i++) {
        pmt::pmt_t gate = pmt::dict_ref(msg, pmt::from_float(i), pmt::PMT_NIL);
        if(pmt::eq(gate, pmt::PMT_NIL)) {
          continue;
        }
        set_params_to_list(qubit_id, gate::RO, gate);
      }

    }

    void
    measurements_Detector_impl::handle_SYNC_msg(pmt::pmt_t msg)
    {
    }


    void
    measurements_Detector_impl::copy_wave_result_t(wave_result_t* copyto, wave_result_t* orig)
    {
      copyto->is_detected_RO = orig->is_detected_RO;
      copyto->qubit_angle = orig->qubit_angle;
    }

    int
    measurements_Detector_impl::get_ZERO_wave_point(gr_complex* RO_wave, int len, gate::gate_param_t RO_gate, RO_wave_state_machine_t* state_machine)
    {
      gr_complex* _RO_wave = RO_wave;

      int vec_size = len;
      int start_point = 0;
      int detected_I_zero_linecnt = 0;
      int detected_Q_zero_linecnt = 0;
      double detected_I_prev_data = 0.0;
      double detected_Q_prev_data = 0.0;
      bool is_detected_wave = false;

      if(state_machine->wave_status == processing_START) {
        if(check_proc_time(GR_QUANTUM_CDET_FIR_DELAY_RATE, RO_gate, state_machine)) {
          state_machine->wave_status = proccessing_PHASE;
          state_machine->proc_timer.start(); 
        }
        return len;
      } else if(state_machine->wave_status == proccessing_PHASE) {
        if(check_proc_time(1 - GR_QUANTUM_CDET_FIR_DELAY_RATE, RO_gate, state_machine)) {
          state_machine->wave_status = proccessing_END;
          state_machine->proc_timer.start(); 
        }
      } else if(state_machine->wave_status == proccessing_END) {
        if(check_proc_time((1-GR_QUANTUM_CDET_FIR_DELAY_RATE)/2.0, RO_gate, state_machine)) {
          state_machine->wave_status = END_PHASE;
        }
        return len;
      }

      double min_I_amp = -RO_gate.I_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;
      double max_I_amp = RO_gate.I_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;
      double min_Q_amp = -RO_gate.Q_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;
      double max_Q_amp = RO_gate.Q_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;

      for(int i = 0; i < vec_size; i++) {
        if(!(min_I_amp < _RO_wave[i].real() && _RO_wave[i].real() <= max_I_amp)) {
          if(detected_I_zero_linecnt < GR_QUANTUM_CDET_AMP_ZERO_CHECK_NUM) {
            detected_I_prev_data = _RO_wave[i].real();
            detected_I_zero_linecnt++;
            continue;
          }
          start_point = i-(GR_QUANTUM_CDET_AMP_ZERO_CHECK_NUM+1);
          if(start_point < 0) {
            start_point = 0;
          }
          detected_I_zero_linecnt = 0;
          detected_I_prev_data = 0.0;
          if(state_machine->wave_status == CHECKING) {
            state_machine->detected_wave_type = I_wave;
            state_machine->proc_timer.start(); 
            state_machine->wave_status = processing_START;
          }
          break;
        } else {
          detected_I_zero_linecnt = 0;
          detected_I_prev_data = 0.0;
        }

        if(!(min_Q_amp < _RO_wave[i].imag() && _RO_wave[i].imag() <= max_Q_amp)) {
          if(detected_Q_zero_linecnt < GR_QUANTUM_CDET_AMP_ZERO_CHECK_NUM) {
            detected_Q_prev_data = _RO_wave[i].imag();
            detected_Q_zero_linecnt++;
            continue;
          }
          start_point = i-(GR_QUANTUM_CDET_AMP_ZERO_CHECK_NUM+1);
          if(start_point < 0) {
            start_point = 0;
          }
          detected_Q_zero_linecnt = 0;
          detected_Q_prev_data = 0.0;
          if(state_machine->wave_status == CHECKING) {
            state_machine->detected_wave_type = Q_wave;
            state_machine->proc_timer.start(); 
            state_machine->wave_status = processing_START;
          }
          break;
        } else {
          detected_Q_zero_linecnt = 0;
          detected_Q_prev_data = 0.0;
        }
      }
      return start_point;      
    }

    void
    measurements_Detector_impl::reset_wave_state(RO_wave_state_machine_t* state_machine)
    {
      if(state_machine->proc_timer.is_stopped()) {
        state_machine->proc_timer.start();
      }
      state_machine->proc_timer.stop();
      state_machine->wave_status = CHECKING;
    }

    void
    measurements_Detector_impl::reset_phase_state(RO_wave_state_machine_t* state_machine)
    {
      state_machine->phase_status = FIRST_PHASE;

      state_machine->prev_phase_data = 0.0f;
      state_machine->prev_phase_data_abs = 0.0f;
      state_machine->diff_phase_adv;

      state_machine->check_cycle_cnt = 0;
      state_machine->check_cycle_len = 0;
      state_machine->check_phase_rate = 0.0f;
      state_machine->line_cycle_cnt = 0;
      state_machine->total_diff_cycle_cnt = 0;

      state_machine->is_first_riseup_wave = false;
      state_machine->is_first_check = true;

      state_machine->is_detected_phase = false;
      state_machine->detected_phase_data = 0.0f;

    }


    void
    measurements_Detector_impl::reset_state_with_END(RO_wave_state_machine_t* state_machine)
    {
      if(state_machine->wave_status == END_PHASE) {
        reset_wave_state(state_machine);
        reset_phase_state(state_machine);
      }
    }
    void
    measurements_Detector_impl::reset_state(RO_wave_state_machine_t* state_machine)
    {
      reset_wave_state(state_machine);
      reset_phase_state(state_machine);
    }

    void
    measurements_Detector_impl::calc_changing_phase(RO_wave_state_machine_t* state_machine, wave_result_t* wave_ret)
    {
      state_machine->detected_phase_data = 3.141592653589793 * (state_machine->check_cycle_len/state_machine->total_diff_cycle_cnt);
      state_machine->is_detected_phase = true;
      wave_ret->qubit_angle = state_machine->detected_phase_data;
      wave_ret->is_detected_RO = true;
    }


    bool
    measurements_Detector_impl::detecte_changing_phase(float* buf_vec, int total_buf_vec_size, int qubit_ID, RO_wave_state_machine_t* state_machine, wave_result_t* wave_ret)
    {
      int buf_size = total_buf_vec_size;
      float* _buf_vec = buf_vec;
      wave_result_t* _wave_ret = wave_ret;
      bool is_riseup_wave = false;
      float cur_phase = 0.0f;
      float diff_phase = 0.0f;
      float diff_phase_abs = 0.0f;
      float prev_diff_phase_abs = 0.0f;
      float prev_data;
      float prev_data_abs;
      float cur_data;
      float cur_data_abs;
      int total_diff_abs = 0;
      bool is_start_point = false;


      //Already Detected Qubit...
      if(state_machine->is_detected_phase) {
        return false;
      }


      prev_data = state_machine->prev_phase_data;
      prev_data_abs = state_machine->prev_phase_data_abs;
      for(int i = 0; i < buf_size; i++) {
        cur_data  = _buf_vec[i];
        cur_data_abs  = std::abs(cur_data);

        //Set First ZERO point.
        if(state_machine->phase_status == FIRST_PHASE) {
          gate::gate_param_t RO_gate = d_RO_gate_params[qubit_ID];

          if(sample_rate() > RO_gate.freq) {
            state_machine->check_cycle_len = sample_rate()/RO_gate.freq;
          } else {
            state_machine->check_cycle_len = RO_gate.freq/sample_rate();
          }
          state_machine->check_cycle_cnt = 0;
          state_machine->check_phase_rate = state_machine->check_cycle_len * GR_QUANTUM_CDET_DIFF_CYCLE_RATE;
          if(_buf_vec[i] <= _buf_vec[i+1]) {
            is_riseup_wave = true;
          } else {
            is_riseup_wave = false;
          }
          state_machine->is_first_riseup_wave = is_riseup_wave;
          state_machine->diff_phase_adv = std::abs(cur_data_abs - std::abs(_buf_vec[i+1]));
          state_machine->phase_status = searching_PHASE;
          prev_data = cur_data;
          prev_data_abs = cur_data_abs;

          state_machine->is_first_check = true;
          continue;
        }

        //First Check
        diff_phase_abs = std::abs(cur_data_abs - prev_data_abs);
        if(diff_phase_abs < state_machine->diff_phase_adv*2.0) {
          if((cur_data*prev_data) <= 0) {
            if(prev_data < cur_data) {
              is_riseup_wave = true;
            } else {
              is_riseup_wave = false;
            }
            if(state_machine->is_first_riseup_wave == is_riseup_wave) {
              is_start_point = true;
            }
          }
        }
        state_machine->diff_phase_adv = (state_machine->diff_phase_adv + diff_phase_abs) / 2.0;
        state_machine->check_cycle_cnt++;
        prev_diff_phase_abs = diff_phase_abs;
        prev_data = cur_data;
        prev_data_abs = cur_data_abs;
        state_machine->prev_phase_data = cur_data;
        state_machine->prev_phase_data_abs = cur_data_abs;

        if(!is_start_point) continue;
        is_start_point = false;

        if(state_machine->phase_status == searching_PHASE) {
          //Check Changeing Phase
          state_machine->line_cycle_cnt++;
          state_machine->total_diff_cycle_cnt += (state_machine->check_cycle_len - state_machine->check_cycle_cnt);
          if(state_machine->total_diff_cycle_cnt >= state_machine->check_cycle_len) state_machine->total_diff_cycle_cnt -= state_machine->check_cycle_len;
          if(state_machine->total_diff_cycle_cnt <= -state_machine->check_cycle_len) state_machine->total_diff_cycle_cnt += state_machine->check_cycle_len;
          if(state_machine->line_cycle_cnt < GR_QUANTUM_CDET_LINE_CYCLE_NUM) {
            state_machine->check_cycle_cnt = 0;
            continue;
          }
          //Check changing phase in range.
          total_diff_abs = std::abs(state_machine->total_diff_cycle_cnt);
          if(((state_machine->check_cycle_len - state_machine->check_phase_rate) < total_diff_abs && total_diff_abs < (state_machine->check_cycle_len + state_machine->check_phase_rate)) || (0 <= total_diff_abs && total_diff_abs < state_machine->check_phase_rate)) {
            state_machine->line_cycle_cnt = 0;
            state_machine->total_diff_cycle_cnt = 0;
            state_machine->check_cycle_cnt = 0;
            continue;
          }
          //Wrong detect in first changing 0 -> freq phase
          if(state_machine->is_first_check == true) {
            state_machine->is_first_check = false;
            state_machine->line_cycle_cnt = 0;
            state_machine->check_cycle_cnt = 0;
            state_machine->total_diff_cycle_cnt = 0;
            continue;
          }

          state_machine->phase_status = END_PHASE;

        }

        if(state_machine->phase_status == END_PHASE) {
          calc_changing_phase(state_machine, _wave_ret);
          return true;
        }
      }
      return false;
    }

    bool
    measurements_Detector_impl::check_proc_time(float mag_rate, gate::gate_param_t RO_gate, RO_wave_state_machine_t* state_machine)
    {
      long cur_proc_time_cnt = 0;
      bool is_end = false;
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      auto cur_proc_time = boost::chrono::nanoseconds(state_machine->proc_timer.elapsed().wall);
      cur_proc_time_cnt = cur_proc_time.count();
#else
      auto cur_proc_time = boost::chrono::microseconds(state_machine->proc_timer.elapsed().wall);
      cur_proc_time_cnt = cur_proc_time.count() / 1000;
#endif
      double proc_RO_time = RO_gate.proc_time * time_scale_rate() * mag_rate;
      //Check over proc time.        
      if(proc_RO_time <= cur_proc_time_cnt) {
        if(state_machine->proc_timer.is_stopped()) {
          state_machine->proc_timer.start();
        }
        state_machine->proc_timer.stop();
        is_end = true;
      }
      return is_end;
    }

    void
    measurements_Detector_impl::analyze_RO_waves(wave_result_t* wave_ret, int qubit_id, gr_complex* RO_wave, int len)
    {
      int vec_size = len;
      int vec_max_size = GR_QUANTUM_CDET_FFT_SIZE;

      gr_complex* _RO_wave = RO_wave;
      float* vec_I = RO_vector_I_data();
      float* vec_Q = RO_vector_Q_data();
      double I_prev_data = 0.0;
      double  Q_prev_data = 0.0;
      bool is_end_RO_wave = false;
      int start_point = 0;

      wave_result_t* _wave_ret = wave_ret;
      
      gate::gate_param_t RO_gate = d_RO_gate_params[qubit_id];
      RO_wave_state_machine_t* state_machine = &(d_RO_wave_state_machine[qubit_id]);

      double min_I_amp = -RO_gate.I_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;
      double max_I_amp = RO_gate.I_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;
      double min_Q_amp = -RO_gate.Q_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;
      double max_Q_amp = RO_gate.Q_amp*GR_QUANTUM_CDET_AMP_ZERO_RATE;

      start_point = get_ZERO_wave_point(_RO_wave, vec_size, RO_gate, state_machine);
      if(state_machine->wave_status == proccessing_PHASE) {
        if(start_point > 0) {
          vec_size -= start_point;
          std::memcpy(&_RO_wave[0], &_RO_wave[start_point], sizeof(gr_complex) * vec_size);
        }
      } else if(state_machine->wave_status == END_PHASE) {
        if(!(state_machine->is_detected_phase)) {
          calc_changing_phase(state_machine, _wave_ret);
        }
        reset_state_with_END(state_machine);
        return;
      } else {
        return;
      }

      volk_32fc_deinterleave_32f_x2(vec_I,
                                    vec_Q,
                                    _RO_wave,
                                    vec_size);

      float* target_vec;
      if(state_machine->detected_wave_type == I_wave) {
        target_vec = vec_I;
      } else if(state_machine->detected_wave_type == Q_wave) {
        target_vec = vec_Q;
      }

      detecte_changing_phase(target_vec, vec_size, qubit_id, state_machine, _wave_ret);
      reset_state_with_END(state_machine);
      return;

    }

    int
    measurements_Detector_impl::work(int noutput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
    {
      gr::thread::scoped_lock lock(d_setlock);

      int vec_size = GR_QUANTUM_CDET_FFT_SIZE;
      int item_size = noutput_items;
      int proc_items_unit = item_size/vec_size;
      int port_set_num = input_items.size()/qubit_num();
      int proc_item_num = 0;
      int proced_item_num = 0;

      for (int unit = 0; unit <= proc_items_unit; unit++) {
        std::map<QUANTUM_QUBID_ID, wave_result_t> result;

        if(unit == proc_items_unit) {
          proc_item_num = (item_size%vec_size);
          if(proc_item_num == 0) continue; 
        } else {
          proc_item_num = vec_size;
        }
        for(int in_port = 0; in_port < port_set_num; in_port++) {
          for(int RO_port = 0; RO_port < (int)qubit_num(); RO_port++) {
            QUANTUM_QUBID_ID qubit_id = RO_port+1;
            filter::kernel::fir_filter_ccc* filter = d_RO_filter[qubit_id];
            if(filter == NULL) continue;
            wave_result_t wave_ret = result[qubit_id];


            gr_complex* _RO_wave_orig = (gr_complex*)input_items[RO_port + (in_port*qubit_num())];

            filter->filterN(RO_wave_buf(), &_RO_wave_orig[unit*vec_size], proc_item_num);

            analyze_RO_waves(&wave_ret,
                              qubit_id,
                              RO_wave_buf(),
                              proc_item_num);

            copy_wave_result_t(&result[qubit_id], &wave_ret);

          }
          for(int RO_port = 0; RO_port < (int)qubit_num(); RO_port++) {
            QUANTUM_QUBID_ID qubit_id = in_port+1;
            wave_result_t wave_ret = result[qubit_id];
            if(wave_ret.is_detected_RO) {
              std::string port_name;
              pmt::pmt_t qubit_ret_msg = pmt::make_dict();

              float q_angle = wave_ret.qubit_angle;
              // if(!qubit_pole()) {
              //   q_angle = -q_angle;
              // }
              qubit_ret_msg = pmt::dict_add(qubit_ret_msg, pmt::from_float(qubit::ID), pmt::from_float(qubit_id));
              qubit_ret_msg = pmt::dict_add(qubit_ret_msg, pmt::from_float(qubit::ANGLE), pmt::from_float(q_angle));
              qubit_ret_msg = pmt::dict_add(qubit_ret_msg, pmt::from_float(qubit::POLE), pmt::from_float(qubit_pole()));

              if(((int)qubit_num()) == 1) {
                port_name = "result";
              } else {
                port_name = "result" + std::to_string(qubit_id-1);
              }
              message_port_pub(pmt::mp(port_name), qubit_ret_msg);
            }
          }
        }
      }
      return noutput_items;
    }

  } /* namespace quantum */
} /* namespace gr */
