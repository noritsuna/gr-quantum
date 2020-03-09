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

#include "controllers_Analyzer_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>
#include <limits>
#include <volk/volk.h>


static const int GR_QUANTUM_CSIM_FFT_SIZE = 1024;
static const int GR_QUANTUM_CSIM_PORT_BASE_NUM = 3;
static const float GR_QUANTUM_CSIM_MAG_AMP_RATE = 5.0f;
static const bool GR_QUANTUM_CSIM_STRICT_IQ_CHECK_MODE = false;

namespace gr {
  namespace quantum {

    controllers_Analyzer::sptr
    controllers_Analyzer::make(double qubit_num, double samples_per_sec, double time_scale_rate, double freq_threshold_rate, double amp_threshold_rate, double bw_threshold_rate, double proc_time_threshold_rate, bool show_SYNC_port)
    {
      return gnuradio::get_initial_sptr
        (new controllers_Analyzer_impl(qubit_num, samples_per_sec, time_scale_rate, freq_threshold_rate, amp_threshold_rate, bw_threshold_rate, proc_time_threshold_rate, show_SYNC_port));
    }

    controllers_Analyzer_impl::controllers_Analyzer_impl(double qubit_num, double samples_per_sec, double time_scale_rate, double freq_threshold_rate, double amp_threshold_rate, double bw_threshold_rate, double proc_time_threshold_rate, bool show_SYNC_port)
      : sync_block("controllers_Analyzer",
                      io_signature::make(GR_QUANTUM_CSIM_PORT_BASE_NUM, -1, sizeof(gr_complex)),
                      io_signature::make(0, 0, sizeof(gr_complex))),
      d_port_ANALY_out(pmt::mp("analyzed_data")),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in")),
      d_port_sync_in(pmt::mp("SYNC_CLK_in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Analyzer");
      set_qubit_num(qubit_num);
      set_sample_rate(samples_per_sec);
      set_freq_threshold_rate(freq_threshold_rate);
      set_amp_threshold_rate(amp_threshold_rate);
      set_bw_threshold_rate(bw_threshold_rate);
      set_SYNC_port(show_SYNC_port);

      d_fft = new fft::fft_real_fwd(GR_QUANTUM_CSIM_FFT_SIZE);
      d_gate_ffted_I_data = new gr_complex[GR_QUANTUM_CSIM_FFT_SIZE];
      d_gate_ffted_Q_data = new gr_complex[GR_QUANTUM_CSIM_FFT_SIZE];
      d_CTRL_ffted_I_data = new gr_complex[GR_QUANTUM_CSIM_FFT_SIZE];
      d_CTRL_ffted_Q_data = new gr_complex[GR_QUANTUM_CSIM_FFT_SIZE];
      d_RO_ffted_I_data = new gr_complex[GR_QUANTUM_CSIM_FFT_SIZE];
      d_RO_ffted_Q_data = new gr_complex[GR_QUANTUM_CSIM_FFT_SIZE];
      d_fft_freq_step_Hz = (double)(samples_per_sec)/(double)(GR_QUANTUM_CSIM_FFT_SIZE);

      d_gate_I_wave = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_gate_Q_wave = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_CTRL_I_wave = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_CTRL_Q_wave = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_RO_I_wave = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_RO_Q_wave = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_gate_mag_I_data = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_gate_mag_Q_data = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_CTRL_mag_I_data = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_CTRL_mag_Q_data = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_RO_mag_I_data = new float[GR_QUANTUM_CSIM_FFT_SIZE];
      d_RO_mag_Q_data = new float[GR_QUANTUM_CSIM_FFT_SIZE];

      d_signale_maker = new controlled_sig_source<gr_complex>(samples_per_sec, controlled_sig_source<gr_complex>::QUANTUM_SIN_WAVE, 1, 0.0);


#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      d_is_nanosec_mode = true;
      set_time_scale_rate(time_scale_rate);
#else
      d_is_nanosec_mode = false;
      set_time_scale_rate(time_scale_rate/1000);
#endif
      set_proc_time_threshold_rate(proc_time_threshold_rate);
      d_proc_rate_timer.start();

      message_port_register_out(d_port_out);
      message_port_register_out(d_port_ANALY_out);
      message_port_register_in(d_port_in);
      message_port_register_in(d_port_sync_in);
      set_msg_handler(d_port_in, boost::bind(&controllers_Analyzer_impl::handle_cmd_msg, this, _1));
      set_msg_handler(d_port_sync_in, boost::bind(&controllers_Analyzer_impl::handle_SYNC_msg, this, _1));

    }

    controllers_Analyzer_impl::~controllers_Analyzer_impl()
    {
      delete d_signale_maker;
      delete d_fft;
      delete d_gate_ffted_I_data;
      delete d_gate_ffted_Q_data;
      delete d_CTRL_ffted_I_data;
      delete d_CTRL_ffted_Q_data;
      delete d_RO_ffted_I_data;
      delete d_RO_ffted_Q_data;
      delete d_gate_I_wave;
      delete d_gate_Q_wave;
      delete d_CTRL_I_wave;
      delete d_CTRL_Q_wave;
      delete d_RO_I_wave;
      delete d_RO_Q_wave;
      delete d_gate_mag_I_data;
      delete d_gate_mag_Q_data;
      delete d_CTRL_mag_I_data;
      delete d_CTRL_mag_Q_data;
      delete d_RO_mag_I_data;
      delete d_RO_mag_Q_data;
    }

    bool
    controllers_Analyzer_impl::start()
    {
      return block::start();
    }

    void
    controllers_Analyzer_impl::set_qubit_num(double qubit_num)
    {
      d_qubit_num = qubit_num;
    }
    double
    controllers_Analyzer_impl::qubit_num()
    {
      return d_qubit_num;
    }

    void
    controllers_Analyzer_impl::set_sample_rate(double rate)
    {
      //changing the sample rate performs a reset of state params
      d_samps_per_us = rate/1e6;
    }

    double
    controllers_Analyzer_impl::sample_rate()
    {
      return d_samps_per_us * 1e6;
    }

    void
    controllers_Analyzer_impl::set_time_scale_rate(double rate)
    {
      d_time_scale_rate = rate;
    }

    double
    controllers_Analyzer_impl::time_scale_rate()
    {
      return d_time_scale_rate;
    }


    void
    controllers_Analyzer_impl::set_freq_threshold_rate(double freq_threshold_rate)
    {
      d_freq_threshold_rate = freq_threshold_rate/100.0;
    }
    double
    controllers_Analyzer_impl::freq_threshold_rate()
    {
      return d_freq_threshold_rate;
    }

    void
    controllers_Analyzer_impl::set_amp_threshold_rate(double amp_threshold_rate)
    {
      d_amp_threshold_rate = amp_threshold_rate/100.0;
    }
    double
    controllers_Analyzer_impl::amp_threshold_rate()
    {
      return d_amp_threshold_rate;
    }

    void
    controllers_Analyzer_impl::set_bw_threshold_rate(double bw_threshold_rate)
    {
      d_bw_threshold_rate = bw_threshold_rate/100.0;
    }
    double
    controllers_Analyzer_impl::bw_threshold_rate()
    {
      return d_bw_threshold_rate;
    }

    void
    controllers_Analyzer_impl::set_proc_time_threshold_rate(double proc_time_threshold_rate)
    {
      d_proc_time_threshold_rate= proc_time_threshold_rate/100.0;
    }
    double
    controllers_Analyzer_impl::proc_time_threshold_rate()
    {
      return d_proc_time_threshold_rate;
    }

    void
    controllers_Analyzer_impl::set_SYNC_port(bool is_use) {
        d_SYNC_port = !is_use;
    }

    bool
    controllers_Analyzer_impl::SYNC_port() const{
      return d_SYNC_port;
    }

    gr_complex* 
    controllers_Analyzer_impl::gate_ffted_I_data()
    {
      return &d_gate_ffted_I_data[0];
    }
    gr_complex* 
    controllers_Analyzer_impl::gate_ffted_Q_data()
    {
      return &d_gate_ffted_Q_data[0];
    }
    gr_complex* 
    controllers_Analyzer_impl::CTRL_ffted_I_data()
    {
      return &d_CTRL_ffted_I_data[0];
    }
    gr_complex* 
    controllers_Analyzer_impl::CTRL_ffted_Q_data()
    {
      return &d_CTRL_ffted_Q_data[0];
    }
    gr_complex* 
    controllers_Analyzer_impl::RO_ffted_I_data()
    {
      return &d_RO_ffted_I_data[0];
    }
    gr_complex* 
    controllers_Analyzer_impl::RO_ffted_Q_data()
    {
      return &d_RO_ffted_Q_data[0];
    }
    float* 
    controllers_Analyzer_impl::gate_I_wave()
    {
      return &d_gate_I_wave[0];
    }
    float* 
    controllers_Analyzer_impl::gate_Q_wave()
    {
      return &d_gate_Q_wave[0];
    }
    float* 
    controllers_Analyzer_impl::CTRL_I_wave()
    {
      return &d_CTRL_I_wave[0];
    }
    float* 
    controllers_Analyzer_impl::CTRL_Q_wave()
    {
      return &d_CTRL_Q_wave[0];
    }
    float* 
    controllers_Analyzer_impl::RO_I_wave()
    {
      return &d_RO_I_wave[0];
    }
    float* 
    controllers_Analyzer_impl::RO_Q_wave()
    {
      return &d_RO_Q_wave[0];
    }
    float* 
    controllers_Analyzer_impl::gate_mag_I_data()
    {
      return &d_gate_mag_I_data[0];
    }
    float* 
    controllers_Analyzer_impl::gate_mag_Q_data()
    {
      return &d_gate_mag_Q_data[0];
    }
    float* 
    controllers_Analyzer_impl::CTRL_mag_I_data()
    {
      return &d_CTRL_mag_I_data[0];
    }
    float* 
    controllers_Analyzer_impl::CTRL_mag_Q_data()
    {
      return &d_CTRL_mag_Q_data[0];
    }
    float* 
    controllers_Analyzer_impl::RO_mag_I_data()
    {
      return &d_RO_mag_I_data[0];
    }
    float* 
    controllers_Analyzer_impl::RO_mag_Q_data()
    {
      return &d_RO_mag_Q_data[0];
    }
    

    void
    controllers_Analyzer_impl::copy_gate_param_t(gate::gate_param_t* copyto, gate::gate_param_t* orig)
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
    controllers_Analyzer_impl::set_params_to_list(int qubit_id, gate::gate_type_t type, pmt::pmt_t gate)
    {
      pmt::pmt_t gate_type = pmt::dict_ref(gate, pmt::from_float(gate::GATE_TYPE), pmt::PMT_NIL);
      if(pmt::eq(gate_type, pmt::PMT_NIL)) return;
      int gate_type_int = (int)pmt::to_float(gate_type);
      if(gate_type_int == type) {
        gate::gate_param_t gate_tmp;
        qubit_state_machine_t *stat = &(d_qubits_state_machine[qubit_id]);

        gate_tmp.type = type;
        gate_tmp.freq= pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::FREQ), pmt::PMT_NIL));
        gate_tmp.I_amp = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::I_AMP), pmt::PMT_NIL));
        gate_tmp.Q_amp = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::Q_AMP), pmt::PMT_NIL));
        gate_tmp.I_bw = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::I_BW), pmt::PMT_NIL));
        gate_tmp.Q_bw = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::Q_BW), pmt::PMT_NIL));
        gate_tmp.proc_time = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::PROC_TIME), pmt::PMT_NIL));

        gate_tmp.DC_mode = (bool)(pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::CTRL_DC_MODE), pmt::PMT_NIL)));

        switch(type) {
          case gate::INIT:
//TODO Do not yet support the multi INIT
            copy_gate_param_t(&(stat->INIT_params_list[stat->INIT_stacked_num++]), &gate_tmp);
            break;
          case gate::CNOT:
            copy_gate_param_t(&(stat->CTRL_params), &gate_tmp);
            break;
          case gate::RO:
            copy_gate_param_t(&(stat->RO_params), &gate_tmp);
            break;
          case gate::JUNC:
            copy_gate_param_t(&(stat->JUNC_params_list[qubit_id]), &gate_tmp);
            break;
          default:
            copy_gate_param_t(&(stat->gates_params_list[type]), &gate_tmp);
            break;
        }        
      }
    }

    void
    controllers_Analyzer_impl::handle_cmd_msg(pmt::pmt_t msg)
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
        qubit_state_machine_t *stat = &(d_qubits_state_machine[qubit_id]);
        stat->gate_proc_status.proc_status = NOP;
        stat->CTRL_proc_status.proc_status = NOP;
        stat->RO_proc_status.proc_status = NOP;
        stat->JUNC_proc_status.proc_status = NOP;
        stat->INIT_proc_status.proc_status = NOP;
        stat->JUNC_proc_status.JUNC_num = -1;
        stat->INIT_proc_status.INIT_num = -1;
        stat->JUNC_counter = 0;
        stat->INIT_stacked_num = 0;
        break;
      }

      for(int i = 0; i <= gate_len; i++) {
        pmt::pmt_t gate = pmt::dict_ref(msg, pmt::from_float(i), pmt::PMT_NIL);
        if(pmt::eq(gate, pmt::PMT_NIL)) {
          continue;
        }
        
        set_params_to_list(qubit_id, gate::X, gate);
        set_params_to_list(qubit_id, gate::Y, gate);
        set_params_to_list(qubit_id, gate::Z, gate);
        set_params_to_list(qubit_id, gate::H, gate);
        set_params_to_list(qubit_id, gate::T, gate);
        set_params_to_list(qubit_id, gate::S, gate);
        set_params_to_list(qubit_id, gate::INIT, gate);
        set_params_to_list(qubit_id, gate::JUNC, gate);
        set_params_to_list(qubit_id, gate::CNOT, gate);
        set_params_to_list(qubit_id, gate::RO, gate);
      }

      message_port_pub(d_port_out, msg);

    }

    void
    controllers_Analyzer_impl::handle_SYNC_msg(pmt::pmt_t msg)
    {
    }

    void
    controllers_Analyzer_impl::convert_waves2IQmags(gr_complex* gate_wave, gr_complex* CTRL_wave, gr_complex* RO_wave)
    {
      int fft_size = GR_QUANTUM_CSIM_FFT_SIZE;
      gr_complex* _gate_wave = gate_wave;
      gr_complex* _CTRL_wave = CTRL_wave; 
      gr_complex* _RO_wave = RO_wave;

      float* _gate_I_wave = gate_I_wave();
      float* _gate_Q_wave = gate_Q_wave();
      gr_complex* _gate_ffted_I_data = gate_ffted_I_data();
      gr_complex* _gate_ffted_Q_data = gate_ffted_Q_data();
      float* _gate_mag_I_data = gate_mag_I_data();
      float* _gate_mag_Q_data = gate_mag_Q_data();

      float* _CTRL_I_wave = CTRL_I_wave();
      float* _CTRL_Q_wave = CTRL_Q_wave();
      gr_complex* _CTRL_ffted_I_data = CTRL_ffted_I_data();
      gr_complex* _CTRL_ffted_Q_data = CTRL_ffted_Q_data();
      float* _CTRL_mag_I_data = CTRL_mag_I_data();
      float* _CTRL_mag_Q_data = CTRL_mag_Q_data();

      float* _RO_I_wave = RO_I_wave();
      float* _RO_Q_wave = RO_Q_wave();
      gr_complex* _RO_ffted_I_data = RO_ffted_I_data();
      gr_complex* _RO_ffted_Q_data = RO_ffted_Q_data();
      float* _RO_mag_I_data = RO_mag_I_data();
      float* _RO_mag_Q_data = RO_mag_Q_data();

      volk_32fc_deinterleave_32f_x2(_gate_I_wave, _gate_Q_wave, _gate_wave, fft_size);
      volk_32fc_deinterleave_32f_x2(_CTRL_I_wave, _CTRL_Q_wave, _CTRL_wave, fft_size);
      volk_32fc_deinterleave_32f_x2(_RO_I_wave, _RO_Q_wave, _RO_wave, fft_size);

      memcpy(d_fft->get_inbuf(), _gate_I_wave, fft_size);
      d_fft->execute();
      memcpy(_gate_ffted_I_data, d_fft->get_outbuf(), fft_size);
      memcpy(d_fft->get_inbuf(), _gate_Q_wave, fft_size);
      d_fft->execute();
      memcpy(_gate_ffted_Q_data, d_fft->get_outbuf(), fft_size);

      memcpy(d_fft->get_inbuf(), _CTRL_I_wave, fft_size);
      d_fft->execute();
      memcpy(_CTRL_ffted_I_data, d_fft->get_outbuf(), fft_size);
      memcpy(d_fft->get_inbuf(), _CTRL_Q_wave, fft_size);
      d_fft->execute();
      memcpy(_CTRL_ffted_Q_data, d_fft->get_outbuf(), fft_size);

      memcpy(d_fft->get_inbuf(), _RO_I_wave, fft_size);
      d_fft->execute();
      memcpy(_RO_ffted_I_data, d_fft->get_outbuf(), fft_size);
      memcpy(d_fft->get_inbuf(), _RO_Q_wave, fft_size);
      d_fft->execute();
      memcpy(_RO_ffted_Q_data, d_fft->get_outbuf(), fft_size);

      gr_complex* _gate_ffted_I_data_mag = gate_ffted_I_data();
      gr_complex* _gate_ffted_Q_data_mag = gate_ffted_Q_data();
      gr_complex* _CTRL_ffted_I_data_mag = CTRL_ffted_I_data();
      gr_complex* _CTRL_ffted_Q_data_mag = CTRL_ffted_Q_data();
      gr_complex* _RO_ffted_I_data_mag = RO_ffted_I_data();
      gr_complex* _RO_ffted_Q_data_mag = RO_ffted_Q_data();
      volk_32fc_magnitude_32f_u(_gate_mag_I_data, _gate_ffted_I_data_mag, fft_size);
      volk_32fc_magnitude_32f_u(_gate_mag_Q_data, _gate_ffted_Q_data_mag, fft_size);
      volk_32fc_magnitude_32f_u(_CTRL_mag_I_data, _CTRL_ffted_I_data_mag, fft_size);
      volk_32fc_magnitude_32f_u(_CTRL_mag_Q_data, _CTRL_ffted_Q_data_mag, fft_size);
      volk_32fc_magnitude_32f_u(_RO_mag_I_data, _RO_ffted_I_data_mag, fft_size);
      volk_32fc_magnitude_32f_u(_RO_mag_Q_data, _RO_ffted_Q_data_mag, fft_size);
    }

    void
    controllers_Analyzer_impl::analyze_qubit_waves(wave_result_t* wave_ret, int qubit_id, gr_complex* gate_wave, gr_complex* CTRL_wave, gr_complex* RO_wave)
    {
      int fft_size = GR_QUANTUM_CSIM_FFT_SIZE;
      double param_val;
      gate::gate_type_t ret_gate;
      gate::gate_type_t ret_CTRL;
      gate::gate_type_t ret_RO;
      QUANTUM_INIT_COUNTER ret_INIT;
      QUANTUM_QUBID_ID ret_JUNC;
      gr_complex* _gate_wave = gate_wave;
      gr_complex* _CTRL_wave = CTRL_wave; 
      gr_complex* _RO_wave = RO_wave;
      float* _gate_mag_I_data = gate_mag_I_data();
      float* _gate_mag_Q_data = gate_mag_Q_data();
      float* _CTRL_mag_I_data = CTRL_mag_I_data();
      float* _CTRL_mag_Q_data = CTRL_mag_Q_data();
      float* _RO_mag_I_data = RO_mag_I_data();
      float* _RO_mag_Q_data = RO_mag_Q_data();

      qubit_state_machine_t *stat_machine = &(d_qubits_state_machine[qubit_id]);

      std::map<gate::gate_type_t, gate::gate_param_t> gates_list = stat_machine->gates_params_list;
      std::map<gate::gate_type_t, gate::gate_param_t> CTRL_list;
      copy_gate_param_t(&CTRL_list[gate::CNOT], &(stat_machine->CTRL_params));
      std::map<gate::gate_type_t, gate::gate_param_t> RO_list;
      copy_gate_param_t(&RO_list[gate::RO], &(stat_machine->RO_params));
      std::map<QUANTUM_QUBID_ID, gate::gate_param_t> JUNC_list = stat_machine->JUNC_params_list;
      std::map<QUANTUM_INIT_COUNTER, gate::gate_param_t> INIT_list = stat_machine->INIT_params_list;

      convert_waves2IQmags(_gate_wave, _CTRL_wave, _RO_wave);

      ret_gate = analyze_gate_type_by_mag(_gate_mag_I_data, _gate_mag_Q_data, gates_list);
      if(ret_gate != gate::NONE) {
        if(check_BW_proc_time(&(stat_machine->gate_proc_status), gates_list[ret_gate], ret_gate, proccessing_BW)) {
          wave_ret->is_detected_gate = false;
        }
      } else if(stat_machine->gate_proc_status.proc_status != NOP){
        gate::gate_type_t proc_type = stat_machine->gate_proc_status.processing_gate_type;
        if(check_BW_proc_time(&(stat_machine->gate_proc_status), gates_list[stat_machine->gate_proc_status.processing_gate_type], stat_machine->gate_proc_status.processing_gate_type, END_BW)) {
          wave_ret->is_detected_gate = true;
          wave_ret->detected_gate_type = proc_type;
        } else {
          stat_machine->gate_proc_status.proc_status = NOP;
        }
      } else {
        wave_ret->is_detected_gate = false;
      }

      ret_CTRL = analyze_gate_type_by_mag(_CTRL_mag_I_data, _CTRL_mag_Q_data, CTRL_list);
      if(ret_CTRL != gate::NONE) {
        if(check_BW_proc_time(&(stat_machine->CTRL_proc_status), CTRL_list[ret_CTRL], ret_CTRL, proccessing_BW)) {
          wave_ret->is_detected_CTRL = false;
        }
      } else if(stat_machine->CTRL_proc_status.proc_status != NOP){
        gate::gate_type_t proc_type = stat_machine->CTRL_proc_status.processing_gate_type;
        if(check_BW_proc_time(&(stat_machine->CTRL_proc_status), CTRL_list[stat_machine->CTRL_proc_status.processing_gate_type], stat_machine->CTRL_proc_status.processing_gate_type, END_BW)) {
          wave_ret->is_detected_CTRL = true;
          wave_ret->detected_gate_type = proc_type;
        } else {
          stat_machine->CTRL_proc_status.proc_status = NOP;
        }
      } else {
        wave_ret->is_detected_CTRL = false;
      }

      ret_RO = analyze_gate_type_by_mag(_RO_mag_I_data, _RO_mag_Q_data, RO_list);
      if(ret_RO != gate::NONE) {
        if(check_BW_proc_time(&(stat_machine->RO_proc_status), RO_list[ret_RO], ret_RO, proccessing_BW)) {
          wave_ret->is_detected_RO = false;
        }
      } else if(stat_machine->RO_proc_status.proc_status != NOP){
        gate::gate_type_t proc_type = stat_machine->RO_proc_status.processing_gate_type;
        if(check_BW_proc_time(&(stat_machine->RO_proc_status), RO_list[stat_machine->RO_proc_status.processing_gate_type], stat_machine->RO_proc_status.processing_gate_type, END_BW)) {
          wave_ret->is_detected_RO = true;
          wave_ret->detected_gate_type = proc_type;
        } else {
          stat_machine->RO_proc_status.proc_status = NOP;
        }
      } else {
        wave_ret->is_detected_RO = false;
      }

      ret_INIT = analyze_INIT_by_mag(_gate_mag_I_data, _gate_mag_Q_data, INIT_list);
      if(ret_INIT != -1) {
        if(check_BW_proc_time(&(stat_machine->INIT_proc_status), INIT_list[ret_INIT], ret_INIT, proccessing_BW)) {
          wave_ret->is_detected_INIT = false;
        }
      } else if(stat_machine->INIT_proc_status.proc_status != NOP){
        int INIT_num = stat_machine->INIT_proc_status.INIT_num;
        if(check_BW_proc_time(&(stat_machine->INIT_proc_status), INIT_list[stat_machine->INIT_proc_status.INIT_num], stat_machine->INIT_proc_status.INIT_num, END_BW)) {
          wave_ret->is_detected_INIT = true;
          wave_ret->detected_INIT_num = INIT_num;
        } else {
          stat_machine->INIT_proc_status.proc_status = NOP;
        }
      } else {
        wave_ret->is_detected_INIT = false;
      }

      ret_JUNC = analyze_JUNC_by_mag(_RO_mag_I_data, _RO_mag_Q_data, JUNC_list);
      if(ret_JUNC != -1) {
        if(check_BW_proc_time(&(stat_machine->JUNC_proc_status), JUNC_list[ret_JUNC], ret_JUNC, proccessing_BW)) {
          wave_ret->is_detected_JUNC = false;
        }
      } else if(stat_machine->JUNC_proc_status.proc_status != NOP){
        int JUNC_num = stat_machine->JUNC_proc_status.JUNC_num;
        if(check_BW_proc_time(&(stat_machine->JUNC_proc_status), JUNC_list[stat_machine->JUNC_proc_status.JUNC_num], stat_machine->JUNC_proc_status.JUNC_num, END_BW)) {
          wave_ret->is_detected_JUNC = true;
          wave_ret->detected_JUNC_qubit_id = JUNC_num;
        } else {
          stat_machine->JUNC_proc_status.proc_status = NOP;
        }
      } else {
        wave_ret->is_detected_JUNC = false;
      }

    }
    double
    controllers_Analyzer_impl::volt2db(double amp)
    {
      if(amp == 0.0) return 0.0;
      return 20*std::log10(amp);
    }
    double
    controllers_Analyzer_impl::mag2amp(double mag)
    {
      if(mag <= 0.0) return 0.0;
      double fft_size = (double)GR_QUANTUM_CSIM_FFT_SIZE;
      double mag_rate = fft_size/GR_QUANTUM_CSIM_MAG_AMP_RATE;
      double amp = mag/mag_rate;

      return amp;
    }

    bool
    controllers_Analyzer_impl::check_I_wave_null(gate::gate_param_t gate)
    {
      if(gate.I_amp == 0.0) true;
      if(gate.I_bw == 0.0) true;

      return false;
    }
    bool
    controllers_Analyzer_impl::check_Q_wave_null(gate::gate_param_t gate)
    {
      if(gate.Q_amp == 0.0) true;
      if(gate.Q_bw == 0.0) true;

      return false;
    }


    QUANTUM_QUBID_ID
    controllers_Analyzer_impl::analyze_JUNC_by_mag(float* mag_I_data, float* mag_Q_data, std::map<QUANTUM_QUBID_ID, gate::gate_param_t> params_list)
    {
      for(auto it = begin(params_list); it != end(params_list); ++it) {
        if(analyze_params_by_mag(mag_I_data, mag_Q_data, it->second)) return it->first;
      }
      return -1;
    }
    QUANTUM_INIT_COUNTER
    controllers_Analyzer_impl::analyze_INIT_by_mag(float* mag_I_data, float* mag_Q_data, std::map<QUANTUM_INIT_COUNTER, gate::gate_param_t> params_list)
    {
      for(auto it = begin(params_list); it != end(params_list); ++it) {
        if(analyze_params_by_mag(mag_I_data, mag_Q_data, it->second)) return it->first;
      }
      return -1;
    }
    gate::gate_type_t
    controllers_Analyzer_impl::analyze_gate_type_by_mag(float* mag_I_data, float* mag_Q_data, std::map<gate::gate_type_t, gate::gate_param_t> params_list)
    {
      for(auto it = begin(params_list); it != end(params_list); ++it) {
        if(analyze_params_by_mag(mag_I_data, mag_Q_data, it->second)) return it->first;
      }
      return gate::NONE;
    }
    bool
    controllers_Analyzer_impl::analyze_params_by_mag(float* mag_I_data, float* mag_Q_data, gate::gate_param_t params)
    {
        bool is_I_wave_null = check_I_wave_null(params);
        if(!is_I_wave_null) {
          if(!check_wave_range(mag_I_data, params.freq, params.I_amp, params.I_bw, params.DC_mode)) return false;
        }
        bool is_Q_wave_null = check_Q_wave_null(params);
        if(!is_Q_wave_null) {
          if(!check_wave_range(mag_Q_data, params.freq, params.Q_amp, params.Q_bw, params.DC_mode)) return false;
        }
        if(is_I_wave_null && is_Q_wave_null) return false;

        return true;
    }

    bool
    controllers_Analyzer_impl::check_BW_proc_time(qubit_proc_state_t* proc_state, gate::gate_param_t gate, int state_type, proc_status_type_t exec_flag)
    {
      bool is_detected = false;
      int processing_type = gate::NONE;
      bool reset_processing_type = false;
      bool update_processing_type = false;
      switch(gate.type) {
        case gate::INIT:
          processing_type = proc_state->INIT_num;
          break;
        case gate::JUNC:
          processing_type = proc_state->JUNC_num;
          break;
        default:
          processing_type = proc_state->processing_gate_type;
          break;
      }

      if(proc_state->proc_status == proccessing_BW && exec_flag == END_BW) {
        proc_state->proc_timer.stop();
        proc_state->proc_status = NOP;
        reset_processing_type = true;
        if(GR_QUANTUM_CSIM_STRICT_IQ_CHECK_MODE == true && processing_type != state_type) {
          is_detected = false;
        } else {
          double real_proc_BW_time_min;
          double real_proc_BW_time_max;
          double target_proc_BW_time = gate.proc_time;
          long cur_proc_time_cnt;
          real_proc_BW_time_min = (target_proc_BW_time - target_proc_BW_time*proc_time_threshold_rate()) * time_scale_rate();
          real_proc_BW_time_max = (target_proc_BW_time + target_proc_BW_time*proc_time_threshold_rate()) * time_scale_rate();
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
          auto cur_proc_time = boost::chrono::nanoseconds(proc_state->proc_timer.elapsed().wall);
          cur_proc_time_cnt = cur_proc_time.count();
#else
          auto cur_proc_time = boost::chrono::microseconds(proc_state->proc_timer.elapsed().wall);
          cur_proc_time_cnt = cur_proc_time.count() / 1000;
#endif          

          if(real_proc_BW_time_min <= cur_proc_time_cnt && cur_proc_time_cnt <= real_proc_BW_time_max) {
            is_detected = true;
          } else {
            is_detected = false;
          }
        }
      } else if(proc_state->proc_status == NOP && exec_flag == proccessing_BW) {
        proc_state->proc_status = proccessing_BW;
        update_processing_type = true;
        if(proc_state->proc_timer.is_stopped()) {
          proc_state->proc_timer.start();
        } else {
          proc_state->proc_timer.stop();
          proc_state->proc_timer.start();
        }
        is_detected = false;
      } else if(proc_state->proc_status == proccessing_BW && exec_flag == proccessing_BW) {
        is_detected = false;
      } else {
        is_detected = false;
      }


      switch(gate.type) {
        case gate::INIT:
          if(reset_processing_type) {
            proc_state->INIT_num = -1;
          } else if(update_processing_type) {
            proc_state->INIT_num = state_type;
          }
          break;
        case gate::JUNC:
          if(reset_processing_type) {
            proc_state->JUNC_num = -1;
          } else if(update_processing_type) {
            proc_state->JUNC_num = state_type;
          }
          break;
        case gate::RO:
          if(reset_processing_type) {
            proc_state->processing_gate_type = gate::NONE;
          } else if(update_processing_type) {
            proc_state->processing_gate_type = gate::RO;
          }
          break;
        case gate::CNOT:
          if(reset_processing_type) {
            proc_state->processing_gate_type = gate::NONE;
          } else if(update_processing_type) {
            proc_state->processing_gate_type = gate::CNOT;
          }
          break;
        default:
          if(reset_processing_type) {
            proc_state->processing_gate_type = gate::NONE;
          } else if(update_processing_type) {
            proc_state->processing_gate_type = gate.type;
          }
          break;
      }

      return is_detected;
    }

    bool
    controllers_Analyzer_impl::check_wave_range(float* mag_data, double target_freq, double target_amp, double target_bw, bool DC_mode)
    {
      int fft_size = GR_QUANTUM_CSIM_FFT_SIZE;
      double freq_min_start_range;
      double freq_max_start_range;
      double freq_min_end_range;
      double freq_max_end_range;
      double amp_min_range;
      double amp_max_range;
      double bw_min_range;
      double bw_max_range;
      int target_freq_min_start_array_num;
      int target_freq_max_start_array_num;
      int target_freq_min_end_array_num;
      int target_freq_max_end_array_num;
      int target_freq_start_array_size;
      int target_freq_end_array_size;
      double cur_freq;
      double cur_amp;
      bool is_start_freq_amp_OK = false;
      bool is_end_freq_amp_OK = false;
      bool is_freq_amp_ON_OK = false;
      float* _mag_data = mag_data;

      double target_freq_threshold = target_freq*freq_threshold_rate();
      if(target_bw <= target_freq_threshold) target_freq_threshold = target_bw - target_bw*bw_threshold_rate();
      if(target_freq_threshold <= 1.0) target_freq_threshold = 2.0;

      double target_bw_half = target_bw/2.0;
      double target_freq_min = target_freq - target_bw_half - target_bw_half*bw_threshold_rate();
      if(target_freq_min < 0) target_freq_min = 0;
      double target_freq_max = target_freq + target_bw_half + target_bw_half*bw_threshold_rate();


      if(!DC_mode && (target_freq == 0.0 || target_amp == 0.0 ||  target_bw == 0.0)) return false;


      freq_min_start_range = target_freq_min - target_freq_threshold;
      if(freq_min_start_range < 0.0) freq_min_start_range = 0.0;
      freq_max_start_range = target_freq_min + target_freq_threshold;

      freq_min_end_range = target_freq_max - target_freq_threshold;
      if(freq_min_end_range < 0.0) freq_min_end_range = 0.0;
      freq_max_end_range = target_freq_max + target_freq_threshold;

      amp_min_range = target_amp - target_amp*amp_threshold_rate();
      if(amp_min_range <= 0.0) amp_min_range = target_amp/100.0f;
      amp_max_range = target_amp + target_amp*amp_threshold_rate();


      target_freq_min_start_array_num = freq_min_start_range/d_fft_freq_step_Hz;
      if(target_freq_min_start_array_num >= GR_QUANTUM_CSIM_FFT_SIZE) target_freq_min_start_array_num = GR_QUANTUM_CSIM_FFT_SIZE-1;
      target_freq_max_start_array_num = freq_max_start_range/d_fft_freq_step_Hz;
      if(target_freq_max_start_array_num >= GR_QUANTUM_CSIM_FFT_SIZE) target_freq_max_start_array_num = GR_QUANTUM_CSIM_FFT_SIZE-1;

      target_freq_min_end_array_num = freq_min_end_range/d_fft_freq_step_Hz;
      if(target_freq_min_end_array_num >= GR_QUANTUM_CSIM_FFT_SIZE) target_freq_min_end_array_num = GR_QUANTUM_CSIM_FFT_SIZE-1;
      target_freq_max_end_array_num = freq_max_end_range/d_fft_freq_step_Hz;
      if(target_freq_max_end_array_num >= GR_QUANTUM_CSIM_FFT_SIZE) target_freq_max_end_array_num = GR_QUANTUM_CSIM_FFT_SIZE-1;

      target_freq_start_array_size = target_freq_max_start_array_num - target_freq_min_start_array_num;
      target_freq_end_array_size = target_freq_max_end_array_num - target_freq_min_end_array_num;


      


      // Check the amp of the Start BW & End BW only, NOT Check the amp of All the BW Range.
      for(int i = target_freq_max_start_array_num; i > target_freq_min_start_array_num; --i) {
        cur_amp = mag2amp(_mag_data[i]);
        if(is_freq_amp_ON_OK == false) {
          if(amp_min_range <= cur_amp && cur_amp <= amp_max_range) {
            if(DC_mode) return true;
            is_freq_amp_ON_OK = true;
          }
        }
        if(is_freq_amp_ON_OK == true) {
          // Cheking End of Wave Edge
          if(amp_min_range > cur_amp) {
            is_start_freq_amp_OK = true;
            break;
          }
          // Cheking Half-value angle Area
          if(i == target_freq_min_start_array_num+1) {
            for(int j = 0; j < target_freq_start_array_size*4; j++) {
              cur_amp = mag2amp(_mag_data[target_freq_min_start_array_num-j]);
              if(amp_min_range > cur_amp) {
                is_start_freq_amp_OK = true;
                break;
              }
              if(target_freq_min_start_array_num-j <= 0) {
                break;
              }
            }
          }
        }
      }
      if(!is_start_freq_amp_OK) {
        return false;
      }

      is_freq_amp_ON_OK = false;
      for(int i = target_freq_min_end_array_num; i < target_freq_max_end_array_num; i++) {
        cur_amp = mag2amp(_mag_data[i]);
        if(is_freq_amp_ON_OK == false) {
          if(amp_min_range <= cur_amp && cur_amp <= amp_max_range) {
            if(DC_mode) return true;
            is_freq_amp_ON_OK = true;
          }
        }
        if(is_freq_amp_ON_OK == true) {
          // Cheking End of Wave Edge
          if(amp_min_range > cur_amp) {
            is_end_freq_amp_OK = true;
            break;
          }
          // Cheking Half-value angle Area
          if(i == target_freq_max_end_array_num-1) {
            for(int j = 0; j < target_freq_end_array_size*4; j++) {
              cur_amp = mag2amp(_mag_data[target_freq_max_end_array_num+j]);
              if(amp_min_range > cur_amp) {
                is_end_freq_amp_OK = true;
                break;
              }
              if(target_freq_max_end_array_num+j >= GR_QUANTUM_CSIM_FFT_SIZE-1) {
                break;
              }
            }
          }
        }
      }
      if(!is_end_freq_amp_OK) {
        return false;
      }

      return true;
    }

    void
    controllers_Analyzer_impl::copy_wave_result_t(wave_result_t* copyto, wave_result_t* orig)
    {
      copyto->detected_gate_type = orig->detected_gate_type;
      copyto->detected_JUNC_qubit_id = orig->detected_JUNC_qubit_id;
      copyto->detected_INIT_num = orig->detected_INIT_num;

      copyto->is_detected_gate = orig->is_detected_gate;
      copyto->is_detected_CTRL = orig->is_detected_CTRL;
      copyto->is_detected_RO = orig->is_detected_RO;
      copyto->is_detected_INIT = orig->is_detected_INIT;
      copyto->is_detected_JUNC = orig->is_detected_JUNC;
    }
    
    int
    controllers_Analyzer_impl::work(int noutput_items,
                        gr_vector_const_void_star &input_items,
                        gr_vector_void_star &output_items)
    {
      gr::thread::scoped_lock lock(d_setlock);

      long cur_proc_time_cnt;
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      auto cur_proc_time = boost::chrono::nanoseconds(d_proc_rate_timer.elapsed().wall);
      cur_proc_time_cnt = cur_proc_time.count();
#else
      auto cur_proc_time = boost::chrono::microseconds(d_proc_rate_timer.elapsed().wall);
      cur_proc_time_cnt = cur_proc_time.count() / 1000;
#endif
      if(time_scale_rate() > cur_proc_time_cnt) {
        return noutput_items;
      }
      d_proc_rate_timer.stop();
      d_proc_rate_timer.start();

      int fft_size = GR_QUANTUM_CSIM_FFT_SIZE;
      int proc_items_unit = noutput_items/fft_size;
      int proc_items_num = proc_items_unit*fft_size;
      int port_set_num = input_items.size()/GR_QUANTUM_CSIM_PORT_BASE_NUM;
      if(proc_items_unit == 0) {
        return 0;
      } else if(time_scale_rate() > 1000) {
        proc_items_unit = 1;
      }

      for (int unit = 0; unit < proc_items_unit; unit++) {
        std::map<QUANTUM_QUBID_ID, wave_result_t> result;
        
        // Analize each qubit's port
        for (int in_port = 0; in_port < port_set_num; in_port++) {
          wave_result_t wave_ret;
          QUANTUM_QUBID_ID qubit_id = in_port+1;
          gr_complex* _gate_wave_array = (gr_complex*)input_items[controllers_Analyzer_impl::GATE + (in_port*GR_QUANTUM_CSIM_PORT_BASE_NUM)];
          gr_complex* _CTRL_wave_array = (gr_complex*)input_items[controllers_Analyzer_impl::CTRL + (in_port*GR_QUANTUM_CSIM_PORT_BASE_NUM)]; 
          gr_complex* _RO_wave_array = (gr_complex*)input_items[controllers_Analyzer_impl::RO + (in_port*GR_QUANTUM_CSIM_PORT_BASE_NUM)]; 
          gr_complex* _gate_wave = &_gate_wave_array[unit*fft_size];
          gr_complex* _CTRL_wave = &_CTRL_wave_array[unit*fft_size];
          gr_complex* _RO_wave = &_RO_wave_array[unit*fft_size];

          analyze_qubit_waves(&wave_ret,
                              qubit_id,
                              _gate_wave, 
                              _CTRL_wave,
                              _RO_wave);

          copy_wave_result_t(&result[qubit_id], &wave_ret);
        }

        for (int in_port = 0; in_port < port_set_num; in_port++) {
          QUANTUM_QUBID_ID qubit_id = in_port+1;
          wave_result_t wave_ret = result[qubit_id];
          qubit_state_machine_t *stat = &(d_qubits_state_machine[qubit_id]);

          if(wave_ret.is_detected_gate) {
            stat->stacked_qubits.push(stat->gates_params_list[wave_ret.detected_gate_type]);
          }
          if(wave_ret.is_detected_INIT) {
            stat->stacked_qubits.push(stat->INIT_params_list[wave_ret.detected_INIT_num]);
          }


          if(wave_ret.is_detected_CTRL) {
            stat->stacked_qubits.push(stat->CTRL_params);
          }
          if(wave_ret.is_detected_RO) {
            stat->stacked_qubits.push(stat->RO_params);
          }

          if(wave_ret.is_detected_JUNC) {
            std::queue<gate::CTRL_junction_relation_t> relation_queue = d_CTRL_junction_relation_list[wave_ret.detected_JUNC_qubit_id];
            gate::CTRL_junction_relation_t relation;
            relation.qubit_ID = qubit_id;
            relation.number = stat->JUNC_counter++;
            relation_queue.push(relation);

            stat->stacked_qubits.push(stat->JUNC_params_list[wave_ret.detected_JUNC_qubit_id]);
          }          
 
        }
        for (int in_port = 0; in_port < port_set_num; in_port++) {
          QUANTUM_QUBID_ID qubit_id = in_port+1;
          wave_result_t wave_ret = result[qubit_id];
          qubit_state_machine_t *stat = &(d_qubits_state_machine[qubit_id]);
          if(wave_ret.is_detected_RO) {
            while (!stat->stacked_qubits.empty()) {
              gate::gate_param_t qubit_params = stat->stacked_qubits.front();
              gate* gate_d;
              if(qubit_params.type == gate::JUNC) {
                std::queue<gate::CTRL_junction_relation_t> relataion_queue = d_CTRL_junction_relation_list[wave_ret.detected_JUNC_qubit_id];
                gate_d = new gate(gate::JUNC_LIST, qubit_id, relataion_queue);
              } else if(qubit_params.type == gate::CNOT) {
                std::queue<gate::CTRL_junction_relation_t> relataion_queue = d_CTRL_junction_relation_list[wave_ret.detected_JUNC_qubit_id];
                gate_d = new gate(gate::CNOT, qubit_id, relataion_queue);
              } else {
                gate_d = new gate(qubit_params.type, qubit_id);
              }
              pmt::pmt_t gates_params = pmt::make_vector(1, gate_d->get_parameters());
              pmt::vector_set(gates_params, 0, gate_d->get_parameters());
              message_port_pub(d_port_ANALY_out, gates_params);
              stat->stacked_qubits.pop();
              delete gate_d;
            }
          }
        }
      }
      return proc_items_num;
    }

  } /* namespace quantum */
} /* namespace gr */
