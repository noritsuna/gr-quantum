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

#include "controllers_Generator_impl.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>
#include <limits>
#include <volk/volk.h>
#include <cmath>


static const int GR_QUANTUM_CSIM_FFT_SIZE = 1024;
static const int GR_QUANTUM_CSIM_PORT_BASE_NUM = 3;
static const float GR_QUANTUM_CSIM_MAG_AMP_RATE = 5.0f;

namespace gr {
  namespace quantum {

    controllers_Generator::sptr
    controllers_Generator::make(double qubit_num, double samples_per_sec, double time_scale_rate, bool show_SYNC_port)
    {
      return gnuradio::get_initial_sptr
        (new controllers_Generator_impl(qubit_num, samples_per_sec, time_scale_rate, show_SYNC_port));
    }

    controllers_Generator_impl::controllers_Generator_impl(double qubit_num, double samples_per_sec, double time_scale_rate, bool show_SYNC_port)
      : hier_block2("controllers_Generator",
                      io_signature::make(0, 0, 0),
                      io_signature::make(qubit_num, qubit_num, sizeof(gr_complex))),
      d_port_out(pmt::mp("out")),
      d_port_in(pmt::mp("in")),
      d_port_SIM_in(pmt::mp("simulated_data")),
      d_port_SYNC_in(pmt::mp("SYNC_CLK_in"))
    {
      configure_default_loggers(d_logger, d_debug_logger, "Generator");
      set_qubit_num(qubit_num);
      set_sample_rate(samples_per_sec);
      set_SYNC_port(show_SYNC_port);



#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      d_is_nanosec_mode = true;
      set_time_scale_rate(time_scale_rate);
#else
      d_is_nanosec_mode = false;
      set_time_scale_rate(time_scale_rate/1000);
#endif
      d_proc_rate_timer.start();

    

      for(int i = 0; i < qubit_num; i++) {
        QUANTUM_QUBID_ID qubit_id = i+1;
        RO_wave_synthesizer_t* synthe = &(d_RO_wave_synthesizers[qubit_id]);

        synthe->readout_I_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_SIN_WAVE, 1, 0.0, 0.0, 0.0);
        synthe->readout_Q_src_base = analog::sig_source_f::make(sample_rate(), analog::GR_SIN_WAVE, 1, 0.0, 0.0, 0.0);
        synthe->readout_I_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_SIN_WAVE, 1, 0.0, 0.0, 0.0);
        synthe->readout_Q_src_bw = analog::sig_source_f::make(sample_rate(), analog::GR_SIN_WAVE, 1, 0.0, 0.0, 0.0);

        synthe->readout_I_src_add = blocks::add_ff::make(1);
        synthe->readout_Q_src_add = blocks::add_ff::make(1);

        synthe->readout_IQ_mixer = blocks::float_to_complex::make(1);

        connect(synthe->readout_I_src_base, 0, synthe->readout_I_src_add, 0);
        connect(synthe->readout_I_src_bw, 0, synthe->readout_I_src_add, 1);
        connect(synthe->readout_Q_src_base, 0, synthe->readout_Q_src_add, 0);
        connect(synthe->readout_Q_src_bw, 0, synthe->readout_Q_src_add, 1);

        connect(synthe->readout_I_src_add, 0, synthe->readout_IQ_mixer, 0);
        connect(synthe->readout_Q_src_add, 0, synthe->readout_IQ_mixer, 1);

        connect(synthe->readout_IQ_mixer, 0, self(), i);        
      }
      
      callback_message_port = controllers_callback_message::make();

      message_port_register_hier_in(d_port_in);
      message_port_register_hier_in(d_port_SYNC_in);
      message_port_register_hier_in(d_port_SIM_in);

      callback_message_port->set_msg_handler(d_port_in, boost::bind(&controllers_Generator_impl::handle_cmd_msg, this, _1));
      callback_message_port->set_msg_handler(d_port_SYNC_in, boost::bind(&controllers_Generator_impl::handle_SYNC_msg, this, _1));
      callback_message_port->set_msg_handler(d_port_SIM_in, boost::bind(&controllers_Generator_impl::handle_simurated_msg, this, _1));


      msg_connect(self(), d_port_in, callback_message_port, d_port_in);
      msg_connect(self(), d_port_SYNC_in, callback_message_port, d_port_SYNC_in);
      msg_connect(self(), d_port_SIM_in, callback_message_port, d_port_SIM_in);

      message_port_register_hier_out(d_port_out);
      callback_message_port->message_port_register_out(d_port_out);
      msg_connect(callback_message_port, d_port_out, self(), d_port_out);


    }

    controllers_Generator_impl::~controllers_Generator_impl()
    {
    }

    void
    controllers_Generator_impl::set_qubit_num(double qubit_num)
    {
      d_qubit_num = qubit_num;
    }
    double
    controllers_Generator_impl::qubit_num()
    {
      return d_qubit_num;
    }

    void
    controllers_Generator_impl::set_sample_rate(double rate)
    {
      //changing the sample rate performs a reset of state params
      d_samps_per_us = rate/1e6;
    }

    double
    controllers_Generator_impl::sample_rate()
    {
      return d_samps_per_us * 1e6;
    }

    void
    controllers_Generator_impl::set_time_scale_rate(double rate)
    {
      d_time_scale_rate = rate;
    }

    double
    controllers_Generator_impl::time_scale_rate()
    {
      return d_time_scale_rate;
    }

    void
    controllers_Generator_impl::set_SYNC_port(bool is_use) {
        d_SYNC_port = !is_use;
    }

    bool
    controllers_Generator_impl::SYNC_port() const{
      return d_SYNC_port;
    }

    

    void
    controllers_Generator_impl::copy_gate_param_t(gate::gate_param_t* copyto, gate::gate_param_t* orig)
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
    controllers_Generator_impl::set_params_to_list(int qubit_id, gate::gate_type_t type, pmt::pmt_t gate)
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

      }
    }



    bool
    controllers_Generator_impl::stop_waveform(int qubit_id, gate::gate_param_t gate_params)
    {
      analog::sig_source_f::sptr I_wave_src_base;
      analog::sig_source_f::sptr Q_wave_src_base;
      analog::sig_source_f::sptr I_wave_src_bw;
      analog::sig_source_f::sptr Q_wave_src_bw;

      RO_wave_synthesizer_t* synthe = &(d_RO_wave_synthesizers[qubit_id]);

      I_wave_src_base = synthe->readout_I_src_base;
      Q_wave_src_base = synthe->readout_Q_src_base;
      I_wave_src_bw = synthe->readout_I_src_bw;
      Q_wave_src_bw = synthe->readout_Q_src_bw;

      I_wave_src_base->set_amplitude(0.0);
      Q_wave_src_base->set_amplitude(0.0);
      I_wave_src_bw->set_amplitude(0.0);
      Q_wave_src_bw->set_amplitude(0.0);
      I_wave_src_base->set_phase(0.0);
      Q_wave_src_base->set_phase(0.0);
      I_wave_src_bw->set_phase(0.0);
      Q_wave_src_bw->set_phase(0.0);

      return true;
    }

    bool
    controllers_Generator_impl::change_waveform(int qubit_id, gate::gate_param_t gate_params, float angle)
    {
      analog::sig_source_f::sptr I_wave_src_base;
      analog::sig_source_f::sptr Q_wave_src_base;
      analog::sig_source_f::sptr I_wave_src_bw;
      analog::sig_source_f::sptr Q_wave_src_bw;

      RO_wave_synthesizer_t* synthe = &(d_RO_wave_synthesizers[qubit_id]);

      I_wave_src_base = synthe->readout_I_src_base;
      Q_wave_src_base = synthe->readout_Q_src_base;
      I_wave_src_bw = synthe->readout_I_src_bw;
      Q_wave_src_bw = synthe->readout_Q_src_bw;
      
      float angle_rad = deg2rad(angle);
      I_wave_src_base->set_phase(I_wave_src_base->phase()+angle_rad);
      Q_wave_src_base->set_phase(Q_wave_src_base->phase()+angle_rad);
      I_wave_src_bw->set_phase(I_wave_src_bw->phase()+angle_rad);
      Q_wave_src_bw->set_phase(Q_wave_src_bw->phase()+angle_rad);

      return true;
    }


    bool
    controllers_Generator_impl::start_waveform(int qubit_id, gate::gate_param_t gate_params)
    {
      double freq;
      double freq_min;
      double I_amp;
      double Q_amp;
      double I_bw;
      double Q_bw;
      double proc_time;

      bool CTRL_dc_mode;

      analog::sig_source_f::sptr I_wave_src_base;
      analog::sig_source_f::sptr Q_wave_src_base;
      analog::sig_source_f::sptr I_wave_src_bw;
      analog::sig_source_f::sptr Q_wave_src_bw;

      RO_wave_synthesizer_t* synthe = &(d_RO_wave_synthesizers[qubit_id]);

      I_wave_src_base = synthe->readout_I_src_base;
      Q_wave_src_base = synthe->readout_Q_src_base;
      I_wave_src_bw = synthe->readout_I_src_bw;
      Q_wave_src_bw = synthe->readout_Q_src_bw;



      freq = gate_params.freq;
      I_amp = gate_params.I_amp;
      Q_amp = gate_params.Q_amp;
      I_bw = gate_params.I_bw;
      Q_bw = gate_params.Q_bw;
      proc_time = gate_params.proc_time;

      CTRL_dc_mode = gate_params.DC_mode;

      if(CTRL_dc_mode)
      {
        freq = 0.0;
        I_bw = 0.0;
        Q_bw = 0.0;
        I_wave_src_base->set_waveform(analog::GR_CONST_WAVE);
        Q_wave_src_base->set_waveform(analog::GR_CONST_WAVE);
        I_wave_src_bw->set_waveform(analog::GR_CONST_WAVE);
        Q_wave_src_bw->set_waveform(analog::GR_CONST_WAVE);
      } else {
        I_wave_src_base->set_waveform(analog::GR_SIN_WAVE);
        Q_wave_src_base->set_waveform(analog::GR_SIN_WAVE);
        I_wave_src_bw->set_waveform(analog::GR_SIN_WAVE);
        Q_wave_src_bw->set_waveform(analog::GR_SIN_WAVE);
        I_bw = I_bw/2;
        Q_bw = Q_bw/2;
        freq_min = freq - I_bw;
        if(freq_min < 0) freq_min = 0; 
        I_wave_src_base->set_frequency(freq_min);
        freq_min = freq - Q_bw;
        if(freq_min < 0) freq_min = 0; 
        Q_wave_src_base->set_frequency(freq_min);
        I_wave_src_bw->set_frequency(freq+I_bw);
        Q_wave_src_bw->set_frequency(freq+Q_bw);
      }

      I_wave_src_base->set_amplitude(I_amp);
      Q_wave_src_base->set_amplitude(Q_amp);
      I_wave_src_bw->set_amplitude(I_amp);
      Q_wave_src_bw->set_amplitude(Q_amp);

      I_wave_src_base->set_phase(0.0);
      Q_wave_src_base->set_phase(0.0);
      I_wave_src_bw->set_phase(0.0);
      Q_wave_src_bw->set_phase(0.0);

      return true;
    }

    void
    controllers_Generator_impl::proccessing_wait(long wait_time_ns)
    {

#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      boost::posix_time::time_duration sleep_time = boost::posix_time::nanoseconds(wait_time_ns);
#else      
      boost::posix_time::time_duration sleep_time = boost::posix_time::microseconds(wait_time_ns);
#endif
      boost::system_time st = boost::get_system_time() + sleep_time;
      boost::unique_lock<boost::mutex> lk(sleep_mutex);
      while(sleep_condition.timed_wait(lk, st));
    }


    double
    controllers_Generator_impl::deg2rad(double degree) 
    {
      return (degree) * (3.141592653589793 / 180.0);
    }

    void
    controllers_Generator_impl::handle_simurated_msg(pmt::pmt_t msg)
    {

      pmt::pmt_t qubit_id_PMT = pmt::dict_ref(msg, pmt::from_float(qubit::ID), pmt::PMT_NIL);
      if(pmt::eq(qubit_id_PMT, pmt::PMT_NIL)) return;
      int qubit_id = pmt::to_float(qubit_id_PMT);
      pmt::pmt_t qubit_state_PMT = pmt::dict_ref(msg, pmt::from_float(qubit::STATE), pmt::PMT_NIL);
      if(pmt::eq(qubit_state_PMT, pmt::PMT_NIL)) return;
      int qubit_state = pmt::to_float(qubit_state_PMT);

      float qubit_angle = 0.0;
      pmt::pmt_t qubit_angle_PMT = pmt::dict_ref(msg, pmt::from_float(qubit::ANGLE), pmt::PMT_NIL);
      if(!pmt::eq(qubit_angle_PMT, pmt::PMT_NIL)) {
        qubit_angle = pmt::to_float(qubit_angle_PMT);
      }

      switch(qubit_state) {
        case qubit::START:
          start_waveform(qubit_id, d_RO_gate_params[qubit_id]);
          break;
        case qubit::PROC:
          change_waveform(qubit_id, d_RO_gate_params[qubit_id], qubit_angle);
          break;
        case qubit::END:
          stop_waveform(qubit_id, d_RO_gate_params[qubit_id]);
          break;
        default:
          stop_waveform(qubit_id, d_RO_gate_params[qubit_id]);
          break;
      }

    }
    void
    controllers_Generator_impl::handle_cmd_msg(pmt::pmt_t msg)
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

        qubit_id = pmt::to_double(pmt::dict_ref(gate, pmt::from_float(gate::QUBIT_ID), pmt::PMT_NIL));
      }

      for(int i = 0; i <= gate_len; i++) {
        pmt::pmt_t gate = pmt::dict_ref(msg, pmt::from_float(i), pmt::PMT_NIL);
        if(pmt::eq(gate, pmt::PMT_NIL)) {
          continue;
        }
        set_params_to_list(qubit_id, gate::RO, gate);
      }

      callback_message_port->message_port_pub(d_port_out, msg);

    }

    void
    controllers_Generator_impl::handle_SYNC_msg(pmt::pmt_t msg)
    {
    }


  } /* namespace quantum */
} /* namespace gr */
