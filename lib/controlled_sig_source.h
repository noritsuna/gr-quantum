/* -*- c++ -*- */
/*
 * Copyright 2004,2012,2018 Free Software Foundation, Inc.
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


#ifndef INCLUDED_GR_QUANTUM_CONTROLLED_SIG_SOURCE_H
#define INCLUDED_GR_QUANTUM_CONTROLLED_SIG_SOURCE_H

#include <gnuradio/fxpt_nco.h>

namespace gr {
namespace quantum {

template <class T>
class controlled_sig_source
{
public:
    enum quantum_waveform_t {
        QUANTUM_CONST_WAVE = 1,
        QUANTUM_SIN_WAVE = 2,
        QUANTUM_COS_WAVE = 4,
        QUANTUM_SQR_WAVE = 8,
        QUANTUM_TRI_WAVE = 16,
        QUANTUM_SAW_WAVE = 32,
        QUANTUM_AMP_RISING_WAVE = 64,
        QUANTUM_AMP_DROPING_WAVE = 128,
        QUANTUM_FREQ_RISING_WAVE = 256,
        QUANTUM_FREQ_DROPING_WAVE = 512
    };
    struct quantum_wave_param_t {
        double start_frequency;
        double end_frequency;
        double start_amplitude;
        double end_amplitude;
        float phase;
        T offset;
    };

    controlled_sig_source(double sampling_freq,
                    quantum_waveform_t waveform,
                    double wave_freq,
                    double ampl,
                    T offset = 0,
                    float phase = 0);
    ~controlled_sig_source();


    double sampling_freq() const { return d_sampling_freq; }
    quantum_waveform_t waveform() const { return d_waveform; }
    double frequency() const { return d_frequency; }
    double amplitude() const { return d_ampl; }
    T offset() const { return d_offset; }
    float phase() const { return d_nco.get_phase(); }

    int exec(int noutput_items, gr_vector_void_star& output_items);

    void set_sampling_freq(double sampling_freq);
    void set_waveform(quantum_waveform_t waveform);
    void set_frequency(double frequency);
    void set_amplitude(double ampl);
    void set_offset(T offset);
    void set_phase(float phase);

private:
    double d_sampling_freq;
    quantum_waveform_t d_waveform;
    double d_frequency;
    double d_ampl;
    T d_offset;
    gr::fxpt_nco d_nco;

};

} /* namespace quantum */
} /* namespace gr */

#endif /* INCLUDED_GR_QUANTUM_CONTROLLED_SIG_SOURCE_H */
