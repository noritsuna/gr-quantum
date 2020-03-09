/* -*- c++ -*- */
/*
 * Copyright 2004,2010,2012,2018 Free Software Foundation, Inc.
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
#include <config.h>
#endif

#include "controlled_sig_source.h"
#include <gnuradio/gr_complex.h>
#include <gnuradio/math.h>

#include <algorithm>
#include <stdexcept>

namespace gr {
namespace quantum {

template <class T>
controlled_sig_source<T>::controlled_sig_source(double sampling_freq,
                                    quantum_waveform_t waveform,
                                    double frequency,
                                    double ampl,
                                    T offset,
                                    float phase)
{
    this->set_sampling_freq(sampling_freq);
    this->set_waveform(waveform);
    this->set_amplitude(ampl);
    this->set_offset(offset);
    this->set_frequency(frequency);
    this->set_phase(phase);
}

template <class T>
controlled_sig_source<T>::~controlled_sig_source()
{
}

template <class T>
int controlled_sig_source<T>::exec(int noutput_items,
                             gr_vector_void_star& output_items)
{
    T* optr = (T*)output_items[0];
    T t;

    switch (d_waveform) {
    case QUANTUM_CONST_WAVE:
        t = (T)d_ampl + d_offset;
        std::fill_n(optr, noutput_items, t);
        break;

    case QUANTUM_SIN_WAVE:
        d_nco.sin(optr, noutput_items, d_ampl);
        if (d_offset == 0)
            break;

        for (int i = 0; i < noutput_items; i++) {
            optr[i] += d_offset;
        }
        break;

    case QUANTUM_COS_WAVE:
        d_nco.cos(optr, noutput_items, d_ampl);
        if (d_offset == 0)
            break;

        for (int i = 0; i < noutput_items; i++) {
            optr[i] += d_offset;
        }
        break;

        /* The square wave is high from -PI to 0. */
    case QUANTUM_SQR_WAVE:
        t = (T)d_ampl + d_offset;
        for (int i = 0; i < noutput_items; i++) {
            if (d_nco.get_phase() < 0)
                optr[i] = t;
            else
                optr[i] = d_offset;
            d_nco.step();
        }
        break;

        /* The triangle wave rises from -PI to 0 and falls from 0 to PI. */
    case QUANTUM_TRI_WAVE:
        for (int i = 0; i < noutput_items; i++) {
            double t = d_ampl * d_nco.get_phase() / GR_M_PI;
            if (d_nco.get_phase() < 0)
                optr[i] = static_cast<T>(t + d_ampl + d_offset);
            else
                optr[i] = static_cast<T>(-1 * t + d_ampl + d_offset);
            d_nco.step();
        }
        break;

        /* The saw tooth wave rises from -PI to PI. */
    case QUANTUM_SAW_WAVE:
        for (int i = 0; i < noutput_items; i++) {
            t = static_cast<T>(d_ampl * d_nco.get_phase() / (2 * GR_M_PI) + d_ampl / 2 +
                               d_offset);
            optr[i] = t;
            d_nco.step();
        }
        break;
    default:
        throw std::runtime_error("analog::sig_source: invalid waveform");
    }

    return noutput_items;
}

template <>
int controlled_sig_source<gr_complex>::exec(int noutput_items,
                                      gr_vector_void_star& output_items)
{
    gr_complex* optr = (gr_complex*)output_items[0];
    gr_complex t;

    switch (d_waveform) {
    case QUANTUM_CONST_WAVE:
        t = (gr_complex)d_ampl + d_offset;
        std::fill_n(optr, noutput_items, t);
        break;

    case QUANTUM_SIN_WAVE:
    case QUANTUM_COS_WAVE:
        d_nco.sincos(optr, noutput_items, d_ampl);
        if (d_offset == gr_complex(0, 0))
            break;

        for (int i = 0; i < noutput_items; i++) {
            optr[i] += d_offset;
        }
        break;

        /* Implements a real square wave high from -PI to 0.
         * The imaginary square wave leads by 90 deg.
         */
    case QUANTUM_SQR_WAVE:
        for (int i = 0; i < noutput_items; i++) {
            if (d_nco.get_phase() < -1 * GR_M_PI / 2)
                optr[i] = gr_complex(d_ampl, 0) + d_offset;
            else if (d_nco.get_phase() < 0)
                optr[i] = gr_complex(d_ampl, d_ampl) + d_offset;
            else if (d_nco.get_phase() < GR_M_PI / 2)
                optr[i] = gr_complex(0, d_ampl) + d_offset;
            else
                optr[i] = d_offset;
            d_nco.step();
        }
        break;

        /* Implements a real triangle wave rising from -PI to 0 and
         * falling from 0 to PI. The imaginary triangle wave leads by
         * 90 deg.
         */
    case QUANTUM_TRI_WAVE:
        for (int i = 0; i < noutput_items; i++) {
            if (d_nco.get_phase() < -1 * GR_M_PI / 2) {
                optr[i] =
                    gr_complex(d_ampl * d_nco.get_phase() / GR_M_PI + d_ampl,
                               -1 * d_ampl * d_nco.get_phase() / GR_M_PI - d_ampl / 2) +
                    d_offset;
            } else if (d_nco.get_phase() < 0) {
                optr[i] = gr_complex(d_ampl * d_nco.get_phase() / GR_M_PI + d_ampl,
                                     d_ampl * d_nco.get_phase() / GR_M_PI + d_ampl / 2) +
                          d_offset;
            } else if (d_nco.get_phase() < GR_M_PI / 2) {
                optr[i] = gr_complex(-1 * d_ampl * d_nco.get_phase() / GR_M_PI + d_ampl,
                                     d_ampl * d_nco.get_phase() / GR_M_PI + d_ampl / 2) +
                          d_offset;
            } else {
                optr[i] = gr_complex(-1 * d_ampl * d_nco.get_phase() / GR_M_PI + d_ampl,
                                     -1 * d_ampl * d_nco.get_phase() / GR_M_PI +
                                         3 * d_ampl / 2) +
                          d_offset;
            }
            d_nco.step();
        }
        break;

        /* Implements a real saw tooth wave rising from -PI to PI.
         * The imaginary saw tooth wave leads by 90 deg.
         */
    case QUANTUM_SAW_WAVE:
        for (int i = 0; i < noutput_items; i++) {
            if (d_nco.get_phase() < -1 * GR_M_PI / 2) {
                optr[i] =
                    gr_complex(d_ampl * d_nco.get_phase() / (2 * GR_M_PI) + d_ampl / 2,
                               d_ampl * d_nco.get_phase() / (2 * GR_M_PI) +
                                   5 * d_ampl / 4) +
                    d_offset;
            } else {
                optr[i] =
                    gr_complex(d_ampl * d_nco.get_phase() / (2 * GR_M_PI) + d_ampl / 2,
                               d_ampl * d_nco.get_phase() / (2 * GR_M_PI) + d_ampl / 4) +
                    d_offset;
            }
            d_nco.step();
        }
        break;
    default:
        throw std::runtime_error("analog::sig_source: invalid waveform");
    }

    return noutput_items;
}


template <class T>
void controlled_sig_source<T>::set_sampling_freq(double sampling_freq)
{
    d_sampling_freq = sampling_freq;
    d_nco.set_freq(2 * GR_M_PI * this->d_frequency / this->d_sampling_freq);
}

template <class T>
void controlled_sig_source<T>::set_waveform(quantum_waveform_t waveform)
{
    d_waveform = waveform;
}

template <class T>
void controlled_sig_source<T>::set_frequency(double frequency)
{
    d_frequency = frequency;
    d_nco.set_freq(2 * GR_M_PI * this->d_frequency / this->d_sampling_freq);
}

template <class T>
void controlled_sig_source<T>::set_amplitude(double ampl)
{
    d_ampl = ampl;
}

template <class T>
void controlled_sig_source<T>::set_offset(T offset)
{
    d_offset = offset;
}

template <class T>
void controlled_sig_source<T>::set_phase(float phase)
{
    d_nco.set_phase(phase);
}

template class controlled_sig_source<std::int8_t>;
template class controlled_sig_source<std::int16_t>;
template class controlled_sig_source<std::int32_t>;
template class controlled_sig_source<float>;
template class controlled_sig_source<gr_complex>;
} /* namespace quantum */
} /* namespace gr */
