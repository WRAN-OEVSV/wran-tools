/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef KEYER_HPP
#define KEYER_HPP

#include <vector>
#include <deque>
#include <tr1/memory>
#include <complex>
#include <utility>
#include <cstdint>

struct space_t
{
    space_t(
        float d
    )   : duration(d) {
    }
    float duration;
};
extern const space_t char_space;
extern const space_t word_space;

class keyer {
    float                t_unit_;    // time for 'di'
    unsigned             handicap_;  // longer spaces
    double               a_;
    float                f_s_;       // sampling frequency
    double               q_;         // damping factor
    std::complex<double> o_;         // complex oscillator frequency
    double               g_;         // damping state
    std::complex<double> x_;         // oscillator state
    bool                 on_;        // keyer state
    std::uint64_t        n_t_;       // sample phase of current series
    std::deque<std::pair<bool, float>>
                         tq_;        // keyer time series
    std::deque<std::complex<float>>
                         fb_;        // frame buffer
public:
    keyer(
        float   cpm = 60        // characters per minute
        , float f_0 = 441.0     // oszilator frequency / Hz
        , unsigned handicap = 0 // handicap (longer spaces)
        , float a   = 0.5       // amplitude
        , float f_s = 44100.0   // sample rate / Hz
        );

    void
    key_times(
        std::vector<std::pair<bool, float> > time_series // on-off times in sec
        );

    float
    pending_time(
        );

    void
    didah(
        const std::string& didah
        );

    void
    space(
        const space_t& s
        );

    void
    space(
        float duration
        );

    void
    mark(
        float duration
        );

    void
    send(
        const std::string& message
        );


    std::size_t
    get_frame(
        std::complex<float>* frame,
        std::size_t frame_size
        );
};

#endif // KEYER_HPP
