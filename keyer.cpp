/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * Based on a cw keyer, originally developed by me in 2009 for the morsepod
 * program.
*/

#include "keyer.hpp"
#include "morsecode.hpp"

#include <complex>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <cmath>
using std::atan2;

#include <cctype>
using std::toupper;

#include <utility>
using std::pair, std::make_pair;

#include <string>
using std::string;

#include <vector>
using std::vector;

#include <cstdint>
using std::uint64_t;

namespace {
    const float pi = atan2(0,-1);
}

const space_t char_space(3);
const space_t word_space(7);

keyer::keyer(
    float cpm
    , float f_0
    , unsigned handicap
    , float a
    , float f_s
)   : t_unit_(6.0/cpm)
    , handicap_(handicap)
    , a_(a)
    , f_s_(f_s)
    , q_(0.1)//(1.0-exp(-10.0f/(t_unit_*f_s)))
    , o_(exp(2.0i*acos(-1)*double(f_0/f_s)))
    , g_(1.0)
    , x_(conj(o_))
    , on_(true)
    , n_t_(0)
     {
}

void
keyer::key_times(
    vector<pair<bool, float> > time_series
) {
    copy(time_series.begin(), time_series.end(), back_inserter(tq_));
}

float
keyer::pending_time(
    ) {
  float duration = 0;
  for (auto& t: tq_) duration += t.second;
  return duration;
}

// convert symbolic di dah notation to time series
void
keyer::didah(
    const std::string& didah
){
    string::const_iterator i;
    vector<float> t;
    if (!tq_.empty()) {
        if (tq_.back().first)
            tq_.push_back(make_pair(false, t_unit_));
    }
    for (i=didah.begin(); i!=didah.end(); ++i) {
        if ('.' == *i) {
            tq_.push_back(make_pair(true, t_unit_));
            tq_.push_back(make_pair(false, t_unit_));
        }
        else if ('_' == *i) {
            tq_.push_back(make_pair(true, 3*t_unit_));
            tq_.push_back(make_pair(false, t_unit_));
        }
        else if (' ' == *i)
            space(char_space);
        else if ('|' == *i)
            space(word_space);
    }
}

void
keyer::space(
    const space_t& s
){
    if (!tq_.empty()) {
        if (tq_.back().first)
            tq_.push_back(make_pair(false, s.duration*t_unit_*(handicap_+1)));
        else
            tq_.back().second = s.duration*t_unit_*(handicap_+1);
    }
    else
        tq_.push_back(make_pair(false, s.duration*t_unit_*(handicap_+1)));
}

void
keyer::space(
    float duration
    ){
  tq_.push_back(make_pair(false, duration));
}

void
keyer::mark(
    float duration
    ){
  tq_.push_back(make_pair(true, duration));
}

void
keyer::send(
    const string& message
) {
    string::size_type i,j;
    i = message.find_first_not_of(" ");
    j = message.find_last_not_of(" ");
    string m(message.substr(i,j-i+1));
    string::const_iterator it;
    bool mark = false;
    for (it=m.begin(); it!=m.end(); ++it) {
        if (mark) {
            if (' ' == *it)
                space(word_space);
            else
                space(char_space);
            mark = false;
        }
        if (' ' != *it) {
            didah(char_code(toupper(*it)));
            mark = true;
        }
    }
    space(word_space);
}

size_t
keyer::get_frame(
    complex<float>* frame, size_t frame_size
) {
  size_t count = 0;
  if ( not tq_.empty()) {
    uint64_t n_t_duration = f_s_*tq_.front().second;
    while ( not tq_.empty() and count < frame_size) {
        x_ *= o_;
        g_ = (1-q_)*g_ + q_* (tq_.front().first?1.0f:0.0f);

        frame[count++] = a_*g_*x_;
        if (++n_t_ >= n_t_duration) {
            n_t_ = 0;
            tq_.pop_front();
          }
      }
    }
  return count;
}
