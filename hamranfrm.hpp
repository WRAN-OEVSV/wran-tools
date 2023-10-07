/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef _HAMRANFRM_HPP
#define _HAMRANFRM_HPP

#include <complex> // NB: Must be included before liquid.h !
#include <liquid/liquid.h>
#include <cstddef>
#include <vector>

class hrframe {

protected:
  std::vector<unsigned char> sca;

public:
  static const double      frame_len;       // in s
  static const double      bandwidth;       // in Hz
  static const double      carrier_spacing; // in Hz
  static const std::size_t prefix_divider;
  static const std::size_t pilot_fraction;

  const double      sample_rate;     // in Hz
  const std::size_t subcarriers;
  const std::size_t available_carriers; // pilot + data
  const std::size_t guards;
  const std::size_t used_carriers; // data
  const std::size_t prefix_len;
  const std::size_t taper_len;

  hrframe(double sample_rate, std::size_t prefix_fraction);
  ~hrframe();

};


class hrframegen : public hrframe {
  ofdmflexframegen fg;

  typedef struct {
    modulation_scheme mod_scheme;
    fec_scheme fec0;
    fec_scheme fec1;
  } phy_t;
  static const phy_t phy_prop[];

public:
  hrframegen(double sample_rate,
             std::size_t prefix_fraction, std::size_t phy_mode);
  ~hrframegen();
  void assemble(const unsigned char* header,
                const unsigned char* payload, std::size_t payload_len);
  bool write(std::complex<float>* buffer, std::size_t buffer_len);
};

typedef int (fn_framesync_callback) (unsigned char*,
                                     int, unsigned char*, unsigned int,
                                     int, framesyncstats_s, void*);
extern "C" fn_framesync_callback hrframesync_wrap_callback;


class hrframesync : public hrframe {
  ofdmflexframesync fs;

protected:
  friend fn_framesync_callback hrframesync_wrap_callback;
  virtual int callback(unsigned char* header, bool header_valid,
                       unsigned char* payload, unsigned int payload_len,
                       bool payload_valid, framesyncstats_s stats);

public:
  hrframesync(double sample_rate, std::size_t prefix_fraction);
  ~hrframesync();
  bool execute(std::complex<float>* buffer, std::size_t buffer_len);
  bool execute(std::complex<uint8_t>* buffer, std::size_t buffer_len);
};

#endif // _HAMRANFRM_HPP
