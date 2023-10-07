/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <iostream>
using std::cout, std::endl;

#include <boost/format.hpp>
using boost::format;

#include <string>
using std::string;

#include "hamranfrm.hpp"

#include <cstddef>
using std::size_t;

#include <complex>
using std::complex;
using namespace std::literals::complex_literals;

#include <cmath>
using std::fmod;

#include <stdexcept>
using std::runtime_error;

#include <cassert>

const double hrframe::frame_len       =    10e-3; // s
const double hrframe::bandwidth       = 2e6*0.92; // Hz
const double hrframe::carrier_spacing =      4e3; // Hz
const size_t hrframe::prefix_divider  =       50;
const size_t hrframe::pilot_fraction  =        5;

const hrframegen::phy_t hrframegen::phy_prop[] =
{
  {LIQUID_MODEM_PSK2,    LIQUID_FEC_NONE,        LIQUID_FEC_NONE},    //  1
  {LIQUID_MODEM_QPSK,    LIQUID_FEC_CONV_V27,    LIQUID_FEC_REP3},    //  2
  {LIQUID_MODEM_QPSK,    LIQUID_FEC_CONV_V27,    LIQUID_FEC_NONE},    //  3
  {LIQUID_MODEM_QPSK,    LIQUID_FEC_CONV_V27P23, LIQUID_FEC_NONE},    //  4
  {LIQUID_MODEM_QPSK,    LIQUID_FEC_CONV_V27P23, LIQUID_FEC_NONE},    //  5
  {LIQUID_MODEM_QPSK,    LIQUID_FEC_CONV_V27P56, LIQUID_FEC_NONE},    //  6
  {LIQUID_MODEM_QAM16,   LIQUID_FEC_CONV_V27,    LIQUID_FEC_NONE},    //  7
  {LIQUID_MODEM_QAM16,   LIQUID_FEC_CONV_V27P23, LIQUID_FEC_NONE},    //  8
  {LIQUID_MODEM_QAM16,   LIQUID_FEC_CONV_V27P34, LIQUID_FEC_NONE},    //  9
  {LIQUID_MODEM_QAM16,   LIQUID_FEC_CONV_V27P56, LIQUID_FEC_NONE},    // 10
  {LIQUID_MODEM_QAM64,   LIQUID_FEC_CONV_V27,    LIQUID_FEC_NONE},    // 11
  {LIQUID_MODEM_QAM64,   LIQUID_FEC_CONV_V27P23, LIQUID_FEC_NONE},    // 12
  {LIQUID_MODEM_QAM64,   LIQUID_FEC_CONV_V27P34, LIQUID_FEC_NONE},    // 13
  {LIQUID_MODEM_QAM64,   LIQUID_FEC_CONV_V27P56, LIQUID_FEC_NONE}     // 14
};

hrframe::hrframe(double sample_rate, size_t prefix_fraction)
  : sample_rate(sample_rate),
    subcarriers(sample_rate/carrier_spacing),
    available_carriers(bandwidth/carrier_spacing),
    guards((subcarriers - available_carriers)/2),
    used_carriers(available_carriers - available_carriers/pilot_fraction),
    prefix_len((subcarriers*prefix_fraction)/prefix_divider),
    taper_len(prefix_len/16) {

  assert(0 == fmod(bandwidth, carrier_spacing));
  assert(0 == fmod(subcarriers, 2));
  assert(1 <= frame_len*carrier_spacing);

  if (sample_rate < bandwidth)
    throw runtime_error("sample rate must be larger than bandwidth");

  if (0 != fmod(sample_rate, carrier_spacing*prefix_divider))
    throw runtime_error("sample_rate must be multiple of carrier_spacing*prefix_divider");

  sca.resize(subcarriers);
  for (size_t i = 0;                  i < guards-1;             ++i)
      sca[(i+subcarriers/2) % subcarriers] = OFDMFRAME_SCTYPE_NULL; // guard band

  for (size_t i = guards;             i < subcarriers/2; ++i)
      sca[((i+subcarriers/2) % subcarriers) - 1] = (0 == i%pilot_fraction)? OFDMFRAME_SCTYPE_PILOT :  OFDMFRAME_SCTYPE_DATA;

  for (size_t i = subcarriers/2; i < subcarriers/2+2; ++i)
    sca[(i+subcarriers/2) % subcarriers] = OFDMFRAME_SCTYPE_NULL; // DC block

  for (size_t i = subcarriers/2;      i < subcarriers-guards; ++i)
      sca[((i+subcarriers/2) % subcarriers) + 1] = (0 == i%pilot_fraction)? OFDMFRAME_SCTYPE_PILOT :  OFDMFRAME_SCTYPE_DATA;

  for (size_t i = subcarriers-guards+1; i < subcarriers;        ++i)
      sca[(i+subcarriers/2) % subcarriers] = OFDMFRAME_SCTYPE_NULL; // guard_band]

//  for (size_t i = 0;                  i < guards;             ++i)
//      sca[(i+subcarriers/2) % subcarriers] = OFDMFRAME_SCTYPE_NULL; // guard band

//  for (size_t i = guards;             i < subcarriers-guards; ++i)
//      sca[(i+subcarriers/2) % subcarriers] = (0 == i%pilot_fraction)? OFDMFRAME_SCTYPE_PILOT :  OFDMFRAME_SCTYPE_DATA;

//  for (size_t i = subcarriers-guards; i < subcarriers;        ++i)
//      sca[(i+subcarriers/2) % subcarriers] = OFDMFRAME_SCTYPE_NULL; // guard_band]

}

hrframe::~hrframe() {

}

hrframegen::hrframegen(double sample_rate, size_t prefix_fraction, size_t phy_mode)
  : hrframe(sample_rate, prefix_fraction) {

  ofdmflexframegenprops_s fgp;
  ofdmflexframegenprops_init_default(&fgp);

  fgp.check = LIQUID_CRC_32;
  fgp.mod_scheme = phy_prop[phy_mode-1].mod_scheme;
  fgp.fec0       = phy_prop[phy_mode-1].fec0;
  fgp.fec1       = phy_prop[phy_mode-1].fec1;

  fg = ofdmflexframegen_create(subcarriers, prefix_len, taper_len, sca.data(), &fgp);
  ofdmflexframegen_print(fg);
  //ofdmframe_print_sctype(sca.data(), subcarriers);
}

void hrframegen::assemble(const unsigned char* header, const unsigned char* payload, size_t payload_len) {
  ofdmflexframegen_assemble(fg, header, payload, payload_len);
  //ofdmflexframegen_print(fg);
}

bool hrframegen::write(complex<float>* buffer, size_t buffer_len) {
  int result = ofdmflexframegen_write(fg, buffer, buffer_len);
  // Amplitudes > 1.0 overdrive the ADC resulting in splatter.
  // TODO: The optimum scale factor needs to be determined.
  for (size_t i=0; i < buffer_len; ++i) buffer[i] *= 0.275;
  return (1 == result)?true:false;
}

hrframegen::~hrframegen() {
  ofdmflexframegen_destroy(fg);
}

extern "C" {
  int hrframesync_wrap_callback(unsigned char* _header,
                int _header_valid,
                unsigned char* _payload,
                unsigned int _payload_len,
                int _payload_valid,
                framesyncstats_s _stats,
                void* _userdata) {
    return static_cast<hrframesync*>(_userdata)->callback(
          _header, _header_valid, _payload, _payload_len, _payload_valid,
          _stats);
    return 0;
  }
}

hrframesync::hrframesync(double sample_rate, size_t prefix_fraction)
  : hrframe(sample_rate, prefix_fraction) {
  fs = ofdmflexframesync_create(subcarriers, prefix_len, taper_len,
                                sca.data(), hrframesync_wrap_callback, this);
  ofdmflexframesync_print(fs);
}

hrframesync::~hrframesync() {
  ofdmflexframesync_print(fs);
  ofdmflexframesync_destroy(fs);
}

int hrframesync::callback(unsigned char* header, bool header_valid,
             unsigned char* payload, unsigned int payload_len,
             bool payload_valid, framesyncstats_s stats) {
  cout << format("%6.1f, %6.1fe-6") % stats.rssi % (1e6*stats.cfo);
  if (nullptr != payload) {
    if (payload_valid)
      cout << ", " << payload_len << ": " << string(reinterpret_cast<char*>(payload)).substr(0, payload_len);
    else
      cout << ", PAYLOAD INVALID";
    }
  else
    cout << ", PAYLOAD MISSING";
  cout << endl;
  return 0;
}

bool hrframesync::execute(complex<float>* buffer, size_t buffer_len) {
  ofdmflexframesync_execute(fs, buffer, buffer_len);
  return true;
}

bool hrframesync::execute(complex<uint8_t>* buffer, size_t buffer_len) {
  // convert 8 bit to complex float, as e.g. for rtl sdr
  complex<float> fbuffer[buffer_len];
  for (size_t n=0; n<buffer_len; ++n)
      fbuffer[n] = (buffer[n].real() / 127.5 - 1) + 1i*(buffer[n].imag() / 127.5 -1 );
  return execute(fbuffer, buffer_len);
}
