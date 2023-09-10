/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "frame.hpp"

#include <cstddef>
using std::size_t;

#include <complex>
using std::complex;

#include <cassert>

const size_t framegenerator::frames_per_second =  100;
const size_t framegenerator::subcarriers       = 1024;
const size_t framegenerator::guards            =  232;
const size_t framegenerator::pilotfrac         =    7;
const size_t framegenerator::datacarriers      =  480;

const framegenerator::phy_t framegenerator::phy_prop[] =
{
  {LIQUID_MODEM_UNKNOWN, LIQUID_FEC_UNKNOWN,     LIQUID_FEC_UNKNOWN}, //  0
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

// (sym_per_frame + 4)*subcarriers*(1+2**(-spfxs) == samp_per_frame
const framegenerator::frame_t framegenerator::spfxs_prop[] =
{ // sym_per_frame, samp_per_frame, spfxs
  {             12,         32768}, // 0
  {             18,         33792}, // 1
  {             22,         33280}, // 2
  {             24,         32256}, // 3
  {             26,         32640}, // 4
  {             27,         32736}  // 5
};

framegenerator::framegenerator(size_t cpfxs, size_t phymode) {

  ofdmflexframegenprops_s fgp;
  ofdmflexframegenprops_init_default(&fgp);

  fgp.check = LIQUID_CRC_NONE;
  fgp.mod_scheme = phy_prop[phymode].mod_scheme;
  fgp.fec0       = phy_prop[phymode].fec0;
  fgp.fec1       = phy_prop[phymode].fec1;

  size_t cp_len    = subcarriers >> cpfxs;
  size_t taper_len = cp_len/4;

  // subcarrier allocation array (null/pilot/data)
  unsigned char sca[subcarriers];

  assert(datacarriers == ((subcarriers - 2*guards)*(pilotfrac-1))/pilotfrac);

  for (size_t i = 0;                  i < guards;             ++i)
    sca[i] = OFDMFRAME_SCTYPE_NULL; // guard band

  for (size_t i = guards;             i < subcarriers-guards; ++i)
    sca[i] = (0 == i%pilotfrac)? OFDMFRAME_SCTYPE_PILOT :  OFDMFRAME_SCTYPE_DATA;

  for (size_t i = subcarriers-guards; i < subcarriers;        ++i)
    sca[i] = OFDMFRAME_SCTYPE_NULL; // guard_band]

  fg = ofdmflexframegen_create(subcarriers, cp_len, taper_len, sca, &fgp);
}

void framegenerator::assemble(const unsigned char* header, const unsigned char* payload, size_t payload_len) {
  ofdmflexframegen_assemble(fg, header, payload, payload_len);
}

bool framegenerator::write(complex<float>* buffer, size_t buffer_len) {
  int result = ofdmflexframegen_write(fg, buffer, buffer_len);
  return (1 == result)?true:false;
}

framegenerator::~framegenerator() {
  ofdmflexframegen_destroy(fg);
}
