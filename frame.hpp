/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef _FRAME_HPP
#define _FRAME_HPP

#include <complex> // NB: Must be included before liquid.h !
#include <liquid/liquid.h>
#include <cstddef>


class framegenerator {
  ofdmflexframegen fg;
  static const std::size_t guards;
  static const std::size_t pilotfrac;

public:
  static const std::size_t frames_per_second;
  static const std::size_t subcarriers;
  static const std::size_t datacarriers;

  typedef struct {modulation_scheme mod_scheme; fec_scheme fec0; fec_scheme fec1;} phy_t;
  static const phy_t phy_prop[];

  typedef struct {size_t sym_per_frame; uint32_t samp_per_frame; } frame_t;
  static const frame_t spfxs_prop[];

  framegenerator(std::size_t cpfxs, std::size_t phymode);
  ~framegenerator();

  void assemble(const unsigned char* header, const unsigned char* payload, std::size_t payload_len);
  bool write(std::complex<float>* buffer, std::size_t buffer_len);
};

#endif // _FRAME_HPP
