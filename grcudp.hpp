/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/* A simple UDP port implementation that can be read with gnuradio's
   UDP soource block. */

#ifndef _GRCUDP_HPP
#define _GRCUDP_HPP

#include <string>

#include <complex>

#include <boost/asio.hpp>
using namespace boost::asio;

#include <boost/system/system_error.hpp>
using boost::system::error_code;

class grcudp {

  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket socket;
  boost::asio::ip::udp::endpoint grc;
  boost::system::error_code err;

  struct { uint64_t seq; std::complex<float> iq[183]; } grcbuf;
  std::size_t index;

public:
  grcudp(const std::string& address  = "127.0.0.1", unsigned short port = 2000);
  ~grcudp();

  void send(std::complex<float>* buf, std::size_t len);
  void send(std::complex<uint8_t>* buf, std::size_t len);

};

#endif // _GRCUDP_HPP
