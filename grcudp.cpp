/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "grcudp.hpp"

#include <string>
using std::string;

#include <complex>
using std::complex;
using namespace std::literals::complex_literals;


#include <cstddef>
using std::size_t;

#include <boost/asio.hpp>
using namespace boost::asio;

#include <boost/system/system_error.hpp>
using boost::system::error_code;

grcudp::grcudp(const string& address, unsigned short port)
  : socket(io_service) {
  socket.open(ip::udp::v4());
  grc = ip::udp::endpoint(ip::address::from_string(address), port);
  grcbuf.seq = 0;
  index = 0;
}

grcudp::~grcudp() {
  socket.close();
}

void grcudp::send(complex<float>* buf, size_t len) {
  size_t written = 0;
  while (written < len) {
      while (index < 183 and written < len)
          grcbuf.iq[index++] = buf[written++];
      if (183 == index) {
          socket.send_to(buffer(&grcbuf, sizeof(grcbuf)), grc, 0, err);
          ++grcbuf.seq;
          index = 0;
        }
    }
}

void grcudp::send(complex<uint8_t>* buf, size_t len) {
  // convert 8 bit to complex float, as e.g. for rtl sdr
  complex<float> fbuf[len];
  for (size_t n=0; n<len; ++n)
      fbuf[n] = (buf[n].real() / 127.5 - 1) + 1i*(buf[n].imag() / 127.5 -1 );
  send(fbuf, len);
}
