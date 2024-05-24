/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * Simulation of the HAMRAN modem.
 */

#include "config.hpp"
#include "hamranfrm.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using po::options_description, po::value, po::variables_map, po::store,
  po::positional_options_description, po::command_line_parser, po::notify,
  po::parse_command_line;

#include <cstdint>
using std::uint8_t, std::uint32_t, std::uint64_t, std::uintmax_t;

#include <cstdlib>
using std::size_t;

#include <bitset>
using std::bitset;

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl;

#include <fstream>
using std::ofstream, std::ios;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <map>
using std::map;

#include <complex>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <string>
using std::string;

#include <vector>
using std::vector;

#include <random>
using std::random_device, std::mt19937, std::uniform_int_distribution;

#include <algorithm>
using std::copy, std::fill_n;

#include <boost/format.hpp>
using boost::format, boost::str;

#include <liquid/liquid.h>

const double sample_rate = 4e6;

size_t   cpf               = 0; // cyclic prefix len, 0 ... prefix_divider
size_t   phy_mode          = 1; // physical layer mode;

class hrframegen_stats : public hrframegen {
  uint32_t        seed;
  mt19937         msggen;
  uniform_int_distribution<unsigned short> symbol;
  uint64_t        offset; // invocation count of msggen
  vector<uint8_t> msg;

public:
  hrframegen_stats(double sample_rate,
                   size_t prefix_fraction, size_t phy_mode,
                   size_t packet_size = 256, uint32_t seed = 230361)
    : hrframegen(sample_rate, -12.0, prefix_fraction, phy_mode),
      seed(seed), msggen(seed), symbol(0, 255), offset(0), msg(packet_size) {
  }

  void assemble(bool restart = false) {
    if (restart) {
        msggen.seed(seed);
        offset = 0;
      }
    uint64_t header = offset;
    for (size_t n=0; n<msg.size(); ++n, ++offset) msg[n] = symbol(msggen);
    hrframegen::assemble(reinterpret_cast<const unsigned char*>(&header),
                         msg.data(), msg.size());
  }
};

class hrframesync_stats : public hrframesync {
  uint32_t        seed;
  mt19937         msggen;
  uniform_int_distribution<unsigned short> symbol;
  uint64_t        offset; // invocation count of msggen

public:

  uintmax_t       num_bytes_valid;
  uintmax_t       num_bits_valid;
  uintmax_t       num_payloadbits_valid;

  hrframesync_stats(double sample_rate, std::size_t prefix_fraction, uint32_t seed = 230361)
    : hrframesync(sample_rate, prefix_fraction) ,
      seed(seed), msggen(seed), symbol(0, 255), offset(0), num_bytes_valid(0),
      num_bits_valid(0), num_payloadbits_valid(0) {}

  int callback(unsigned char* header, bool header_valid,
                       unsigned char* payload, unsigned int payload_len,
                       bool payload_valid, framesyncstats_s stats) override {

    // the header holds the number of invocations of the tx random generator
    // sync the rx generator to the tx generator and compare message bytes
    if (header_valid and nullptr != payload) {
        uint64_t h = *reinterpret_cast<uint64_t*>(header);
        if (h > offset) {
            msggen.discard(h-offset);
            offset = h;
          }
        else if (h < offset) {
            msggen.seed(seed);
            msggen.discard(h);
          }
        for (size_t n=0; n< payload_len; ++n, ++offset) {
            uint8_t s = symbol(msggen);
            if (payload[n] == s) ++num_bytes_valid;
            size_t bits_valid = bitset<8>(compl(s xor payload[n])).count();
            num_bits_valid += bits_valid;
            if (payload_valid) num_payloadbits_valid += bits_valid;
          }
      }

    return 0;
  };

};

int main(int argc, char* argv[])
{

  try {

    options_description opts("Options");
    opts.add_options()
        ("help,h", "Print usage information.")
        ("version", "Print version.")
        ("cpf",        value<size_t>()->default_value(12), "Cyclic prefix len: 0...50")
        ("phy",        value<size_t>()->default_value(1),  "Physical layer mode: 1 ... 14")
        ("packsiz,s",  value<size_t>()->default_value(256), "Packet size")
        ;

    //auto vm = options.parse(argc, argv);
    variables_map vm;
    store(parse_command_line(argc, argv, opts), vm);
    if (vm.count("help")) {
        cout << "wrbeacon beacon generator for WRAN project." << endl;
        cout << opts << endl;
        return EXIT_SUCCESS;
    }

    if (vm.count("version")) {
        cout <<PROJECT_VER << endl;
        return EXIT_SUCCESS;
    }

    cpf = vm["cpf"].as<size_t>();
    if (0 == cpf or  cpf > hrframegen::prefix_divider)
      throw runtime_error("prefix not in range");

    phy_mode = vm["phy"].as<size_t>();
    if (0 == phy_mode or phy_mode > 14)
      throw runtime_error("invalid phymode requested");

    //float noise_floor  = -60.0f;
    // seed for random message
    uint32_t seed      = 230361;
    size_t packet_size = vm["packsiz"].as<size_t>();

    {
      hrframegen_stats fg(sample_rate, cpf, phy_mode, seed);

      cout << "hrsim parameters:" << endl;
      cout << format("    prefix frac         :   %-u\n")    % cpf;
      cout << format("    phymode             :   %-u\n")    % phy_mode;
      cout << format("    samplerate          :   %g MHz\n") % (1e-6*sample_rate);
      //cout << format("    noise_floor         :   %g dB\n")  % noise_floor;
      cout << format("    packet size         :   %-u\n")    % packet_size;
      fg.print();
      cout << endl;
    }


    // trx and rx buffers
    vector<complex<float>> tx_buffer(512);
    vector<complex<float>> rx_buffer(512);

    cout << "     SNR,   frames,  headers,   payval,  bytesrx, bytesval,    bitsval,   samples,     kbps" << endl;
    for (float SNRdB = 0.0; SNRdB < 40.0; SNRdB += 3) {

        hrframegen_stats fg(sample_rate, cpf, phy_mode, packet_size, seed);
        hrframesync_stats fs(sample_rate, cpf, seed);

        channel_cccf channel = channel_cccf_create();
        // keep the signal at constant level, but lower the noise fllor
        channel_cccf_add_awgn(channel, -SNRdB, SNRdB);

        for (size_t f=0; f<10'000; ++f) { // frame loop
            // create a message frame, restart on frame number 0
            fg.assemble(0 == f);

            // send message frame
            bool last = fg.write(tx_buffer.data(), tx_buffer.size());
            while (not last) {
                // channel simulation
                channel_cccf_execute_block(channel, tx_buffer.data(), tx_buffer.size(), rx_buffer.data());
                //copy(tx_buffer.begin(), tx_buffer.end(), rx_buffer.begin());
                fs.execute(rx_buffer.data(), rx_buffer.size());
                last = fg.write(tx_buffer.data(), tx_buffer.size());
            }
            // channel simulation
            channel_cccf_execute_block(channel, tx_buffer.data(), tx_buffer.size(), rx_buffer.data());
            //copy(tx_buffer.begin(), tx_buffer.end(), rx_buffer.begin());
            fs.execute(rx_buffer.data(), rx_buffer.size());

//            // some inter frame spacing
//            fill_n(tx_buffer.data(), tx_buffer.size(), 0);
//            // channel simulation
//            channel_cccf_execute_block(channel, tx_buffer.data(), tx_buffer.size(), rx_buffer.data());
//            //copy(tx_buffer.begin(), tx_buffer.end(), rx_buffer.begin());
//            fs.execute(rx_buffer.data(), rx_buffer.size());
          }
        framedatastats_s s = fs.get_framedatastats();
        cout << format("%8.1f, %8d, %8d, %8d, %8d, %8d, %10d, %8d, %8.3f\n")
                % SNRdB
                % s.num_frames_detected
                % s.num_headers_valid
                % s.num_payloads_valid
                % s.num_bytes_received
                % fs.num_bytes_valid
                % fs.num_bits_valid
                % fs.num_samples
                % (1e-3*(sample_rate*fs.num_payloadbits_valid)/fs.num_samples);

        channel_cccf_destroy(channel);
    }

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
