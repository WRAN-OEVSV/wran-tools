/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A tool for generating a beacon signal for the wran project, but only
 * stored to disk.
 */

#include "config.hpp"
#include "wranfrm.hpp"

#include <cxxopts.hpp>

#include <cstdlib>
using std::size_t;

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl;

#include <fstream>
using std::ofstream, std::ios;

#include <filesystem>
using std::filesystem::path;

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

#include <boost/format.hpp>
using boost::format, boost::str;

const double sample_rate = 4e6;

// some global shared variables for use by the threads
//atomic<uint64_t> rx_timestamp      = 0; // latest timestamp from rx, we know of
size_t   cpf               = 0; // cyclic prefix len, 0 ... prefix_divider
size_t   phy_mode          = 1; // physical layer mode;

// this is a hack, proper implementation pending
string global_message;

// The control thread.
int main(int argc, char* argv[])
{

  try {

    cxxopts::Options options("wrbeacon", "Beacon generator for WRAN project.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("f,filename", "Output file name", cxxopts::value<path>()->default_value("beacon.cfile"))
        ("cpf", "Cyclic prefix len: 0...50", cxxopts::value<size_t>()->default_value(("12")))
        ("phy", "Physical layer mode: 1 ... 14", cxxopts::value<size_t>()->default_value("1"))
        ("message", "Beacon message", cxxopts::value<string>()->default_value("OE1XTU"));
        ;
    options.parse_positional({"message"});
    options.positional_help("message");
    options.show_positional_help();

    auto vm = options.parse(argc, argv);

    if (vm.count("help")) {
        cout << options.help() << endl;
        return EXIT_SUCCESS;
    }

    if (vm.count("version")) {
        cout <<PROJECT_VER << endl;
        return EXIT_SUCCESS;
    }

    // copy to global variable, this is preliminary and will be replaced
    // TODO: replace by message queue
    global_message = vm["message"].as<string>();
    //for (size_t n=0; n<100; ++n) global_message += "c"; // Test different msg len.

    cpf = vm["cpf"].as<size_t>();
    if (0 == cpf or  cpf > wrframegen::prefix_divider)
      throw runtime_error("prefix not in range");

    phy_mode = vm["phy"].as<size_t>();
    if (0 == phy_mode or phy_mode > 14)
      throw runtime_error("invalid phymode requested");

    cout << "wrbeacon-sim parameters:" << endl;
    cout << format("Prefix frac: %2d\n")             % cpf;
    cout << format("Samplerate: %g MHz\n")          % (1e-6*sample_rate);
    cout << format("Phymode:    %2d\n")             % phy_mode;
    cout << endl;

    ofstream out(vm["filename"].as<path>(), ios::binary|ios::out);

    unsigned char header[8] = {0,0,0,0,0,0,0,0};

    wrframegen fg(sample_rate, cpf, phy_mode);
    size_t samp_per_frame = sample_rate*wrframegen::frame_len;

    vector<complex<float>> tx_buffer(fg.subcarriers + fg.prefix_len);
    size_t tx_timestamp = 0;

    for (size_t n=0; n<10; ++n) {
        tx_timestamp = 0;
        fg.assemble(header, reinterpret_cast<unsigned char*>(global_message.data()), global_message.size());
        bool last = fg.write(tx_buffer.data(), tx_buffer.size());
        while (not last) {
            out.write(reinterpret_cast<char*>(tx_buffer.data()), tx_buffer.size()*sizeof(complex<float>));
            tx_timestamp += tx_buffer.size();
            last = fg.write(tx_buffer.data(), tx_buffer.size());

          }
        out.write(reinterpret_cast<char*>(tx_buffer.data()), tx_buffer.size()*sizeof(complex<float>));
        tx_timestamp += tx_buffer.size();
        tx_buffer[0] = 0;
        for (size_t m = tx_timestamp; m < samp_per_frame; ++m)
            out.write(reinterpret_cast<char*>(tx_buffer.data()), sizeof(complex<float>));

    }

    cout << "Exiting and cleaning up." << endl;

    cout << "Control thread stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
