/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A simple receiver for the wran beacon test using a RTL dongle.
 */

#include "config.hpp"
#include "hamranfrm.hpp"

#include <liquid/liquid.h>

#include <cxxopts.hpp>

#include <cstdlib>
using std::size_t;

#include <cstdint>

#include <complex>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
using std::ifstream, std::ios;

#include <filesystem>
using std::filesystem::path;

#include <exception>
using std::exception;

#include <stdexcept>
using std::runtime_error;

#include <boost/format.hpp>
using boost::format, boost::str;

const double sample_rate = 4e6;

size_t cpf      = 0;
size_t phy_mode = 1;

int main(int argc, char* argv[]) {

  try {

    cxxopts::Options options("wrrx_sim", "Beacon receiver for WRAN project.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("f,filename", "Output file name", cxxopts::value<path>()->default_value("wrrx_rtl.cfile"))
        ("cpf", "Cyclic prefix len: 0...50", cxxopts::value<size_t>()->default_value(("12")))
        ("phy", "Physical layer mode: 1 ... 14", cxxopts::value<size_t>()->default_value("1"))
        ;

    auto vm = options.parse(argc, argv);

    if (vm.count("help")) {
        cout << options.help() << endl;
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

    cout << "wrrx-sim parameters:" << endl;
    cout << format("Prefix len: %2d\n")             % cpf;
    cout << format("Samplerate: %g MHz\n")          % (1e-6*sample_rate);
    cout << format("Phymode:    %2d\n")             % phy_mode;
    cout << endl;

    ifstream in(vm["filename"].as<path>(), ios::binary|ios::in);

    hrframesync fs(sample_rate, cpf);

    complex<uint8_t> rtlbuf[8192];

    while(in.read(reinterpret_cast<char*>(rtlbuf), 8192*sizeof(complex<uint8_t>))) {
        fs.execute(rtlbuf, 8192);
    }
    // drop the last samples to the floor

    cout << "Stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
