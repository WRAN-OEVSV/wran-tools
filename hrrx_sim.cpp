/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A simple receiver for the wran beacon test using a RTL dongle.
 */

#include "config.hpp"
#include "hamranfrm.hpp"
#include "AudioFile.h"

#include <liquid/liquid.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using po::options_description, po::value, po::variables_map, po::store,
  po::positional_options_description, po::command_line_parser, po::notify,
  po::parse_command_line;

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

#include <string>
using std::string;

#include <vector>
using std::vector;

#include <boost/format.hpp>
using boost::format, boost::str;

const double sample_rate = 4e6;

size_t cpf      = 0;
size_t phy_mode = 1;

class rxframe : public hrframesync {

  size_t num_valid_frames;

public:
  rxframe(double sample_rate, size_t prefix_fraction)
    : hrframesync(sample_rate, prefix_fraction),
      num_valid_frames(0) {
  }

protected:
  int callback(unsigned char* header, bool header_valid,
                             unsigned char* payload, unsigned int payload_len,
                             bool payload_valid, framesyncstats_s stats) {
    //wrframesync::callback(header, header_valid, payload, payload_len, payload_valid, stats);
    cout << format("rssi: %6.1f dB") % stats.rssi;
    if (payload_valid) {
        ++num_valid_frames;
        cout << format(" : payload %6d : ") % num_valid_frames;
        for (unsigned n =0; n<payload_len; ++n) cout << payload[n];

      }
    cout << endl;
    return 0;
  }
};

int main(int argc, char* argv[]) {

  path infile;

  try {

    options_description opts("Options");
    opts.add_options()
        ("help,h", "Print usage information.")
        ("version", "Print version.")
        ("cpf",        value<size_t>()->default_value(12),           "Cyclic prefix len: 0...50")
        ("phy",        value<size_t>()->default_value(1),            "Physical layer mode: 1 ... 14")
        ;

    options_description pos_opts;
    pos_opts.add_options()
        ("infile", value<path>(&infile), "Input file name")
        ;
    positional_options_description pos;
    pos.add("infile", 1);

    options_description all_opts;
    all_opts.add(opts).add(pos_opts);

    variables_map vm;
    store(command_line_parser(argc, argv).options(all_opts).positional(pos).run(), vm);
    notify(vm);

    if (vm.count("help")) {
        cout << "wrrx-sim Beacon receiver for WRAN project." << endl;
        cout << "Usage: wrrx-sim [options] infile" << endl;
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

    cout << "wrrx-sim parameters:" << endl;
    cout << format("Prefix len: %2d\n")             % cpf;
    cout << format("Samplerate: %g MHz\n")          % (1e-6*sample_rate);
    cout << format("Phymode:    %2d\n")             % phy_mode;
    cout << endl;

    AudioFile<float> in;
    if (!in.load(infile.c_str()))
      throw runtime_error("File loading error.");

    cout << infile << endl;
    in.printSummary();
    size_t numSamples = in.getNumSamplesPerChannel();

    if (sample_rate*3 != in.getSampleRate()*5)
      throw runtime_error("File has unusable sample rate.");

    if ( 2!= in.getNumChannels())
      throw runtime_error("File has unusable channel number.");

    rxframe fs(sample_rate, cpf);
    vector<complex<float>> buf_4MHz(40*512);   // 24*512*5/3
    vector<complex<float>> buf_2_4MHz(24*512); // 24*512
    rresamp_cccf rs;
    rs = rresamp_cccf_create_default(buf_4MHz.size(), buf_2_4MHz.size());

    size_t offset = 0;
    while (offset+buf_2_4MHz.size() < numSamples) {
        for (size_t n=0; n<buf_2_4MHz.size(); ++n) {
            buf_2_4MHz[n] = complex<float>(in.samples[0][offset+n], in.samples[1][offset+n]);
          }
        rresamp_cccf_execute(rs, buf_2_4MHz.data(), buf_4MHz.data());
        fs.execute(buf_4MHz.data(), buf_4MHz.size());
        offset += buf_2_4MHz.size();
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
