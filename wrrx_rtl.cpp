/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A simple receiver for the wran beacon test using a RTL dongle.
 */

#include "config.hpp"
#include "wranfrm.hpp"

#include <rtl-sdr.h>

#include <cxxopts.hpp>

#include <cstdlib>
using std::size_t;

#include <cstdint>

#include <complex> // NB: Must be included before liquid.h !
#include <liquid/liquid.h>
using std::complex;
using namespace std::literals::complex_literals;

#include <iostream>
using std::cin, std::cout, std::cerr, std::endl;

#include <fstream>
using std::ofstream, std::ios;

#include <utility>
using std::swap;

#include <string>
using std::string, std::getline;

#include <vector>
using std::vector;

#include <array>
using std::array;

#include <exception>
using std::exception;

#include <stdexcept>
using std::runtime_error;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <chrono>
using namespace std::literals::chrono_literals;

#include <atomic>
using std::atomic;

#include <thread>
using std::jthread, std::stop_token, std::this_thread::sleep_for;

#include <future>
using std::async, std::launch;

#include <boost/format.hpp>
using boost::format, boost::str;

namespace {

  volatile sig_atomic_t signal_status = 0;
  void signal_handler(int signal) {
    signal_status = signal;
  }

}

const double sample_rate = 2.4e6; // rtlsdr specific

size_t cpf      = 0;
//size_t phy_mode = 1;

complex<float> buf_4MHz[40*512];   // 24*512*5/3
complex<float> buf_2_4MHz[24*512]; // 24*512
rresamp_cccf rs;

void receive_cb(unsigned char*buf, uint32_t len, void* ctx) {
  wrframesync& fs (*static_cast<wrframesync*>(ctx));
  complex<uint8_t>* buf_8 = reinterpret_cast<complex<uint8_t>*>(buf);
  for (size_t n=0; n<24*512; ++n) buf_2_4MHz[n] = (buf_8[n].real()/127.5-1) + 1i*(buf_8[n].imag()/127.5-1);
  rresamp_cccf_execute(rs, buf_2_4MHz, buf_4MHz);
  fs.execute(buf_4MHz, 40*512);
}

void receive(stop_token stoken, rtlsdr_dev_t* dev) {
  wrframesync fs(sample_rate*5/3, cpf);
  // We need to resample, due to a bug in liquiddsp.
  rs = rresamp_cccf_create_default(40*512, 24*512);
  rtlsdr_read_async(dev, receive_cb, &fs, 0, 2*24*512);
  cout << "Receive thread stopping."  << endl;
}

int main(int argc, char* argv[]) {

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c

  rtlsdr_dev_t* dev = nullptr;

  try {

    cxxopts::Options options("wrrx_rtl", "Beacon receiver for WRAN project.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("freq", "Center frequency.", cxxopts::value<double>()->default_value("53e6"))
        ("cpf", "Cyclic prefix len: 0...50", cxxopts::value<size_t>()->default_value(("12")))
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
    if (0 == cpf or  cpf > wrframegen::prefix_divider)
      throw runtime_error("prefix not in range");

//    phy_mode = vm["phy"].as<size_t>();
//    if (0 == phy_mode or phy_mode > 14)
//      throw runtime_error("invalid phymode requested");

    double freq       = vm["freq"].as<double>();

    cout << "wrrx-rtl parameters:" << endl;
    cout << format("Freq:       %g MHz\n") % (1e-6*freq);
    cout << format("Prefix len: %2d\n")             % cpf;
    cout << format("Samplerate: %g MHz\n")          % (1e-6*sample_rate);
    //cout << format("Phymode:    %2d\n")             % phy_mode;
    cout << endl;

    size_t n = rtlsdr_get_device_count();
    if (n < 1) throw runtime_error("No device found");

    // use first device for now
    char manufact[256];
    char product[256];
    char serial[256];
    const char* name = rtlsdr_get_device_name(0);
    cout << name << endl;
    rtlsdr_get_device_usb_strings(0, manufact, product, serial);
    cout << manufact << endl;
    cout << product << endl;
    cout << serial << endl;

    if (0 != rtlsdr_open(&dev, 0))
      throw runtime_error("Cannot open device.");

    rtlsdr_set_center_freq(dev, static_cast<uint32_t>(freq));
    rtlsdr_set_freq_correction(dev, 45);
    rtlsdr_set_direct_sampling(dev, 0);
    rtlsdr_set_sample_rate(dev, static_cast<uint32_t>(sample_rate));
    rtlsdr_set_tuner_bandwidth(dev, 0); // automatic
    rtlsdr_set_tuner_gain_mode(dev, 0);
    rtlsdr_set_agc_mode(dev, 0);
    //rtlsdr_set_tuner_gain(dev, 496);
    //rtlsdr_set_bias_tee(dev, 1);
    rtlsdr_reset_buffer(dev);

    jthread rx_thread(receive, dev);

    cout << "Beacon rx control thread: enter EOF (Ctrl-D) or empty line to end." << endl;
    string line;
    while(getline(cin, line)) {
        if (line.empty()) break;
      }
    rtlsdr_cancel_async(dev);

    cout << "Exiting and cleaning up." << endl;

    rx_thread.request_stop();
    rx_thread.join();

    rtlsdr_close(dev);
    dev = nullptr;

    cout << "Stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    if (nullptr != dev)
      rtlsdr_close(dev);
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    if (nullptr != dev)
      rtlsdr_close(dev);
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
