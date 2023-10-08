/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A simple receiver for the wran beacon test using a RTL dongle.
 */

#include "config.hpp"
#include "hamranfrm.hpp"

#include <complex> // NB: Must be included before liquid.h !
#include <liquid/liquid.h>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include "grcudp.hpp"

#include <hackrf.h>

#include <cxxopts.hpp>

#include <iostream>
using std::cin, std::cout, std::cerr, std::endl;

#include <string>
using std::string, std::getline;

#include <exception>
using std::exception;

#include <stdexcept>
using std::runtime_error;

#include <csignal>
using std::sig_atomic_t, std::signal;


namespace {

  volatile sig_atomic_t signal_status = 0;
  void signal_handler(int signal) {
    signal_status = signal;
  }

}

int receive_cb(hackrf_transfer* transfer) {
  complex<int8_t>* cbuf = reinterpret_cast<complex<int8_t>*>(transfer->buffer);
  int len = transfer->valid_length/sizeof(complex<int8_t>);
  //grcudp& udp(*static_cast<grcudp*>(transfer->rx_ctx));
  hrframesync& fs (*static_cast<hrframesync*>(transfer->rx_ctx));
  //udp.send(cbuf, len);
  fs.execute(cbuf, len);
  return 0;
}

int main(int argc, char* argv[]) {

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c

  hackrf_device*  dev = nullptr;

  try {

    cxxopts::Options options("wrrx_hackrf", "Beacon receiver for WRAN project.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("vgagain", "Baseband gain 0 ... 62dB in 2dB steps", cxxopts::value<int>()->default_value("26"))
        ("lnagain", "IF gain 0 ... 47dB in 1dB steps", cxxopts::value<int>()->default_value("25"))
        ("ampgain", "IF gain 0 or 11dB", cxxopts::value<int>()->default_value("0"))
        ("cpf", "Cyclic prefix len: 0...50", cxxopts::value<size_t>()->default_value(("12")))
        ;

    auto vm = options.parse(argc, argv);

    if (vm.count("help")) {
        cout << options.help() << endl;
        cout << "Version: " << PROJECT_VER << endl;
        cout << "using" << endl;
        cout << "hackrf_library_version: " << hackrf_library_version() << endl;
        cout << "hackrf_library_release: " << hackrf_library_release() << endl;
        return EXIT_SUCCESS;
    }

    if (vm.count("version")) {
        cout << PROJECT_VER << endl;
        return EXIT_SUCCESS;
    }

    size_t cpf = vm["cpf"].as<size_t>();
    if (0 == cpf or  cpf > hrframegen::prefix_divider)
      throw runtime_error("prefix not in range");

    int vga_gain = vm["vgagain"].as<int>();
    int lna_gain = vm["lnagain"].as<int>();
    int amp_gain = vm["ampgain"].as<int>();

    hackrf_init();
    hackrf_device_list_t* devicelist = hackrf_device_list();
    if (devicelist->serial_numbers !=0) {
        int idx = 0;
        for (; idx < devicelist->devicecount; ++idx) {
            if (HACKRF_SUCCESS == hackrf_device_list_open(devicelist, idx, &dev)) {
                char* serial_number = devicelist->serial_numbers[idx];
                cout << "[" << idx << "] " << (serial_number?serial_number:"") << endl;
                break;
              }
          }
        if (idx == devicelist->devicecount)
          throw runtime_error("all devices already in use");
      }

    // set 53MHz carrier, 3.5MHz filter bw, and 4MHz samplerate
    hackrf_set_freq_explicit(dev, 2'150'000'000, 2'097'000'000, RF_PATH_FILTER_LOW_PASS);
    hackrf_set_sample_rate_manual(dev, 20'000'000, 5);
    hackrf_set_baseband_filter_bandwidth(dev, 3'500'000);

    hackrf_set_vga_gain(dev,  vga_gain); // baseband gain, 0-62 in 2dB steps
    hackrf_set_lna_gain(dev,  lna_gain); // IF gain, 0-47 in 1dB steps
    hackrf_set_amp_enable(dev, amp_gain); // RF gain, 0-11 in 11dB steps

    //grcudp udp;
    hrframesync fs(4'000'000, cpf);
    hackrf_start_rx(dev, receive_cb, &fs);

    cout << "Beacon rx control thread: enter EOF (Ctrl-D) or empty line to end." << endl;

    string line;
    while(getline(cin, line)) {
        if (line.empty()) break;
      }

    hackrf_stop_rx(dev);

    cout << "Exiting and cleaning up." << endl;

    hackrf_close(dev);
    dev = nullptr;
    hackrf_device_list_free(devicelist);
    hackrf_exit();

    cout << "Stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    if (nullptr != dev)
      hackrf_close(dev);
    return EXIT_FAILURE;
  } catch(...) {
    if (nullptr != dev)
      hackrf_close(dev);
    cerr << "Exception of unknown reason." << endl;
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
