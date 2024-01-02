/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A simple receiver for the wran beacon test using the lime mini 2.0.
 */

#include "config.hpp"
#include "wranfrm.hpp"

#include <cxxopts.hpp>

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <cstdlib>
using std::size_t;

#include <cstdint>

#include <complex> // NB: Must be included before liquid.h !
#include <liquid/liquid.h>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <iostream>
using std::cin, std::cout, std::cerr, std::clog, std::endl;

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

#include <functional>
using std::ref;


#include <boost/format.hpp>
using boost::format, boost::str;



namespace {

  const lime::LogLevel default_LogLevel = lime::LOG_LEVEL_INFO;
  lime::LogLevel max_LogLevel = default_LogLevel;

  void limeSuiteLogHandler(const lime::LogLevel level, const char* message) {
    if (level <= max_LogLevel) {
        switch(level) {
          //rem: throwing on error avoids overly verbose error handling code
          case lime::LOG_LEVEL_CRITICAL: throw runtime_error(message);
          case lime::LOG_LEVEL_ERROR:    throw runtime_error(message);
          case lime::LOG_LEVEL_WARNING: clog << format("Warning: %s\n") % message; return;
          case lime::LOG_LEVEL_INFO:    clog << format("Info: %s\n") % message; return;
          case lime::LOG_LEVEL_DEBUG:   clog << format("Debug: %s\n") % message; return;
          }
      }
  }

  volatile sig_atomic_t signal_status = 0;
  void signal_handler(int signal) {
    signal_status = signal;
  }

}

const double sample_rate = 4e6; // lime specific

// some global shared variables for use by the threads
atomic<uint64_t> rx_timestamp      = 0; // latest timestamp from rx, we know of
atomic<size_t>   cpf               = 0; // cyclic prefix len, 0 ... prefix_divider

//complex<float> buf_4MHz[40*512];   // 24*512*5/3
//complex<float> buf_2_4MHz[24*512]; // 24*512
//rresamp_cccf rs;

//void receive_cb(unsigned char*buf, uint32_t len, void* ctx) {
//  hrframesync& fs (*static_cast<hrframesync*>(ctx));
//  complex<uint8_t>* buf_8 = reinterpret_cast<complex<uint8_t>*>(buf);
//  for (size_t n=0; n<24*512; ++n) buf_2_4MHz[n] = (buf_8[n].real()/127.5-1) + 1i*(buf_8[n].imag()/127.5-1);
//  rresamp_cccf_execute(rs, buf_2_4MHz, buf_4MHz);
//  fs.execute(buf_4MHz, 40*512);
//}

void receive(stop_token stoken, lms_stream_t& rx_stream) {
  wrframesync fs(sample_rate, cpf);

  vector<complex<float>> rx_buffer(1024*4);
  lms_stream_meta_t meta;
  int ret = LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &meta, 1000);
  rx_timestamp = meta.timestamp +  ret;

  while(!stoken.stop_requested()) {
      ret = LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &meta, 1000);
      rx_timestamp = meta.timestamp + ret;
      fs.execute(rx_buffer.data(), rx_buffer.size());
  }

  cout << "Receive thread stopping."  << endl;


  // We need to resample, due to a bug in liquiddsp.
//  rs = rresamp_cccf_create_default(40*512, 24*512);
//  rtlsdr_read_async(dev, receive_cb, &fs, 0, 2*24*512);
}

int main(int argc, char* argv[]) {

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  lms_device_t* dev = nullptr;


  try {

    cxxopts::Options options("wrrx_lime", "Beacon receiver for WRAN project.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("freq", "Center frequency.", cxxopts::value<double>()->default_value("53e6"))
        ("cpf", "Cyclic prefix len: 0...50", cxxopts::value<size_t>()->default_value(("12")))
        ("gain", "Gain factor 0 ... 1.0", cxxopts::value<double>()->default_value("0.7"))
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

    double band_width = 2.5e6;
    double freq       = vm["freq"].as<double>();
    double gain       = vm["gain"].as<double>();

    cout << "wrrx-lime parameters:" << endl;
    cout << format("Freq:       %g MHz\n") % (1e-6*freq);
    cout << format("Prefix len: %2d\n")             % cpf;
    cout << format("Samplerate: %g MHz\n")          % (1e-6*sample_rate);
    cout << endl;

    // Try to open devices until one is available or fail if none.
    lms_info_str_t list[8];
    int numdev = LMS_GetDeviceList(list);
    if (numdev < 1) lime::error("No device found");
    int n = 0;
    for (; n < numdev; ++n) {
        try {
          LMS_Open(&dev, list[n], nullptr);
          lime::info("Device: %s", list[n]);
          break;
        } catch(runtime_error& e) { /* possibly busy, try next one */ }
      }
    if (n == numdev)
      lime::error("all devices already in use");

    LMS_Init(dev);

    LMS_SetSampleRate(dev,     sample_rate, 0);

    // initialize receive direction
    LMS_EnableChannel(dev,     LMS_CH_RX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_RX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_RX, 0, LMS_PATH_LNAW);
    LMS_SetNormalizedGain(dev, LMS_CH_RX, 0, gain);
    LMS_Calibrate(dev,         LMS_CH_RX, 0, band_width, 0);

    lms_stream_t rx_stream;
    rx_stream.channel = 0;
    rx_stream.fifoSize = 8*1024;
    rx_stream.throughputVsLatency = 0.5;
    rx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    rx_stream.isTx = false;
    LMS_SetupStream(dev, &rx_stream);

    LMS_StartStream(&rx_stream);

    // initialize transmit direction, even if we are not using it!
    LMS_EnableChannel(dev,     LMS_CH_TX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_TX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_TX, 0, LMS_PATH_TX2);
    LMS_SetNormalizedGain(dev, LMS_CH_TX, 0, gain);
    LMS_Calibrate(dev,         LMS_CH_TX, 0, band_width, 0);

    lms_stream_t tx_stream;
    tx_stream.channel = 0;
    tx_stream.fifoSize = 1024;
    tx_stream.throughputVsLatency = 0.5;
    assert(8*sizeof(float) == 32);
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;
    LMS_SetupStream(dev, &tx_stream);
    LMS_StartStream(&tx_stream);

    // start the receiver thread
    jthread rx_thread(receive, ref(rx_stream));

    cout << "Beacon rx control thread: enter EOF (Ctrl-D) or empty line to end." << endl;
    string line;
    while(getline(cin, line)) {
        if (line.empty()) break;
      }

    cout << "Exiting and cleaning up." << endl;

    rx_thread.request_stop();
    rx_thread.join();

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(dev, &tx_stream);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, false);

    LMS_StopStream(&rx_stream);
    LMS_DestroyStream(dev, &rx_stream);
    LMS_EnableChannel(dev, LMS_CH_RX, 0, false);

    LMS_Close(dev);
    dev = nullptr;

    cout << "Stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    if (nullptr != dev)
      LMS_Close(dev);
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    if (nullptr != dev)
      LMS_Close(dev);
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
