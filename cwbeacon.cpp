/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A CW Beacon for the HAMRAN project.
 */

#include "config.hpp"

#include "keyer.hpp"

#include <cxxopts.hpp>

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <iostream>
using std::cout, std::cin, std::cerr, std::clog, std::endl;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <atomic>
using std::atomic;

#include <string>
using std::string, std::getline;

#include <map>
using std::map;

#include <cmath>
using std::acos, std::cos, std::sin;

#include <complex>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <algorithm>
using std::max;

#include <limits>
using std::numeric_limits;

#include <thread>
using std::jthread, std::stop_token, std::this_thread::sleep_for;

#include <mutex>
using std::mutex;

#include <condition_variable>
using std::condition_variable;

#include <chrono>
using std::chrono::high_resolution_clock;
using namespace std::literals::chrono_literals;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <vector>
using std::vector;

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
          //rem: throwing on error avoids verbose error handling code
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

map<string, uint8_t> gpio_map{{"TXwoBP", 0xbf}, {"TX6m", 0xb3}, {"TX2m", 0xb7}, {"TX70cm", 0xbb}};
const double sample_rate = 2.5e6; // MSps

// some global shared variables for use by the threads
atomic<uint64_t> rx_timestamp = 0; // latest timestamp from rx, we know of
atomic<double>   tone         = 100e3; //kHz

string message; // the beacon text.
mutex message_mutex;

// Note: LMS_SendStream and LMS_RecvStream are the ONLY thread safe functions
// of limesuite.
// The transmit thread:
void transmit(stop_token stoken, lms_stream_t& tx_stream) {

  keyer k(60, tone, 0, 1.0, sample_rate);

  const int buffer_size = 1024*8;
  complex<float> tx_buffer[buffer_size];

  while(!stoken.stop_requested()) {
      size_t size = k.get_frame(tx_buffer, buffer_size);
      if (0 == size) {
          message_mutex.lock();
          k.send(message);
          message_mutex.unlock();
          k.space(2.0);
          k.mark(20.0);
          float fill_time = 60.0 - k.pending_time();
          while (fill_time < 0.0)
            fill_time += 60.0;
          k.space(fill_time);
          size = k.get_frame(tx_buffer, buffer_size);
        }

    LMS_SendStream(&tx_stream, tx_buffer, size, nullptr, 1000);
  }

  cout << "Transmit thread stopping." << endl;
}

// The control thread.
int main(int argc, char* argv[])
{

  signal(SIGINT,  signal_handler); // install handler to catch ctrl-c
  signal(SIGTERM, signal_handler);

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  lms_device_t* dev = nullptr;

  try {

    cxxopts::Options options("wrsiggen", "WRAN signal generator tool.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("mode", "TXwoBP, TX6m, TX2m, TX70cm", cxxopts::value<string>()->default_value("TXwoBP"))
        ("freq", "Center frequency.", cxxopts::value<double>()->default_value("52.9e6"))
        ("txpwr", "Tx Pwr. in in dBm (-26dBm ... 10dBm)", cxxopts::value<int>()->default_value("-10"))
        ("tone", "Modulation freqeuncy.", cxxopts::value<double>()->default_value("100e3"))
        ("message", "Beacon message", cxxopts::value<string>()->default_value("OE1XTU"))
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

    message = vm["message"].as<string>(); // no need for message_mutex yet.

    if (1 != gpio_map.count(vm["mode"].as<string>()))
      throw runtime_error(str(format("%s is an invalid mode") % vm["mode"].as<string>()));

    double freq  = vm["freq"].as<double>();
    double txpwr = vm["txpwr"].as<int>();

    tone  = vm["tone"].as<double>();

    cout << "Parameters:" << endl;
    cout << format("Freq: %5.1f MHz\n")    % (1e-6*freq);
    cout << format("Peak Pwr:   %g dBm\n") % txpwr;
    cout << format("Samplerate: %g MHz\n") % (1e-6*sample_rate);
    cout << format("Mode: %s\n")           % vm["mode"].as<string>();
    cout << format("Tone: %3.3f kHz\n")    % (1e-3*tone);
    cout << format("Message: %s\n")        % message;
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
        } catch(runtime_error& e) { /* possibly busy, try next one */}
      }
    if (n == numdev)
      lime::error("No device could be obened");

    LMS_Init(dev);

    lms_range_t lpf_tx;
    LMS_GetLPFBWRange(dev, LMS_CH_TX, &lpf_tx);
    cout << format("Tx Filter Range: min %.3f MHz, max %.3f MHz, step %.1f Hz\n") % (1e-6*lpf_tx.min) % (1e-6*lpf_tx.max) % (lpf_tx.step);

    uint8_t gpio_dir = 0xFF;
    LMS_GPIODirWrite(dev, &gpio_dir, 1);

    uint8_t gpio_val = 0;
    LMS_GPIODirRead(dev, &gpio_val, 1);
    lime::debug("GPIODIR: 0x%02x", unsigned(gpio_val));

    gpio_val = gpio_map[vm["mode"].as<string>()];
    lime::info("GPIO: 0x%02x", unsigned(gpio_val));
    LMS_GPIOWrite(dev, &gpio_val, 1);

    LMS_SetSampleRate(dev, sample_rate, 0);

    // initialize transmit direction
    LMS_EnableChannel(dev,  LMS_CH_TX, 0, true);
    LMS_SetLOFrequency(dev, LMS_CH_TX, 0, freq);
    LMS_SetAntenna(dev,     LMS_CH_TX, 0, LMS_PATH_TX2);
    LMS_SetGaindB(dev,      LMS_CH_TX, 0, 61+txpwr);
    LMS_SetGFIRLPF(dev,     LMS_CH_TX, 0, true, 1.8e6);
    LMS_SetLPFBW(dev,       LMS_CH_TX, 0, 5e6);

    lms_stream_t tx_stream;
    tx_stream.channel = 0;
    tx_stream.fifoSize = 256*1024;
    tx_stream.throughputVsLatency = 0.5;
    assert(8*sizeof(float) == 32);
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;

    LMS_Calibrate(dev,      LMS_CH_TX, 0, 4e6, 0);
    LMS_SetupStream(dev, &tx_stream);
    LMS_StartStream(&tx_stream);

    // start the thread
    jthread tx_thread(transmit, ref(tx_stream));

    cout << "cwbeacon control thread: stop with SIGINT (Ctrl-C) or SIGTERM" << endl;
    while (0 == signal_status) {
        sleep_for(100ms);
    }
    cout << "Caught signal " << signal_status << ", exiting ..." << endl;

    tx_thread.request_stop();
    tx_thread.join();

    cout << "Cleaning up ..." << endl;

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(dev, &tx_stream);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, false);

//    LMS_StopStream(&rx_stream);
//    LMS_DestroyStream(dev, &rx_stream);
    LMS_EnableChannel(dev, LMS_CH_RX, 0, false);

    gpio_val = 0x00;
    LMS_GPIOWrite(dev, &gpio_val, 1);

    LMS_Close(dev);
    dev = nullptr;

    cout << "Control thread stopping." << endl;

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
