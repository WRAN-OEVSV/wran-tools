/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A tool for testing the signal paths of the WRAN box. The local oszillator
 * frequency, the band filters, the gain, and a amplitude modulation tone can
 * be set.
 */

#include "config.hpp"

#include <cxxopts.hpp>

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <cstdlib>
using std::size;

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <map>
using std::map;

#include <cmath>
using std::acos, std::cos, std::sin;
const double Pi = acos(-1);

#include <complex>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <chrono>
using std::chrono::high_resolution_clock;
using namespace std::literals::chrono_literals;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <thread>
using std::jthread, std::stop_token;

#include <mutex>
using std::mutex;

#include <condition_variable>
using std::condition_variable;

#include <atomic>
using std::atomic;

#include <string>
using std::string, std::getline;

#include <vector>
using std::vector;

#include <functional>
using std::ref;

#include <chrono>
using std::chrono::high_resolution_clock, std::chrono::duration_cast, std::chrono::nanoseconds;

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

//atomic<uint64_t> timestamp;
const uint16_t samples_per_frame = 33280;
const uint16_t frames_per_second = 100;


// The transmit thread.
void transmit(stop_token stoken, lms_stream_t& tx_stream, uint64_t start_time) {

  vector<complex<float>> tx_buffer(3328);
  uint64_t timestamp = start_time;
  complex<double> w = exp(2i*Pi*0.001);
  complex<double> osz = conj(w);
  lms_stream_meta_t meta;

  while(!stoken.stop_requested()) {
      for (size_t m=0; m<tx_buffer.size(); ++m) tx_buffer[m] = (osz*=w);
      timestamp += samples_per_frame;
      meta.timestamp = timestamp;
      meta.waitForTimestamp = true;
      meta.flushPartialPacket = true;
      LMS_SendStream(&tx_stream, tx_buffer.data(), tx_buffer.size(), &meta, 1000);
  }

  cout << "Transmit thread stopping." << endl;
}

// The receive thread
void receive(stop_token stoken, lms_stream_t& rx_stream, lms_stream_t& tx_stream) {
  vector<complex<float>> rx_buffer(1024*4);
  lms_stream_meta_t meta;
  LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &meta, 1000);
  jthread tx_thread(transmit, ref(tx_stream), meta.timestamp);
  while(!stoken.stop_requested()) {
      LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &meta, 1000);
      // The receive thread does nothing but transfer the samples yet.
  }
  cout << "Receive thread stopping." << endl;
  tx_thread.request_stop();
  tx_thread.join();
}

// The control thread.
int main(int argc, char* argv[])
{

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  lms_device_t* dev = nullptr;

  try {

    cxxopts::Options options("wrbeacon", "Beacon generator for WRAN project.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("freq", "Center frequency.", cxxopts::value<double>()->default_value("53e6"))
        ("gain", "Gain factor 0 ... 1.0", cxxopts::value<double>()->default_value("0.7"))
        ;
    options.parse_positional({"freq", "gain"});
    options.positional_help("freq gain");
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

    cout << "wrbeacon parameters:" << endl;
    cout << format("Freq: %5.1f MHz\n") % (1e-6*vm["freq"].as<double>());
    cout << format("Gain: %0.3f\n") % vm["gain"].as<double>();
    cout << endl;

    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) lime::error("No device found");
    lime::info("Device: %s", list[0]);

    LMS_Open(&dev, list[0], nullptr);
    LMS_Init(dev);

    double samp_rate  = samples_per_frame*frames_per_second;
    double band_width = samp_rate;
    double freq       = vm["freq"].as<double>();
    double gain       = vm["gain"].as<double>();

    // Set up GPIO.
    uint8_t gpio_dir = 0xFF;
    LMS_GPIODirWrite(dev, &gpio_dir, 1);

    uint8_t gpio_val = 0;
    LMS_GPIODirRead(dev, &gpio_val, 1);
    lime::debug("GPIODIR: 0x%02x", unsigned(gpio_val));

    gpio_val = 0;
    LMS_GPIOWrite(dev, &gpio_val, 1);

    // Set up TX indicator on GPIO0.
    // https://github.com/myriadrf/LimeSDR-Mini-v2_GW/issues/3
    uint16_t fpga_val = 0;
    LMS_ReadFPGAReg(dev, 0x00c0, &fpga_val);
    fpga_val &= 0xfffe;
    LMS_WriteFPGAReg(dev, 0x00c0, fpga_val);

    // TXANT_PRE
    fpga_val = static_cast<uint16_t>(10.5e-6*samp_rate);
    LMS_WriteFPGAReg(dev, 0x0010, fpga_val);

    // TXANT_POST
    fpga_val = static_cast<uint16_t>(11.5e-6*samp_rate);;
    LMS_WriteFPGAReg(dev, 0x0011, fpga_val);

    LMS_SetSampleRate(dev,     samp_rate, 0);

    // initialize receive direction
    LMS_EnableChannel(dev,     LMS_CH_RX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_RX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_RX, 0, LMS_PATH_LNAW);
    LMS_SetNormalizedGain(dev, LMS_CH_RX, 0, gain);
    LMS_Calibrate(dev,         LMS_CH_RX, 0, band_width, 0);

    lms_stream_t rx_stream;
    rx_stream.channel = 0;
    rx_stream.fifoSize = 256*1024;
    rx_stream.throughputVsLatency = 0.5;
    rx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    rx_stream.isTx = false;
    LMS_SetupStream(dev, &rx_stream);

    LMS_StartStream(&rx_stream);

    // initialize transmit direction
    LMS_EnableChannel(dev,     LMS_CH_TX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_TX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_TX, 0, LMS_PATH_TX2);
    LMS_SetNormalizedGain(dev, LMS_CH_TX, 0, gain);
    LMS_Calibrate(dev,         LMS_CH_TX, 0, band_width, 0);

    lms_stream_t tx_stream;
    tx_stream.channel = 0;
    tx_stream.fifoSize = 256*1024;
    tx_stream.throughputVsLatency = 0.5;
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;
    LMS_SetupStream(dev, &tx_stream);

    LMS_StartStream(&tx_stream);
    jthread rx_thread(receive, ref(rx_stream), ref(tx_stream));
    // transmit is started from inside receive thread

    cout << "Beacon control thread: enter EOF (Ctrl-D) or empty line to end." << endl;
    string line;
    while(getline(cin, line)) {
        if (line.empty()) break;
    }
    cout << "Exiting and cleaning up." << endl;

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(dev, &tx_stream);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, false);

    rx_thread.request_stop();
    rx_thread.join();
    LMS_StopStream(&rx_stream);
    LMS_DestroyStream(dev, &rx_stream);
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
