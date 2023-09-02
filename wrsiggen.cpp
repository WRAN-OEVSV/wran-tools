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

#include <iostream>
using std::cout;
using std::cerr;
using std::clog;
using std::endl;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <string>
using std::string;

#include <map>
using std::map;

#include <cmath>
using std::acos;
using std::cos;
using std::sin;

#include <complex>
using std::complex;
using std::exp;
using std::conj;

using namespace std::literals::complex_literals;

#include <chrono>
using std::chrono::high_resolution_clock;
using namespace std::literals::chrono_literals;

#include <csignal>
using std::sig_atomic_t;
using std::signal;

#include <vector>
using std::vector;

#include <boost/format.hpp>
using boost::format;
using boost::str;

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

int main(int argc, char* argv[])
{

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  try {

    cxxopts::Options options("wrsiggen", "WRAN signal generator tool.");
    options.add_options()
        ("h,help", "Print usage information.")
        ("version", "Print version.")
        ("mode", "TXwoBP, TX6m, TX2m, TX70cm", cxxopts::value<string>()->default_value("TXwoBP"))
        ("freq", "Center frequency.", cxxopts::value<double>()->default_value("53e6"))
        ("gain", "Gain factor 0 ... 1.0", cxxopts::value<double>()->default_value("0.7"))
        ("tone", "Modulation freqeuncy.", cxxopts::value<double>()->default_value("100e3"))
        ;
    options.parse_positional({"mode", "freq", "gain", "tone"});
    options.positional_help("mode freq gain tone");
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

    if (1 != gpio_map.count(vm["mode"].as<string>()))
      throw runtime_error(str(format("%s is an invalid mode") % vm["mode"].as<string>()));

    cout << format("Mode: %s\n") % vm["mode"].as<string>();
    cout << format("Freq: %5.1f MHz\n") % (1e-6*vm["freq"].as<double>());
    cout << format("Gain: %0.3f\n") % vm["gain"].as<double>();
    cout << format("Tone: %3.3f kHz\n") % (1e-3*vm["tone"].as<double>());
    cout << endl;
    cout << "Hint: press Ctrl-C if you want to abort before 10 minutes expired." << endl;
    cout << endl;

    vector<lms_info_str_t> list(LMS_GetDeviceList(nullptr));
    if (list.empty()) lime::error("No device list found");
    int n = LMS_GetDeviceList(list.data());
    if (n < 1) lime::error("No device found");
    lime::info("Device: %s", list[0]);

    lms_device_t* dev = nullptr;
    LMS_Open(&dev, list[0], nullptr);
    LMS_Init(dev);

    uint8_t gpio_dir = 0xFF;
    LMS_GPIODirWrite(dev, &gpio_dir, 1);

    uint8_t gpio_val = 0;
    LMS_GPIODirRead(dev, &gpio_val, 1);
    lime::debug("GPIODIR: 0x%02x", unsigned(gpio_val));

    gpio_val = gpio_map[vm["mode"].as<string>()];
    lime::info("GPIO: 0x%02x", unsigned(gpio_val));
    LMS_GPIOWrite(dev, &gpio_val, 1);

    double samp_rate = 2.5e6;
    double freq      = vm["freq"].as<double>();
    double gain      = vm["gain"].as<double>();
    double tone      = vm["tone"].as<double>();

    LMS_EnableChannel(dev, LMS_CH_TX, 0, true);
    LMS_SetSampleRate(dev, samp_rate, 0);
    LMS_SetLOFrequency(dev, LMS_CH_TX, 0, freq);
    LMS_SetAntenna(dev, LMS_CH_TX, 0, LMS_PATH_TX2);
    //LMS_SetLPFBW(dev, LMS_CH_TX, 0, 5e6);
    LMS_SetNormalizedGain(dev, LMS_CH_TX, 0, gain);
    LMS_Calibrate(dev, LMS_CH_TX, 0, samp_rate, 0);

    lms_stream_t tx_stream;
    tx_stream.channel = 0;
    tx_stream.fifoSize = 256*1024;
    tx_stream.throughputVsLatency = 0.5;
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;
    LMS_SetupStream(dev, &tx_stream);

    const int buffer_size = 1024*4;
    complex<float> tx_buffer[buffer_size];

    // The oscillator is implemented by rotation of a complex<double>
    // to lower the noise floor compared to the basic example.
    // Phase increment per step:
    complex<double> w = exp(2.0i*acos(-1)*tone/samp_rate);
    // Initialize the oscillator:
    complex<double> y = conj(w);

    // initialize the buffer
    for (size_t n = 0; n<buffer_size; ++n)
        tx_buffer[n] = exp(2.0i*acos(-1)*tone/samp_rate*double(n));

    LMS_StartStream(&tx_stream);

    auto t1 = high_resolution_clock::now();
    auto t2 = t1;
    while (high_resolution_clock::now() - t1 < 600s && SIGINT != signal_status) {
        // Fill the buffer with new oscillator values
        for (size_t n = 0; n<buffer_size; ++n)  tx_buffer[n] = (y*=w);
        int ret = LMS_SendStream(&tx_stream, tx_buffer, buffer_size, nullptr, 1000);
        if (ret != buffer_size)
          cerr << "Error: samples sent: " << ret << "/" << buffer_size << endl;
        if (high_resolution_clock::now()-t2 > 10s) {
            t2 = high_resolution_clock::now();
            lms_stream_status_t status;
            LMS_GetStreamStatus(&tx_stream, & status);
            lime::info("TX data rate: %.3f MB/s", 1e-6*status.linkRate);
        }
    }

    cout << "Exiting and cleaning up." << endl;

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(dev, &tx_stream);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, false);

    gpio_val = 0x00;
    LMS_GPIOWrite(dev, &gpio_val, 1);

    LMS_Close(dev);

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;
}
