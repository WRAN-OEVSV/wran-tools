/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A tool for generating a beacon signal based on the work of Marek Honek
 * in his master thesis: SDR OFDM Frame Generation according to IEEE 802.22.
 * https://doi.org/10.34726/hss.2022.74390, but heavily adapted by OE1RSA.
 */

#include "config.hpp"
#include "hamranfrm.hpp"

#include <cxxopts.hpp>

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <cstdlib>
using std::size_t;

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl;

#include <fstream>
using std::ofstream;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <map>
using std::map;

#include <initializer_list>
using std::initializer_list;

#include <complex>
using std::complex, std::exp, std::conj;
using namespace std::literals::complex_literals;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <thread>
using std::jthread, std::stop_token;
using std::this_thread::sleep_for;

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
using std::chrono::high_resolution_clock, std::chrono::duration_cast,
  std::chrono::nanoseconds;
using namespace std::literals::chrono_literals;

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
atomic<size_t>   phy_mode          = 1; // physical layer mode;

// this is a hack, proper implementation pending
string global_message;

// Note: LMS_SendStream and LMS_RecvStream are the ONLY thread safe functions
// of limesuite.
// The transmit thread:
void transmit(stop_token stoken, lms_stream_t& tx_stream, uint64_t start_time) {

  string message = global_message; // I know this is unproteced
  unsigned char header[8] = {0,0,0,0,0,0,0,0};

  hrframegen fg(sample_rate, cpf, phy_mode);
  uint32_t samp_per_frame = sample_rate*hrframegen::frame_len;

  vector<complex<float>> tx_buffer(fg.subcarriers + (fg.subcarriers / cpf));
  uint64_t tx_timestamp = start_time;
  lms_stream_meta_t meta;

  while(!stoken.stop_requested()) {
      tx_timestamp += samp_per_frame;
      while(not stoken.stop_requested()
            and rx_timestamp + samp_per_frame < tx_timestamp) {
          sleep_for(10us); // In a bidirectionional setting waiting here could
                           // be replaced by transmit data preparation.
        }

      if (not stoken.stop_requested()) {
          fg.assemble(header, reinterpret_cast<unsigned char*>(message.data()), message.size());
          bool last = fg.write(tx_buffer.data(), tx_buffer.size());
          meta.timestamp = tx_timestamp;
          meta.waitForTimestamp = true;
          while (not last) {
              LMS_SendStream(&tx_stream, tx_buffer.data(), tx_buffer.size(), &meta, 1000);
              // TODO: I do not yet understand the write function completely.
              // The documentaion and source of liquid appear incomplete.
              last = fg.write(tx_buffer.data(), tx_buffer.size());
              meta.waitForTimestamp = false;
              meta.flushPartialPacket = false;
            }
          meta.flushPartialPacket = true;
          LMS_SendStream(&tx_stream, tx_buffer.data(), tx_buffer.size(), &meta, 1000);
        }
    }

  cout << "Transmit thread stopping." << endl;
}

// The receive thread:
void receive(stop_token stoken, lms_stream_t& rx_stream, lms_stream_t& tx_stream) {
  vector<complex<float>> rx_buffer(1024*4);
  lms_stream_meta_t meta;
  int ret = LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &meta, 1000);
  jthread tx_thread(transmit, ref(tx_stream), meta.timestamp);
  rx_timestamp = meta.timestamp +  ret;

  while(!stoken.stop_requested()) {
      ret = LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &meta, 1000);
      rx_timestamp = meta.timestamp + ret;
      // The receive thread does not do much yet, but fetch samples and update time.
  }

  cout << "Receive thread stopping."  << endl;

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
    if (0 == cpf or  cpf > hrframegen::prefix_divider)
      throw runtime_error("prefix not in range");

    phy_mode = vm["phy"].as<size_t>();
    if (0 == phy_mode or phy_mode > 14)
      throw runtime_error("invalid phymode requested");

    double band_width = 2.5e6;
    double freq       = vm["freq"].as<double>();
    double gain       = vm["gain"].as<double>();

    cout << "wrbeacon parameters:" << endl;
    cout << format("Freq:       %g MHz\n") % (1e-6*freq);
    cout << format("Gain:       %5.02f \n")         % vm["gain"].as<double>();
    cout << format("Prefix len: %2d\n")             % cpf;
    cout << format("Samplerate: %g MHz\n")          % (1e-6*sample_rate);
    cout << format("Phymode:    %2d\n")             % phy_mode;
    cout << endl;

    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) lime::error("No device found");
    lime::info("Device: %s", list[0]);

    LMS_Open(&dev, list[0], nullptr);
    LMS_Init(dev);

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
    fpga_val = static_cast<uint16_t>(10.5e-6*sample_rate);
    LMS_WriteFPGAReg(dev, 0x0010, fpga_val);

    // TXANT_POST
    fpga_val = static_cast<uint16_t>(11.5e-6*sample_rate);
    LMS_WriteFPGAReg(dev, 0x0011, fpga_val);

    LMS_SetSampleRate(dev,     sample_rate, 0);

    // initialize receive direction
    LMS_EnableChannel(dev,     LMS_CH_RX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_RX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_RX, 0, LMS_PATH_LNAW);
    LMS_SetNormalizedGain(dev, LMS_CH_RX, 0, gain);
    LMS_Calibrate(dev,         LMS_CH_RX, 0, band_width, 0);

    lms_stream_t rx_stream;
    rx_stream.channel = 0;
    rx_stream.fifoSize = 1024;
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
    assert(8*sizeof(float) == 32);
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;
    LMS_SetupStream(dev, &tx_stream);

    LMS_StartStream(&tx_stream);

    // start the threads
    jthread rx_thread(receive, ref(rx_stream), ref(tx_stream));
    // transmit is started from inside receive thread

    cout << "Beacon control thread: enter EOF (Ctrl-D) or empty line to end." << endl;
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
