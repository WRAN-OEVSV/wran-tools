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
#include "keyer.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using po::options_description, po::value, po::variables_map, po::store,
  po::positional_options_description, po::command_line_parser, po::notify;

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <complex> // NB: Must be included before liquid.h !
#include <liquid/liquid.h>

#include <cstdlib>
using std::size_t;

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <map>
using std::map;

#include <initializer_list>
using std::initializer_list;

#include <complex>
using std::complex, std::abs;
using namespace std::literals::complex_literals;

#include <algorithm>
using std::max;

#include <limits>
using std::numeric_limits;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <thread>
using std::jthread, std::stop_token, std::this_thread::sleep_for;

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
atomic<double>   tone          = 100e3; // Hz
string message;                         // the beacon text.
mutex message_mutex;

// Note: LMS_SendStream and LMS_RecvStream are the ONLY thread safe functions
// of limesuite.
// The transmit thread:
void transmit(stop_token stoken, lms_stream_t& tx_stream, uint64_t start_time) {

  unsigned int count = 0;
  //keyer k(60, tone, 0, 1.0, sample_rate);

  float sample_max = numeric_limits<float>::min();

  unsigned char header[8] = {0,0,0,0,0,0,0,0};

  hrframegen fg(cpf, phy_mode);
  uint32_t samp_per_frame = sample_rate*hrframegen::frame_len;

  vector<complex<float>> wr_buffer(fg.subcarriers + fg.prefix_len);
  vector<complex<float>> tx_buffer(wr_buffer.size()*sample_rate/fg.sample_rate);
  rresamp_cccf rs = rresamp_cccf_create_default(tx_buffer.size(), wr_buffer.size());
  rresamp_cccf_set_scale(rs, 0.245f);

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
          bool last = fg.write(wr_buffer.data(), wr_buffer.size());
          meta.timestamp = tx_timestamp;
          meta.waitForTimestamp = true;
          while (not last) {
              rresamp_cccf_execute(rs, wr_buffer.data(), tx_buffer.data());
              for (size_t i=0; i<tx_buffer.size(); ++i) sample_max = max(sample_max, abs(tx_buffer[i]));
              LMS_SendStream(&tx_stream, tx_buffer.data(), tx_buffer.size(), &meta, 1000);
              // TODO: I do not yet understand the write function completely.
              // The documentaion and source of liquid appear incomplete.
              last = fg.write(wr_buffer.data(), wr_buffer.size());
              meta.waitForTimestamp = false;
              meta.flushPartialPacket = false;
            }
          meta.flushPartialPacket = true;
          rresamp_cccf_execute(rs, wr_buffer.data(), tx_buffer.data());
          for (size_t i=0; i<tx_buffer.size(); ++i) sample_max = max(sample_max, abs(tx_buffer[i]));
          LMS_SendStream(&tx_stream, tx_buffer.data(), tx_buffer.size(), &meta, 1000);
        }
    }

  cout << "sample_max = " << sample_max << ", 1/sample_max = " << 1/sample_max << endl;
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
  signal(SIGTERM, signal_handler);

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  lms_device_t* dev = nullptr;

  try {

    options_description opts("Options");
    opts.add_options()
        ("help,h", "Print usage information.")
        ("version", "Print version.")
        ("freq", value<double>()->default_value(53e6), "Center frequency.")
        ("txpwr", value<int>()->default_value(0), "Tx Pwr. in in dBm (-26dBm ... 10dBm)")
        ("tone", value<double>()->default_value(100e3), "Modulation freqeuncy.")
        ("cpf", value<size_t>()->default_value(1), "Cyclic prefix len")
        ("phy", value<size_t>()->default_value(2), "Physical layer mode: 1 ... 14")
        ;

    options_description pos_opts;
    pos_opts.add_options()
        ("message", value<string>()->default_value("OE1XTU"), "Beacon message")
        ;
    positional_options_description pos;
    pos.add("message", 1);

    options_description all_opts;
    all_opts.add(opts).add(pos_opts);

    variables_map vm;
    store(command_line_parser(argc, argv).options(all_opts).positional(pos).run(), vm);
    notify(vm);


//    options.parse_positional({"message"});
//    options.positional_help("message");
//    options.show_positional_help();

//    auto vm = options.parse(argc, argv);

    if (vm.count("help")) {
        cout << "hrbeacon beacon generator for WRAN project." << endl;
        cout << "Usage: hrbeacon [options] [message]" << endl;
        cout << "  message default is " << vm["message"].as<string>() << endl;
        cout << opts << endl;
        return EXIT_SUCCESS;
    }

    if (vm.count("version")) {
        cout <<PROJECT_VER << endl;
        return EXIT_SUCCESS;
    }

    message = vm["message"].as<string>();  // no need for message_mutex yet.

    cpf = vm["cpf"].as<size_t>();
    if (0 == cpf or  cpf > hrframegen::prefix_divider)
      throw runtime_error("prefix not in range");

    phy_mode = vm["phy"].as<size_t>();
    if (0 == phy_mode or phy_mode > 14)
      throw runtime_error("invalid phymode requested");

    //double band_width = 2.0e6;
    double freq       = vm["freq"].as<double>();
    double txpwr      = vm["txpwr"].as<int>();

    tone  = vm["tone"].as<double>();

    cout << "hrbeacon. Version " PROJECT_VER << endl;

    cout << "Parameters:" << endl;
    cout << format("Freq:       %g MHz\n") % (1e-6*freq);
    cout << format("Peak Pwr:   %g dBm\n") % vm["txpwr"].as<int>();
    cout << format("Samplerate: %g MHz\n") % (1e-6*sample_rate);
    cout << format("Prefix len: %2d\n")    % cpf;
    cout << format("Phymode:    %2d\n")    % phy_mode;
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

    // Set up GPIO.
    uint8_t gpio_dir = 0xFF;
    LMS_GPIODirWrite(dev, &gpio_dir, 1);

    uint8_t gpio_val = 0;
    LMS_GPIODirRead(dev, &gpio_val, 1);
    lime::debug("GPIODIR: 0x%02x", unsigned(gpio_val));


// This is the fast, data synchronous TX/TX signaling
#if 0
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
#endif

    LMS_SetSampleRate(dev,     sample_rate, 0);

    // initialize receive direction
    LMS_EnableChannel(dev,     LMS_CH_RX, 0, true);
    LMS_SetAntenna(dev,        LMS_CH_RX, 0, LMS_PATH_LNAW);
    LMS_SetLOFrequency(dev,    LMS_CH_RX, 0, freq);
    LMS_SetNormalizedGain(dev, LMS_CH_RX, 0, 1.0);
    LMS_SetGFIRLPF(dev,        LMS_CH_RX, 0, true, 1.86e6);
    LMS_SetLPFBW(dev,          LMS_CH_RX, 0, 5e6);

    lms_stream_t rx_stream;
    rx_stream.channel = 0;
    rx_stream.fifoSize = 1024;
    rx_stream.throughputVsLatency = 0.5;
    rx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    rx_stream.isTx = false;

    // initialize transmit direction
    LMS_EnableChannel(dev,     LMS_CH_TX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_TX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_TX, 0, LMS_PATH_TX2);
    LMS_SetGaindB(dev,         LMS_CH_TX, 0, 61+txpwr);
    LMS_SetGFIRLPF(dev,        LMS_CH_TX, 0, true, 1.86e6);
    LMS_SetLPFBW(dev,          LMS_CH_TX, 0, 5e6);

    lms_stream_t tx_stream;
    tx_stream.channel = 0;
    tx_stream.fifoSize = 256*1024;
    tx_stream.throughputVsLatency = 0.5;
    assert(8*sizeof(float) == 32);
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;

    LMS_Calibrate(dev,         LMS_CH_RX, 0, 5e6, 0);
    LMS_Calibrate(dev,         LMS_CH_TX, 0, 5e6, 0);

    LMS_SetupStream(dev, &rx_stream);
    LMS_SetupStream(dev, &tx_stream);

    LMS_StartStream(&rx_stream);
    LMS_StartStream(&tx_stream);

    // turn on the PA
    gpio_val = 0xb3; // 0xbf; //0xb3;
    LMS_GPIOWrite(dev, &gpio_val, 1);

    // start the threads
    jthread rx_thread(receive, ref(rx_stream), ref(tx_stream));
    // transmit is started from inside receive thread

    cout << "hrbeacon control thread: stop with SIGINT (Ctrl-C) or SIGTERM" << endl;
    while (0 == signal_status) {
        sleep_for(100ms);
    }
    cout << "Caught signal " << signal_status << ", exiting ..." << endl;

    rx_thread.request_stop();
    rx_thread.join();

    cout << "Cleaning up ..." << endl;

    // turn off PA
    gpio_val = 0x00;
    LMS_GPIOWrite(dev, &gpio_val, 1);

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(dev, &tx_stream);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, false);

    LMS_StopStream(&rx_stream);
    LMS_DestroyStream(dev, &rx_stream);
    LMS_EnableChannel(dev, LMS_CH_RX, 0, false);

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
