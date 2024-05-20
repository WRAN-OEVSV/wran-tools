/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A tool to control the GPIO pins of the OEVSV WRAN box. If you modify
 * the code please note to take care that the both lower end bits always
 * sahll are at the same level to ensure a consistent state of the PA and TX/RX
 * switch.
 * https://limesdr-mini.myriadrf.org/
 */

#include "config.hpp"

//#include <cxxopts.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
using po::options_description, po::value, po::variables_map, po::store,
  po::positional_options_description, po::command_line_parser, po::notify,
  po::parse_command_line;

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <boost/format.hpp>
using boost::format;


#include <cstdlib>

#include <cctype>
using std::isxdigit;

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl;

#include <string>
using std::string;
using std::getline;

#include <exception>
using std::exception;

#include <stdexcept>
using std::runtime_error;

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
}

int main(int argc, char* argv[]) {

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  lms_device_t* dev = nullptr;

  try {

    options_description opts("Options");
    opts.add_options()
        ("help,h", "Print usage information.")
        ("version", "Print version.")
        ("gpio", value<string>(), "[x](r|t|t6|t2|t70) with x opt. for upper nibble")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, opts), vm);
    notify(vm);

//    options.parse_positional("gpio");
//    options.positional_help("gpio");
//    options.show_positional_help();

//    auto vm = options.parse(argc, argv);

    if (vm.count("help")) {
        cout << "wrgpio GPIO manipulation tool." << endl;
        cout << opts << endl;
        return EXIT_SUCCESS;
    }

    if (vm.count("version")) {
        cout <<PROJECT_VER << endl;
        return EXIT_SUCCESS;
    }

    cout << "WRAN GPIO tool. Version " PROJECT_VER << endl;

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

#if 0
    // Set board gpio override to lime 1 mode
    // see LimeSDR-Mini_Gateware_Description pp19
    uint16_t fpga_val = 0;
    LMS_ReadFPGAReg(dev, 0x00c0, &fpga_val);
    fpga_val |= 0x00ff;
    LMS_WriteFPGAReg(dev, 0x00c0, fpga_val);
#endif

    // Set all GPIO pins to output
    const uint8_t gpio_dir = 0xFF;
    LMS_GPIODirWrite(dev, &gpio_dir, 1);

    uint8_t gpio_val = 0;
    LMS_GPIODirRead(dev, &gpio_val, 1);
    lime::debug("GPIODIR: 0x%02x", unsigned(gpio_val));

    bool interactive = 0 == vm.count("gpio");
    string line = interactive?"":vm["gpio"].as<string>();

    if (interactive) {
        cout << "Enter [hexdigit]mode, or EOF (Ctrl-D) to end." << endl;
        cout << "    Optional hexdigit selects front panel LED's." << endl;
        cout << "    Mode is one of r, t, t6, t2, t70" << endl;
        cout << "e.g. ft6 switch on all LED's and PA with 6m filters." << endl;
      }

    while(not interactive or getline(cin, line)) {
        gpio_val = 0;

        size_t pos = 0;
        if (line.empty()) break;
        if (isxdigit(line[0])) {
            gpio_val |= stoi(line.substr(0,1), &pos, 16) << 4;
            line = line.substr(1);
        }
        if      ("r"   == line) { gpio_val |= 0x00; }
        else if ("t"   == line) { gpio_val |= 0x0f; }
        else if ("t6"  == line) { gpio_val |= 0x03; }
        else if ("t2"  == line) { gpio_val |= 0x07; }
        else if ("t70" == line) { gpio_val |= 0x0b; }
        else { cout << "unrecognized input: " << line << endl;}
        cout << format("Set GPIO to: 0x%02x") % unsigned(gpio_val) << endl;
        LMS_GPIOWrite(dev, &gpio_val, 1);
        if (not interactive) break;
    }

    if (interactive) {
        cout << "Exit and reset GPIO to 0x00." << endl;

        gpio_val = 0x00;
        LMS_GPIOWrite(dev, &gpio_val, 1);
      }

    LMS_Close(dev);

  } catch(const exception& e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception: unknown reason." << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
