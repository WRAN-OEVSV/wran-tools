/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A tool to control the GPIO pins of the OEVSV WRAN box. If you modify
 * the code please note to take care that the both lower end bits always
 * sahll are at the same level to ensure a consistent state of the PA and TX/RX
 * switch.
 */

#include "config.hpp"

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <boost/format.hpp>
using boost::format;

#include<curses.h>

#include <cstdlib>

#include <cctype>
using std::isxdigit;

#include <iostream>
using std::cout;
using std::cerr;
using std::clog;
using std::cin;
using std::endl;

#include <string>
using std::string;

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

int main() {

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  try {

    cout << "WRAN GPIO tool." << endl;

    lms_info_str_t list[8];
    int n = LMS_GetDeviceList(list);
    if (n < 1) lime::error("No device found");
    cout << "Found device: " << list[0] << endl;

    lms_device_t* dev = nullptr;
    LMS_Open(&dev, list[0], nullptr);

    LMS_Init(dev);

    // Set all GPIO pins to output
    const uint8_t gpio_dir = 0xFF;
    LMS_GPIODirWrite(dev, &gpio_dir, 1);

    uint8_t gpio_val = 0;

    cout << "Enter [hexdigit]mode, or EOF (Ctrl-D) to end." << endl;
    cout << "    Optional hexdigit selects front panel LED's." << endl;
    cout << "    Mode is one of r, t, t6, t2, t70" << endl;
    cout << "e.g. ft6 switch on all LED's and PA with 6m filters." << endl;
    string line;
    while(getline(cin, line)) {
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
    }
    cout << "Exit and reset GPIO to 0x00." << endl;

    gpio_val = 0;
    LMS_GPIOWrite(dev, &gpio_val, 1);

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
