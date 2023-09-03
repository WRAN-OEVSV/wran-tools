<!--
SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>

SPDX-License-Identifier: CC-BY-3.0
-->

wran-tools
==========

The wran-tools are intended to support hardware development and trouble
shooting of the OEVSV-WRAN modem.

What can I do with these tools?
-------------------------------

At the moment programs

* **wrgpio** for controlling of the GPIO pins located at LimeSDR Mini and
* **wrsiggen** a program to generate a sinus signal
* **wrbeacon** a program that evenually shall generate a beacon.
* 
are available. The programs are intended for use with the LimeSDR Mini v2.2, 
but might work with the Lime Mini 1.

Installation
------------

The *wran-tools* can be built with the help of cmake and a C++ compiler. If
you are using debian or ubuntu, the prerequisites can be installed with

    apt-get install liblimesuite-dev \
                    libcxxopts-dev libboost-dev libboost-system-dev

Of course every other method installing the prerequisites is fine too, just
take care, that the cmake config files get installed too.

Clone the repository to a covneient directory, choose a name for a build
directory, crate it and *cd* into it.

    cmake <path/to/git/source/dir>
    cmake --build .

If you want you may install the binaries to the prefix path. For details
please consult the cmake user guide. If you need the tools q&d, just try to
run them from the build directory.

Usage
-----

    wrgpio

The *wrgpio* tool runs a little loop, taking one line input from the keyboard
and writing its values to the GPIO pins of the LimeMini. The commands are
made of one optional hex character plus one of *r, t, t6, t2, t70*. The leading
hex character, if present, turns on one of the LED's on the front panel, while
the second part switches the power amplifier, the TX/RX switch and the
bandfilters. The leading hex character allows independent control of all the
bits, while the second part takes care of the fact that the TX/RX and PA shall
be allways both on or both off.

Example:

    bt70<enter>

will turn on LED 1,3,4, switch on PA and TX, and select the 70cm bandfilter.
You may end the program by hitting return on an empty line. The tool will set
all bits to zero before quitting.

    wrsiggen

The *wrsiggen* tool is intended to test the signal path of the modem plus 
amplifier. Using this tool the modem will produce RF output at a choosen level.
Please take care that you have connected a sufficient dummy load and consider
that sending a CW tone for an extended period of time may cause a lot of
heating of the PA. If left unattended the program will stop after 10 minutes.
It is recommended, however, to end the program by sending an interrupt signal,
i.e. Ctrl-C, as soon as you have finished your particular measurement.

    wrsiggen --help

will show a help screen with the various parameters and their default values.
You either may specify all of the parameters in a positional manner without
the need to write the parameter names, or only specify specific parameters
while leaving others at their default value.

Please note that the gain adjustment reacts "bitingly". So be careful and
increase the value only slowly considering the current consumption of the modem.

Contributing
------------

The WRAN modem is a complex project, which is carried out by a small group of
enthusiastic Austrian radio amateurs. We are very happy if you want to to
support our work by contributing to the code base. Just send and email to one
of the authors or just me at oe1rsa@oevsv.at

