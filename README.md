<!--
SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>

SPDX-License-Identifier: CC-BY-3.0
-->

hamran-tools
============

The hamran-tools are intended to support hardware development and trouble
shooting of the OEVSV-HAMRAN (formerly WRAN) modem.

What can I do with these tools?
-------------------------------

Purpose of programs:

* **gpio-lime**    control the GPIO pins located at LimeSDR Mini
* **siggen-lime**  generate a sinus signal
* **cwbeacon**     generate a CW beacon (lime)
* **hrbeacon**     generate a mixed CW / OFDM beacon
* **hrbeacon-sim** generate a file of OFDM signal
* **hrbeacon-txrx-sim** simulate beacon tx / rx under various SNR
* **hrrx-* **      various beacon receivers for rtl, hackrf and lime

The programs are intended for use with the LimeSDR Mini v2.2, 
but might work with the Lime Mini 1.

Installation
------------

The *hamran-tools* can be built with the help of cmake and a C++ compiler. If
you are using debian or ubuntu, the prerequisites can be installed with the
apt-get tool. I have also managed to cross build for the odroid platform and
windows. The process, however may be more contrieved. Also the list of 
prerequisites may be changing particularly during the early development phase
and version zero releases. If you are in doubt, check the CMakeLists.txt file.

I am a believer of the orthogonality principle, meaning this code is using 
libraries, the libraries are not part of it. So you will need to create a
build environment by yourself. There are no magic downloaders of used libraries.

The upside is, that package manager will like the approach since the code only
depends to a very small amount on platform peculiarities and avoids duplication,
the downside, howver is that it is harder for you to compile the programs if you
are not used to compile programs. Since this is open source, you should be able
to find someone who compiles the programs for you. (Possibly you could buy him a
bear or something she likes in return.) 

Please understand that I have not the time to explain how to use the CMake tool,
instead head over to their geŕeat documentation site: 
https://cmake.org/documentation/

Since the tools are meant to be hacked on, you are better off learning to
compile them yourself. There will be a separate procedure for the final 
*product* of this project.

Usage
-----

The tools have a built in help page thet can be invoked by giving the *--help* 
switch on the command line, which

    <tool> --help

will show a help screen with the various parameters and their default values.
You either may specify all of the parameters in a positional manner without
the need to write the parameter names, or only specify specific parameters
while leaving others at their default value.

For further details dont be shy and study the source, exercising one of your
four rights of free software: **use**,**study**,**share**,**improve**.

Contributing
------------

The HAMRAN modem is a complex project, which is carried out by a small group of
enthusiastic Austrian radio amateurs. We are very happy if you want to
support our work by contributing to the code base. Just send and email to one
of the authors or just me at oe1rsa@oevsv.at

