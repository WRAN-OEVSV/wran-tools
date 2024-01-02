/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MORSECODE_HPP
#define MORSECODE_HPP

#include <string>

std::string
char_code(
    char c
);

extern const std::string begin_of_message;
extern const std::string end_of_message;

#endif // MORSECODE_HPP
