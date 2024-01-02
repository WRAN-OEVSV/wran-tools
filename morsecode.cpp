/*
 * SPDX-FileCopyrightText: 2023 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * Morse code lookup table, originally developed by me in 2009 for the
 * morsepod program.
*/

#include "morsecode.hpp"

#include <string>
using std::string;

#include <map>
using std::map;

#include <utility>
using std::pair;

#include <locale>
using std::toupper;

namespace {

  const map<char, const char*> code = {
    {'A',  "._"}
    , {'B',  "_..."}
    , {'C',  "_._."}
    , {'D',  "_.."}
    , {'E',  "."}
    , {'F',  ".._."}
    , {'G',  "__."}
    , {'H',  "...."}
    , {'I',  ".."}
    , {'J',  ".___"}
    , {'K',  "_._"}
    , {'L',  "._.."}
    , {'M',  "__"}
    , {'N',  "_."}
    , {'O',  "___"}
    , {'P',  ".__."}
    , {'Q',  "__._"}
    , {'R',  "._."}
    , {'S',  "..."}
    , {'T',  "_"}
    , {'U',  ".._"}
    , {'V',  "..._"}
    , {'W',  ".__"}
    , {'X',  "_.._"}
    , {'Y',  "_.__"}
    , {'Z',  "__.."}
    , {'1',  ".____"}
    , {'2',  "..___"}
    , {'3',  "...__"}
    , {'4',  "...._"}
    , {'5',  "....."}
    , {'6',  "_...."}
    , {'7',  "__..."}
    , {'8',  "___.."}
    , {'9',  "____."}
    , {'0',  "_____"}
    , {'.',  "._._._"}
    , {',',  "__..__"}
    , {'?',  "..__.."}
    , {'\'', ".____."}
    , {'/',  "_.._."}
    , {'(',  "_.__."}
    , {')',  "_.__._"}
    , {':',  "___..."}
    , {';',  "_._._."}
    , {'=',  "_..._"}
    , {'+',  "._._."}
    , {'-',  "_...._"}
    , {'_',  "..__._"}
    , {'"',  "._.._."}
    , {'@',  ".__._."}
    , {'*',  "_.._"}
  };

}

string
char_code(
    char c
) {
    auto it = code.find(toupper(c));
    if (it != code.end())
        return it->second;
    else
        return code.find('?')->second;
}

const string begin_of_message("_._._");
const string end_of_message("._._.");
