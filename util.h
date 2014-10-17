/**
 *  stm32tpl --  STM32 C++ Template Peripheral Library
 *
 *  Copyright (c) 2010-2014 Anton B. Gusev aka AHTOXA
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  file          : util.h
 *  description   : Misc utility functions.
 *  created on    : 07.06.2011
 *
 */

#ifndef STM32TPL_UTIL_H_INCLUDED
#define STM32TPL_UTIL_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

char HexChar(char ch);
void strreverse(char* begin, char* end);
char *str_upr(char *s);
char* itoa(int value, char* s, int base);
long small_atoi(char * s);
double small_atof(const char * s);
uint32_t small_atoh(char * s);

char small_isxdigit (unsigned char c);
uint8_t small_isspace (char c);
char * trim(char *p);
char * GetToken(char *s, char **last);
char * GetToken2(char *s, char **last, const char * dlims);
char * get_float_token(double* f, char **last);
char * get_u16_token(uint16_t* Int, char **last);
char * get_int_token(int* Int, char **last);
char * get_byte_token(uint8_t* c, char **last);
void pad(char * s, int width, char padder);

#ifdef __cplusplus
}
#endif

#endif
