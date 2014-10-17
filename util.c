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
 *  file          : util.c
 *  description   : Misc utility functions.
 *  created on    : 07.06.2011
 *
 */

#include "util.h"
#include <string.h>
#include <stdlib.h>

char small_isxdigit (unsigned char c)
{

   if (( c >= '0' && c <= '9')  ||
       ( c >= 'a' && c <= 'f')  ||
       ( c >= 'A' && c <= 'F'))
   	return 1;
   return 0;

}

char HexChar(char ch)
{
	static char const *const HexChars = "0123456789ABCDEF";
	return HexChars[ch&0x0F];
}

void strreverse(char* begin, char* end)
{
	while (end > begin)
	{
		char aux = *end;
		*end-- = *begin;
		*begin++ = aux;
	}
}

char* itoa(int value, char* s, int base)
{
	if (base < 2 || base > 16)
	{
		*s = '\0';
		return s;
	}

	int sign = value;
	char* ptr = s;

	do {
		*ptr++ = HexChar(abs(value % base));
		value /= base;
	} while (value);
	if (sign < 0)
		*ptr++ = '-';

	*ptr-- = '\0';
	strreverse(s, ptr);
	return s;
}

long small_atoi(char * s)
{
	long rv=0;
	char sign = 0;

	// skip till we find either a digit or '+' or '-'
	while (*s)
	{
		if (*s <= '9' && *s >= '0')
			break;
		if (*s == '-' || *s == '+')
			break;
		s++;
	}

	sign = (*s == '-');
	if (*s == '-' || *s == '+')
		s++;

	while (*s && *s >= '0' && *s <= '9')
	{
		rv = (rv * 10) + (*s - '0');
		s++;
	}

	return (sign ? -rv : rv);
}

const char* delims="\r\n\t :";

char *str_upr(char *s)
{
	char c;
	char * s1 =s;
	while(*s1)
	{
		c = *s1;
		if (c >= 'a' && c <= 'z') c -= ('a'-'A');
		*s1++ = c;
	}
	return s;
}

void pad(char * s, int width, char padder)
{
	int l;
	if (*s == '-') s++;

	l = strlen(s);
	width -= l;
	if (width < 0) return;
	memmove(s + width, s, l+1);
	memset(s, padder, width);
}

double small_atof(const char * s)
{
	char buf[16];
	strncpy(buf, s, 16);

	char * dot = strchr(buf, '.');
	if (!dot) dot = strchr(buf, ',');
	if (!dot)
		return small_atoi(buf);

	*dot++ = 0;
	double res = small_atoi(dot);
    int l = strlen(dot);
	while (l--)
	  res /= 10;
	double intpart = small_atoi(buf);
	if (intpart > 0.0)
		res += intpart;
	else
		res = intpart - res;

	return res;
}

char * trim(char *p)
{
	int len;

	len = strlen(p);
	while ((len > 0) && strchr(delims, p[len - 1] )) {
		p[len - 1] = '\0';
		len--;
	}
	while((*p)&&(strchr(delims,*p)))p++;
	return(p);
}

char * GetToken(char *s, char **last)
{
	const char *spanp;
	int c, sc;
	char *tok;

	if (s == 0 && (s = *last) == 0)
		return 0;

cont:
	c = *s++;
	for (spanp = delims; (sc = *spanp++) != 0; )
	{
		if (c == sc)
			goto cont;
	}

	if (c == 0)		/* no non-delimiter characters */
	{
		*last = 0;
		return 0;
	}

	tok = s - 1;

	for (;;)
	{
		c = *s++;
		spanp = delims;
		do
		{
			if ((sc = *spanp++) == c)
			{
				if (c == 0)
					s = 0;
				else
				{
					char* w = s - 1;
					*w = '\0';
				}
				*last = s;
				return tok;
			}
		}
		while (sc != 0);
	}
	return 0;
}

char * get_float_token(double* f, char **last)
{
	char *token = GetToken(0, last);
	if (token) *f = small_atof(token);
	return token;
}

char * get_u16_token(uint16_t* Int, char **last)
{
	char *token=GetToken(0, last);
	if (token) *Int=small_atoi(token);
	return token;
}

char * get_int_token(int* Int, char **last)
{
	char *token=GetToken(0, last);
	if (token) *Int=small_atoi(token);
	return token;
}

char * get_byte_token(uint8_t* c, char **last)
{
	char *token=GetToken(0, last);
	if (token) *c = small_atoi(token);
	return token;
}

char * GetToken2(char *s, char **last, const char * dlims)
{
	const char *spanp;
	int c, sc;
	char *tok;

	if (s == 0 && (s = *last) == 0)
		return 0;

cont:
	c = *s++;
	for (spanp = dlims; (sc = *spanp++) != 0; )
	{
		if (c == sc)
			goto cont;
	}

	if (c == 0)		/* no non-delimiter characters */
	{
		*last = 0;
		return 0;
	}

	tok = s - 1;

	while (1)
	{
		c = *s++;
		spanp = dlims;
		do
		{
			if ((sc = *spanp++) == c)
			{
				if (c == 0)
					s = 0;
				else
				{
					char *w = s - 1;
					*w = '\0';
				}
				*last = s;
				return tok;
			}
		}
		while (sc != 0);
	}
	return 0;
}


