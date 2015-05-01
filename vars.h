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
 *  file          : vars.h
 *  description   : variable and variable list class templates.
 *  created on    : 31.05.2010
 *
 */

#ifndef STM32TPL_VARS_H_INCLUDED
#define STM32TPL_VARS_H_INCLUDED

#include "textstream.h"
#include <cstring>

/**
 * Variable handler type.
 */
using VarHandler = int (*)(char *, TextStream&);

// forward declarations
class VarList;
class Variable;
typedef Variable* PVar;
static inline void AddVariable(PVar cmd);

/**
 * Variable class.
 * Holds two command handlers - GET handler and SET handler.
 * Also holds variable name and pointer to next variable in list.
 */
class Variable
{
public:
	Variable(char const* aname, VarHandler aget, VarHandler aset = nullptr)
		: name_ { aname }
		, get_  { aget }
		, set_  { aset }
	{
		AddVariable(this);
	}
	char const* Name() { return name_; }
	int Set(char* args, TextStream& stream) { return set_ ? set_(args, stream) : false; }
	int Get(char* args, TextStream& stream) { return get_ ? get_(args, stream) : false; }
private:
	friend class VarList;
	Variable* next_ = nullptr;
	char const* name_;
	VarHandler get_;
	VarHandler set_;
	bool Match(char const* aname) { return (!strcmp(name_, aname)); }
};

/**
 * class VarList - list of variables.
 */
class VarList
{
public:
	static PVar Find(char const* aname)
	{
		for(PVar var = First(); var; var = Next(var))
		{
			if (var->Match(aname))
				return var;
		}
		return 0;
	}
	static PVar First() { return Head(); }
	static PVar Next(PVar var) { return var->next_; }
private:
	friend void AddVariable(PVar cmd);

	static PVar& Head()
	{
		static PVar head_ = 0;
		return head_;
	}

	static void Add(PVar var)
	{
		PVar& head = Head();
		var->next_ = head;
		head = var;
	}
};

inline void AddVariable(PVar var)
{
	VarList::Add(var);
}

#define xstr(s) str(s)
#define str(s) #s

#define DEFINE_VARIABLE(varName) \
extern const char variable_##varName##_name[] = xstr(varName); \
int variable_##varName##_get(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))); \
int variable_##varName##_set(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))); \
Variable variable_##varName##_object(	\
	variable_##varName##_name,		\
	variable_##varName##_get,		\
	variable_##varName##_set)		\

#define VARIABLE_GET_HANDLER(varName) \
int variable_##varName##_get(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))) \

#define VARIABLE_SET_HANDLER(varName) \
int variable_##varName##_set(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))) \

#define DEFINE_VARIABLE_RO(varName) \
extern const char variable_##varName##_name[] = xstr(varName); \
int variable_##varName##_get(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))); \
Variable variable_##varName##_object(	\
	variable_##varName##_name,		\
	variable_##varName##_get)		\

/**
 * read-only variable declaration and get handler (combined)
 */
#define RO_VARIABLE_GET_HANDLER(varName) \
	DEFINE_VARIABLE_RO(varName); \
	VARIABLE_GET_HANDLER(varName)

#endif // STM32TPL_VARS_H_INCLUDED
