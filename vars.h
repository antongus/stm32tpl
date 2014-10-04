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
#include <string.h>

/**
 * variable handlers typedef
 */
typedef int (*VarHandler)(char *, TextStream&);

/**
 * forward declarations
 */
class VarList;
class Variable;
typedef Variable* PVar;
static inline void addVariable(PVar cmd);

/**
 * Variable class.
 */
class Variable
{
private:
	friend class VarList;
	Variable* next_;
	char const* name_;
	VarHandler get_;
	VarHandler set_;
	Variable(const Variable&) {}
protected:
	bool match(char const* aname) { return (!strcmp(name_, aname)); }
public:
	__attribute__((__noinline__))
	Variable(
		char const* aname,
		VarHandler aget,
		VarHandler aset = 0
		)
		: next_(0)
		, name_(aname)
		, get_(aget)
		, set_(aset)
	{
		addVariable(this);
	}
	char const* getName() { return name_; }
	virtual int set(char* args, TextStream& stream) { return set_ ? set_(args, stream) : false; }
	virtual int get(char* args, TextStream& stream) { return get_ ? get_(args, stream) : false; }
};

/**
 * class VarList - list of variables.
 */
class VarList
{
private:
	friend void addVariable(PVar cmd);

	static PVar& getHead()
	{
		static PVar head_ = 0;
		return head_;
	}

	static void add(PVar var)
	{
		PVar& head = getHead();
		var->next_ = head;
		head = var;
	}
public:
	__attribute__((__noinline__))
	static PVar find(char const* aname)
	{
		for(PVar var = first(); var; var = next(var))
		{
			if (var->match(aname))
				return var;
		}
		return 0;
	}
	static PVar first() { return getHead(); }
	static PVar next(PVar var) { return var->next_; }
};

inline void addVariable(PVar var)
{
	VarList::add(var);
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
