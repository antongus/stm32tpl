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
 *  file          : commands.h
 *  description   : Templates and macros for command interpreter.
 *  created on    : 26.03.2011
 *
 */

#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

#include "util.h"
#include "textstream.h"
#include <string.h>

/**
 * command handler typedef
 */
typedef int (*CommandHandler)(char *, TextStream&);

/**
 * forward declarations
 */
class Interpreter;
class InterpreterCommand;
typedef InterpreterCommand* PInterpreterCommand;
static inline void addInterpreterCommand(PInterpreterCommand cmd);

/**
 * Command class.
 */
class InterpreterCommand
{
protected:
	friend class Interpreter;
	PInterpreterCommand next_;		///< pointer to next command
	const char * name_;			///< command name
	CommandHandler handler_;
	InterpreterCommand(const InterpreterCommand&) {}
public:
	__attribute__((__noinline__))
	InterpreterCommand(const char * aname, CommandHandler ahandler)
		: next_(0)
		, name_(aname)
		, handler_(ahandler)
	{
		addInterpreterCommand(this);
	}
	int handle(char * args, TextStream& stream) { return handler_(args, stream); }
	const char * getName() { return name_; }
	PInterpreterCommand getNext() { return next_; }
};

/**
 * Interpreter class.
 * Holds command list, finds appropriate command
 * and performs command invocation with rest of command line.
 */
class Interpreter
{
private:
	friend void addInterpreterCommand(PInterpreterCommand cmd);

	static void add(PInterpreterCommand cmd)
	{
		PInterpreterCommand& head = getHead();
		cmd->next_ = head;
		head = cmd;
	}
public:
	__attribute__((__noinline__))
	static PInterpreterCommand& getHead()
	{
		static PInterpreterCommand head_ = 0;
		return head_;
	}

	static int parse(char* s, TextStream& stream)
	{
		char *lt = s;
		char *cmd = GetToken(0, &lt);
		if (!cmd) return true;

		for(PInterpreterCommand command = getHead(); command; command = command->next_)
		{
			if (!strcmp(command->name_, cmd))
				return command->handle(lt, stream);
		}
		return false;
	}
};

inline void addInterpreterCommand(InterpreterCommand * cmd)
{
	Interpreter::add(cmd);
}


/**
 * Macros
 */
#define xstr(s) str(s)
#define str(s) #s

#define INTERPRETER_COMMAND(cmd_name) \
int command_##cmd_name##_handler(char * args, TextStream& stream); \
InterpreterCommand command_##cmd_name##_object(xstr(cmd_name), command_##cmd_name##_handler);	\
int command_##cmd_name##_handler(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))) \

#define INTERPRETER_COMMAND_ID(cmd_name, id) \
int command_##id##_handler(char * args, TextStream& stream); \
InterpreterCommand command_##id##_object(xstr(cmd_name), command_##id##_handler);	\
int command_##id##_handler(char * args __attribute__((unused)), TextStream& stream __attribute__((unused))) \

#endif // COMMANDS_H_INCLUDED
