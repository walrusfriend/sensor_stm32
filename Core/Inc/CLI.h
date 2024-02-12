/*
 * Class parses UART channel to commands
 * It adds a char to buffer, analyze the buffer string and return the current command state
 * TODO Write a manual "how to put this module to the project"
 * TODO Move all initialization to compile time
*/

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include <string>
#include <vector>

namespace CLI {

	enum CommandRoles {
		SERVICE_COMMAND,
		USER_COMMAND,
		DEBUG_COMMAND
	};
	enum StatusCode {
		PARSED,
		BUSY,
		MAX_SIZE_REACHED,
		UNKNOWN_COMMAND
	};

	class Command {
	public:
		Command(const std::string_view& other_name,
				CommandRoles other_role, void(*func)()) :
			name(other_name), role(other_role), callback(func) {};

		std::string name;
		CommandRoles role;
		void(*callback)();
	};

}


class CommandLineInterpreter {
public:
	CommandLineInterpreter(const uint16_t maxSize);
	CommandLineInterpreter(const std::vector<CLI::Command> &commands, const uint16_t maxSize);
	virtual ~CommandLineInterpreter();

public:
	// Container for an arguments of a command
	std::vector<std::string> args;

public:
	// Add a symbol to the buffer and analyse the string for end and max length
	CLI::StatusCode process(const char& sym);

	void add_commands(std::vector<CLI::Command>& commands);

private:
	// Find string
	CLI::StatusCode parse_command();

private:
	uint16_t max_size;
	std::string command = "";
	std::vector<CLI::Command> command_list;
};

#endif /* INC_CLI_H_ */
