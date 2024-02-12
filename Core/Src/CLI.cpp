#include <CLI.h>

// Constructor of the class
// @argument	size - max length of a command
CommandLineInterpreter::CommandLineInterpreter(const uint16_t size = 64) : max_size(size) {
	// Make a command list
	// Reserve first and second positions to BUSY and NOT_A_COMMAND states
	command_list.emplace_back("BUSY", CLI::CommandRoles::SERVICE_COMMAND, [](){});
}

// Constructor of the class
// @argument 	commands - list of the commands to compare with
// @argument	size - max length of a command
CommandLineInterpreter::CommandLineInterpreter(const std::vector<CLI::Command> &commands,
											   const uint16_t size = 64) :
	max_size(size)
{
	// Make a command list
	// Reserve the first position to BUSY state
	command_list.emplace_back("BUSY", CLI::CommandRoles::SERVICE_COMMAND, [](){});

	// Add user's commands to the list
	if(commands.size() > 0)
		command_list.insert(command_list.end(), commands.begin(), commands.end());
}

CommandLineInterpreter::~CommandLineInterpreter() {
}

// Add a symbol to the buffer and analyse the string for end and max length
// @sym 	- sym that taken from UART
// @retval	- status
CLI::StatusCode CommandLineInterpreter::process(const char& sym) {
	// When we find a terminate symbol that stop the receiving and start the processing of a command
	if (sym == '\n') {
		return parse_command();
	}

	// Return NOT A COMMAND when current size bigger that max size
	if (command.size() > max_size) {
		command.clear();
		return CLI::StatusCode::MAX_SIZE_REACHED;
	}

	// Usually \r meets with \n  in \r\n combination
	// Check \r before \n and skip it
	if (sym != '\r')
		command.push_back(sym); // Add sym to buffer

	// Return a busy state cause we're reading now
	return CLI::StatusCode::BUSY;
}

// Add commands to the command list
void CommandLineInterpreter::add_commands(std::vector<CLI::Command>& commands) {
	for (auto user_command : commands) {
		// Check for duplications
		bool isExist = false;

		for (auto existing_command : command_list) {
			if (user_command.name == existing_command.name) {
				isExist = true;
				break;
			}
		}

		if (!isExist)
			command_list.push_back(user_command);
	}
}

// Tries to recognise a command in the buffer
// @retval	- status
CLI::StatusCode CommandLineInterpreter::parse_command() {
	// Clear arguments buffer
	args.clear();

	// Parse string to the first space to split off a command and keys
	// (Most of commands looks like "command key1 key2 ..."
	size_t pos = 0;
	while (pos < command.size()) {
		if (command[pos] == ' ')
			break;
		++pos;
	}
	std::string str;
	str.append(command, 0, pos++);

	// Save the argument of the command
	std::string tmp;
	for (size_t i = pos; i < command.size(); ++i) {
		if (command[i] == ' ') {
			args.push_back(tmp);
			tmp.clear();
		}
		else
			tmp += command[i];
	}
	args.push_back(tmp);
	tmp.clear();

	// Try to find the command in the command list
	// Skip first two commands in a list because they are reserved to BUSY and NOT A COMMAND states
	for (size_t i = 2; i < command_list.size(); ++i) {
		if (str == command_list[i].name) {
			command_list[i].callback();
			str.clear();
			command.clear();
			return CLI::StatusCode::PARSED;
		}
	}

	command.clear();
	return CLI::StatusCode::UNKNOWN_COMMAND;
}
