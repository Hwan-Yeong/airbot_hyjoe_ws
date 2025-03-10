#include "application/process.hpp"

ProcessService::ProcessService() = default;
ProcessService::~ProcessService() = default;

std::string
ProcessService::execute_command(std::string command) const
{
	const char *_command = command.c_str();

	try
	{
		const int &command_result = system(_command);

		if (command_result == 0)
		{
			FILE *pipe = popen(_command, "r");
			if (!pipe)
			{
				printf("Failed to run command: %s", command);
				return "";
			}

			char buffer[128];
			std::string result = "";

			while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
			{
				result += buffer;
			}

			const int &exit_status = pclose(pipe);

			return result;
		}
		else
		{
			printf("Failed to run command: %s", command);
			return "";
		}
	}
	catch (const std::exception &e)
	{
		printf("Failed to run command: %s", command);
		return "";
	}
}

std::vector<std::string>
ProcessService::split(const std::string& str, char delimiter) const
{
	std::vector<std::string> tokens;
	std::stringstream ss(str);
	std::string token;
	while (std::getline(ss, token, delimiter))
	{
		tokens.push_back(token);
	}
	return tokens;
}
