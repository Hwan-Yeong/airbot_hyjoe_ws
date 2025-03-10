#ifndef PROCESS_HPP
#define PROCESS_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class ProcessService final
{
public:
	explicit ProcessService();
	virtual ~ProcessService();
	std::string execute_command(std::string command) const;
	std::vector<std::string> split(const std::string &str, char delimiter) const;
public:
	using SharedPtr = std::shared_ptr<ProcessService>;
};

#endif // PROCESS_HPP
