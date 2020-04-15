#include <iostream>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char const* argv[])
{
	auto h = argv;
	auto c = argc;

	std::cout << h << c << std::endl;

	int i = 10;
	while (i)
	{
		std::cout << i-- << std::endl;
	}
	return 0;
}
