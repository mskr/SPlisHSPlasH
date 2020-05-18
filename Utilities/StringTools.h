#ifndef __StringTools_h__
#define __StringTools_h__

#include <vector>
#include <iostream>

namespace Utilities
{
	/** \brief Tools to handle std::string objects 
	*/
	class StringTools
	{
	public:

		static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ")
		{
			std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
			std::string::size_type pos = str.find_first_of(delimiters, lastPos);

			while (std::string::npos != pos || std::string::npos != lastPos)
			{
				tokens.push_back(str.substr(lastPos, pos - lastPos));
				lastPos = str.find_first_not_of(delimiters, pos);
				pos = str.find_first_of(delimiters, lastPos);
			}
		}

		static std::string line(std::istream& in) {
			std::string result;
			std::getline(in, result);
			return result;
		}

		static std::vector<std::string> strings(std::string input = "split on    whitespace   ", char separator = ' ') {
			std::vector<std::string> result;
			std::string next = "";
			for (size_t i = 0; i < input.length(); i++) {
				if (input[i] != separator) {
					next += input[i];
				} else if (!next.empty()) {
					result.push_back(next);
					next = "";
				}
			}
			if (!next.empty()) result.push_back(next);
			return result;
		}

	};
}

#endif
