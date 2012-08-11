/*
 * ds_library.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: cookao
 */

#include "ds_library.h"
#include "ds_common.h"
#include "iostream"
#include "fstream"
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

 ds_common::NetList* import(std::string file, std::string toplevel){

	static const boost::regex module_regex = "module\s+(\w+)\s+\(\w+\s*,?\s*)*\);";

	ds_common::NetList* netlist = 0;

	std::ifstream input(file.c_str());

	std::string line;
	std::string buffer;
	std::string delims = ";";
	if (input.is_open()){

		netlist = new ds_common::NetList();

		std::vector<std::string> tokens;
		std::vector<std::string> netlist_tokens;
		while (!input.eof()){
			get_next(input, tokens);
			std::vector<std::string>::iterator it = tokens.begin();
			for (;it != tokens.end();it++){

				std::string s = *it;
				boost::smatch what;
				if (boost::regex_match(s, what, module_regex)) {

				}

			}

			tokens.clear();
		}

	} else {
		std::cout << "Unable to open file " << file << std::endl;
	}


	return (netlist);
}

void parse_value(std::string, ds_common::NetList* netlist){

	 static const boost::regex port = "";
	 static const boost::regex instance = "";
}

void get_next(const std::ifstream& s, std::vector<std::string>& to_parse){

	static const std::string delims =";";
	std::string line;
	std::string buffer;

	do {

		if (s.eof())
			break;
		std::getline(s, line);
		buffer.append(line);


	} while (buffer.find(';')!=-1);

	boost::algorithm::split(to_parse, buffer, boost::algorithm::is_any_of(delims));

}

