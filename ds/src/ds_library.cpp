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
#include <boost/lexical_cast.hpp>

const std::string ds_library::Library::lib_name = "../gate_lib/default_lib";
ds_library::LibraryFactory* ds_library::LibraryFactory::instance = 0;

ds_common::NetList* ds_library::import(const std::string& file, const std::string& toplevel){

	std::ifstream input(file.c_str());
	std::string line;
	std::vector<std::string> tokens;

	ds_common::NetList *netlist = 0;

	if (input.is_open()){

		while (!input.eof()){

			std::getline(input ,line);
			tokens.push_back(line);
		}
		typedef std::vector<std::string>::iterator IT;
		netlist = import_verilog<IT>(tokens.begin(), tokens.end(), toplevel);

	} else {
		std::cout << "Unable to open file " << file << std::endl;
	}

	return netlist;
}

template<typename Iterator>
ds_common::NetList* ds_library::import_verilog(Iterator begin, Iterator end, const std::string& toplevel){



	return 0;
}
