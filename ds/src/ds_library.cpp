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

ds_library::LibraryFactory* ds_library::LibraryFactory::instance = 0;

ds_common::NetList* ds_library::import(const std::string& file, const std::string& toplevel){

	boost::regex module_regex("\\s*module\\s+(\\w+)\\s+\\(((\\s*\\w+\\s*,?\\s*)+)\\)");

	ds_common::NetList* netlist = 0;

	std::ifstream input(file.c_str());

	std::string line;
	std::string buffer;
	std::string delims = ";";
	if (input.is_open()){

		std::vector<std::string> tokens;
		std::vector<std::string> netlist_tokens;

		while (!input.eof()){

			if (netlist == 0) {

				ds_library::get_next(input, tokens);
				std::vector<std::string>::iterator it = tokens.begin();

				for (;it != tokens.end();it++){

					std::string statement = *it;

					boost::smatch what;
					if (boost::regex_match(statement, what, module_regex)) {
						if (what[1] == toplevel) {
							std::cout << "Toplevel: " << toplevel << " found..." << std::endl;
							netlist = new ds_common::NetList();
							netlist->set_name(toplevel);
							std::vector<std::string>::iterator r = it;
							for (;r != tokens.end();r++){
								std::string left_over = *r;
								netlist_tokens.push_back(left_over);
							}
						}
			        }
			    }
				tokens.clear();
			} else {

				ds_library::get_next(input, netlist_tokens);
				std::vector<std::string>::iterator it = netlist_tokens.begin();

				for (;it != netlist_tokens.end();it++){

					std::string statement = *it;
					bool finish = ds_library::parse_value(statement, netlist);
					if (finish)
						return netlist;
				}
			}

		}

	} else {
		std::cout << "Unable to open file " << file << std::endl;
	}

	return (netlist);
}

bool ds_library::parse_value(const std::string& value, ds_common::NetList* netlist){

	//static const boost::regex regex_port("\\s*(input|output|wire)\\s*((\\w+)(\\[(\\d+\\:\\d+)\\])?\\s?,?\\s?)+");
	static const boost::regex regex_port("\\s*(input|output|wire)\\s*(.*)");
	boost::smatch what;
	if (boost::regex_match(value, what, regex_port)) {
		std::cout << what[1] << " " << what[2] <<	std::endl;
		std::string definition = what[1];
		std::string instances = what[2];

		static const std::string delims =",";
		std::vector<std::string> to_parse;
		boost::algorithm::split(to_parse, instances, boost::algorithm::is_any_of(delims));
		std::vector<std::string>::iterator it = to_parse.begin();
		for (;it != to_parse.end();it++){
			std::string s = *it;
			boost::algorithm::trim(s);
			std::size_t marker = s.find(':');
			if (marker == std::string::npos){
				ds_library::add_ports(s,definition,netlist);
			} else {
				int l_pos = s.find('[');
				int r_pos = s.find(']');
				std::string left = s.substr(l_pos+1,marker);
				std::string right = s.substr(marker+1,r_pos);
				int l = std::atoi(left.c_str());
				int r = std::atoi(right.c_str());
				int from = r;
				int to = l;
				if (r > l){
					from = l;
					to = r;
				}

				for (int i=from;i<=to;i++){
					std::string p = boost::lexical_cast<std::string>(i);
					std::string name = s.substr(0, s.find('[')) + "[" + p + "]";
					ds_library::add_ports(name,definition,netlist);
				}
			}
		}
	 }
	 return false;
}

void ds_library::get_next(std::ifstream& s, std::vector<std::string>& to_parse){

	static const std::string delims =";";
	static const std::string end ="endmodule";
	std::string line;
	std::string buffer;

	do {

		if (s.eof())
			break;
		std::getline(s ,line);
		buffer.append(line);
		if (line.find(end) != std::string::npos){
			boost::algorithm::trim(line);
			if (line == end)
				break;
		}
	} while (buffer.find(delims) == std::string::npos);

	boost::algorithm::split(to_parse, buffer, boost::algorithm::is_any_of(delims));

}

void ds_library::add_ports(const std::string& value, const std::string& type, ds_common::NetList* netlist){
	static const std::string in = "input";
	static const std::string out = "output";
	static const std::string wire = "wire";
	std::cout << "adding " << value << " " << type << std::endl;
	if (type == wire){
		ds_common::Signal* signal = new ds_common::Signal(value);
		netlist->add_signal(signal);
	} else if(type == in){
		ds_common::PortType t = ds_common::IN;
		ds_common::PortBit *pb = new ds_common::PortBit(value, netlist, t);
		netlist->add_port(pb);
	} else if (type==out){
		ds_common::PortType t = ds_common::OUT;
		ds_common::PortBit *pb = new ds_common::PortBit(value, netlist, t);
		netlist->add_port(pb);
	} else {
		std::cout << "Undefined import type: " << type << std::endl;
	}
}

