/*
 * ds_library.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: cookao
 */

#include "ds_library.h"
#include "ds_common.h"
#include <iostream>
#include <fstream>
#include <errno.h>
#include <boost/exception/all.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/lambda/algorithm.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>

ds_library::LibraryFactory* ds_library::LibraryFactory::instance = 0;
void ds_library::Library::load_nodes(){

	using namespace ds_lg;

	std::list<LGNode*> list;
	LGNode *node = new LGNode1I("not", f_not);
	list.push_back(node);
	node = new LGNode1I("not1", f_not);
	list.push_back(node);
	node = new LGNode1I("buf", f_buf);
	list.push_back(node);
	node = new LGNode1I("buf1", f_buf);
	list.push_back(node);

	node = new LGNode2I("and", f_and2);
	list.push_back(node);
	node = new LGNode2I("and2", f_and2);
	list.push_back(node);
	node = new LGNode2I("or", f_or2);
	list.push_back(node);
	node = new LGNode2I("or2", f_or2);
	list.push_back(node);
	node = new LGNode2I("nand", f_nand2);
	list.push_back(node);
	node = new LGNode2I("nand2", f_nand2);
	list.push_back(node);
	node = new LGNode2I("nor", f_nor2);
	list.push_back(node);
	node = new LGNode2I("nor2", f_nor2);
	list.push_back(node);
	node = new LGNode2I("xor", f_xor2);
	list.push_back(node);
	node = new LGNode2I("xor2", f_xor2);
	list.push_back(node);
	node = new LGNode2I("xnor", f_xnor2);
	list.push_back(node);
	node = new LGNode2I("xnor2", f_xnor2);
	list.push_back(node);

	node = new LGNode3I("and3", f_and3);
	list.push_back(node);
	node = new LGNode3I("or3",f_or3);
	list.push_back(node);
	node = new LGNode3I("nand3",f_nand3);
	list.push_back(node);
	node = new LGNode3I("nor3",f_nor3);
	list.push_back(node);
	node = new LGNode3I("xor3",f_xor3);
	list.push_back(node);
	node = new LGNode3I("xnor3",f_xnor3);
	list.push_back(node);

	node = new LGNode4I("and4",f_and4);
	list.push_back(node);
	node = new LGNode4I("or4",f_or4);
	list.push_back(node);
	node = new LGNode4I("nand4",f_nand4);
	list.push_back(node);
	node = new LGNode4I("nor4",f_nor4);
	list.push_back(node);
	node = new LGNode4I("xor4",f_xor4);
	list.push_back(node);
	node = new LGNode4I("xnor4",f_xnor4);
	list.push_back(node);

	node = new LGNode5I("and5",f_and5);
	list.push_back(node);
	node = new LGNode5I("or5",f_or5);
	list.push_back(node);
	node = new LGNode5I("nand5",f_nand5);
	list.push_back(node);
	node = new LGNode5I("nor5",f_nor5);
	list.push_back(node);
	node = new LGNode5I("xor5",f_xor5);
	list.push_back(node);
	node = new LGNode5I("xnor5",f_xnor5);
	list.push_back(node);

	node = new LGNode6I("and6",f_and6);
	list.push_back(node);
	node = new LGNode6I("or6",f_or6);
	list.push_back(node);
	node = new LGNode6I("nand6",f_nand6);
	list.push_back(node);
	node = new LGNode6I("nor6",f_nor6);
	list.push_back(node);
	node = new LGNode6I("xor6",f_xor6);
	list.push_back(node);
	node = new LGNode6I("xnor6",f_xnor6);
	list.push_back(node);

	node = new LGNode7I("and7",f_and7);
	list.push_back(node);
	node = new LGNode7I("or7",f_or7);
	list.push_back(node);
	node = new LGNode7I("nand7",f_nand7);
	list.push_back(node);
	node = new LGNode7I("nor7",f_nor7);
	list.push_back(node);
	node = new LGNode7I("xor7",f_xor7);
	list.push_back(node);
	node = new LGNode7I("xnor7",f_xnor7);
	list.push_back(node);

	node = new LGNode8I("and8",f_and8);
	list.push_back(node);
	node = new LGNode8I("or8",f_or8);
	list.push_back(node);
	node = new LGNode8I("nand8",f_nand8);
	list.push_back(node);
	node = new LGNode8I("nor8",f_nor8);
	list.push_back(node);
	node = new LGNode8I("xor8",f_xor8);
	list.push_back(node);
	node = new LGNode8I("xnor8",f_xnor8);
	list.push_back(node);

	typedef std::list<ds_lg::LGNode*>::iterator IT;
	for (IT it=list.begin();it!=list.end();it++){
		LGNode *n = *it;
		prototypes[n->getType()] = n;
	}

}

ds_library::Library* ds_library::LibraryFactory::loadLibrary(const std::string& name){
	Library *lib = 0;
	library_map_t::const_iterator it = map.find(name);
	if (it != map.end())
		lib = it->second;
	else {
		const char* d = getenv("DS");
		std::string path = d?d:"";
		path += "/gate_lib/" + name;
		try {
			lib = new Library(path);
			map[name] = lib;
		}
		catch (ds_library::parse_error &ex){
			std::string const * msg  = boost::get_error_info<ds_common::errmsg_info>(ex);
			std::cout << "Error:" << *msg << std::endl;
		}
	}
	return lib;
}

void ds_library::Library::load(const std::string &lib_name){

	std::ifstream input(lib_name.c_str());
	std::string line;

	if (input.is_open()){

		int line_num = 0;
		while (!input.eof()){

			std::getline(input ,line);
			if (!line.empty()) {
				bool p = parse_library<std::string::iterator>(line.begin(), line.end(), gate_map, prototypes, functions);
				if (!p){

					boost::algorithm::trim(line);
					if (line[0]!='#') {
						std::cout << "Error parsing library file: " << lib_name << ":" << line << std::endl;
						BOOST_THROW_EXCEPTION(ds_library::parse_error()
						<< ds_common::errmsg_info("Error parsing library file: " + lib_name + ":" + line));
					}
				}
			}
			line_num++;
		}
	} else {
		std::cout << "error loading library " << std::endl;
		BOOST_THROW_EXCEPTION(file_read_error() << boost::errinfo_errno(errno));
	}
}

void ds_library::Library::close(){
	typedef std::map<std::string, ds_lg::LGNode*>::const_iterator IT;
	for (IT it = prototypes.begin(); it!=prototypes.end();it++){
		std::string name = it->first;
		ds_lg::LGNode* n = it->second;
		delete(n);
	}
	prototypes.clear();
}

ds_structural::NetList* ds_library::import(const std::string& file, const std::string& toplevel, const Library* lib){

	if (file.find_last_of(".v") != file.size() - 2)
		BOOST_THROW_EXCEPTION(parse_error()
		<< ds_common::errmsg_info("Only verilog design supported at the moment"));

	std::ifstream input(file.c_str());
	std::string line;
	std::stringstream ss;

	ds_structural::NetList *netlist = 0;

	if (input.is_open()){

		while (!input.eof()){
			std::getline(input ,line);
			ss << line << ' ';
		}

		input.close();

		typedef std::string::iterator STR_IT;
		std::vector<ds_library::parse_netlist> netlists;
		ds_library::nxp_verilog_parser<STR_IT> p;

		std::string nl_str = ss.str();
		using boost::spirit::ascii::space;
		bool parse =  boost::spirit::qi::phrase_parse(nl_str.begin(), nl_str.end(), p, space, netlists);
		if (!parse) {
			BOOST_THROW_EXCEPTION(parse_error()
					<< ds_common::errmsg_info("Error parsing verilog file"));
		}
		typedef std::vector<ds_library::parse_netlist>::iterator IT;
		IT top = std::find_if(netlists.begin(), netlists.end(), bind(&parse_netlist::nl_name, _1) == toplevel);
		if (top == netlists.end()){
			BOOST_THROW_EXCEPTION(parse_error()
					<< ds_common::errmsg_info("Design " + toplevel + " not found"));
		}



	} else {
		BOOST_THROW_EXCEPTION(file_read_error() << boost::errinfo_errno(errno));
	}
	return netlist;
}

ds_structural::NetList* convert(ds_library::parse_netlist nl, const ds_library::Library *lib){
	ds_structural::NetList *netlist = new ds_structural::NetList();
	netlist->set_instance_name(nl.nl_name);
	std::vector<std::string> temp_names;
	std::vector<std::string> names;
	ds_library::aggregate_visitor<std::vector<std::string> > visitor(temp_names);
	BOOST_FOREACH( ds_library::verilog_declaration p, nl.inputs )
	{
		temp_names.clear();
		boost::apply_visitor(visitor, p);
		BOOST_FOREACH( std::string n, temp_names )
		{
			ds_structural::PortBit *pb = new ds_structural::PortBit(n,ds_structural::DIR_IN);
			pb->setGate(netlist);
			netlist->add_port(pb);
			ds_structural::Signal *s = new ds_structural::Signal(n);
			netlist->add_signal(s);
		}
	}
	BOOST_FOREACH( ds_library::verilog_declaration p, nl.outputs )
	{
		temp_names.clear();
		boost::apply_visitor(visitor, p);
		BOOST_FOREACH( std::string n, temp_names )
		{
			ds_structural::PortBit *pb = new ds_structural::PortBit(n,ds_structural::DIR_OUT);
			pb->setGate(netlist);
			netlist->add_port(pb);
			ds_structural::Signal *s = new ds_structural::Signal(n);
			netlist->add_signal(s);
		}
	}
	BOOST_FOREACH( ds_library::verilog_declaration p, nl.signals )
	{
		temp_names.clear();
		boost::apply_visitor(visitor, p);
		BOOST_FOREACH( std::string n, temp_names )
		{
			ds_structural::Signal *s = new ds_structural::Signal(n);
			netlist->add_signal(s);
		}
	}
	BOOST_FOREACH( ds_library::parse_nl_implicit_instance implicit, nl.instances )
	{
		std::string type = implicit.type;
		std::size_t numPorts = implicit.ports.size();
		ds_structural::Gate *g = lib->getGate(type,numPorts);
		g->set_instance_name(implicit.name);
		netlist->add_gate(g);
		typedef std::vector<std::string>::iterator PORT_IT;
		std::vector<ds_structural::PortBit*> all_ports;
		std::for_each(g->get_inputs()->begin(),g->get_inputs()->end(), boost::bind(&std::vector<ds_structural::PortBit*>::push_back, all_ports, _1));
		std::for_each(g->get_outputs()->begin(),g->get_outputs()->end(), boost::bind(&std::vector<ds_structural::PortBit*>::push_back, all_ports, _1));
		std::vector<ds_structural::PortBit*>::iterator pi = all_ports.begin();
		for (PORT_IT it = implicit.ports.begin();it!=implicit.ports.end();it++){
			ds_structural::PortBit* pb = *pi;
			pi++;
			std::string signal_name = *it;
			ds_structural::Signal *signal = netlist->find_signal(signal_name);
			if (signal==0){
				signal = new ds_structural::Signal(signal_name);
				netlist->add_signal(signal);
			}
			signal->add_receiver(pb);
		}
	}
	return netlist;
}

template<typename Iterator>
ds_structural::NetList* ds_library::import_verilog(Iterator begin, Iterator end, const std::string& toplevel, const ds_library::Library* lib){
	return 0;
}
