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

ds_library::LibraryFactory* ds_library::LibraryFactory::instance = 0;

ds_structural::NetList* ds_library::import(const std::string& file, const std::string& toplevel){

	std::ifstream input(file.c_str());
	std::string line;
	std::vector<std::string> tokens;

	ds_structural::NetList *netlist = 0;

	if (input.is_open()){

		while (!input.eof()){

			std::getline(input ,line);
			tokens.push_back(line);
		}
		typedef std::vector<std::string>::iterator IT;
		netlist = import_verilog<IT>(tokens.begin(), tokens.end(), toplevel);

	} else {
		 throw file_read_error() << boost::errinfo_errno(errno);
	}

	return netlist;
}

void ds_library::Library::load_nodes(){

	using namespace ds_lg;

	std::list<LGNode*> list;
	LGNode *node = new LGNode1I("not1", f_not);
	list.push_back(node);
	node = new LGNode1I("buf1", f_buf);
	list.push_back(node);

	node = new LGNode2I("and2", f_and2);
	list.push_back(node);
	node = new LGNode2I("or2", f_or2);
	list.push_back(node);
	node = new LGNode2I("nand2", f_nand2);
	list.push_back(node);
	node = new LGNode2I("nor2", f_nor2);
	list.push_back(node);
	node = new LGNode2I("xor2", f_xor2);
	list.push_back(node);
	node = new LGNode2I("xnor2", f_xnor2);

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
void ds_library::Library::load(const std::string &lib_name){

	std::ifstream input(lib_name.c_str());
	std::string line;

	if (input.is_open()){

		int line_num = 0;
		while (!input.eof()){

			std::getline(input ,line);
			bool p = parse_library<std::string::iterator>(line.begin(), line.end(), &gate_map, prototypes);
			if (!p){
				BOOST_THROW_EXCEPTION(ds_library::parse_error()
				<< ds_common::errmsg_info("Error parsing library file: " + lib_name + ":" + line));
			}
			line_num++;
		}


	} else {
		 throw file_read_error() << boost::errinfo_errno(errno);
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

