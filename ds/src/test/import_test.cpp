/*
 * import_test.cpp
 *
 *  Created on: 22.04.2013
 *      Author: cookao
 */

#include <boost/test/unit_test.hpp>
#include "import_test.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"

using namespace ds_library;
using namespace ds_structural;
using namespace ds_workspace;

void import_test::import_netlist(const std::string& name) {

	LibraryFactory *factory = LibraryFactory::getInstance();
	Library *defaultLib = factory->load_library();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	Workspace *wp = ds_workspace::Workspace::get_workspace();
	wp->add_library(defaultLib);
	ds_structural::NetList* nl = 0;
	try {

		std::string file = path + "/files/" + name;
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		nl->remove_floating_signals();
		bool r = nl->remove_unused_gates();
		while (r)
			r = nl->remove_unused_gates();
		BOOST_CHECK(nl->check_netlist());
		ds_lg::LeveledGraph* lg = nl->build_leveled_graph();
		BOOST_CHECK(lg->sanity_check());
		delete nl;

	} catch (boost::exception& e){

		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
	BOOST_ASSERT(nl!=0);
}

void import_test::parse_verilog(const std::string& name) {

	const char* d = getenv("DS");
	std::string path = d?d:"";
	std::vector<ds_library::parse_netlist> netlists;
	bool parse = false;
	std::string file;

	file = path + "/files/" + name;
	std::cout << "... parsing " << file << std::endl;
	parse = parse_verilog(file, netlists);
	BOOST_ASSERT(parse);
	BOOST_ASSERT(netlists.size() >=1);
	netlists.clear();
}
