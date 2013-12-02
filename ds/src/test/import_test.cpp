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

void import_test::import_netlist(const std::string& name) {

	ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	ds_structural::NetList* nl = 0;
	try {

		std::string file = path + "/files/" + name;
		BOOST_LOG_TRIVIAL(info) << "Importing " << file;
		nl = ds_workspace::load_netlist(file, "top");
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
