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
#include "ds_timing.h"
#include <iostream>
#include <fstream>

void import_test::import_netlist(const std::string& name) {

	ds_library::Library *lib = ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	ds_structural::NetList* nl = 0;
	try {

		std::string file = path + "/files/" + name;
		BOOST_LOG_TRIVIAL(info) << "Importing " << file;
		nl = ds_workspace::load_netlist("top", file);
		BOOST_ASSERT(nl!=0);
		BOOST_CHECK(nl->check_netlist());
		ds_lg::LeveledGraph* lg = nl->get_sim_graph(lib);
		BOOST_CHECK(lg->sanity_check());
		delete nl;

	} catch (boost::exception& e){

		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
}

void import_test::import_sdf(const std::string& name) {

	ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	ds_library::Library *lib = ds_library::load_default_lib();
	try {

		std::string sdf_file = path + "/files/" + name + ".sdf";
		BOOST_LOG_TRIVIAL(info) << "Importing " << sdf_file;
		ds_timing::sdf_data data;
		bool p = ds_timing::parse_sdf(sdf_file, data);

		std::string design_file = path + "/files/" + name + ".v";
		ds_structural::NetList *nl = ds_workspace::load_netlist("top", design_file);
		ds_timing::TLeveledGraph* lg = nl->get_ts_graph(lib);
		ds_timing::annotate(data, lg);

		BOOST_CHECK(p);

	} catch (boost::exception& e){

		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
}

void import_test::parse_verilog(const std::string& name) {
	ds_library::load_default_lib();
	const char* d = getenv("DS");
	std::string path = d?d:"";
	std::vector<ds_library::parse_netlist> netlists;
	bool parse = false;
	std::string file;

	file = path + "/files/" + name;
	parse = parse_verilog(file, netlists);
	BOOST_ASSERT(parse);
	BOOST_ASSERT(netlists.size()>=1);
	netlists.clear();
}

void import_test::dump_verilog(const std::string& name){
	ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	ds_structural::NetList* nl = 0;
	try {
		std::string file = path + "/files/" + name;
		BOOST_LOG_TRIVIAL(info) << "Reading " << file;
		nl = ds_workspace::load_netlist("top", file);
		std::string dump_file = path + "/files/" + "dump_" + name;
		BOOST_LOG_TRIVIAL(info) << "Dumping " << file;
		std::ofstream verilog(dump_file);
		nl->dump_verilog_implicit(verilog);
		delete nl;
	} catch (boost::exception& e){
		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
}

void import_test::import_scan_map(const std::string& name){
	ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	std::string file = path + "/files/" + name;
	try {
		BOOST_LOG_TRIVIAL(info) << "Getting scan chain map for " << name;
		ds_structural::CombinationalScanMap *sm = ds_structural::get_combinational_scan_map(file);
		int output_num = sm->get_output_chains();
		int input_num = sm->get_input_chains();
		BOOST_LOG_TRIVIAL(info) << "# Input chains: " << input_num << ". # Output chains: " << output_num;
		for (int i=0;i<output_num;i++){
			BOOST_LOG_TRIVIAL(trace) << "# Length of output chain " << i << ": " << sm->get_output_chain_length(i);
			BOOST_LOG_TRIVIAL(trace) << "First element of output chain " << i << ":" << sm->get_output_signal(i,0);
		}
		for (int i=0;i<input_num;i++){
			BOOST_LOG_TRIVIAL(info) << "# Length of input chain " << i << ": " << sm->get_input_chain_length(i);
			BOOST_LOG_TRIVIAL(info) << "First element of input chain " << i << ":" << sm->get_input_signal(i,0);
		}
	} catch (boost::exception& e){
		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
}

