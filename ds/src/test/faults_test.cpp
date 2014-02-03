/*
 * faults_test.cpp
 *
 *  Created on: 08.11.2013
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include "faults_test.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"
#include "ds_faults.h"


void faults_test::get_collapsed_faults(const std::string& name) {


	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	ds_library::Library* lib = ds_library::load_default_lib();
	ds_structural::NetList* nl = 0;
	try {

		std::string file = path + "/files/" + name;
		nl = ds_workspace::load_netlist("top",file);
		BOOST_CHECK(nl->check_netlist());

		ds_lg::LeveledGraph* lg = nl->get_sim_graph(lib);
		BOOST_CHECK(lg->sanity_check());

		std::map<ds_faults::SAFaultDescriptor*, std::list<ds_faults::SAFaultDescriptor*>* > *fault_classes = 0;
		fault_classes = ds_faults::get_fault_universe(nl);
		BOOST_LOG_TRIVIAL(info) << name << ": # of collapsed stuck-at faults " << fault_classes->size();
		ds_faults::delete_faults(fault_classes);
		delete nl;

	} catch (boost::exception& e){
		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
	BOOST_ASSERT(nl!=0);
}



