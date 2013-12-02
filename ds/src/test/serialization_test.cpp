/*
 * serialization_test.cpp
 *
 *  Created on: 30.11.2013
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>
#include "serialization_test.h"
#include "ds_pattern.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"

using namespace ds_library;
using namespace ds_structural;
using namespace ds_workspace;


void serialization_test::serialize_paterns(const std::string& name) {
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	std::string file = path + "/files/" + name;
	BOOST_LOG_TRIVIAL(info) << "... serializing " << file;
	ds_pattern::CombinationalPatternProvider* p = ds_pattern::load_pattern_blocks(file, false);

	std::string p_name = path + "/files/" + name + ".ds";

	{
		std::ofstream ofs(p_name);
		boost::archive::text_oarchive oa(ofs);
		oa << p;
	}

	BOOST_LOG_TRIVIAL(info) << "Pattern provider dumped... " << file;

	ds_pattern::CombinationalPatternProvider* r = 0;
	{

		std::ifstream ifs(p_name);
		boost::archive::text_iarchive ia(ifs);
		ia >> r;
	}

	BOOST_LOG_TRIVIAL(info) << "Pattern provider read back... " << file;

	BOOST_CHECK( p->num_ports() == r->num_ports() );
	BOOST_CHECK(  p->get_output_offset() == r->get_output_offset() );
	BOOST_CHECK(  p->get_num_inputs() ==  r->get_num_inputs() );
	BOOST_CHECK( p->get_num_outputs() == r->get_num_outputs() );
	BOOST_CHECK( p->num_blocks() == r->num_blocks() );

	while (p->has_next()){

		BOOST_CHECK( r->has_next() );

		ds_pattern::SimPatternBlock *p_b = p->next();
		ds_pattern::SimPatternBlock *r_b = r->next();

		for (std::size_t i=0;i<p->num_ports();i++){
			BOOST_ASSERT( p_b->values[i].v == r_b->values[i].v );
			BOOST_ASSERT( p_b->values[i].x == r_b->values[i].x );
		}

	}
}

void serialization_test::serialize_netlist(const std::string& name) {
	ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	std::string file = path + "/files/" + name;
	BOOST_LOG_TRIVIAL(info) << "... reading " << file;
	ds_structural::NetList* n = ds_workspace::load_netlist(file, "top");
	BOOST_CHECK(n->check_netlist());
	//ds_lg::LeveledGraph* lg = n->build_leveled_graph();
	//BOOST_CHECK(lg->sanity_check());

	BOOST_LOG_TRIVIAL(info) << " Design :" << file << " imported";
	std::string d_name = path + "/files/" + name + ".ds";
	ds_structural::save_netlist(d_name, n);


	BOOST_LOG_TRIVIAL(info) << "Netlist dumped... " << file;

	ds_structural::NetList* r = ds_structural::load_netlist(d_name);

	BOOST_LOG_TRIVIAL(info) << "Netlist read back... " << file;

	//BOOST_CHECK(r->check_netlist());
	//ds_lg::LeveledGraph* r_lg = n->build_leveled_graph();
	//BOOST_CHECK(r_lg->sanity_check());
}

