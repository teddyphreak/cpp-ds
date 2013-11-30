/*
 * serialization_test.cpp
 *
 *  Created on: 30.11.2013
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/log/trivial.hpp>
#include <fstream>
#include "serialization_test.h"
#include "ds_pattern.h"


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
		boost::archive::binary_oarchive oa(ofs);
		oa << p;
	}

	BOOST_LOG_TRIVIAL(info) << "Pattern provider dumped... " << file;

	ds_pattern::CombinationalPatternProvider* r = 0;
	{

		std::ifstream ifs(p_name);
		boost::archive::binary_iarchive ia(ifs);
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



