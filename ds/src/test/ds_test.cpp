/*
  * ds_test.cpp
 *
 *  Created on: 22.04.2013
 *      Author: cookao
 */

#include <boost/test/unit_test.hpp>
#include "import_test.h"
#include "lib_test.h"
#include "pattern_test.h"
#include "sim_test.h"
#include "faults_test.h"
#include "serialization_test.h"
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

boost::unit_test::test_suite* init_unit_test_suite( int argc, char* argv[] ) {

	boost::log::core::get()->set_filter(
		boost::log::trivial::severity >= boost::log::trivial::trace
	);

	boost::unit_test::test_suite* test = BOOST_TEST_SUITE( "DSTest" );

	serialization_test::serialize_ts *serial = new serialization_test::serialize_ts("serialize_ts");
	test->add( serial );

//	faults_test::faults_ts *faults = new faults_test::faults_ts("faults_ts");
//	test->add( faults );
//
//	sim_test::sim_ts *sim = new sim_test::sim_ts("sim_ts");
//	test->add( sim );
//
//	import_test::import_ts *import = new import_test::import_ts("import_ts");
//	test->add( import );
//
//	lib_test::lib_ts *lib = new lib_test::lib_ts("lib_ts");
//	test->add( lib );
//
//	pattern_test::pattern_ts *pattern = new pattern_test::pattern_ts("pattern_ts");
//	test->add( pattern );

    return test;
}



