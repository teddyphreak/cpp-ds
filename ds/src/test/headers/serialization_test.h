/*
 * ds_serialization.h
 *
 *  Created on: 30.11.2013
 *      Author: cookao
 */

#ifndef DS_SERIALIZATION_H_
#define DS_SERIALIZATION_H_

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>

namespace serialization_test{

	void serialize_patterns(const std::string& name);

	void serialize_netlist(const std::string& name);

	struct serialize_ts : public boost::unit_test::test_suite
	{
		serialize_ts(const std::string& name):boost::unit_test::test_suite(name)
		{
			netlist_to_test.push_back( "p45k_nan.v" );
			netlist_to_test.push_back( "p100k_nan.v" );
			netlist_to_test.push_back( "p141k_nan.v" );
			netlist_to_test.push_back( "p267k_nan.v" );
			netlist_to_test.push_back( "p269k_nan.v" );
			netlist_to_test.push_back( "p279k_nan.v" );
			netlist_to_test.push_back( "p286k_nan.v" );
			netlist_to_test.push_back( "p295k_nan.v" );
			netlist_to_test.push_back( "p330k_nan.v" );

//			pattern_to_test.push_back( "p45k" );
//			pattern_to_test.push_back( "p100k" );
//			pattern_to_test.push_back( "p141k" );
//			pattern_to_test.push_back( "p267k" );
//			pattern_to_test.push_back( "p269k" );
//			pattern_to_test.push_back( "p279k" );
//			pattern_to_test.push_back( "p286k" );
//			pattern_to_test.push_back( "p295k" );
//			pattern_to_test.push_back( "p330k" );

			add( BOOST_PARAM_TEST_CASE( &serialize_netlist, netlist_to_test.begin(), netlist_to_test.end() ) );
			add( BOOST_PARAM_TEST_CASE( &serialize_patterns, pattern_to_test.begin(), pattern_to_test.end() ) );
		}
		std::list<std::string> pattern_to_test;
		std::list<std::string> netlist_to_test;
	};

}

#endif /* DS_SERIALIZATION_H_ */
