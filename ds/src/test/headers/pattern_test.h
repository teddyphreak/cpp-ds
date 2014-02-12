/*
 * pattern_test.h
 *
 *  Created on: 22.04.2013
 *      Author: cookao
 */

#ifndef PATTERN_TEST_H_
#define PATTERN_TEST_H_

#include <boost/test/parameterized_test.hpp>
#include <list>
#include <string>

namespace pattern_test {

	void wgl_combinational_test(std::string& name);
	void wgl_transition_test(std::string& name);

	struct pattern_ts : public boost::unit_test::test_suite
	{
		pattern_ts(const std::string& name):boost::unit_test::test_suite(name)
		{
			combinational.push_back( "p45k.wgl" );
			combinational.push_back( "p100k.wgl" );
			combinational.push_back( "p141k.wgl" );
			combinational.push_back( "p239k.wgl" );
			combinational.push_back( "p259k.wgl" );
			combinational.push_back( "p267k.wgl" );
			combinational.push_back( "p269k.wgl" );
			combinational.push_back( "p279k.wgl" );
			combinational.push_back( "p286k.wgl" );
			combinational.push_back( "p295k.wgl" );
			combinational.push_back( "p30k.wgl" );
	       // add( BOOST_PARAM_TEST_CASE( &wgl_combinational_test, combinational.begin(), combinational.end() ) );


			transition.push_back( "p45k_nan_patterns.wgl" );
	        add( BOOST_PARAM_TEST_CASE( &wgl_transition_test, transition.begin(), transition.end() ) );

		}
		std::list<std::string> combinational;
		std::list<std::string> transition;
	};

}



#endif /* PATTERN_TEST_H_ */
