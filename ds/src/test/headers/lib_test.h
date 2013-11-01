/*
 * lib_test.h
 *
 *  Created on: 22.04.2013
 *      Author: cookao
 */

#ifndef LIB_TEST_H_
#define LIB_TEST_H_

#include <string>

namespace lib_test {

	void lib_parse_test();
	void lib_open_test();
	void lib_gate_query_test();

	struct lib_ts : public boost::unit_test::test_suite
	{
		lib_ts(const std::string& name):boost::unit_test::test_suite(name)
		{
	        add( BOOST_TEST_CASE( &lib_parse_test ) );
	        add( BOOST_TEST_CASE( &lib_open_test ) );
	        add( BOOST_TEST_CASE( &lib_gate_query_test ) );
	    }
	};
}


#endif /* LIB_TEST_H_ */
