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

	void wgl_import_test(std::string& name);

	struct pattern_ts : public boost::unit_test::test_suite
	{
		pattern_ts(const std::string& name):boost::unit_test::test_suite(name)
		{
			files_to_test.push_back( "atpg.07.wgl" );

	        add( BOOST_PARAM_TEST_CASE( &wgl_import_test, files_to_test.begin(), files_to_test.end() ) );

		}
		std::list<std::string> files_to_test;
	};

}



#endif /* PATTERN_TEST_H_ */
