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

	void serialize_paterns(const std::string& name);


	struct serialize_ts : public boost::unit_test::test_suite
	{
		serialize_ts(const std::string& name):boost::unit_test::test_suite(name)
		{
			patternd_to_test.push_back( "p45k.wgl" );

			add( BOOST_PARAM_TEST_CASE( &serialize_paterns, patternd_to_test.begin(), patternd_to_test.end() ) );
		}
		std::list<std::string> patternd_to_test;
	};

}

#endif /* DS_SERIALIZATION_H_ */
