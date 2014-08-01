/*
 * diagnosis_test.h
 *
 *  Created on: Apr 4, 2014
 *      Author: cookao
 */

#ifndef DIAGNOSIS_TEST_H_
#define DIAGNOSIS_TEST_H_

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <list>
#include <string>

namespace diagnosis_test {

	void diagnose(const std::string& design, const std::string& wgl_file, const std::string& top, const std::string& faults, const unsigned int& circuits, const unsigned int& iterations);

	void t_diagnosis_test();

	struct diagnosis_ts : public boost::unit_test::test_suite
	{

		diagnosis_ts(const std::string& name):boost::unit_test::test_suite(name)
	    {
			add( BOOST_TEST_CASE( &t_diagnosis_test ) );
	    }
	};

}

#endif /* DIAGNOSIS_TEST_H_ */
