/*
 * faults_test.h
 *
 *  Created on: 08.11.2013
 *      Author: cookao
 */

#ifndef FAULTS_TEST_H_
#define FAULTS_TEST_H_

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <list>
#include <string>

namespace faults_test {

	void get_collapsed_faults(const std::string& name);
	void import_faults(const std::string& name);

	struct faults_ts : public boost::unit_test::test_suite
	{
		faults_ts(const std::string& name):boost::unit_test::test_suite(name)
	    {
			files_to_test.push_back( "cpu_ip_t03.v" );
//	        files_to_test.push_back( "p45k.v" );
//	        files_to_test.push_back( "p100k.v" );
//	        files_to_test.push_back( "p141k.v" );
//	        files_to_test.push_back( "p239k.v" );
//	        files_to_test.push_back( "p259k.v" );
//	        files_to_test.push_back( "p279k.v" );
//	        files_to_test.push_back( "p286k.v" );
//	        files_to_test.push_back( "p295k.v" );
//	        files_to_test.push_back( "p330k.v" );
//
//	        faults_to_import.push_back( "p45k_nan_faults" );
//	        faults_to_import.push_back( "p100k_nan_faults" );
//	        faults_to_import.push_back( "p141k_nan_faults" );
//	        faults_to_import.push_back( "p267k_nan_faults" );
//	        faults_to_import.push_back( "p269k_nan_faults" );
//	        faults_to_import.push_back( "p279k_nan_faults" );
//	        faults_to_import.push_back( "p295k_nan_faults" );
//	        faults_to_import.push_back( "p330k_nan_faults" );

	        add( BOOST_PARAM_TEST_CASE( &get_collapsed_faults, files_to_test.begin(), files_to_test.end() ) );
	     //   add( BOOST_PARAM_TEST_CASE( &import_faults, faults_to_import.begin(), faults_to_import.end() ) );
	    }
	    std::list<std::string> files_to_test;
	    std::list<std::string> faults_to_import;
	};

}


#endif /* FAULTS_TEST_H_ */
