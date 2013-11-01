/*
 * import_test.h
 *
 *  Created on: 22.04.2013
 *      Author: cookao
 */

#ifndef IMPORT_TEST_H_
#define IMPORT_TEST_H_

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <list>
#include <string>

namespace import_test {

	void import_netlist(const std::string& name);
	void parse_verilog(const std::string& name);

	struct import_ts : public boost::unit_test::test_suite
	{
		import_ts(const std::string& name):boost::unit_test::test_suite(name)
	    {
			files_to_test.push_back( "cpu_ip_cmb07.v" );
			//files_to_test.push_back( "cpu_ip.v" );
	        files_to_test.push_back( "p45k.v" );
	        files_to_test.push_back( "p100k.v" );
	        files_to_test.push_back( "p141k.v" );
	        files_to_test.push_back( "p239k.v" );
	        files_to_test.push_back( "p259k.v" );
	        files_to_test.push_back( "p279k.v" );
	        files_to_test.push_back( "p286k.v" );
	        files_to_test.push_back( "p295k.v" );
	        files_to_test.push_back( "p330k.v" );

	        add( BOOST_PARAM_TEST_CASE( &import_netlist, files_to_test.begin(), files_to_test.end() ) );
	        add( BOOST_PARAM_TEST_CASE( &parse_verilog, files_to_test.begin(), files_to_test.end() ) );
	    }
	    std::list<std::string> files_to_test;
	};

}


#endif /* IMPORT_TEST_H_ */
