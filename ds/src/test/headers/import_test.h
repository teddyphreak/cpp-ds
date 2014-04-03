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
	void dump_verilog(const std::string& name);
	void import_scan_map(const std::string& name);

	struct import_ts : public boost::unit_test::test_suite
	{
		import_ts(const std::string& name):boost::unit_test::test_suite(name)
	    {
			circuits_to_test.push_back( "cpu_ip_fwdctrl_opc_t03.v" );
//	    	circuits_to_test.push_back( "p45k_nan.v" );
//	    	circuits_to_test.push_back( "p100k.v" );
//	    	circuits_to_test.push_back( "p141k.v" );
//	    	circuits_to_test.push_back( "p239k.v" );
//	    	circuits_to_test.push_back( "p259k.v" );
//	    	circuits_to_test.push_back( "p279k.v" );
//	    	circuits_to_test.push_back( "p286k.v" );
//	    	circuits_to_test.push_back( "p295k.v" );
//	    	circuits_to_test.push_back( "p330k.v" );

	    	chains_to_test.push_back( "p45k.chain_info" );
	    	chains_to_test.push_back( "p100k.chain_info" );
	    	chains_to_test.push_back( "p141k.chain_info" );
	    	chains_to_test.push_back( "p239k.chain_info" );
	    	chains_to_test.push_back( "p259k.chain_info" );
	    	chains_to_test.push_back( "p279k.chain_info" );
	    	chains_to_test.push_back( "p286k.chain_info" );
	    	chains_to_test.push_back( "p295k.chain_info" );
	    	chains_to_test.push_back( "p330k.chain_info" );

	        add( BOOST_PARAM_TEST_CASE( &import_netlist, circuits_to_test.begin(), circuits_to_test.end() ) );
	      //  add( BOOST_PARAM_TEST_CASE( &parse_verilog, circuits_to_test.begin(), circuits_to_test.end() ) );
	      //  add( BOOST_PARAM_TEST_CASE( &dump_verilog, circuits_to_test.begin(), circuits_to_test.end() ) );
	      //  add( BOOST_PARAM_TEST_CASE( &import_scan_map, chains_to_test.begin(), chains_to_test.end() ) );
	    }
	    std::list<std::string> circuits_to_test;
	    std::list<std::string> chains_to_test;
	};

}


#endif /* IMPORT_TEST_H_ */
