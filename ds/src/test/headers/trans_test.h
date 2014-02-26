/*
 * trans_test.h
 *
 *  Created on: 27.01.2014
 *      Author: cookao
 */

#ifndef TRANS_TEST_H_
#define TRANS_TEST_H_

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <list>
#include <string>

namespace trans_test {

	void convert_sff_netlist(const std::string& name);
	void fix_nxp_netlist(const std::string& name);

	struct trans_ts : public boost::unit_test::test_suite
	{
		trans_ts(const std::string& name):boost::unit_test::test_suite(name)
	    {
			circuits_to_fix.push_back( "p45k" );
			circuits_to_fix.push_back( "p100k" );
			circuits_to_fix.push_back( "p141k" );
			circuits_to_fix.push_back( "p267k" );
			circuits_to_fix.push_back( "p269k" );
			circuits_to_fix.push_back( "p279k" );
			circuits_to_fix.push_back( "p286k" );
			circuits_to_fix.push_back( "p295k" );
			circuits_to_fix.push_back( "p330k" );

	    	circuits_to_transform.push_back( "p45k_nan" );
	    	circuits_to_transform.push_back( "p100k_nan" );
	    	circuits_to_transform.push_back( "p141k_nan" );
	    	circuits_to_transform.push_back( "p267k_nan" );
	    	circuits_to_transform.push_back( "p269k_nan" );
	    	circuits_to_transform.push_back( "p279k_nan" );
	    	circuits_to_transform.push_back( "p286k_nan" );
	    	circuits_to_transform.push_back( "p295k_nan" );
	    	circuits_to_transform.push_back( "p330k_nan" );

	        add( BOOST_PARAM_TEST_CASE( &convert_sff_netlist, circuits_to_transform.begin(), circuits_to_transform.end() ) );
//	        add( BOOST_PARAM_TEST_CASE( &fix_nxp_netlist, circuits_to_fix.begin(), circuits_to_fix.end() ) );
	    }
	    std::list<std::string> circuits_to_transform;
	    std::list<std::string> circuits_to_fix;
	};

}


#endif /* TRANS_TEST_H_ */
