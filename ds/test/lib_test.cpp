/*
 * lib_test.cpp
 *
 *  Created on: Sep 16, 2012
 *      Author: cookao
 */
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>
#include "ds_library.h"
#include <iostream>
#include <boost/spirit/include/qi.hpp>

	BOOST_AUTO_TEST_CASE( lib_test_open ) {
		using namespace ds_library;
		LibraryFactory *factory = LibraryFactory::getInstance();
		Library *defaultLib = factory->loadLibrary();
		BOOST_REQUIRE(defaultLib !=0);
		factory->remove_library();

		std::string path_inconsistent = "default_lib_inconsistent";
		Library *inconsistentLib = factory->loadLibrary(path_inconsistent);
		BOOST_REQUIRE(inconsistentLib ==0);
		factory->remove_library(path_inconsistent);

		std::string path_unknown = "default_lib_unknown";
		Library *unknownLib = factory->loadLibrary(path_unknown);
		BOOST_REQUIRE(unknownLib ==0);
		factory->remove_library(path_unknown);
	}

	BOOST_AUTO_TEST_CASE( lib_test_parse ) {
		using namespace ds_library;
		std::string t("and8,and8,1,o1@o,8,i1@a,i2@b,i3@c,i4@d,i5@e,i6@f,i7@g,i8@h");
		parse_lib_node n;
		lib_parser<std::string::iterator> p;
		lib_skipper<std::string::iterator> skipper;
		bool parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
		BOOST_CHECK(parse);
		t = "and8,and8,1,o1@o,8,i1@a,i2@b,i3@c,i4@d,i5@e,i6@f,i7@g,i8@h# this is a comment";
		parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
		BOOST_CHECK(parse);
		BOOST_CHECK(!n.flexible);
		t = "and8,and8,1,o1@o,8,i1@a,  i2@b, i3@c,i4@d, i5 @e,i6@ f,i7@g,i8@h";
		parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
		BOOST_CHECK(parse);
		t = "and8,and8,1,o1@o,8,i1@1,i2@b,i3@c,i4@d,i5 @e,i6@f,i7@g,i8@h";
		parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
		BOOST_CHECK(!parse);
		t = "and8,and8,1,o1@o,i1@a,i2@b,i3@c,i4@d,i5 @e,i6@f,i7@g,i8@h";
		parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
		BOOST_CHECK(!parse);
		t = "and8*,and8,1,o1@o,8,i1@a,  i2@b, i3@c,i4@d, i5 @e,i6@ f,i7@g,i8@h";
		parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
		BOOST_CHECK(parse);
		BOOST_CHECK(n.flexible);
	}

	BOOST_AUTO_TEST_CASE( lib_gate_query ) {
		using namespace ds_library;
		using namespace ds_structural;
		LibraryFactory *factory = LibraryFactory::getInstance();
		Library *defaultLib = factory->loadLibrary();

		Gate* g = defaultLib->getGate("and4",5);
		BOOST_REQUIRE(g!=0);
		BOOST_REQUIRE(g->get_type()=="and4");
		BOOST_REQUIRE(g->get_lgn()!=0);
		BOOST_REQUIRE(g->get_num_ports()==5);
		std::size_t inputs = 0;
		typedef port_container::const_iterator IT;
		for (IT it=g->get_inputs()->begin();it!=g->get_inputs()->end();it++){
			std::string name = (*it)->get_instance_name();
			BOOST_REQUIRE(!(g->get_mapping(name)).empty());
			inputs++;
		}
		std::size_t outputs = 0;
		typedef port_container::const_iterator IT;
		for (IT it=g->get_outputs()->begin();it!=g->get_outputs()->end();it++){
			std::string name = (*it)->get_instance_name();
			BOOST_REQUIRE(!(g->get_mapping(name)).empty());
			outputs++;
		}
		BOOST_REQUIRE(g->get_num_ports() == inputs + outputs);

		delete(g);
		g = defaultLib->getGate("or",7);
		BOOST_REQUIRE(g!=0);
		BOOST_REQUIRE(g->get_type()=="or");
		BOOST_REQUIRE(g->get_lgn()!=0);
		BOOST_REQUIRE(g->get_num_ports()==7);
		BOOST_REQUIRE(g->get_inputs()->size()==6);
		BOOST_REQUIRE(g->get_outputs()->size()==1);
		inputs = 0;
		typedef port_container::const_iterator IT;
		for (IT it=g->get_inputs()->begin();it!=g->get_inputs()->end();it++){
			std::string name = (*it)->get_instance_name();
			BOOST_REQUIRE(!(g->get_mapping(name)).empty());
			inputs++;
		}
		outputs = 0;
		typedef port_container::const_iterator IT;
		for (IT it=g->get_outputs()->begin();it!=g->get_outputs()->end();it++){
			std::string name = (*it)->get_instance_name();
			BOOST_REQUIRE(!(g->get_mapping(name)).empty());
			outputs++;
		}
		BOOST_REQUIRE(g->get_num_ports() == inputs + outputs);
		delete(g);
	}

	BOOST_AUTO_TEST_CASE(verilog_parser) {
		using namespace ds_library;

		const char* d = getenv("DS");
		std::string path = d?d:"";
		path += "/files/p45k.v";
		std::ifstream input(path.c_str());
		std::stringstream ss;
		std::string line;
		BOOST_REQUIRE(input.is_open());

		while (!input.eof()){
			std::getline(input ,line);
			ss << line << ' ';
		}
		input.close();

		typedef std::string::iterator IT;
		std::vector<ds_library::parse_netlist> netlists;
		ds_library::parse_netlist netlist;
		ds_library::nxp_verilog_parser<IT> p;

		std::string nl_str = ss.str();
		using boost::spirit::ascii::space;
		bool parse =  boost::spirit::qi::phrase_parse(nl_str.begin(), nl_str.end(), p, space, netlists);
		BOOST_REQUIRE(parse);
	}
