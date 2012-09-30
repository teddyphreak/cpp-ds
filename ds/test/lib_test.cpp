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
#include "ds_workspace.h"

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

	BOOST_AUTO_TEST_CASE(netlist_import) {

		using namespace ds_library;
		using namespace ds_structural;
		using namespace ds_workspace;

		LibraryFactory *factory = LibraryFactory::getInstance();
		Library *defaultLib = factory->loadLibrary();
		const char* d = getenv("DS");
		std::string path = d?d:"";

		Workspace *wp = ds_workspace::Workspace::get_workspace();
		wp->add_library(defaultLib);

		std::string file = path + "/files/p45k.v";
		std::cout << "... importing " << file << std::endl;
		ds_structural::NetList *nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p100k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p141k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p239k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p259k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p267k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p269k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p279k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p286k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p295k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;

		file = path + "/files/p330k.v";
		std::cout << "... importing " << file << std::endl;
		nl = ds_library::import(file, "top", wp);
		BOOST_CHECK(nl!=0);
		delete nl;
	}

	BOOST_AUTO_TEST_CASE(verilog_parser) {
		using namespace ds_library;

		const char* d = getenv("DS");
		std::string path = d?d:"";

		std::string file = path + "/files/p45k.v";
		std::cout << "...parsing " << file << std::endl;
		std::vector<ds_library::parse_netlist> netlists;
		bool parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size() == 2);
		netlists.clear();

		file = path + "/files/p100k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p141k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p239k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p259k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p267k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p269k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p279k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p286k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p295k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();

		file = path + "/files/p330k.v";
		std::cout << "... parsing " << file << std::endl;
		parse = parse_verilog(file, netlists);
		BOOST_CHECK(parse);
		BOOST_CHECK(netlists.size()>0);
		netlists.clear();
	}

