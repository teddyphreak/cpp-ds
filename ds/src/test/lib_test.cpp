/*
 * lib_test.cpp
 *
 *  Created on: Sep 16, 2012
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include "ds_library.h"
#include "ds_structural.h"
#include "lib_test.h"

using namespace ds_library;
using namespace ds_structural;

void lib_test::lib_open_test() {

	LibraryFactory *factory = LibraryFactory::getInstance();
	Library *defaultLib = factory->load_library();
	BOOST_ASSERT(defaultLib !=0);
	factory->remove_library();
	std::string path_inconsistent = "default_lib_inconsistent";
	Library *inconsistentLib = factory->load_library(path_inconsistent);
	BOOST_ASSERT(inconsistentLib == 0);
	factory->remove_library(path_inconsistent);
	std::string path_unknown = "default_lib_unknown";
	Library *unknownLib = factory->load_library(path_unknown);
	BOOST_ASSERT(unknownLib ==0);
	factory->remove_library(path_unknown);
}

void lib_test::lib_parse_test() {

	std::string t("and8,and8,1,o1@o,8,i1@a,i2@b,i3@c,i4@d,i5@e,i6@f,i7@g,i8@h");
	parse_lib_node n;
	lib_parser<std::string::iterator> p;
	lib_skipper<std::string::iterator> skipper;
	bool parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
	BOOST_ASSERT(parse);
	t = "and8,and8,1,o1@o,8,i1@a,i2@b,i3@c,i4@d,i5@e,i6@f,i7@g,i8@h# this is a comment";
	parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
	BOOST_ASSERT(parse);
	BOOST_ASSERT(!n.flexible);
	t = "and8,and8,1,o1@o,8,i1@a,  i2@b, i3@c,i4@d, i5 @e,i6@ f,i7@g,i8@h";
	parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
	BOOST_ASSERT(parse);
	t = "and8,and8,1,o1@o,8,i1@1,i2@b,i3@c,i4@d,i5 @e,i6@f,i7@g,i8@h";
	parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
	BOOST_ASSERT(!parse);
	t = "and8,and8,1,o1@o,i1@a,i2@b,i3@c,i4@d,i5 @e,i6@f,i7@g,i8@h";
	parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
	BOOST_ASSERT(!parse);
	t = "and8*,and8,1,o1@o,8,i1@a,  i2@b, i3@c,i4@d, i5 @e,i6@ f,i7@g,i8@h";
	parse =  boost::spirit::qi::phrase_parse(t.begin(), t.end(), p, skipper, n);
	BOOST_ASSERT(parse);
	BOOST_ASSERT(n.flexible);
}

void lib_test::lib_gate_query_test() {

	LibraryFactory *factory = LibraryFactory::getInstance();
	Library *defaultLib = factory->load_library();
		Gate* g = defaultLib->get_gate("and4",5);
	BOOST_ASSERT(g!=0);
	BOOST_ASSERT(g->get_type()=="and4");
	BOOST_ASSERT(g->get_lgn()!=0);
	BOOST_ASSERT(g->get_num_ports()==5);
	std::size_t inputs = 0;
	typedef port_container::const_iterator IT;
	for (IT it=g->get_inputs()->begin();it!=g->get_inputs()->end();it++){
		std::string name = (*it)->get_instance_name();
		BOOST_ASSERT(!(g->get_mapping(name)).empty());
		inputs++;
	}
	std::size_t outputs = 0;
	typedef port_container::const_iterator IT;
	for (IT it=g->get_outputs()->begin();it!=g->get_outputs()->end();it++){
		std::string name = (*it)->get_instance_name();
		BOOST_ASSERT(!(g->get_mapping(name)).empty());
		outputs++;
	}
	BOOST_ASSERT(g->get_num_ports() == inputs + outputs);
	delete(g);
	g = defaultLib->get_gate("or",7);
	BOOST_ASSERT(g!=0);
	BOOST_ASSERT(g->get_type()=="or");
	BOOST_ASSERT(g->get_lgn()!=0);
	BOOST_ASSERT(g->get_num_ports()==7);
	BOOST_ASSERT(g->get_inputs()->size()==6);
	BOOST_ASSERT(g->get_outputs()->size()==1);
	inputs = 0;
	typedef port_container::const_iterator IT;
	for (IT it=g->get_inputs()->begin();it!=g->get_inputs()->end();it++){
		std::string name = (*it)->get_instance_name();
		BOOST_ASSERT(!(g->get_mapping(name)).empty());
		inputs++;
	}
	outputs = 0;
	typedef port_container::const_iterator IT;
	for (IT it=g->get_outputs()->begin();it!=g->get_outputs()->end();it++){
		std::string name = (*it)->get_instance_name();
		BOOST_ASSERT(!(g->get_mapping(name)).empty());
		outputs++;
	}
	BOOST_ASSERT(g->get_num_ports() == inputs + outputs);
	delete(g);
}








