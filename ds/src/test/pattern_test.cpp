/*
 * pattern_test.cpp
 *
 *  Created on: 22.04.2013
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include "pattern_test.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"
#include "ds_pattern.h"

using namespace ds_library;
using namespace ds_structural;
using namespace ds_workspace;
using namespace ds_pattern;


void pattern_test::wgl_combinational_test(std::string& name) {

	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	try {
		std::string file = path + "/files/" + name;
		BOOST_LOG_TRIVIAL(info) << "... importing " << file;
		ds_pattern::CombinationalPatternProvider* p = ds_pattern::load_combinational_blocks(file, false);

		namespace spirit = boost::spirit;
		std::ifstream input(file.c_str());
		input.unsetf(std::ios::skipws);
		ds_pattern::scan_data rawPatterns;

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_pattern::wgl_parser<spirit::istream_iterator> parser;
		spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, rawPatterns);
		input.close();

		ds_pattern::CombinationalPatternList pl(rawPatterns,false);

		BOOST_CHECK(rawPatterns.cycles.size() == pl.get_vector_count());

		std::vector<std::string> rawData;
		ds_pattern::combinational_port_data_extractor extrator(&rawData);
		for ( ds_pattern::tester_cycle c: rawPatterns.cycles )
		{
			boost::apply_visitor(extrator, c);
		}

		for (std::size_t i=0;i<rawData.size();i++){

			std::string pattern = rawData[i];

			BOOST_CHECK(pattern.size() == pl.get_port_count());
			for (std::size_t pIdx=0;pIdx<pattern.size();pIdx++){
				char expected = pattern[pIdx];
				PatternValue pv = pl.get_port_values(i);
				char val = '-';
				if (pv.get(pIdx) == ds_common::BIT_0)
					val = '0';
				else if (pv.get(pIdx) == ds_common::BIT_1)
					val = '1';
				else if (pv.get(pIdx) == ds_common::BIT_X)
					val = 'X';
				BOOST_CHECK(val == expected);
			}


			BOOST_CHECK(pattern.size() == p->num_ports());
			int block = i / ds_common::WIDTH;
			int offset = i % ds_common::WIDTH;

			for (std::size_t pIdx=0;pIdx<pattern.size();pIdx++){

				if (pattern[pIdx] != 'X'){

					SimPatternBlock& pb = p->get_block(block);
					int64 xVal = (pb.values[pIdx].x >> offset) & 0x1;
					BOOST_CHECK(xVal == 0);
					int64 val = (pb.values[pIdx].v >> offset) & 0x1;
					int64 expected = pattern[pIdx] == '1' ? 1:0;
					BOOST_CHECK(val == expected);
					if (val != expected){
						BOOST_LOG_TRIVIAL(error) << "VALUE MISMATCH: pattern: " << pIdx <<  " block: " << block << " offset " << offset;
					}

				} else {
					SimPatternBlock& pb = p->get_block(block);
					int64 xVal = (pb.values[pIdx].x >> offset) & 0x1;
					BOOST_CHECK(xVal == 1);
					if (xVal != 1){
						BOOST_LOG_TRIVIAL(error) << "X MISMATCH: pattern: " << pIdx <<  " block: " << block << " offset: " << offset;
					}
				}

			}


		}
	}catch (boost::exception& e){
		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
			BOOST_LOG_TRIVIAL(error) << *mi;
			std::cerr << "Error: " << *mi;
		}
	}
}

void pattern_test::wgl_transition_test(std::string& name) {

	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	try {
		std::string file = path + "/files/" + name;
		BOOST_LOG_TRIVIAL(info) << "Importing " << file;

		ds_pattern::scan_data data;
		bool parse = ds_pattern::parse_transition_wgl(file, data);
		BOOST_CHECK(parse);
		BOOST_LOG_TRIVIAL(info) << "WGL file parsed " << file;
		ds_pattern::SequentialPatternList pl(data);
		BOOST_LOG_TRIVIAL(info) << "Pattern list generated...";
		BOOST_CHECK(data.cycles.size() == pl.get_vector_count());
		ds_pattern::SequentialPatternProvider provider(pl);
		BOOST_LOG_TRIVIAL(info) << "Pattern provider generated...";

	}catch (boost::exception& e){
			if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) ){
				BOOST_LOG_TRIVIAL(error) << *mi;
				std::cerr << "Error: " << *mi;
			}
		}
	}


