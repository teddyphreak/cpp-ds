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


void pattern_test::wgl_import_test(std::string& name) {

	const char* d = getenv("DS");
	std::string path = d?d:"";
	try {
		std::string file = path + "/files/" + name;
		std::cout << "... importing " << file << std::endl;
		ds_pattern::CombinationalPatternProvider* p = ds_pattern::load_pattern_blocks(file, false);

		namespace spirit = boost::spirit;
		std::ifstream input(file.c_str());
		input.unsetf(std::ios::skipws);
		ds_pattern::scan_data rawPatterns;

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_pattern::wgl_parser<spirit::istream_iterator> parser;
		spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, rawPatterns);
		input.close();

		ds_pattern::PatternList pl(rawPatterns,false);

		BOOST_CHECK(rawPatterns.scan.size() == pl.get_pattern_count());

		for (std::size_t i=0;i<rawPatterns.scan.size();i++){

			std::string pattern = rawPatterns.scan[i];

			BOOST_CHECK(pattern.size() == pl.get_port_count());
			for (std::size_t pIdx=0;pIdx<pattern.size();pIdx++){
				char expected = pattern[pIdx];
				PatternValue pv = pl.get_port_values(i);
				char val = '-';
				if (pv.get(pIdx) == ds_simulation::BIT_0)
					val = '0';
				else if (pv.get(pIdx) == ds_simulation::BIT_1)
					val = '1';
				else if (pv.get(pIdx) == ds_simulation::BIT_X)
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
						std::cout << std::hex << "VALUE MISMATCH: pattern: " << pIdx <<  " block: " << block << " offset " << offset << std::endl;
					}

				} else {
					SimPatternBlock& pb = p->get_block(block);
					int64 xVal = (pb.values[pIdx].x >> offset) & 0x1;
					BOOST_CHECK(xVal == 1);
					if (xVal != 1){
						std::cout << std::hex << "X MISMATCH: pattern: " << pIdx <<  " block: " << block << " offset: " << offset << std::endl;
					}
				}

			}


		}
	}catch (boost::exception& e){
		if( std::string *mi = boost::get_error_info<ds_common::errmsg_info>(e) )
			 std::cerr << "Error: " << *mi;
	}
}


