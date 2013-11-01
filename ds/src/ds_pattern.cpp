/*
 * ds_pattern.cpp
 *
 *  Created on: 24.03.2013
 *      Author: cookao
 */
#include "ds_pattern.h"
#include <fstream>
#include "ds_common.h"
#include "ds_simulation.h"
#include "ds_lg.h"
#include <boost/exception/all.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>

ds_pattern::PatternList* ds_pattern::parse_wgl(const std::string& file, bool discard_x){

	namespace spirit = boost::spirit;

	std::cout << file << std::endl;
	std::ifstream input(file.c_str());
	input.unsetf(std::ios::skipws);
	ds_pattern::PatternList* pl = 0;

	ds_pattern::scan_data rawPatterns;

	if (input.is_open()){

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_pattern::wgl_parser<spirit::istream_iterator> parser;
		bool parse =  spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, rawPatterns);
		input.close();

		if (parse)
			pl = new ds_pattern::PatternList(rawPatterns, discard_x);


	} else {
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
	return pl;
}

ds_pattern::CombinationalPatternProvider* ds_pattern::load_pattern_blocks(const std::string& file, bool discard_x){
	ds_pattern::PatternList* pl = ds_pattern::parse_wgl(file, discard_x);
	ds_pattern::CombinationalPatternProvider* pattern_blocks = new ds_pattern::CombinationalPatternProvider(*pl);
	delete pl;
	return pattern_blocks;
}

ds_pattern::PatternList::PatternList(const scan_data& sd, bool discard_x) {

	port_order.insert(port_order.begin(), sd.order.begin(), sd.order.end());
	inputs = 0;

	for (std::size_t i=0;i<sd.ports.inputs.size();i++){
		ds_pattern::signal sig = sd.ports.inputs[i];
		if (sig.bus == true) {
			inputs += sig.left - sig.right + 1;
		} else {
			inputs++;
		}
	}
	outputs = 0;
	for (std::size_t i=0;i<sd.ports.outputs.size();i++){
		ds_pattern::signal sig = sd.ports.outputs[i];
		if (sig.bus == true) {
			outputs += sig.left - sig.right + 1;
		} else {
			outputs++;
		}
	}

	std::vector<std::string> local;
	local.insert(local.begin(), sd.scan.begin(), sd.scan.end());
	if (discard_x == true) {
		std::remove_if(
				local.begin(),
				local.end(),
				[&] (std::string const& i) { return std::string::npos != i.find('X'); }
		);
	}

	for (auto v = local.begin(); v!=local.end(); v++){
		std::string s = *v;

		ds_pattern::PatternValue val(inputs + outputs);
		for (std::size_t i=0;i<inputs + outputs;i++){
			char c = s.at(i);
			val.set(i,ds_simulation::BIT_X);
			if (c=='1'){
				val.set(i,ds_simulation::BIT_1);
			}
			if (c=='0'){
				val.set(i,ds_simulation::BIT_0);
			}
		}
		ports.push_back(val);
	}
}

ds_pattern::CombinationalPatternProvider::CombinationalPatternProvider(const scan_data& sc, bool discard_x):
		CombinationalPatternProvider(PatternList(sc,discard_x)){}

ds_pattern::CombinationalPatternProvider::CombinationalPatternProvider(const PatternList& pl):total_ports(pl.get_port_count()),
		output_offset(pl.get_num_inputs()), num_inputs(pl.get_num_inputs()), num_outputs(pl.get_num_outputs()){
	ports.insert(ports.begin(), pl.begin_port_order(), pl.end_port_order());
	reset();

	for (unsigned int i=0;i<pl.get_pattern_count() / ds_common::WIDTH + 1; i++){
		int last = ds_common::WIDTH;
		if (i*ds_common::WIDTH + last > pl.get_pattern_count())
			last =  pl.get_pattern_count() % ds_common::WIDTH;
		SimPatternBlock b;
		for (unsigned int j=0;j<pl.get_num_inputs() + pl.get_num_outputs();j++){
			b.num_patterns = last;
			b.values.push_back(ds_lg::lg_v64(0x0L,-1L));
		}
		for (int j=0;j<last;j++){
			PatternValue pv = pl.get_port_values(i*ds_common::WIDTH + j);
			for (unsigned int k=0;k<pl.get_num_inputs() + pl.get_num_outputs();k++){
				int64 v = 0L;
				int64 x = 0L;
				if (pv.get(k) == ds_simulation::BIT_0){
					v = 0L;
					x = 0L;
					x = 1L << j;
				} else if (pv.get(k) == ds_simulation::BIT_1){
					v = 1L << j;
					x = 0L;
					x = 1L << j;
				}

				b.values[k].v |= v;
				b.values[k].x &= ~x;

			}

		}
		values.push_back(b);
	}
}

std::size_t ds_pattern::CombinationalPatternProvider::get_offset(const std::string& name) const {
	std::size_t idx = 0;
	for (;idx<ports.size();idx++){
		if (ports[idx] == name){
			break;
		}
	}
	return idx;
}

std::string ds_pattern::CombinationalPatternProvider::get_name(const int& idx) const {
	return ports[idx];
}