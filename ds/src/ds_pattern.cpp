/*
 * ds_pattern.cpp
 *
 *  Created on: 24.03.2013
 *      Author: cookao
 */
#include "ds_lg.h"
#include "ds_pattern.h"
#include <fstream>
#include "ds_common.h"
#include <boost/exception/all.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>
#include <boost/log/trivial.hpp>

void ds_pattern::CombinationalPatternList::setup_ports(const ds_pattern::scan_data& sd){
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
}

ds_pattern::CombinationalPatternList* ds_pattern::parse_combinational_wgl(const std::string& file, bool compact){

	namespace spirit = boost::spirit;

	std::ifstream input(file.c_str());
	input.unsetf(std::ios::skipws);
	ds_pattern::CombinationalPatternList* pl = 0;

	ds_pattern::scan_data rawPatterns;

	if (input.is_open()){

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_pattern::wgl_parser<spirit::istream_iterator> parser;
		bool parse =  spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, rawPatterns);
		input.close();

		if (parse){
			pl = new ds_pattern::CombinationalPatternList(rawPatterns, compact);
		}

	} else {
		BOOST_LOG_TRIVIAL(error) << "Error parsing wgl file: " << file << ". Device not open";
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
	return pl;
}

ds_pattern::CombinationalPatternProvider* ds_pattern::load_combinational_blocks(const std::string& file, bool compact){
	ds_pattern::CombinationalPatternList* pl = ds_pattern::parse_combinational_wgl(file, compact);
	ds_pattern::CombinationalPatternProvider* pattern_blocks = new ds_pattern::CombinationalPatternProvider(*pl);
	delete pl;
	return pattern_blocks;
}

ds_pattern::SequentialPatternProvider* ds_pattern::load_loc_blocks(const std::string& file){
	ds_pattern::scan_data data;
	ds_pattern::parse_transition_wgl(file, data);;
	ds_pattern::SequentialPatternList pl(data);
	ds_pattern::SequentialPatternProvider* pattern_blocks = new ds_pattern::SequentialPatternProvider(pl);
	return pattern_blocks;
}

ds_pattern::CombinationalPatternList::CombinationalPatternList(const scan_data& sd, bool compact) {

	setup_ports(sd);

	std::vector<std::string> local;
	ds_pattern::combinational_port_data_extractor extrator(&local);
	for ( ds_pattern::tester_cycle c: sd.cycles )
	{
		boost::apply_visitor(extrator, c);
	}

	for (auto v = local.begin(); v!=local.end(); v++){
		std::string s = *v;

		ds_pattern::PatternValue val = transform(s, inputs + outputs, 0);

		if (compact){

			bool compatible = false;

			for (std::size_t i=0;i<cycles.size();i++){
				PatternValue pv = cycles[i].ports;
				if (pv.is_compatible(val)){
					compatible = true;
					if (val.get_specified_bits() > pv.get_specified_bits()){
						cycles[i].ports = val;
					}
					break;
				}
			}
			if (!compatible){
				ate_cycle c(val, "");
				cycles.push_back(c);
			}
		} else {
			ate_cycle c(val, "");
			cycles.push_back(c);
		}
	}
}

bool ds_pattern::PatternValue::is_compatible(const PatternValue& pv){
	boost::dynamic_bitset<> t_x(pv.x);
	t_x |= x;
	boost::dynamic_bitset<> t_v(pv.v);
	t_v ^= v;
	t_v = ~t_v;
	t_v |= t_x;
	t_v = ~t_v;
	return t_v.count()==0;
}

ds_pattern::CombinationalPatternProvider::CombinationalPatternProvider(const scan_data& sc, bool compact):
		CombinationalPatternProvider(CombinationalPatternList(sc,compact)){}

ds_pattern::CombinationalPatternProvider::CombinationalPatternProvider(const CombinationalPatternList& pl):total_ports(pl.get_port_count()),
		output_offset(pl.get_num_inputs()), num_inputs(pl.get_num_inputs()), num_outputs(pl.get_num_outputs()){

	ports.insert(ports.begin(), pl.begin_port_order(), pl.end_port_order());
	reset();

	for (unsigned int i=0;i<pl.get_vector_count() / ds_common::WIDTH + 1; i++){
		int last = ds_common::WIDTH;
		if (i*ds_common::WIDTH + last > pl.get_vector_count())
			last =  pl.get_vector_count() % ds_common::WIDTH;
		SimPatternBlock b(last);
		for (unsigned int j=0;j<pl.get_num_inputs() + pl.get_num_outputs();j++){
			b.values.push_back(ds_lg::lg_v64(0x0L,-1L));
		}
		for (int j=0;j<last;j++){
			PatternValue pv = pl.get_port_values(i*ds_common::WIDTH + j);
			for (unsigned int k=0;k<pl.get_num_inputs() + pl.get_num_outputs();k++){
				int64 v = 0L;
				int64 x = 0L;
				if (pv.get(k) == ds_common::BIT_0){
					v = 0L;
					x = 0L;
					x = 1L << j;
				} else if (pv.get(k) == ds_common::BIT_1){
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

bool ds_pattern::parse_transition_wgl(const std::string& file, ds_pattern::scan_data& rawPatterns){

	namespace spirit = boost::spirit;

	std::ifstream input(file.c_str());
	input.unsetf(std::ios::skipws);

	bool parse = false;

	if (input.is_open()){

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_pattern::wgl_parser<spirit::istream_iterator> parser;
		parse =  spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, rawPatterns);
		input.close();

	} else {
		BOOST_LOG_TRIVIAL(error) << "Error parsing wgl file: " << file << ". Device not open";
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
	return parse;
}

ds_pattern::SequentialPatternList::SequentialPatternList(const scan_data& sd):num_scan_cells(0) {

	setup_ports(sd);
	std::unordered_map<std::string, std::string> value_map;
	std::unordered_map<std::string, int> group_offset;
	std::unordered_map<std::string, int> chain_offset;

	for (std::size_t i=0;i<sd.cell_groups.size();i++){
		ds_pattern::scan_cells scan_cells = sd.cell_groups[i];
		cell_order.push_back(scan_cells.cells);
		group_offset[scan_cells.group] = num_scan_cells;
		cell_names.insert(cell_names.end(), scan_cells.cells.begin(), scan_cells.cells.end());
		num_scan_cells += scan_cells.cells.size();
	}

	for (std::size_t i=0;i<cell_names.size();i++){
		cell_map[cell_names[i]] = i;
	}

	for (std::size_t i=0;i<sd.states.size();i++){
		ds_pattern::scan_state state = sd.states[i];
		chain_offset[state.id] = group_offset[state.group];
		value_map[state.id] = state.data;
	}

	ds_pattern::sequential_extractor extractor(&cycles, chain_offset, value_map, num_scan_cells);
	for ( ds_pattern::tester_cycle c: sd.cycles ){
		boost::apply_visitor(extractor, c);
	}
}

void ds_pattern::sequential_extractor::operator()(const ds_pattern::scan& v) {
	PatternValue scan(2*tc);
	PatternValue ports = ds_pattern::transform(v.port_data, v.port_data.size(), 0);
	for (ds_pattern::scan_directive d:v.directives){
		int offset = 0;
		if (d.direction == "output"){
			offset = tc;
		}

		auto data_it = vm.find(d.chain_data);
		if (data_it != vm.end()){
			std::string data = data_it->second;
			auto it = om.find(d.chain_data);
			if (it!=om.end()){
				int cell_offset = offset + (it->second);
				ds_pattern::transform(data, scan, cell_offset);
			}

		} else {
			BOOST_LOG_TRIVIAL(error) << "Scan values not found: " << d.chain_data;
		}
	}
	ds_pattern::ate_cycle cycle(ports, scan, v.timeplate);
	c->push_back(cycle);
}

void ds_pattern::sequential_extractor::operator()(const ds_pattern::vector& v){
	PatternValue ports = transform(v.port_data, v.port_data.size(), 0);
	ds_pattern::ate_cycle cycle(ports, v.timeplate);
	c->push_back(cycle);
}

ds_pattern::PatternValue ds_pattern::transform(const std::string& s, const std::size_t& length, const int& offset) {
	ds_pattern::PatternValue val(length);
	for (std::size_t i=0;i<s.size();i++){
		char c = s.at(i);
		val.set(offset + i, ds_common::BIT_X);
		if (c=='1'){
			val.set(offset + i, ds_common::BIT_1);
		}
		if (c=='0'){
			val.set(offset +  i, ds_common::BIT_0);
		}
	}
	return val;
}

void ds_pattern::transform(const std::string& s, ds_pattern::PatternValue& val, const int& offset) {
	for (std::size_t i=0;i<s.size();i++){
		char c = s.at(i);
		val.set(offset + i, ds_common::BIT_X);
		if (c=='1'){
			val.set(offset + i, ds_common::BIT_1);
		}
		if (c=='0'){
			val.set(offset +  i, ds_common::BIT_0);
		}
	}
}

ds_pattern::SequentialPatternProvider::SequentialPatternProvider(const ds_pattern::SequentialPatternList& pl): blockIndex(0){

	num_inputs = pl.get_num_inputs();
	num_outputs = pl.get_num_outputs();

	ports.insert(ports.begin(), pl.begin_port_order(), pl.end_port_order());
	for (std::size_t i=0;i<pl.get_num_scan_cells();i++){
		cells.push_back(pl.get_cell_name(i));
	}


	std::size_t num_scan = 0;
	for (std::size_t i=0;i<pl.get_vector_count();i++){
		ds_pattern::PatternValue v = pl.get_scan_values(i);
		if (v.get_size() > 0)
			num_scan++;
	}
	std::size_t vector_offset = pl.get_num_inputs() + pl.get_num_outputs();
	std::size_t cell_offset = vector_offset * 2;

	std::size_t scan_cycles = 0;
	std::size_t idx = 0;

	while(scan_cycles < num_scan){

		ds_pattern::PatternValue p_scan = pl.get_port_values(idx);
		ds_pattern::PatternValue scan = pl.get_scan_values(idx++);
		if (scan.get_size() > 0){

			scan_cycles++;

			std::size_t last = ds_common::WIDTH;

			if ((scan_cycles / ds_common::WIDTH) * ds_common::WIDTH + last > num_scan)
				last =  num_scan % ds_common::WIDTH;

			SimPatternBlock b(last);
			std::size_t total_values = cell_offset + pl.get_num_scan_cells()*2;
			for (std::size_t j=0;j<total_values;j++){
				b.values.push_back(ds_lg::lg_v64(0x0L,-1L));
			}
			set_values(0, cell_offset, scan, b);
			ds_pattern::PatternValue launch = pl.get_port_values(idx++);
			set_values(0, 0, launch, b);
			ds_pattern::PatternValue capture = pl.get_port_values(idx++);
			set_values(0, vector_offset, capture, b);

			std::size_t block_start = scan_cycles;
			std::size_t curr = scan_cycles + 1;

			while(curr - block_start < last ){

				ds_pattern::PatternValue p = pl.get_port_values(idx);
				ds_pattern::PatternValue v = pl.get_scan_values(idx++);

				if (v.get_size() > 0){

					scan_cycles++;

					std::size_t pos = curr - block_start;
					set_values(pos, cell_offset, v, b);
					if ((last != ds_common::WIDTH) && ((scan_cycles - block_start)==last-1))
						break;

					ds_pattern::PatternValue launch = pl.get_port_values(idx++);
					set_values(pos, 0, launch, b);
					ds_pattern::PatternValue capture = pl.get_port_values(idx++);
					set_values(pos, vector_offset, capture, b);

					curr++;
				}
			}
			values.push_back(b);
		}
	}

	std::size_t num_blocks = values.size();
	int output_offset = cell_offset + pl.get_num_scan_cells();
	for (std::size_t i=0;i<num_blocks-1;i++){
		for (std::size_t j=0;j<pl.get_num_scan_cells();j++){
			long av = values[i+1].values[output_offset+j].v & 0x1L;
			long ax = values[i+1].values[output_offset+j].x & 0x1L;

			long vfixed = (values[i].values[output_offset+j].v >> 1) | (av << (ds_common::WIDTH - 1));
			long xfixed = (values[i].values[output_offset+j].x >> 1) | (ax << (ds_common::WIDTH - 1));

			values[i].values[output_offset+j].v = vfixed;
			values[i].values[output_offset+j].x = xfixed;
		}
	}
	if (values[num_blocks-1].num_patterns == 1){
		values.erase(--values.end());
	} else {
		values[num_blocks-1].num_patterns--;
		values[num_blocks-1].mask >>= 1;
		for (std::size_t j=0;j<pl.get_num_scan_cells();j++){
			values[num_blocks-1].values[output_offset+j].v >>= 1;
			values[num_blocks-1].values[output_offset+j].x >>= 1;
			values[num_blocks-1].values[output_offset+j].x |= (1L << (ds_common::WIDTH - 1));
		}
	}
}

void ds_pattern::SequentialPatternProvider::set_values(const std::size_t& slot, const std::size_t& block_offset,
		const ds_pattern::PatternValue pv, ds_pattern::SimPatternBlock& block){
	set_values(slot, block_offset, pv, 0, pv.get_size(), block);
}

void ds_pattern::SequentialPatternProvider::set_values(const std::size_t& slot, const std::size_t& block_offset,
		const ds_pattern::PatternValue pv, const std::size_t& pattern_offset, const std::size_t& length, ds_pattern::SimPatternBlock& block){

	for (std::size_t k=0;k<length;k++){

		int64 v = 0L;
		int64 x = 0L;
		std::size_t offset = pattern_offset + k;
		if (pv.get(offset) == ds_common::BIT_0){
			v = 0L;
			x = 1L << slot;
		} else if (pv.get(offset) == ds_common::BIT_1){
			v = 1L << slot;
			x = 1L << slot;
		}

		std::size_t disp = block_offset + k;
		block.values[disp].v |= v;
		int64 x_mask = ~x;
		block.values[disp].x &= x_mask;
	}
}

int ds_pattern::SequentialPatternProvider::get_port_offset(const std::size_t& vector, const std::string& name) const{
	std::size_t vector_offset = vector * (num_inputs + num_outputs);
	int offset = -1;
	for (std::size_t i=0;i<ports.size();i++){
		if (ports[i]==name){
			offset = i;
			break;
		}
	}
	if (offset == -1){
		BOOST_LOG_TRIVIAL(error) << "Port: " << name << " not found in vector" << vector;
		return offset;
	}
	offset += vector_offset;
	return offset;
}

int ds_pattern::SequentialPatternProvider::get_port_offset(const std::string& name) const{
	int offset = -1;
	for (std::size_t i=0;i<ports.size();i++){
		if (ports[i]==name){
			offset = i;
			break;
		}
	}
	if (offset==-1){
		BOOST_LOG_TRIVIAL(error) << "Port not found: " << name;
	}
	return offset;
}

int ds_pattern::SequentialPatternProvider::get_scan_offset(const std::string& name) const {
	int offset = -1;
	std::string s = "";
	for (std::size_t i=0;i<cells.size();i++){
		s += cells[i] + " ";
		if (cells[i]==name){
			offset = i;
			break;
		}
	}
	if (offset == -1){
		BOOST_LOG_TRIVIAL(error) << "Scan cell not found: " << name;
	}
	return offset;
}

std::string ds_pattern::SequentialPatternProvider::get_name(const int& idx) const{
	int scan_offset = (num_inputs + num_outputs) * 2;
	if (idx < scan_offset){
		return ports[idx % (num_inputs + num_outputs)];
	}
	return cells[idx - scan_offset];
}


