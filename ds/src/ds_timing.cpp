/*
 * ds_timing.cpp
 *
 *  Created on: Apr 9, 2014
 *      Author: cookao
 */

#define FUSION_MAX_VECTOR_SIZE 11

#include "ds_timing.h"
#include "ds_workspace.h"
#include <iostream>
#include <fstream>
#include <boost/log/trivial.hpp>

bool ds_timing::parse_sdf(const std::string& file, ds_timing::sdf_data& sdf){

	namespace spirit = boost::spirit;

	std::ifstream input(file.c_str());
	input.unsetf(std::ios::skipws);
	bool parse = false;

	if (input.is_open()){

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_timing::sdf_parser<spirit::istream_iterator> parser;

		parse =  boost::spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, sdf);

	} else {
		BOOST_LOG_TRIVIAL(error) << "Error parsing verilog file: " << file << ". Device not open";
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
	return parse;
}

ds_timing::v64_ts** ds_timing::TNode1I::get_input(const std::string& name) {
	if (name == "a")return &a;
	return 0;
}

ds_timing::v64_ts** ds_timing::TNode2I::get_input(const std::string& name) {
	if (name == "a")return &a;
	if (name == "b")return &b;
	return 0;
}

ds_timing::v64_ts** ds_timing::TNode3I::get_input(const std::string& name) {
	if (name == "a")return &a;
	if (name == "b")return &b;
	if (name == "c")return &c;
	return 0;
}

ds_timing::v64_ts** ds_timing::TState::get_input(const std::string& name) {
	if (name == "d")return d.get_input("a");
	if (name == "cd")return &cd;
	if (name == "rst")return &rst;
	if (name == "rst_n")return &rst_n;
	if (name == "load")return &load;
	if (name == "load_n")return &load_n;
	if (name == "si")return &si;
	if (name == "se")return &se;
	return 0;
}

void ds_timing::TOutput::hook() {
	sim();
	for (ds_lg::SimulationHook<TNode> *h : hooks){
		o.value.v ^=  h->hook(resolver) & ~o.value.x;
	}
}

void ds_timing::TInput::hook() {
	sim();
	for (ds_lg::SimulationHook<TNode> *h : hooks){
		o.value.v ^=  h->hook(resolver) & ~o.value.x;
	}
}

void ds_timing::TNode1I::hook() {
	ds_timing::v64_ts *sa = a;
	ds_timing::v64_ts va(*a);
	a = &va;
	hook_inputs();
	sim();
	hook_outputs();
	a = sa;
}

void ds_timing::TNode2I::hook() {
	ds_timing::v64_ts *sa = a;
	ds_timing::v64_ts va(*a);
	a = &va;
	ds_timing::v64_ts *sb = b;
	ds_timing::v64_ts vb(*b);
	b = &vb;
	hook_inputs();
	sim();
	hook_outputs();
	a = sa;
	b = sb;
}

void ds_timing::TNode3I::hook() {
	ds_timing::v64_ts *sa = a;
	ds_timing::v64_ts va(*a);
	a = &va;
	ds_timing::v64_ts *sb = b;
	ds_timing::v64_ts vb(*b);
	b = &vb;
	ds_timing::v64_ts *sc = c;
	ds_timing::v64_ts vc(*c);
	c = &vc;
	hook_inputs();
	sim();
	hook_outputs();
	a = sa;
	b = sb;
	c = sc;
}

void ds_timing::TState::hook() {
	ds_timing::v64_ts *sd = *d.get_input("a");
	ds_timing::v64_ts vd(**d.get_input("a"));
	*d.get_input("a") = &vd;
	ds_timing::v64_ts *srst = rst;
	ds_timing::v64_ts vrst(*rst);
	rst = &vrst;
	ds_timing::v64_ts *srst_n = rst_n;
	ds_timing::v64_ts vrst_n(*rst_n);
	rst_n = &vrst_n;
	ds_timing::v64_ts *sload = load;
	ds_timing::v64_ts vload = *load;
	load = &vload;
	ds_timing::v64_ts *sload_n = load_n;
	ds_timing::v64_ts vload_n(*load_n);
	load_n = &vload_n;
	hook_inputs();
	sim();
	hook_outputs();
	*d.get_input("a") = sd;
	rst = srst;
	rst_n = srst_n;
	load = sload;
	load_n = sload_n;
}

void ds_timing::TState::hook_outputs(){
	for (ds_lg::SimulationHook<TNode> *h : hooks){
		ds_timing::v64_ts* p_port = get_output(h->get_hook_port());
		if (p_port!=0){
			int64 activation = h->hook(resolver);
			p_port->value.v ^= activation;// & ~p_port->value.x;
		}
	}
}

void ds_timing::TNode::hook_inputs(){
	for (ds_lg::SimulationHook<TNode> *h : hooks){
		std::string hport = h->get_hook_port();
		ds_timing::v64_ts** input_address = get_input(hport);
		if (input_address!=0){
			ds_timing::v64_ts* p_port = *input_address;
			if (p_port!=0){
				int64 activation = h->hook(resolver);
				p_port->value.v ^= activation & ~p_port->value.x;
			}
		}
	}
}

void ds_timing::TNode::hook_outputs(){
	for (ds_lg::SimulationHook<TNode> *h : hooks){
		ds_timing::v64_ts* p_port = get_output(h->get_hook_port());
		if (p_port!=0){
			int64 activation = h->hook(resolver);
			p_port->value.v ^= activation & ~p_port->value.x;
		}
	}
}

void ds_timing::TLeveledGraph::setup(){
	std::vector<ds_timing::TNode*> temp;
	for (auto it=nodes.begin();it!=nodes.end();it++){
		ds_timing::TNode *n = *it;
		if (!n->has_state()){
			temp.push_back(n);
		}
	}
	nodes.clear();
	nodes.insert(nodes.begin(), temp.begin(), temp.end());
	nodes.insert(nodes.begin(), registers.begin(), registers.end());
}

void ds_timing::TNode::calculate_transitions(ds_workspace::Workspace *workspace) {
	ds_library::LogicFunction f = workspace->get_function(gate->get_type());
	switch(f){
	case ds_library::AND:
		get_latest_transition(0L, 0L);
		break;
	case ds_library::OR:
		get_latest_transition(-1L, -1L);
		break;
	case ds_library::NAND:
		get_latest_transition(0L, -1L);
		break;
	case ds_library::NOR:
		get_latest_transition(-1L,0L);
		break;
	case ds_library::XOR:
	case ds_library::XNOR:
		get_latest_transition();
		break;
	case ds_library::NOT:
	case ds_library::BUF:
		break;
	default:
		BOOST_LOG_TRIVIAL(warning) << "Gate type not found: " << f;
		break;
	}
}

ds_timing::v64_ts* ds_timing::TNode2I::get_transition_input(const std::size_t& index){
	return transitions[index];
}

void ds_timing::TNode2I::get_latest_transition(const ds_common::int64& controlling, const ds_common::int64& controlled){
	for (std::size_t index=0;index<ds_common::WIDTH;index++){
		ds_common::int64 ov = (o.value.v >> index) & 0x01;
		ds_common::int64 ox = (o.value.x >> index) & 0x01;

		ds_common::int64 ci = (controlling >> index) & 0x01;
		ds_common::int64 co = (controlled >> index) & 0x01;

		ds_common::int64 v0 = (a->value.v >> index) & 0x01;
		ds_common::int64 v1 = (b->value.v >> index) & 0x01;

		ds_common::int64 x0 = (a->value.x >> index) & 0x01;
		ds_common::int64 x1 = (b->value.x >> index) & 0x01;

		if (ox == 0x01L){
			if (x0 == x1){
				if (a_ts[index] > b_ts[index]){
					transitions[index] = a;
				} else {
					transitions[index] = b;
				}
			} else if(x0 == ox){
				transitions[index] = a;
			} else {
				transitions[index] = b;
			}
		} else {
			if (ov == co){
				v64_ts *latest = 0;
				if ((x0 != 0x01L) && (v0 == ci)){
					latest = a;
				} else if((x1 != 0x01L) && (v1 == ci)){
					if (latest==0){
						latest = b;
					} else {
						if (b_ts[index] < a_ts[index]){
							latest = b;
						}
					}
				}
				transitions[index] = latest;
			} else {
				if (a_ts[index] > b_ts[index]){
					transitions[index] = a;
				} else {
					transitions[index] = b;
				}
			}
		}
	}
}

void ds_timing::TNode2I::get_latest_transition(){
	for (std::size_t index=0;index<ds_common::WIDTH;index++){
		v64_ts* latest = a;
		double delay = a_ts[index];
		if (b_ts[index] > delay){
			latest = b;
		}
		transitions[index] = latest;
	}
}

ds_timing::v64_ts* ds_timing::TNode3I::get_transition_input(const std::size_t& index){
	return transitions[index];
}

void ds_timing::TNode3I::get_latest_transition(const ds_common::int64& controlling, const ds_common::int64& controlled){
	for (std::size_t index=0;index<ds_common::WIDTH;index++){
		ds_common::int64 ov = (o.value.v >> index) & 0x01;
		ds_common::int64 ox = (o.value.x >> index) & 0x01;

		ds_common::int64 ci = (controlling >> index) & 0x01;
		ds_common::int64 co = (controlled >> index) & 0x01;

		ds_common::int64 v0 = (a->value.v >> index) & 0x01;
		ds_common::int64 v1 = (b->value.v >> index) & 0x01;
		ds_common::int64 v2 = (b->value.v >> index) & 0x01;

		ds_common::int64 x0 = (a->value.x >> index) & 0x01;
		ds_common::int64 x1 = (b->value.x >> index) & 0x01;
		ds_common::int64 x2 = (c->value.x >> index) & 0x01;

		if (ox == 0x01L){
			if ((x0 == x1) && (x1==x2) && (x2==ox)){
				if (a_ts[index] > b_ts[index]){
					if (a_ts[index] > c_ts[index]){
						transitions[index] = a;
					} else {
						transitions[index] = c;
					}
				} else {
					if (b_ts[index] > c_ts[index]){
						transitions[index] = b;
					} else {
						transitions[index] = c;
					}
				}
			} else {
				if ((x0==x1) && (x0==ox)){
					if (a_ts[index] > b_ts[index]){
						transitions[index] = a;
					} else {
						transitions[index] = b;
					}
				} else {
					if (c_ts[index] > b_ts[index]){
						transitions[index] = c;
					} else {
						transitions[index] = b;
					}
				}
			}
		} else {
			if (ov == co){
				double delay;
				v64_ts *latest = 0;
				if ((x0 != 0x01L) && (v0 == ci)){
					latest = a;
					delay = a_ts[index];
				}
				if((x1 != 0x01L) && (v1 == ci)){
					if (latest==0){
						latest = b;
						delay = b_ts[index];
					} else {
						if (b_ts[index] < a_ts[index]){
							latest = b;
							delay = b_ts[index];
						}
					}
				}
				if((x2 != 0x01L) && (v2 == ci)){
					if (latest==0){
						latest = c;
					} else {
						if (b_ts[index] < delay){
							latest = c;
						}
					}
				}
				transitions[index] = latest;
			} else {
				if ((a_ts[index] > b_ts[index]) && (a_ts[index] > c_ts[index])){
					transitions[index] = a;
				} else if ((b_ts[index] > a_ts[index]) && (b_ts[index] > c_ts[index])){
					transitions[index] = b;
				} else {
					transitions[index] = c;
				}
			}
		}
	}
}

void ds_timing::TNode3I::get_latest_transition(){
	for (std::size_t index=0;index<ds_common::WIDTH;index++){
		v64_ts* latest = 0;
		if (a_ts[index] >= b_ts[index] && a_ts[index] >= c_ts[index]){
			latest = a;
		} else if (b_ts[index] >= a_ts[index] && b_ts[index] >= c_ts[index]){
			latest = b;
		} else {
			latest = c;
		}
		transitions[index] = latest;
	}
}

void ds_timing::annotate(const ds_timing::sdf_data& sdf, ds_timing::TLeveledGraph *graph){
	ds_timing::delay_visitor visitor(graph);
	for (const ds_timing::sdf_cell cell:sdf.cells){
		std::string name = cell.instance.instance;
		std::string type = cell.cell_type;
		visitor.set_cell_type(type);
		for (const ds_timing::sdf_delay delay:cell.sdf_spec){
			for (const ds_timing::absolute absolute:delay.specs){
				visitor.set_instance_name(name);
				for (ds_timing::del_def def:absolute.defs){
					boost::apply_visitor(visitor, def);
				}
				visitor.set_timing();
			}
		}
	}
}

std::string ds_timing::get_node_name(const std::string descriptor){
	std::size_t index = descriptor.find('/');
	if (index == std::string::npos){
		return descriptor;
	} else {
		return descriptor.substr(0,index);
	}
}

std::string ds_timing::get_port_name(const std::string descriptor){
	std::size_t index = descriptor.find('/');
	if (index == std::string::npos){
		return "";
	} else {
		return descriptor.substr(index+1,descriptor.size());
	}
}
bool ds_timing::is_port(const std::string descriptor){
	return descriptor.find('/') == std::string::npos;
}

ds_timing::delay_visitor::delay_visitor(ds_timing::TLeveledGraph *g):lg(g){
	wp = ds_workspace::Workspace::get_workspace();
}

void ds_timing::delay_visitor::set_timing(){
	ds_timing::TNode *node = lg->get_node(instance_name);
	for (auto path_it=path_delays.begin();path_it!=path_delays.end();path_it++){
		ds_timing::v64_ts* out = path_it->first;
		ds_timing::IOPathDelay* delay = path_it->second;
		node->set_timing(out, delay);
	}
	for (auto cond_it=condition_delays.begin();cond_it!=condition_delays.end();cond_it++){
		ds_timing::v64_ts* out = cond_it->first;
		ds_timing::ConditionDelay* delay = cond_it->second;
		node->set_timing(out, delay);
	}
}


void ds_timing::delay_visitor::operator()(const ds_timing::sdf_interconnect& instance){

	std::string top_level = lg->get_netlist()->get_instance_name();
	if (cell_type == top_level){

		ds_timing::v64_ts *driver = 0;
		std::string driver_node_name = ds_timing::get_node_name(instance.input);
		ds_timing::TNode *driver_node = lg->get_node(driver_node_name);
		if (ds_timing::is_port(instance.input)){
			driver = driver_node->get_output("o");
		} else {
			std::string driver_port_name = ds_timing::get_port_name(instance.input);
			ds_structural::Gate *g = driver_node->get_gate();
			std::string primitive_port = g->get_mapping(driver_port_name);
			driver = driver_node->get_output(primitive_port);
		}

		ds_timing::v64_ts *reader = 0;
		std::string reader_node_name = ds_timing::get_node_name(instance.output);
		ds_timing::TNode *reader_node = lg->get_node(reader_node_name);
		if (ds_timing::is_port(instance.output)){
			reader = reader_node->get_output("a");
		} else {
			std::string reader_port_name = ds_timing::get_port_name(instance.output);
			ds_structural::Gate *g = reader_node->get_gate();
			std::string primitive_port = g->get_mapping(reader_port_name);
			reader = *reader_node->get_input(primitive_port);
		}

		ds_timing::InterconnectDelay *io_delay = new ds_timing::InterconnectDelay(driver, reader, reader_node, instance.val);
		reader_node->set_timing(reader, io_delay);
	}

}

void ds_timing::TLeveledGraph::adapt(const ds_pattern::SequentialPatternAdapter* adapter){
	// prepare input for simulation: set pattern block and offset
	for (ds_timing::TInput* in:inputs){
		in->set_pattern_block(&pattern_block);
		std::string name = in->get_name();
		std::string port_name = name.substr(name.find('/') + 1);
		std::size_t offset = adapter->get_port_offset(port_name);
		in->set_offset(offset);
		in->set_vector_offset(&vector_offset);
	}
	// prepare outputs for simulation: create observers and attach them to outputs
	for (ds_timing::TOutput* out:outputs){
		std::string name = out->get_name();
		std::string port_name = name.substr(name.find('/') + 1);
		std::size_t offset = adapter->get_port_offset(port_name);
		out->remove_monitors();
		ds_timing::TOutputObserver* observer= new TOutputObserver(offset, &pattern_block, &vector_offset);
		out->add_monitor(observer);
	}
	std::size_t scan_offset = adapter->get_scan_offset();
	for (ds_timing::TState* reg:registers){
		std::string name = reg->get_name();
		std::string cell_name = name.substr(name.find('/') + 1);
		std::size_t offset = adapter->get_scan_offset(cell_name);
		reg->set_pattern_block(&pattern_block);
		reg->set_offset(scan_offset + offset);
	}
}

void ds_timing::delay_visitor::operator()(const ds_timing::sdf_iopath& instance){

	std::string input_port = instance.input.port;
	std::string output_port = instance.output;
	ds_timing::TNode *node = lg->get_node(instance_name);
	ds_structural::Gate *g = node->get_gate();
	std::string input_primitive_port = g->get_mapping(input_port);
	std::string output_primitive_port = g->get_mapping(output_port);
	ds_timing::v64_ts *out = node->get_output(output_primitive_port);

	auto out_it = path_delays.find(out);
	ds_timing::IOPathDelay* current_path = 0;
	if (out_it == path_delays.end()){
		current_path = new ds_timing::IOPathDelay(out, node, wp);
		path_delays[out] = current_path;
	} else {
		current_path = out_it->second;
	}

	current_path->add_rising_delay(input_primitive_port, instance.vals[0]);
	current_path->add_falling_delay(input_primitive_port, instance.vals[1]);
}

void ds_timing::delay_visitor::operator()(const ds_timing::sdf_cond& instance){
	std::string input_port = instance.def.input.port;
	std::string output_port = instance.def.output;
	ds_timing::TNode *node = lg->get_node(instance_name);
	ds_structural::Gate *g = node->get_gate();
	std::string input_primitive_port = g->get_mapping(input_port);
	std::string output_primitive_port = g->get_mapping(output_port);
	ds_timing::v64_ts *out = node->get_output(output_primitive_port);
	ds_timing::v64_ts *in = *node->get_input(input_primitive_port);


	auto out_it = condition_delays.find(out);
	ds_timing::ConditionDelay* current_condition = 0;
	if (out_it == condition_delays.end()){
		current_condition = new ds_timing::ConditionDelay(node, out);
		condition_delays[out] = current_condition;
	} else {
		current_condition = out_it->second;
	}

	ds_timing::ConditionList *list = new ds_timing::ConditionList(in,out);
	for (bool_condition bc: instance.conditions){
		std::string input_name = g->get_mapping(bc.name);
		ds_timing::v64_ts *input = *node->get_input(input_name);
		lg_v64 val(0L,0L);
		if (bc.val)
			val.v=-1L;
		list->add_condition(val, input);
	}
	list->set_rising_delay(instance.def.vals[0]);
	list->set_rising_delay(instance.def.vals[1]);
	current_condition->add_condition_list(list);
}
