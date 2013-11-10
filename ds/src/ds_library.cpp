/*
 * ds_library.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: cookao
 */

#include "ds_library.h"
#include "ds_common.h"
#include "ds_workspace.h"
#include <iostream>
#include <fstream>
#include <errno.h>
#include <stack>
#include <boost/exception/all.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/lambda/algorithm.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

ds_library::LibraryFactory* ds_library::LibraryFactory::instance = 0;

void ds_library::Library::load_nodes(){

	using namespace ds_lg;

	std::list<LGNode*> list;
	LGNode *node = new LGNode1I("not", f_not); types["not"] = ds_library::NOT;
	list.push_back(node);
	node = new LGNode1I("not1", f_not); types["not1"] = ds_library::NOT;
	list.push_back(node);
	node = new LGNode1I("buf", f_buf); types["buf"] = ds_library::BUF;
	list.push_back(node);
	node = new LGNode1I("buf1", f_buf); types["buf1"] = ds_library::BUF;
	list.push_back(node);

	node = new LGNode2I("and", f_and2); types["and"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode2I("and2", f_and2); types["and2"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode2I("or", f_or2); types["or"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode2I("or2", f_or2); types["or2"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode2I("nand", f_nand2); types["nand"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode2I("nand2", f_nand2); types["nand2"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode2I("nor", f_nor2); types["nor"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode2I("nor2", f_nor2); types["nor2"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode2I("xor", f_xor2); types["xor"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode2I("xor2", f_xor2); types["xor2"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode2I("xnor", f_xnor2);  types["xor"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode2I("xnor2", f_xnor2); types["xnor2"] = ds_library::XNOR;
	list.push_back(node);

	node = new LGNode3I("and3", f_and3); types["and3"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode3I("or3",f_or3); types["or"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode3I("nand3",f_nand3); types["nand"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode3I("nor3",f_nor3); types["nor"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode3I("xor3",f_xor3);  types["xor"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode3I("xnor3",f_xnor3);  types["xor"] = ds_library::XOR;
	list.push_back(node);

	node = new LGNode4I("and4",f_and4); types["and4"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode4I("or4",f_or4); types["or4"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode4I("nand4",f_nand4); types["nand4"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode4I("nor4",f_nor4); types["nor4"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode4I("xor4",f_xor4); types["xor4"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode4I("xnor4",f_xnor4); types["xnor4"] = ds_library::XOR;
	list.push_back(node);

	node = new LGNode5I("and5",f_and5); types["and5"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode5I("or5",f_or5); types["or5"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode5I("nand5",f_nand5); types["nand5"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode5I("nor5",f_nor5); types["nor5"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode5I("xor5",f_xor5); types["xor5"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode5I("xnor5",f_xnor5); types["xnor5"] = ds_library::XOR;
	list.push_back(node);

	node = new LGNode6I("and6",f_and6); types["and6"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode6I("or6",f_or6); types["or6"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode6I("nand6",f_nand6); types["nand6"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode6I("nor6",f_nor6); types["nor6"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode6I("xor6",f_xor6); types["xor6"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode6I("xnor6",f_xnor6); types["xnor6"] = ds_library::XOR;
	list.push_back(node);

	node = new LGNode7I("and7",f_and7); types["and7"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode7I("or7",f_or7); types["or7"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode7I("nand7",f_nand7); types["nand7"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode7I("nor7",f_nor7); types["nor7"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode7I("xor7",f_xor7); types["xor7"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode7I("xnor7",f_xnor7); types["xnor7"] = ds_library::XOR;
	list.push_back(node);

	node = new LGNode8I("and8",f_and8); types["and8"] = ds_library::AND;
	list.push_back(node);
	node = new LGNode8I("or8",f_or8); types["or8"] = ds_library::OR;
	list.push_back(node);
	node = new LGNode8I("nand8",f_nand8); types["nand8"] = ds_library::NAND;
	list.push_back(node);
	node = new LGNode8I("nor8",f_nor8); types["nor8"] = ds_library::NOR;
	list.push_back(node);
	node = new LGNode8I("xor8",f_xor8); types["xor8"] = ds_library::XOR;
	list.push_back(node);
	node = new LGNode8I("xnor8",f_xnor8); types["xnor8"] = ds_library::XOR;
	list.push_back(node);

	node = new LGNode3I("mux2",f_mux2);
	list.push_back(node);

	node = new LGState("FD2");
	list.push_back(node);

	// fill up prototype registry
	typedef std::list<ds_lg::LGNode*>::iterator IT;
	for (IT it=list.begin();it!=list.end();it++){
		LGNode *n = *it;
		prototypes[n->get_type()] = n;
	}

}

ds_library::Library* ds_library::LibraryFactory::load_library(const std::string& name){
	Library *lib = 0;
	library_map_t::const_iterator it = map.find(name);
	if (it != map.end())
		lib = it->second;
	else {
		const char* d = getenv("DS");
		std::string path = d?d:"";
		path += "/gate_lib/" + name;
		try {
			lib = new Library(path);
			map[name] = lib;
		}
		catch (ds_common::parse_error &ex){
			std::string const * msg  = boost::get_error_info<ds_common::errmsg_info>(ex);
			std::cout << "Error:" << *msg << std::endl;
		}
	}
	return lib;
}

void ds_library::Library::load(const std::string &lib_name){

	std::ifstream input(lib_name.c_str());
	std::string line;
	if (input.is_open()){

		int line_num = 0;
		while (!input.eof()){

			std::getline(input ,line);
			if (!line.empty()) {
				bool p = parse_library<std::string::iterator>(line.begin(), line.end(), gate_map, prototypes, types, functions, inversion);
				if (!p){

					boost::algorithm::trim(line);
					if (line[0]!='#') {
						std::cout << "Error parsing library file: " << lib_name << ":" << line << std::endl;
						BOOST_THROW_EXCEPTION(ds_common::parse_error()
						<< ds_common::errmsg_info("Error parsing library file: " + lib_name + ":" + line));
					}
				}
			}
			line_num++;
		}
	} else {
		std::cout << "error loading library " << std::endl;
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
}

void ds_library::Library::close(){
	typedef std::map<std::string, ds_lg::LGNode*>::const_iterator IT;
	for (IT it = prototypes.begin(); it!=prototypes.end();it++){
		std::string name = it->first;
		ds_lg::LGNode* n = it->second;
		delete(n);
	}
	prototypes.clear();
}

bool ds_library::parse_verilog(const std::string& file, std::vector<parse_netlist>& netlists){

	namespace spirit = boost::spirit;

	std::ifstream input(file.c_str());
	input.unsetf(std::ios::skipws);
	bool parse = false;

	if (input.is_open()){

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_library::nxp_verilog_parser<spirit::istream_iterator> parser;

		parse =  boost::spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, netlists);

	} else {
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
	return parse;
}

ds_structural::NetList* ds_library::import(const std::string& file, const std::string& toplevel, ds_workspace::Workspace *workspace){

	if (file.find_last_of(".v") != file.size() - 1){
		BOOST_THROW_EXCEPTION(ds_common::parse_error()
		<< ds_common::errmsg_info("Only verilog design supported at the moment"));
	}

	ds_structural::NetList *netlist = 0;
	std::vector<parse_netlist> netlists;
	// parse file and create intermediate netlist representations
	bool parse = ds_library::parse_verilog(file, netlists);

	if (!parse) {
		BOOST_THROW_EXCEPTION(ds_common::parse_error()
				<< ds_common::errmsg_info("Error parsing verilog file"));
	}

	auto top = std::find_if(netlists.begin(), netlists.end(), bind(&parse_netlist::nl_name, _1) == toplevel);
	if (top == netlists.end()){
		std::vector<parse_netlist>::iterator it = netlists.begin();
		BOOST_THROW_EXCEPTION(ds_common::parse_error()
				<< ds_common::errmsg_info("Design " + toplevel + " not found"));
	}

	//Start of dependency check
	typedef std::pair<std::string,std::size_t> dependency_type;
	dependency_visitor dependency_v(workspace);
	dependency_type top_level = dependency_type(top->nl_name, top->ports.size());
	dependency_v.design = top_level;
	std::set<dependency_type> evaluated;
	for ( ds_library::verilog_instance instance: top->instances)
	{
		boost::apply_visitor(dependency_v, instance);
	}
	evaluated.insert(top_level);
	for( dependency_type dp: dependency_v.dependencies )
	{
		std::cout << "dependencies "  << dp.first << dp.second << std::endl;
	}

	if (dependency_v.dependencies.size()==0){

		// no dependencies found: create netlist
		netlist = ds_library::convert(*top, workspace);

	} else {


		std::set<dependency_type> to_evaluate;
		to_evaluate.insert(dependency_v.dependencies.begin(), dependency_v.dependencies.end());
		dependency_v.dependencies.clear();

		//create dependency list one pass at a time
		while (to_evaluate.size()!=0){

			// find all pending dependencies
			for (dependency_type dep: to_evaluate)
			{
				auto parsed = std::find_if(netlists.begin(), netlists.end(),
						[&] (const parse_netlist& n) { return (n.nl_name == dep.first) && (n.ports.size()==dep.second);});

				if (parsed == netlists.end()){
					std::cout << "Design " + toplevel + " not found" << std::endl;
					BOOST_THROW_EXCEPTION(ds_common::parse_error()	<< ds_common::errmsg_info("Design " + toplevel + " not found"));
				}

				// dependency exists in workspace. Query new pending dependencies
				dependency_v.design = dep;
				for (ds_library::verilog_instance instance: parsed->instances)
				{
					boost::apply_visitor(dependency_v, instance);
				}
				for (dependency_type new_dep: dependency_v.dependencies)
				{
					auto dep_it = evaluated.find(new_dep);
					if (dep_it == evaluated.end()){
						// add to list if there are still pending dependencies
						to_evaluate.insert(*dep_it);
					}
				}
			}
		}

		// topological sort to resolve dependencies
		using namespace boost;
		typedef adjacency_list<vecS, vecS, bidirectionalS, std::pair<std::string, int> > Graph;
		std::map<dependency_type, int> node_map;
     	Graph g(evaluated.size());
     	int node_counter = 0;
     	std::vector<dependency_type> evaluated_indexed(evaluated.size());
     	evaluated_indexed.insert(evaluated_indexed.end(), evaluated.begin(), evaluated.end());
     	for (std::set<dependency_type>::iterator it=evaluated.begin();it!=evaluated.end();it++){
     		node_map[*it] = node_counter++;
     	}
     	typedef graph_traits<Graph>::vertex_descriptor Vertex;
		for (auto dep_multi = dependency_v.dep_map.begin(); dep_multi != dependency_v.dep_map.end();dep_multi++){
			Vertex u, v;
			u = vertex(node_map[dep_multi->first], g);
			v = vertex(node_map[dep_multi->second], g);
			add_edge(u, v, g);
		}

		std::list<Vertex>  elaboration_order;
		topological_sort(g, std::front_inserter(elaboration_order));
		for (std::list<Vertex>::iterator it = elaboration_order.begin(); it != elaboration_order.end();it++){
			int key = *it;
			dependency_type dp = evaluated_indexed[key];
			workspace->elaborate_netlist(dp.first, dp.second);

		}

		//all dependencies resolved by now: create netlist
		netlist = ds_library::convert(*top, workspace);
	}

	return netlist;
}

ds_structural::NetList* ds_library::convert(const ds_library::parse_netlist& nl, ds_workspace::Workspace *workspace){
	// allocate netlist
	ds_structural::NetList *netlist = new ds_structural::NetList();
	netlist->set_instance_name(nl.nl_name);
	ds_library::aggregate_visitor aggregate_v(netlist, ds_structural::DIR_IN);
	ds_library::instance_visitor instance_v(netlist, workspace);
	for ( ds_library::verilog_declaration p: nl.inputs )
	{
		boost::apply_visitor(aggregate_v, p);				// create inputs
	}
	aggregate_v.set_port_type(ds_structural::DIR_OUT);
	for ( ds_library::verilog_declaration p: nl.outputs )
	{
		boost::apply_visitor(aggregate_v, p);				// create outputs
	}
	for ( ds_library::verilog_declaration p: nl.inouts )
	{
		boost::apply_visitor(aggregate_v, p);				// create bidireccional ports
	}
	aggregate_v.create_port = false;
	for ( ds_library::verilog_declaration p: nl.signals )
	{
		boost::apply_visitor(aggregate_v, p);				// create signals
	}
	for ( ds_library::verilog_instance instance: nl.instances )
	{
		boost::apply_visitor(instance_v, instance);			// create gates
	}
	for ( ds_library::parse_nl_assignment assignment: nl.assignments )
	{
		// handle assignments
		ds_structural::Signal *s_lhs = netlist->find_signal(assignment.lhs);
		ds_structural::Signal *s_rhs = netlist->find_signal(assignment.rhs);
		if (s_lhs == 0 || s_rhs == 0){
			std::cout << "Warning: assignment signals not found: " <<  assignment.lhs << " <= " << assignment.rhs << std::endl;
		} else {
			// instantiate a buffer and setup inputs and outputs
			ds_structural::Gate *g = workspace->get_gate("buf", 2);
			//TODO Ask the library (workspace) for the port names
			ds_structural::PortBit* in = g->find_port_by_name("i1");
			ds_structural::PortBit* out = g->find_port_by_name("o1");
			g->set_instance_name("assign_" + s_lhs->get_instance_name() + "_" + s_rhs->get_instance_name());
			in->set_signal(s_rhs);
			s_rhs->add_port(in);
			out->set_signal(s_lhs);
			s_lhs->add_port(out);
			netlist->add_gate(g);
			g->set_parent(netlist);
		}
	}

	netlist->remove_floating_signals();

	return netlist;
}

void ds_library::instance_visitor::operator()(const ds_library::parse_nl_implicit_instance& implicit){
	std::string type = implicit.type;
	std::size_t numPorts = implicit.ports.size();
	// first search for a simple gate
	ds_structural::Gate *g = wp->get_gate(type,numPorts);
	if (g!=0){
		// gate found
		g->set_instance_name(implicit.name);
		netlist->add_gate(g);
		g->set_parent(netlist);

		ds_structural::port_container all_ports;
		ds_structural::port_container::iterator port_iterator = all_ports.end();
		all_ports.insert(port_iterator, g->get_outputs()->begin(),g->get_outputs()->end());
		port_iterator = all_ports.end();
		all_ports.insert(port_iterator, g->get_inputs()->begin(),g->get_inputs()->end());
		ds_structural::port_container::iterator pi = all_ports.begin();
		// handle ports in order
		for (auto it = implicit.ports.begin();it!=implicit.ports.end();it++){
			ds_structural::PortBit* pb = *pi;
			pi++;
			std::string signal_name = *it;
			ds_structural::Signal *signal = netlist->find_signal(signal_name);
			if (signal==0){
				// create signal if it does not exist
				signal = new ds_structural::Signal(signal_name);
				netlist->add_signal(signal);
			}
			signal->add_port(pb);
			pb->set_signal(signal);
		}
	} else {
		// search for a netlist
		ds_structural::NetList *nl = wp->get_netlist(type,numPorts);
		ds_structural::port_container all_ports;
		ds_structural::port_container::iterator port_iterator = all_ports.end();
		all_ports.insert(port_iterator, g->get_outputs()->begin(),g->get_outputs()->end());
		port_iterator = all_ports.end();
		all_ports.insert(port_iterator, nl->get_inputs()->begin(),nl->get_inputs()->end());
		ds_structural::port_container::iterator pi = all_ports.begin();
		// handle ports in order
		for (auto it = implicit.ports.begin();it!=implicit.ports.end();it++){
			ds_structural::PortBit* pb = *pi;
			pi++;
			std::string signal_name = *it;
			ds_structural::Signal *signal = netlist->find_signal(signal_name);
			if (signal==0){
				signal = new ds_structural::Signal(signal_name);
				netlist->add_signal(signal);
				if (signal_name == value_0){
					signal->set_value(ds_simulation::BIT_0);
				}
				else if (signal_name == value_1){
					signal->set_value(ds_simulation::BIT_1);
				}
				else if (signal_name == value_X){
					signal->set_value(ds_simulation::BIT_X);
				}
			}
			//connect port signals to the the top level netlist
			ds_structural::Signal *internal = pb->get_signal();
			for (auto inter_it=internal->ports.begin();inter_it!= internal->ports.end();inter_it++){
				ds_structural::PortBit *p = *inter_it;
				p->set_signal(signal);
			}
		}

		// include hierarchical gates with qualified name
		for (auto gi = nl->gates.begin(); gi != nl->gates.end(); gi++){
			ds_structural::Gate *gate = gi->second;
			gate->set_instance_name(implicit.name + "/" + gate->get_instance_name());
			netlist->add_gate(gate);
			gate->set_parent(netlist);
		}
		nl->gates.clear();

		// include hierarchical signals with qualified name
		for (ds_structural::signal_map_t::iterator si = nl->signals.begin();si!=nl->signals.end();si++){
			ds_structural::Signal *s = si->second;
			s->set_name(implicit.name + "/" + s->get_instance_name());
			netlist->add_signal(s);
		}
		nl->signals.clear();
		// include hierarchical internal signals with qualified name
		for (ds_structural::signal_map_t::iterator si = nl->own_signals.begin();si!=nl->own_signals.end();si++){
			ds_structural::Signal *s = si->second;
			s->set_name(implicit.name + "/" + s->get_instance_name());
			netlist->add_signal(s);
		}
		nl->own_signals.clear();

		delete nl;
	}
}

void ds_library::instance_visitor::operator()(const ds_library::parse_nl_explicit_instance& instance) {
	std::string type = instance.type;
	std::vector<std::string> ports;
	for (std::map<std::string, std::string>::const_iterator it = instance.ports.begin();it!=instance.ports.end();it++){
		ports.push_back(it->first);
	}
	//search for gate
	ds_structural::Gate *g = wp->get_gate(type, ports);
	if (g!=0){
		//gate found
		g->set_instance_name(instance.name);
		netlist->add_gate(g);
		g->set_parent(netlist);
		ds_structural::port_container all_ports;
		ds_structural::port_container::iterator port_iterator = all_ports.end();
		all_ports.insert(port_iterator, g->get_outputs()->begin(),g->get_outputs()->end());
		port_iterator = all_ports.end();
		all_ports.insert(port_iterator, g->get_inputs()->begin(),g->get_inputs()->end());
		ds_structural::port_container::iterator pi = all_ports.begin();
		//handle ports
		for (auto it = instance.ports.begin();it!=instance.ports.end();it++){
			std::string formal = it->first;
			std::string actual = it->second;

			ds_structural::PortBit* pb = g->find_port_by_name(formal);
			ds_structural::Signal *signal = netlist->find_signal(actual);
			if (signal==0){
				signal = new ds_structural::Signal(actual);
				if (actual == value_0){
					signal->set_value(ds_simulation::BIT_0);
				}
				else if (actual == value_1){
					signal->set_value(ds_simulation::BIT_1);
				}
				else if (actual == value_X){
					signal->set_value(ds_simulation::BIT_X);
				}

				netlist->add_signal(signal);
			}
			signal->add_port(pb);
			pb->set_signal(signal);
		}
	} else {
		BOOST_THROW_EXCEPTION(ds_common::parse_error()
				<< ds_common::errmsg_info("No gate found" + type + ". Hierarchical explicit instances not supported yet... "));
	}
}

void ds_library::dependency_visitor::operator()(const ds_library::parse_nl_implicit_instance& implicit){
	typedef std::pair<std::string,int> dependency_type;
	if (!wp->is_defined(implicit.type, implicit.ports.size())){
		dependency_type dep(implicit.type,implicit.ports.size());
		dep_map.insert(std::pair<dependency_type, dependency_type>(design, dep));
		dependencies.insert(dep);
	}
}

void ds_library::dependency_visitor::operator()(const ds_library::parse_nl_explicit_instance& instance) {

}

