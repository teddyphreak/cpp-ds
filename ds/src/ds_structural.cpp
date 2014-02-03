/*
 * ds_structural.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: cookao
 */
#include "ds_lg.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"
#include <queue>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/foreach.hpp>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using ds_structural::Gate;
using ds_structural::NetList;
using ds_structural::Signal;
using ds_structural::PortBit;


ds_structural::PortBit::~PortBit(){
	if (signal != 0){
		signal->remove_port(this);
	}
}

void ds_structural::Gate::copy(ds_structural::Gate* g){
	g->set_instance_name(name);
	g->set_type(type);
	for (port_container::iterator it=inputs.begin();it!=inputs.end();it++){
		PortBit *pb = *it;
		PortBit *c = new PortBit(pb->get_instance_name(), g, pb->get_type());
		g->add_port(c);
		c->set_gate(g);
	}
	for (port_container::iterator it=outputs.begin();it!=outputs.end();it++){
		PortBit *pb = *it;
		PortBit *c = new PortBit(pb->get_instance_name(), g, pb->get_type());
		g->add_port(c);
		c->set_gate(g);
	}
	typedef function_map_t::iterator IT;
	for (IT it=mappings.begin();it!=mappings.end();it++){
		g->add_mapping(it->first, it->second);
	}
}

Gate* ds_structural::Gate::clone(){
	Gate *g = new Gate();
	copy(g);
	return g;
}

NetList* ds_structural::NetList::clone(){
	NetList *nl = new NetList();
	//copy ports
	copy(nl);

	//copy gates
	for (auto gate_it = gates.begin();gate_it!=gates.end();gate_it++){
		Gate *g= gate_it->second;
		nl->add_gate(g->clone());
	}
	nl->signal_counter = signal_counter;

	//copy signals
	for (auto sig_it = signals.begin();sig_it!=signals.end();sig_it++){
		Signal *s = sig_it->second;
		Signal *s_copy = new Signal(s->get_instance_name());
		for (auto port_it = s->port_begin(); port_it != s->port_end(); port_it++){
			PortBit *port = *port_it;
			Gate *gate = port->get_gate();
			if (gate != this){
				std::string gate_name = gate->get_instance_name();
				Gate *gate_copy = nl->find_gate(gate_name);
				PortBit *targetPort = gate_copy->find_port_by_name(port->get_instance_name());
				s_copy->add_port(targetPort);
			} else {
				PortBit *targetPort = nl->find_port_by_name(port->get_instance_name());
				s_copy->add_port(targetPort);
			}
		}
		nl->add_signal(s_copy);
	}
	// copy owned signals
	for (auto sig_it = own_signals.begin();sig_it!=own_signals.end();sig_it++){
		Signal *s = sig_it->second;
		Signal *s_copy = new Signal(s->get_instance_name());
		for (auto port_it = s->port_begin(); port_it != s->port_end(); port_it++){
			PortBit *port = *port_it;
			Gate *gate = port->get_gate();
			if (gate != this){
				std::string gate_name = gate->get_instance_name();
				Gate *gate_copy = nl->find_gate(gate_name);
				PortBit *targetPort = gate_copy->find_port_by_name(port->get_instance_name());
				s_copy->add_port(targetPort);
			} else {
				PortBit *targetPort = nl->find_port_by_name(port->get_instance_name());
				s_copy->add_port(targetPort);
			}
		}
		nl->own_signals[s_copy->get_instance_name()] = s_copy;
	}
	return nl;
}

bool ds_structural::NetList::check_netlist(){

	using boost::lambda::_1;

	bool check = true;
	typedef std::map<std::string, ds_structural::PortBit*> port_map;
	typedef std::set<ds_structural::PortBit*> type_set;

	port_map input_ports;
	port_map output_ports;
	type_set internal_input_ports;
	type_set internal_output_ports;

	for (port_container::iterator it = inputs.begin();it!=inputs.end();it++){
		input_ports[(*it)->get_instance_name()] = *it;
	}
	for (port_container::iterator it = outputs.begin();it!=outputs.end();it++){
		output_ports[(*it)->get_instance_name()] = *it;
	}

	// Check for dangling signals
	for (auto it=signals.begin();it!=signals.end();it++){

		ds_structural::Signal *s = it->second;

		if(s->is_fixed())
			continue;

		using boost::lambda::_1;
		ds_structural::PortBit *null = 0;
		sp_container::const_iterator npi = std::find_if(s->port_begin(), s->port_end(), _1==null);

		if (npi!=s->port_end()){
			check = false;
			BOOST_LOG_TRIVIAL(warning) << "Error: signal connected to null port " << s->get_instance_name();
		}

		port_map::iterator in_p = input_ports.find(s->get_instance_name());
		port_map::iterator out_p = output_ports.find(s->get_instance_name());

		std::list<PortBit*>::const_iterator pin = std::find_if(s->port_begin(),s->port_end(),
			boost::lambda::bind(&ds_structural::PortBit::get_type, *_1)==ds_structural::DIR_IN);

		std::list<PortBit*>::const_iterator pout = std::find_if(s->port_begin(),s->port_end(),
			boost::lambda::bind(&ds_structural::PortBit::get_type, *_1)==ds_structural::DIR_OUT ||
			boost::lambda::bind(&ds_structural::PortBit::get_type, *_1)==ds_structural::DIR_INOUT );

		if (in_p == input_ports.end() && out_p == output_ports.end()){

			if (pin==s->port_end() ){
				std::list<PortBit*>::const_iterator opin = std::find_if(s->port_begin(),s->port_end(),
				boost::lambda::bind(&ds_structural::PortBit::get_gate, *_1)==this);
				if (opin == s->port_end()){
					check = false;
					BOOST_LOG_TRIVIAL(warning) << "No input port for " << s->get_instance_name();
				}
			}
			if (pout==s->port_end()){
				std::list<PortBit*>::const_iterator ipin = std::find_if(s->port_begin(),s->port_end(),
				boost::lambda::bind(&ds_structural::PortBit::get_gate, *_1)==this);
				if (ipin == s->port_end()){
					check = false;
					BOOST_LOG_TRIVIAL(warning) << "No output port for " << s->get_instance_name();
				}
			}
		}
		else {
			if (in_p != input_ports.end() && out_p != output_ports.end()) {
				check = false;
				BOOST_LOG_TRIVIAL(warning) << "Error: signal connected to both input and output " << s->get_instance_name();
			}
			if (out_p==output_ports.end()){
				if (pout!=s->port_end()){
					check = false;
					BOOST_LOG_TRIVIAL(warning) << "Netlist input port driven by internal port " << s->get_instance_name();
				}
			}
			if (in_p==input_ports.end()){
				if (pout==s->port_end()){
					check = false;
					BOOST_LOG_TRIVIAL(warning) << "Netlist output port not driven by internal port " << s->get_instance_name();
				}
			}
		}
	}
	for (auto it=signals.begin();it!=signals.end();it++){
		ds_structural::Signal *s = it->second;
		std::list<PortBit*>::const_iterator port_it = s->port_begin();
		std::list<PortBit*>::const_iterator end = s->port_end();
		int drivers = 0;
		for (;port_it!=end;port_it++){
			ds_structural::PortBit *pb = *port_it;
			if (pb->get_type() != ds_structural::DIR_IN){
				if (output_ports.find(pb->get_instance_name()) != output_ports.end()) {
					drivers++;
					if (drivers==2)
						break;
				}
			}
		}
		if (drivers >= 2){
			check = false;
			BOOST_LOG_TRIVIAL(warning) << "Multiple signal drivers for " << s->get_instance_name();
		}
	}
	for (auto it=signals.begin();it!=signals.end();it++){
		ds_structural::Signal *s = it->second;
		for (std::list<PortBit*>::const_iterator p=s->port_begin();p!=s->port_end();p++){
			ds_structural::Gate *g = (*p)->get_gate();
			ds_structural::gate_map_t::iterator owned = gates.find(g->get_instance_name());
			if (owned == gates.end()){
				if (g != this){
					check = false;
					BOOST_LOG_TRIVIAL(warning) << "Foreign gate" << g->get_instance_name();
				}
			}
		}
	}
	for (auto it=gates.begin();it!=gates.end();it++){
		ds_structural::Gate *g = it->second;
		ds_structural::port_container container;
		ds_structural::port_container::const_iterator in_it = g->get_inputs()->begin();
		for (;in_it!=g->get_inputs()->end();in_it++){
			const ds_structural::PortBit *pb = *in_it;
			ds_structural::Signal *s = pb->get_signal();
			auto owned = signals.find(s->get_instance_name());
			if (owned == signals.end()){
				check = false;
				BOOST_LOG_TRIVIAL(warning) << "Foreign signal " << s->get_instance_name();
			}
		}
		ds_structural::port_container::const_iterator out_it = g->get_outputs()->begin();
		for (;out_it!=g->get_outputs()->end();out_it++)
		{
			const ds_structural::PortBit *pb = *out_it;
			ds_structural::Signal *s = pb->get_signal();
			if (s==0)
				continue;

			auto owned = signals.find(s->get_instance_name());
			if (owned == signals.end()){
				check = false;
				BOOST_LOG_TRIVIAL(warning) << "Foreign signal " << s->get_instance_name();
			}
		}
	}
	return check;
}

std::string ds_structural::PortBit::get_qualified_name() const {
	const Gate *g = get_gate();
	return g->get_instance_name() + "/" + get_instance_name();
}

ds_lg::LeveledGraph* ds_structural::NetList::get_sim_graph(ds_library::Library *lib){
	std::map<Gate*,ds_lg::LogicNode*> node_map;
	for (auto it=gates.begin();it!=gates.end();it++){
		Gate *g = it->second;
		ds_lg::LogicNode *n = lib->get_primitive(g->get_type(), g->get_num_ports());
		node_map[g] = n;
	}
	ds_lg::LeveledGraph *lg = new ds_lg::LeveledGraph();
	ds_lg::Input in;
	ds_lg::Output out;
	build_leveled_graph(lg,node_map,in,out);
	bool c = lg->sanity_check();
	if (!c){
		BOOST_LOG_TRIVIAL(warning) << "Sanity check failed";
	}
	return lg;
}

template<class V>
void ds_structural::NetList::build_leveled_graph(ds_lg::LeveledGraphBuilder* builder, std::map<Gate*,V*>& node_map,
		const ds_lg::Input& input_prototype, const ds_lg::Output& output_prototype){

	using ds_lg::LGNode;
	using ds_structural::PortBit;
	using ds_structural::Gate;

	builder->set_netlist(this);

	std::vector<Gate*> state_gates;
	std::stack<Gate*> todo;

	for (auto gate_it = node_map.begin();gate_it!=node_map.end();gate_it++){
		Gate *g = gate_it->first;
		V *n = gate_it->second;
		n->set_gate(g);					// hook every LGN to its corresponding gate
		if (n->has_state()){
			state_gates.push_back(g);
			n->level = 0;				// registers have depth 0
			todo.push(g);
		}
	}

	std::map<std::string, Gate*> trace;
	std::list<V*> level_list;

	std::vector<V*> lg_outputs;
	for (ds_structural::PortBit* pb: outputs)
	{
		ds_lg::Output *out = output_prototype.clone();	// create output
		out->set_gate(this);						// outputs ports belong to the enclosing netlist
		out->set_name(pb->get_instance_name());
		level_list.push_back(out);				// this node is traced back
		builder->add_output(out);
		lg_outputs.push_back(out);
		Signal *s = pb->get_signal();
		for (ds_structural::sp_container::const_iterator pi=s->port_begin();pi!=s->port_end();pi++){
			PortBit *d = *pi;
			if (d->get_type() == ds_structural::DIR_OUT && d!=pb){
				V *driverNode = node_map[d->get_gate()];	// locate LGN of driving gate
				out->inputs.push_back(driverNode);				// update LGN's inputs and outputs
				driverNode->outputs.push_back(out);
				ds_lg::lg_v64** in = out->get_input("a");		//primitive inputs are 'a'
				std::string m = d->get_gate()->get_mapping(d->get_instance_name());
				ds_lg::lg_v64 *driver = driverNode->get_output(m);
				*in = driver;
			}
		}
	}

	std::vector<V*> lg_inputs;
	for (ds_structural::PortBit* pb: inputs)
	{
		ds_lg::Input *in = input_prototype.clone();				// create input
		in->set_gate(this);									// outputs ports belong to the enclosing netlist
		in->set_name(pb->get_instance_name());
		builder->add_input(in);
		lg_inputs.push_back(in);
		in->level = 0;										// inputs have depth 0
		V* t = in;
		ds_lg::lg_v64 *driver = in->get_output("o");		//primitive outputs are 'o'
		trace_lg_forward(pb, t, driver, &trace, &todo, node_map);	//trace inputs forward
		Signal *s = pb->get_signal();
		if (s!=0) {
			for (auto pi=s->port_begin();pi!=s->port_end();pi++){	// setup input and output nodes
				PortBit *pb = *pi;
				if (pb->get_type() == ds_structural::DIR_IN){
					V* receiver = node_map[pb->get_gate()];
					if (receiver!=0){
						in->outputs.push_back(receiver);
						receiver->inputs.push_back(in);
					}
				}
			}
		}
	}
	// continue forward trace with gates not considered so far
	while (!todo.empty()){
		Gate *g = todo.top();
		todo.pop();
		V* lgn = node_map[g];
		for (auto pi=g->get_outputs()->begin();pi!=g->get_outputs()->end();pi++){
			PortBit *pb = *pi;
			std::string port_name = g->get_mapping(pb->get_instance_name());
			ds_lg::lg_v64 *driver = lgn->get_output(port_name);
			trace_lg_forward(pb, lgn, driver, &trace, &todo, node_map);
		}
	}

	// include state inputs in level list
	for (ds_structural::Gate *g: state_gates)
	{
		V* lgn = node_map[g];
		for (V* in: lgn->inputs)
		{
			if (!in->has_state())
				level_list.push_back(in);
		}
	}
	// level gates: push them onto stack till a levelized node is found. Then calculate the levels for the nodes in between
	for (V *lgn: level_list)
	{
		std::stack<V*> st;
		st.push(lgn);
		while (!st.empty()){

			V *t = st.top();

			int max_level = 0;
			std::size_t stack_size = st.size();
			for (V *in: t->inputs)
			{
				if (in->level < 0){
					st.push(in);
				} else {
					if (in->level >= max_level){
						max_level = in->level;
					}
				}
			}
			if (stack_size == st.size()){
				t->level = max_level + 1;
				st.pop();
			}
		}
	}

	//calculate max level
	int max_level = 0;
	for (V* out:lg_outputs){
		if (out->level > max_level){
			max_level = out->level;
		}
	}
	builder->set_levels(max_level + 1);

	// build level map and fill level map with inputs and outputs
	std::map<int, std::set<V*>* > level_map;
	for (int i=0;i<max_level+1;i++){
		level_map[i] = new std::set<V*>();
	}
	for (V *in: lg_inputs)
	{
		level_map[in->level]->insert(in);
	}
	for (V *out: lg_outputs)
	{
		level_map[out->level]->insert(out);
	}
	// fill level map with traced nodes
	for (std::map<std::string, Gate*>::iterator gi = trace.begin(); gi!=trace.end();gi++){
		V *lgn = node_map[gi->second];
		if (lgn->level > 0) {
			level_map[lgn->level]->insert(lgn);

		}
	}
	// add nodes in level order to the nodes container
	for (int i=0;i<max_level+1;i++){
		std::set<V*> *set = level_map[i];
		typename std::set<V*>::iterator it = set->begin();
		for (;it!=set->end();it++){
			V *n = *it;
			builder->add_node(n);
		}
		delete set;
	}
	for  (int i=0;i<max_level+1;i++){
		ds_lg::lg_node_container::iterator it = std::find_if(builder->get_nodes_begin(), builder->get_nodes_end(),
				[&] (const V* n) { return n->level == i;});
		builder->push_level_iterator(it);
	}

	//set up intermediate simulation vectors
	builder->create_simulation_level(max_level+1);

	// count nodes per level
	for (int i=0;i<max_level+1;i++){
		auto cIt = builder->get_level_iterator(i);
		auto nIt = builder->get_nodes_end();
		if (i<max_level)
			nIt = builder->get_level_iterator(i+1);
		unsigned int level_size = 0;
		while (cIt!=nIt){
			level_size++;
			cIt++;
		}

		builder->push_level_width(level_size);
	}

	level_map.clear();

	//drive constant '0' assignments
	Signal *s0 = find_signal(ds_library::value_0);
	if (s0!=0){
		for (auto pi=s0->port_begin();pi!=s0->port_end();pi++){
			drive<V>(*pi, builder->get_constant_0(), node_map);
		}

	}
	//drive constant '1' assignments
	Signal *s1 = find_signal(ds_library::value_1);
	if (s1!=0){
		for (auto pi=s1->port_begin();pi!=s1->port_end();pi++){
			drive<V>(*pi, builder->get_constant_1(), node_map);
		}
	}
	//identify state elements
	for (auto it = builder->get_nodes_begin();it!=builder->get_nodes_end();it++){
		V *n = *it;
		if (n->has_state())
			builder->add_register(n);
	}

}

template <class V, class T>
void ds_structural::NetList::drive(PortBit*pb, T *driver, std::map<Gate*,V*>& node_map){

	// get receiver simulation node
	Gate *g = pb->get_gate();
	std::string m = g->get_mapping(pb->get_instance_name());
	V *target = node_map[g];
	// get the simulation input
	ds_lg::lg_v64** rec = target->get_input(m);
	// connect input to output
	*rec = driver;
}

template <class V, class T>
void ds_structural::NetList::trace_lg_forward(const ds_structural::PortBit *port_bit, V* node,
		T *driver, std::map<std::string, Gate*> *trace, std::stack<Gate*> *todo, std::map<Gate*,V*>& node_map){

	using ds_lg::LGNode;
	using ds_structural::PortBit;
	using ds_structural::Gate;
	using ds_structural::Signal;

	Signal *s = port_bit->get_signal();

	// no signal attached
	if (s==0) {
		return;
	}

	// trace all input ports attached to the driving port
	for (auto pi=s->port_begin();pi!=s->port_end();pi++){
		PortBit *pb = *pi;
		if (pb->get_type() == ds_structural::DIR_IN && pb != port_bit){

			//connect simulation output to input
			drive<V>(pb, driver, node_map);

			// if receiver gate has not been traced, schedule it for tracing
			Gate *g = pb->get_gate();
			V *target = node_map[g];
			auto ti = trace->find(g->get_instance_name());
			if (ti==trace->end()){
				todo->push(g);
				(*trace)[g->get_instance_name()] = g;
			}

			//setup input and output nodes for the driver and receiver
			auto oi = std::find(node->outputs.begin(), node->outputs.end(), target);
			if (oi == node->outputs.end()){
				node->outputs.push_back(target);
				target->inputs.push_back(node);
			}
		}
	}

}

void ds_structural::NetList::get_output_cone(ds_structural::PortBit *pb, std::set<ds_structural::PortBit*> *cone){

	using ds_structural::PortBit;
	using ds_structural::Gate;
	using ds_structural::Signal;

	std::stack<PortBit*> st;
	// if provided port is an input, push the outputs of the same gate
	if(pb->get_type() == ds_structural::DIR_IN){
		Gate *g = pb->get_gate();
		for (auto pb_it = g->get_outputs()->begin(); pb_it!=g->get_outputs()->end();pb_it++){
			st.push(*pb_it);
		}
	} else {
		st.push(pb);
	}

	// trace forward until there are no more gates
	while (!st.empty()){

		PortBit *port = st.top();
		st.pop();
		Signal *s = port->get_signal();

		//check all connections
		for (auto it = s->port_begin();it!=s->port_end();it++){
			PortBit *receiver_port = *it;
			Gate *receiver_gate = receiver_port->get_gate();
			if (receiver_gate == this) {
				// we have reached the top level netlist. (pseudo) primary output found
				cone->insert(receiver_port);
			} else {
				if (receiver_port->get_type() == ds_structural::DIR_IN){
					for (auto op_it = receiver_gate->get_outputs()->begin();
							op_it != receiver_gate->get_outputs()->end();op_it++) {
						//trace the output ports of this gate we have just reaches
						st.push(*op_it);
					}
				}
			}
		}
	}
}

void ds_structural::NetList::remove_floating_signals(){

	auto it = signals.begin();
	while (it!=signals.end()){
		Signal *s = it->second;
		if (s->count_ports() == 0){
			auto del = it++;
			delete(s);
			signals.erase(del);

		} else {
			++it;
		}
	}
}

bool ds_structural::NetList::remove_unused_gates(){

	auto it = gates.begin();
	bool remove = false;
	//check all gates
	while (it!=gates.end()){
		Gate *g = it->second;

		//check if gate is attached: if any of its output ports has more than one connection
		bool isAttached = false;
		const ds_structural::port_container *outputs = g->get_outputs();
		for (auto it=outputs->begin(); it!=outputs->end();it++ ){
			PortBit *pb = *it;
			Signal *spb = pb->get_signal();

			if (spb->count_ports() > 1){
				isAttached = true;
				break;
			}
		}

		if (!isAttached){

			//gate is NOT attached: schedule for removal
			remove = true;
			auto del = it++;
			const ds_structural::port_container *inputs = g->get_inputs();

			for (auto del=inputs->begin(); del!=inputs->end();del++ ){
				PortBit *pb = *del;
				Signal *spb = pb->get_signal();
				if (spb->count_ports() == 1){
					// remove floating signal
					remove_signal(spb);
					delete(spb);
				} else if (spb->count_ports() == 2){
					// check if driver gate is also unused
					std::vector<const Gate*> to_remove;
					find_unused_gates(pb, &to_remove);
					for(const Gate* r:to_remove){
						remove_gate(r);
						delete(r);
					}

				}
			}

			// remove floating signals in the output ports
			for (auto del=outputs->begin(); del!=outputs->end();del++ ){
				PortBit *pb = *del;
				Signal *spb = pb->get_signal();
				if (spb->count_ports() == 1){
					remove_signal(spb);
					delete(spb);
				}
			}

			//remove gate
			gates.erase(del);
			delete(g);

		} else {
			++it;
		}
	}
	return remove;
}


void ds_structural::NetList::find_unused_gates(const PortBit *pb, std::vector<const Gate*> *unused){

	// initialize stack with provided netlist
	std::stack<const PortBit*> todo;
	todo.push(pb);

	while (!todo.empty()){
		const PortBit *current = todo.top();
		todo.pop();
		Signal *s = current->get_signal();

		//proceed only if port is connected to one other port
		if (s->count_ports() == 2) {

			// find driver port at the other end
			PortBit*  driver = *(s->port_begin());
			if (driver == current){
				driver = *(++s->port_begin());
			}

			// remove signal
			remove_signal(s);
			delete(s);

			// potentially unused gate
			Gate *driver_gate = driver->get_gate();

			// watch out for unused ports in the netlist
			if(driver_gate != this) {
				//unused gate
				unused->push_back(driver_gate);
				// trace the inputs of the drver gate to check if the are unused
				for (auto del=driver_gate->get_inputs()->begin(); del!=driver_gate->get_inputs()->end();del++ ){
					PortBit *in = *del;
					todo.push(in);
				}
			} else {
				// unused port
				driver->disconnect();
			}
		}
	}
}

void ds_structural::save_netlist(const std::string& file, ds_structural::NetList *nl){
	std::ofstream ofs(file);
	boost::archive::binary_oarchive oa(ofs);
	oa << nl;
}

ds_structural::NetList* ds_structural::load_netlist(const std::string& file, ds_workspace::Workspace *wp){
	NetList *nl = 0;
	std::ifstream ifs(file);
	boost::archive::binary_iarchive ia(ifs);
	ia >> nl;

	//Fix gate memeber of PortBit
	//Not serialized for circular serialization dependency
	for (PortBit *pb: nl->inputs){
		pb->set_gate(nl);
	}
	for (PortBit *pb: nl->outputs){
		pb->set_gate(nl);
	}
	for (auto it=nl->gates.begin();it!=nl->gates.end();it++){
		Gate* g = it->second;
		for (PortBit *pb: g->inputs){
			pb->set_gate(g);
		}
		for (PortBit *pb: g->outputs){
			pb->set_gate(g);
		}
	}
	return nl;
}

std::string ds_structural::get_implicit_instantiation(ds_structural::Gate* g){
	std::string instance = g->get_type() + "\t" + g->get_instance_name() + " ( ";
	ds_structural::port_container ports;
	const ds_structural::port_container* outputs = g->get_outputs();
	const ds_structural::port_container* inputs = g->get_inputs();
	ports.insert(ports.begin(), inputs->begin(), inputs->end());
	ports.insert(ports.end(), outputs->begin(), outputs->end());
	int num_ports = g->get_num_ports();
	int ports_it = 0;
	for (auto it=ports.begin();it!=ports.end();it++){
		ports_it++;
		const ds_structural::PortBit *pb = *it;
		instance += pb->get_signal()->get_instance_name();
		if (ports_it < num_ports){
			instance += ", ";
		}
	}
	instance += " );";
	return instance;
}

bool ds_structural::parse_nxp_chain_info(const std::string& file, std::vector<ScanPort>& ports){

	namespace spirit = boost::spirit;

	std::ifstream input(file.c_str());
	bool parse = false;

	if (input.is_open()){

		spirit::istream_iterator begin(input);
		spirit::istream_iterator end;

		ds_structural::nxp_chain_info_parser<spirit::istream_iterator> parser;

		parse =  boost::spirit::qi::phrase_parse(begin, end, parser, spirit::ascii::space, ports);

	} else {
		BOOST_LOG_TRIVIAL(error) << "Error parsing chain info file: " << file << ". Device not open";
		BOOST_THROW_EXCEPTION(ds_common::file_read_error() << boost::errinfo_errno(errno));
	}
	return parse;
}

ds_structural::CombinationalScanMap* ds_structural::get_combinational_scan_map(const std::string& file_name){
	std::vector<ds_structural::ScanPort> ports;
	bool parse = ds_structural::parse_nxp_chain_info(file_name, ports);
	if (parse){
		ds_structural::CombinationalScanMap* map = new ds_structural::CombinationalScanMap(ports.begin(), ports.end());
		return map;
	} else {
		return 0;
	}
}

ds_structural::NetList::~NetList(){

	for (signal_map_t::iterator it=signals.begin();it!=signals.end();it++){
		it->second->detach();
		delete it->second;
	}
	for (signal_map_t::iterator it=own_signals.begin();it!=own_signals.end();it++){
		delete it->second;
	}
	for (gate_map_t::iterator it=gates.begin();it!=gates.end();it++){
		delete it->second;
	}

}

ds_structural::Gate::~Gate(){
	for (PortBit* in: inputs){
		delete(in);
	}
	for (PortBit* out: outputs){
		delete(out);
	}
}

