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
	g->set_lgn(lgn->clone());
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
		Signal *copy = new Signal(s->get_instance_name());
		for (auto port_it = s->port_begin(); port_it != s->port_end(); port_it++){
			PortBit *port = *port_it;
			std::string gate_name = port->get_gate()->get_instance_name();
			Gate *g = find_gate(name);
			PortBit *targetPort = g->find_port_by_name(port->get_instance_name());
			copy->add_port(targetPort);
		}
		nl->add_signal(copy);
	}
	// copy owned signals
	for (auto sig_it = own_signals.begin();sig_it!=own_signals.end();sig_it++){
		Signal *s = sig_it->second;
		Signal *copy = new Signal(s->get_instance_name());
		for (auto port_it = s->port_begin(); port_it != s->port_end(); port_it++){
			PortBit *port = *port_it;
			std::string gate_name = port->get_gate()->get_instance_name();
			Gate *g = find_gate(name);
			PortBit *targetPort = g->find_port_by_name(port->get_instance_name());
			copy->add_port(targetPort);
		}
		nl->own_signals[copy->get_instance_name()] = copy;
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
				check = false;
				BOOST_LOG_TRIVIAL(warning) << "No input port for " << s->get_instance_name() << std::endl;
			}
			if (pout==s->port_end()){
				check = false;
				BOOST_LOG_TRIVIAL(warning) << "No output port for " << s->get_instance_name() << std::endl;
			}
		}
		else {
			if (in_p != input_ports.end() && out_p != output_ports.end()) {
				check = false;
				std::cout << "Error: signal connected to both input and output " << s->get_instance_name();
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

ds_lg::LeveledGraph* ds_structural::NetList::clone_leveled_graph(){

	if (lg==0)
		return 0;

	lg = 0;
	for (auto gate_it = gates.begin();gate_it!=gates.end();gate_it++){
		Gate* g = gate_it->second;
		ds_lg::LGNode* n = g->get_lgn();
		g->set_lgn(n->clone());
	}
	return build_leveled_graph();
}

ds_lg::LeveledGraph* ds_structural::NetList::build_leveled_graph(){
	using ds_lg::LGNode;
	using ds_structural::PortBit;
	using ds_structural::Gate;

	if (lg!=0)
		return lg;

	lg = new ds_lg::LeveledGraph();  	// return value
	lg->nl = this;
	lg->iteration = 0;
	std::vector<Gate*> state_gates;
	std::stack<Gate*> todo;

	for (auto gate_it = gates.begin();gate_it!=gates.end();gate_it++){
		Gate* n = gate_it->second;
		n->get_lgn()->set_gate(n);			// hook every LGN to its corresponding gate
		if (n->get_lgn()->has_state()){
			state_gates.push_back(n);
			n->get_lgn()->level = 0;		// registers have depth 0
			todo.push(n);
		}
	}

	std::map<std::string, Gate*> trace;
	std::list<LGNode*> level_list;

	for (ds_structural::PortBit* pb: outputs)
	{
		ds_lg::Output *out = new ds_lg::Output();	// create output
		out->set_gate(this);						// outputs ports belong to the enclosing netlist
		out->set_name(pb->get_instance_name());
		level_list.push_back(out);				// this node is traced back
		lg->add_output(out);
		Signal *s = pb->get_signal();
		for (ds_structural::sp_container::const_iterator pi=s->port_begin();pi!=s->port_end();pi++){
			PortBit *d = *pi;
			if (d->get_type() == ds_structural::DIR_OUT && d!=pb){

				LGNode *driverNode = d->get_gate()->get_lgn();	// locate LGN of driving gate
				out->inputs.push_back(driverNode);				// update LGN's inputs and outputs
				driverNode->outputs.push_back(out);
				ds_lg::lg_v64** in = out->get_input("a");		//primitive inputs are 'a'
				std::string m = d->get_gate()->get_mapping(d->get_instance_name());
				ds_lg::lg_v64 *driver = driverNode->get_output(m);
				*in = driver;

			}
		}
	}

	for (ds_structural::PortBit* pb: inputs)
	{
		ds_lg::Input *in = new ds_lg::Input();				// create input
		in->set_gate(this);									// outputs ports belong to the enclosing netlist
		in->set_name(pb->get_instance_name());
		lg->add_input(in);
		in->level = 0;										// inputs have depth 0
		ds_lg::lg_v64 *driver = in->get_output("o");		//primitive outputs are 'o'
		trace_lg_forward(pb, in, driver, &trace, &todo);	//trace inputs forward
		Signal *s = pb->get_signal();
		if (s!=0) {
			for (auto pi=s->port_begin();pi!=s->port_end();pi++){	// setup input and output nodes
				PortBit *pb = *pi;
				if (pb->get_type() == ds_structural::DIR_IN){
					ds_lg::LGNode* receiver = pb->get_gate()->get_lgn();
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
		LGNode *lgn = g->get_lgn();
		for (auto pi=g->get_outputs()->begin();pi!=g->get_outputs()->end();pi++){
			PortBit *pb = *pi;
			std::string port_name = g->get_mapping(pb->get_instance_name());
			ds_lg::lg_v64 *driver = lgn->get_output(port_name);
			trace_lg_forward(pb, lgn, driver, &trace, &todo);
		}
	}

	// include state inputs in level list
	for (ds_structural::Gate *g: state_gates)
	{
		LGNode *lgn = g->get_lgn();
		for (ds_lg::LGNode *in: lgn->inputs)
		{
			if (!in->has_state())
				level_list.push_back(in);
		}
	}

	// level gates: push them onto stack till a levelized node is found. Then calculate the levels for the nodes in between
	for (ds_lg::LGNode *lgn: level_list)
	{
		std::stack<LGNode*> st;
		st.push(lgn);
		while (!st.empty()){

			LGNode *t = st.top();

			int max_level = 0;
			std::size_t stack_size = st.size();
			for (ds_lg::LGNode *in: t->inputs)
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
	ds_lg::lg_output_container::iterator oi = lg->get_outputs_begin();
	for (;oi!=lg->get_outputs_end();oi++){
		if ((*oi)->level > max_level)
			max_level = (*oi)->level;
	}
	lg->num_levels = max_level + 1;

	// build level map and fill level map with inputs and outputs
	std::map<int, std::set<LGNode*>* > level_map;
	for (int i=0;i<max_level+1;i++){
		level_map[i] = new std::set<LGNode*>();
	}
	for (ds_lg::LGNode *in: lg->inputs)
	{
		level_map[in->level]->insert(in);
	}
	for (ds_lg::LGNode *out: lg->outputs)
	{
		level_map[out->level]->insert(out);
	}
	// fill level map with traced nodes
	for (std::map<std::string, Gate*>::iterator gi = trace.begin(); gi!=trace.end();gi++){
		LGNode *lgn = gi->second->get_lgn();
		if (lgn->level > 0) {
			level_map[lgn->level]->insert(lgn);

		}
	}

	// add nodes in level order to the nodes container
	for (int i=0;i<max_level+1;i++){
		std::set<LGNode*> *set = level_map[i];
		std::set<LGNode*>::iterator it = set->begin();
		for (;it!=set->end();it++){
			LGNode *n = *it;
			lg->add_node(n);
			n->set_leveled_graph(lg);
		}
		delete set;
	}

	//find iterators pointing to the first element of each level
	for  (int i=0;i<max_level+1;i++){
		ds_lg::lg_node_container::iterator it = std::find_if(lg->nodes.begin(), lg->nodes.end(),
				[&] (const LGNode* n) { return n->level == i;});
		lg->levels.push_back(it);
	}

	//set up intermediate simulation vectors
	for  (int i=0;i<max_level+1;i++){
		lg->simulation.push_back(new ds_lg::lg_node_container());
	}

	// count nodes per level
	for (int i=0;i<max_level+1;i++){
		auto cIt = lg->levels[i];
		auto nIt = lg->nodes.end();
		if (i<max_level)
			nIt = lg->levels[i+1];
		unsigned int level_size = 0;
		while (cIt!=nIt){
			level_size++;
			cIt++;
		}

		lg->level_width.push_back(level_size);
	}

	level_map.clear();

	//drive constant '0' assignments
	Signal *s0 = find_signal(ds_library::value_0);
	if (s0!=0){
		for (auto pi=s0->port_begin();pi!=s0->port_end();pi++){
			drive(*pi, &lg->constant_0);
		}

	}

	//drive constant '1' assignments
	Signal *s1 = find_signal(ds_library::value_1);
	if (s1!=0){
		for (auto pi=s1->port_begin();pi!=s1->port_end();pi++){
			drive(*pi, &lg->constant_1);
		}
	}

	//identify state elements
	for (LGNode *n:lg->nodes){
		if (n->has_state())
			lg->registers.push_back(n);
	}

	//check graph integrity
	bool c = lg->sanity_check();

	if (!c)
		BOOST_LOG_TRIVIAL(warning) << "Sanity check failed";


	return lg;
}

void ds_structural::NetList::drive(PortBit*pb, ds_lg::lg_v64 *driver){

	// get receiver simulation node
	Gate *g = pb->get_gate();
	std::string m = g->get_mapping(pb->get_instance_name());
	ds_lg::LGNode *target = g->get_lgn();
	// get the simulation input
	ds_lg::lg_v64** rec = target->get_input(m);
	// connect input to output
	*rec = driver;
}

void ds_structural::NetList::trace_lg_forward(const ds_structural::PortBit *port_bit, ds_lg::LGNode* node,
		ds_lg::lg_v64 *driver, std::map<std::string, Gate*> *trace, std::stack<Gate*> *todo){

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
			drive(pb, driver);

			// if receiver gate has not been traced, schedule it for tracing
			Gate *g = pb->get_gate();
			ds_lg::LGNode *target = g->get_lgn();
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
		ds_lg::LGNode* lgn = wp->get_primitive(g->get_type(), g->get_num_ports());
		g->set_lgn(lgn);
	}




	return nl;
}
