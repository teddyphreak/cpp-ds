/*
 * ds_structural.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: cookao
 */
#include "ds_structural.h"
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/foreach.hpp>
#include "ds_lg.h"
#include <queue>

using ds_structural::Gate;
using ds_structural::NetList;
using ds_structural::Signal;
using ds_structural::PortBit;

void ds_structural::Gate::copy(ds_structural::Gate* g){
	g->set_instance_name(name);
	g->set_type(type);
	g->set_lgn(lgn->clone());
	for (port_container::iterator it=inputs.begin();it!=inputs.end();it++){
		PortBit *pb = *it;
		PortBit *c = new PortBit(pb->get_instance_name(), g, pb->get_type());
		g->add_port(c);
		c->setGate(g);
	}
	for (port_container::iterator it=outputs.begin();it!=outputs.end();it++){
		PortBit *pb = *it;
		PortBit *c = new PortBit(pb->get_instance_name(), g, pb->get_type());
		g->add_port(c);
		c->setGate(g);
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
	copy(nl);
	typedef ds_structural::gate_map_t::iterator GATE_IT;
	for (GATE_IT gate_it = gates.begin();gate_it!=gates.end();gate_it++){
		Gate *g= gate_it->second;
		nl->add_gate(g->clone());
	}
	nl->signal_counter = signal_counter;
	typedef ds_structural::signal_map_t::iterator SIG_IT;
	for (SIG_IT sig_it = signals.begin();sig_it!=signals.end();sig_it++){
		Signal *s = sig_it->second;
		Signal *copy = new Signal(s->get_instance_name());
		typedef std::list<PortBit*>::const_iterator PORT_IT;
		for (PORT_IT port_it = s->port_begin(); port_it != s->port_end(); port_it++){
			PortBit *port = *port_it;
			std::string gate_name = port->get_gate()->get_instance_name();
			Gate *g = find_gate(name);
			PortBit *targetPort = g->find_port_by_name(port->get_instance_name());
			copy->add_port(targetPort);
		}
		nl->add_signal(copy);
	}
	for (SIG_IT sig_it = own_signals.begin();sig_it!=own_signals.end();sig_it++){
		Signal *s = sig_it->second;
		Signal *copy = new Signal(s->get_instance_name());
		typedef std::list<PortBit*>::const_iterator PORT_IT;
		for (PORT_IT port_it = s->port_begin(); port_it != s->port_end(); port_it++){
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
	typedef ds_structural::signal_map_t::iterator SIG_IT;
	typedef ds_structural::gate_map_t::iterator GATE_IT;
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
	for (SIG_IT it=signals.begin();it!=signals.end();it++){
		ds_structural::Signal *s = it->second;

		using boost::lambda::_1;
		ds_structural::PortBit *null = 0;
		sp_container::const_iterator npi = std::find_if(s->port_begin(), s->port_end(), _1==null);
		if (npi!=s->port_end()){
			check = false;
			std::cout << "Error: signal connected to null port " << s->get_instance_name() << std::endl;
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
				std::cout << "Error: no input port for " << s->get_instance_name() << std::endl;
			}
			if (pout==s->port_end()){
				check = false;
				std::cout << "Error: no output port for " << s->get_instance_name() << std::endl;
			}
		} else {
			if (in_p != input_ports.end() && out_p != output_ports.end()) {
				check = false;
				std::cout << "Error: signal connected to both input and output " << s->get_instance_name() << std::endl;
			}
			if (out_p==output_ports.end()){
				if (pout!=s->port_end()){
					check = false;
					std::cout << "Error: netlist input port driven by internal port " << s->get_instance_name() << std::endl;
				}
			}
			if (in_p==input_ports.end()){
				if (pout==s->port_end()){
					check = false;
					std::cout << "Error: netlist output port not driven by internal port " << s->get_instance_name() << std::endl;
				}
			}
		}
	}
	for (SIG_IT it=signals.begin();it!=signals.end();it++){
		ds_structural::Signal *s = it->second;
		std::list<PortBit*>::const_iterator port_it = s->port_begin();
		std::list<PortBit*>::const_iterator end = s->port_begin();
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
			std::cout << "Error: multiple signal drivers for " << s->get_instance_name() << std::endl;
		}
	}
	for (SIG_IT it=signals.begin();it!=signals.end();it++){
		ds_structural::Signal *s = it->second;
		for (std::list<PortBit*>::const_iterator p=s->port_begin();p!=s->port_end();p++){
			ds_structural::Gate *g = (*p)->get_gate();
			ds_structural::gate_map_t::iterator owned = gates.find(g->get_instance_name());
			if (owned == gates.end()){
				if (g != this){
					check = false;
					std::cout << "Error: Foreign gate" << g->get_instance_name() << std::endl;
				}
			}
		}
	}
	for (GATE_IT it=gates.begin();it!=gates.end();it++){
		ds_structural::Gate *g = it->second;
		ds_structural::port_container container;
		ds_structural::port_container::const_iterator in_it = g->get_inputs()->begin();
		for (;in_it!=g->get_inputs()->end();in_it++){
			const ds_structural::PortBit *pb = *in_it;
			ds_structural::Signal *s = pb->get_signal();
			SIG_IT owned = signals.find(s->get_instance_name());
			if (owned == signals.end()){
				check = false;
				std::cout << "Error: Foreign signal " << s->get_instance_name() << std::endl;
			}
		}
		ds_structural::port_container::const_iterator out_it = g->get_outputs()->begin();
		for (;out_it!=g->get_outputs()->end();out_it++)
		{
			const ds_structural::PortBit *pb = *out_it;
			ds_structural::Signal *s = pb->get_signal();
			SIG_IT owned = signals.find(s->get_instance_name());
			if (owned == signals.end()){
				check = false;
				std::cout << "Error: Foreign signal " << s->get_instance_name() << std::endl;
			}
		}
	}

	return check;
}

std::string ds_structural::PortBit::get_qualified_name() const {
	const Gate *g = get_gate();
	return g->get_instance_name() + "/" + get_instance_name();
}

ds_lg::LeveledGraph* ds_structural::NetList::build_leveled_graph(){
	using ds_lg::LGNode;
	using ds_structural::PortBit;
	using ds_structural::Gate;
	ds_lg::LeveledGraph *lg = new ds_lg::LeveledGraph();
	std::vector<Gate*> state_gates;
	std::stack<Gate*> todo;
	typedef ds_structural::gate_map_t::iterator GATE_IT;
	for (GATE_IT gate_it = gates.begin();gate_it!=gates.end();gate_it++){
		Gate* n = gate_it->second;
		if (n->get_lgn()->has_state()){
			state_gates.push_back(n);
			n->get_lgn()->level = 0;
			todo.push(n);
		}
	}

	std::map<std::string, Gate*> trace;
	std::list<LGNode*> levelize_list;

	BOOST_FOREACH(ds_structural::PortBit* pb, outputs)
	{
		ds_lg::Output *out = new ds_lg::Output();
		out->set_gate(this);
		out->set_name(pb->get_instance_name());
		lg->add_output(out);
		levelize_list.push_back(out);
		Signal *s = pb->get_signal();
		for (ds_structural::sp_container::const_iterator pi=s->port_begin();pi!=s->port_end();pi++){
			PortBit *d = *pi;
			if (d != pb){
				LGNode *driver = d->get_gate()->get_lgn();
				out->inputs.push_back(driver);
				driver->outputs.push_back(out);
			}
		}
	}

	BOOST_FOREACH(ds_structural::PortBit* pb, inputs)
	{
		ds_lg::Input *in = new ds_lg::Input();
		in->set_gate(this);
		in->set_name(pb->get_instance_name());
		lg->add_input(in);
		in->level = 0;
		ds_common::int64 *driver = in->get_output('o');
		trace_lg_forward(pb, in, driver, &trace, &todo);
		typedef ds_structural::sp_container::const_iterator PORT_IT;
		Signal *s = pb->get_signal();
		for (PORT_IT pi=s->port_begin();pi!=s->port_end();pi++){
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

	while (!todo.empty()){
		Gate *g = todo.top();
		todo.pop();
		LGNode *lgn = g->get_lgn();
		typedef port_container::const_iterator PORT_IT;
		for (PORT_IT pi=g->get_outputs()->begin();pi!=g->get_outputs()->end();pi++){
			PortBit *pb = *pi;
			std::string port_name = g->get_mapping(pb->get_instance_name());
			char port = port_name[0];
			ds_common::int64 *driver = lgn->get_output(port);
			trace_lg_forward(pb, lgn, driver, &trace, &todo);
		}
	}

	BOOST_FOREACH(ds_structural::Gate *g, state_gates)
	{
		LGNode *lgn = g->get_lgn();
		BOOST_FOREACH(ds_lg::LGNode *in, lgn->inputs)
		{
			levelize_list.push_back(in);
		}
	}

	BOOST_FOREACH(ds_lg::LGNode *lgn, levelize_list)
	{
		std::stack<LGNode*> st;
		st.push(lgn);
		while (!st.empty()){

			LGNode *t = st.top();
			int max_level = -1;
			std::size_t stack_size = st.size();
			BOOST_FOREACH(ds_lg::LGNode *in, t->inputs)
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

	int max_level = 0;
	ds_lg::lg_container::iterator oi = lg->get_outputs_begin();
	for (;oi!=lg->get_outputs_end();oi++){
		if ((*oi)->level > max_level)
			max_level = (*oi)->level;
	}

	lg->max_level = max_level;

	std::map<int, std::set<LGNode*>* > level_map;
	for (int i=0;i<max_level+1;i++){
		level_map[i] = new std::set<LGNode*>();
	}

	BOOST_FOREACH(ds_lg::LGNode *in, lg->inputs)
	{
		level_map[in->level]->insert(in);
	}
	BOOST_FOREACH(ds_lg::LGNode *out, lg->outputs)
	{
		level_map[out->level]->insert(out);
	}
	for (std::map<std::string, Gate*>::iterator gi = trace.begin();gi!=trace.end();gi++){
		LGNode *lgn = gi->second->get_lgn();
		level_map[lgn->level]->insert(lgn);
	}

	for (int i=0;i<max_level;i++){
		std::set<LGNode*> *set = level_map[i];
		std::set<LGNode*>::iterator it = set->begin();
		for (;it!=set->end();it++){
			LGNode *n = *it;
			lg->nodes.push_back(n);
		}
		delete set;
	}


	using boost::lambda::_1;
	for (int i=0;i<max_level+1;i++){
		ds_lg::lg_container::iterator it = std::find_if(lg->nodes.begin(), lg->nodes.end(), boost::lambda::bind(&ds_lg::LGNode::level,*_1)==i);
		lg->levels.push_back(it);
	}
	level_map.clear();

	bool c = lg->sanity_check();
	if (!c)
		std::cout << "Sanity check failed" << std::endl;

	return lg;
}

void ds_structural::NetList::trace_lg_forward(const ds_structural::PortBit *port_bit, ds_lg::LGNode* node,
		ds_common::int64 *driver, std::map<std::string, Gate*> *trace, std::stack<Gate*> *todo){
	using ds_lg::LGNode;
	using ds_structural::PortBit;
	using ds_structural::Gate;
	ds_structural::Signal *s = port_bit->get_signal();
	typedef ds_structural::sp_container::const_iterator PORT_IT;

	for (PORT_IT pi=s->port_begin();pi!=s->port_end();pi++){
		PortBit *pb = *pi;
		if (pb->get_type() == ds_structural::DIR_IN && pb != port_bit){

			Gate *g = pb->get_gate();
			std::string m = g->get_mapping(pb->get_instance_name());
			char c = m[0];
			LGNode *target = g->get_lgn();
			ds_lg::int64** rec = target->get_binding(c);
			*rec = driver;
			std::map<std::string, Gate*>::const_iterator ti = trace->find(g->get_instance_name());
			if (ti==trace->end()){
				todo->push(g);
				(*trace)[g->get_instance_name()] = g;
			}
			typedef std::vector<LGNode*>::iterator DS_IT;
			DS_IT oi = std::find(node->outputs.begin(), node->outputs.end(), target);
			if (oi == node->outputs.end()){
				node->outputs.push_back(target);
				target->inputs.push_back(node);
			}
		}
	}

}
