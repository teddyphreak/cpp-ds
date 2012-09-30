/*
 * ds_structural.cpp
 *
 *  Created on: Sep 18, 2012
 *      Author: cookao
 */
#include "ds_structural.h"
#include "ds_lg.h"

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
	}
	for (port_container::iterator it=outputs.begin();it!=outputs.end();it++){
		PortBit *pb = *it;
		PortBit *c = new PortBit(pb->get_instance_name(), g, pb->get_type());
		g->add_port(c);
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
