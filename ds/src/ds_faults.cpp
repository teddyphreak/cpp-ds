/*
 * ds_faults.cpp
 *
 *  Created on: 31.10.2013
 *      Author: cookao
 */
#include "ds_faults.h"
#include "ds_library.h"
#include "ds_workspace.h"
#include "ds_structural.h"
#include "ds_simulation.h"

using ds_faults::SAFaultDescriptor;

void ds_faults::get_fault_classes(ds_structural::Gate* g, std::vector<std::list<SAFaultDescriptor> >& fault_classes){
	ds_workspace::Workspace *workspace = ds_workspace::Workspace::get_workspace();
	std::string gate_type = g->get_type();
	ds_library::LogicFunction f = workspace->get_function(gate_type);
	switch(f){
	case ds_library::BUF:
		ds_faults::get_gate_faults(g, ds_simulation::BIT_0, ds_simulation::BIT_0, fault_classes);
		break;
	case ds_library::NOT:
		ds_faults::get_gate_faults(g, ds_simulation::BIT_0, ds_simulation::BIT_1, fault_classes);
		break;
	case ds_library::AND:
		ds_faults::get_gate_faults(g, ds_simulation::BIT_0, ds_simulation::BIT_0, fault_classes);
		break;
	case ds_library::OR:
		ds_faults::get_gate_faults(g, ds_simulation::BIT_1, ds_simulation::BIT_0, fault_classes);
		break;
	case ds_library::NAND:
		ds_faults::get_gate_faults(g, ds_simulation::BIT_0, ds_simulation::BIT_1, fault_classes);
		break;
	case ds_library::NOR:
		ds_faults::get_gate_faults(g, ds_simulation::BIT_0, ds_simulation::BIT_0, fault_classes);
		break;
	default:
		const ds_structural::port_container *inputs = g->get_inputs();
		const ds_structural::port_container *outputs = g->get_outputs();
		ds_faults::get_port_faults(inputs->begin(), inputs->end(), fault_classes);
		ds_faults::get_port_faults(outputs->begin(), outputs->end(), fault_classes);
		break;
	}
}

void ds_faults::get_gate_faults(ds_structural::Gate* g, ds_simulation::Value c, ds_simulation::Value i, std::vector<std::list<SAFaultDescriptor> >& fault_classes){

	using ds_structural::PortBit;
	// inverted controlling value
	ds_simulation::Value nc = (c == ds_simulation::BIT_0) ? ds_simulation::BIT_1: ds_simulation::BIT_0;
	//controlling value xor inverting value
	ds_simulation::Value ci = (c == i) ? ds_simulation::BIT_0 : ds_simulation::BIT_1;
	//controlling value xnor inverting value
	ds_simulation::Value nci = (c != i) ? ds_simulation::BIT_0 : ds_simulation::BIT_1;

	//get gate's output port
	PortBit *output_port = *(g->get_outputs()->begin());

	const ds_structural::port_container *ports = g->get_inputs();
	if (ports->size()==1){

		//single input gate (buf/not)

		//get gate's input port
		PortBit *input_port = *(g->get_inputs()->begin());

		//first equivalent class
		std::list<SAFaultDescriptor> f1_class;
		SAFaultDescriptor fic(g->get_instance_name(), output_port->get_instance_name(), c);
		SAFaultDescriptor foci(g->get_instance_name(), input_port->get_instance_name(), ci);
		f1_class.push_back(fic);
		f1_class.push_back(foci);
		fault_classes.push_back(f1_class);

		//second equivalent class
		std::list<SAFaultDescriptor> f2_class;
		SAFaultDescriptor finc(g->get_instance_name(), output_port->get_instance_name(), nc);
		SAFaultDescriptor fonci(g->get_instance_name(), input_port->get_instance_name(), nci);
		f2_class.push_back(finc);
		f2_class.push_back(fonci);
		fault_classes.push_back(f2_class);

	} else {
		// generate one stuck-at-not-c at each input
		for (auto it = ports->begin();it!=ports->end();it++){
			PortBit *pb = *it;
			SAFaultDescriptor f(g->get_instance_name(), pb->get_instance_name(), nc);
			std::list<SAFaultDescriptor> f_class;
			f_class.push_back(f);
			fault_classes.push_back(f_class);
		}
		// generate one stuck-at-c-xnor-i at output
		SAFaultDescriptor f(g->get_instance_name(), output_port->get_instance_name(), nci);
		std::list<SAFaultDescriptor> f_class;
		f_class.push_back(f);
		fault_classes.push_back(f_class);

		//generate equivalence class
		//generate one stuck-at-c-xor-i at output
		SAFaultDescriptor fo(g->get_instance_name(), output_port->get_instance_name(), ci);
		std::list<SAFaultDescriptor> equivalent;
		equivalent.push_back(fo);
		for (auto it = ports->begin();it!=ports->end();it++){
			PortBit *pb = *it;
			//generate one stuck-at-c at each input port
			SAFaultDescriptor f(g->get_instance_name(), pb->get_instance_name(), c);
			equivalent.push_back(f);
		}
		fault_classes.push_back(equivalent);
	}
}

void ds_faults::get_port_faults(ds_structural::port_container::const_iterator begin, ds_structural::port_container::const_iterator end, std::vector<std::list<SAFaultDescriptor> >& fault_classes){
	// no equivalent faults
	for (auto it = begin;it!=end;it++){
		ds_structural::PortBit *pb = *it;

		// generate stuck-at-0 at each port
		SAFaultDescriptor f0(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_simulation::BIT_0);
		std::list<SAFaultDescriptor> f0_class;
		f0_class.push_back(f0);
		fault_classes.push_back(f0_class);

		// generate stuck-at-1 at each port
		SAFaultDescriptor f1(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_simulation::BIT_1);
		std::list<SAFaultDescriptor> f1_class;
		f1_class.push_back(f1);
		fault_classes.push_back(f1_class);
	}
}

void ds_faults::get_fault_classes(ds_structural::NetList* nl, std::map<SAFaultDescriptor, std::list<SAFaultDescriptor> >& representatives){
	std::vector<ds_structural::Gate*> gates;
	nl->get_gates(gates);
	std::cout << "#gates " << gates.size() << std::endl;
	std::vector<std::list<SAFaultDescriptor> > fault_classes;
	std::map<SAFaultDescriptor, SAFaultDescriptor> descriptor_map;
	for (ds_structural::Gate* g:gates){
		ds_faults::get_fault_classes(g, fault_classes);
	}
	std::vector<const ds_structural::PortBit*> ports;
	const ds_structural::port_container* inputs = nl->get_inputs();
	const ds_structural::port_container* outputs = nl->get_outputs();
	ports.insert(ports.begin(), inputs->begin(), inputs->end());
	ports.insert(ports.begin(), outputs->begin(), outputs->end());
	for (const ds_structural::PortBit *pb: ports){
		SAFaultDescriptor sa0 = SAFaultDescriptor(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_simulation::BIT_0);
		std::list<SAFaultDescriptor> list0;
		list0.push_back(sa0);
		SAFaultDescriptor sa1 = SAFaultDescriptor(pb->get_gate()->get_instance_name(), pb->get_instance_name(), ds_simulation::BIT_1);
		std::list<SAFaultDescriptor> list1;
		list1.push_back(sa1);
		fault_classes.push_back(list0);
		fault_classes.push_back(list1);
	}
	for (auto it=fault_classes.begin();it!=fault_classes.end();it++){
		std::list<SAFaultDescriptor> f_class = *it;
		SAFaultDescriptor rep = *f_class.begin();
		representatives[rep] = f_class;
		for (SAFaultDescriptor d:f_class){
			descriptor_map[d] = rep;
		}
	}
	std::cout << "# rep faults " << representatives.size() << std::endl;
	std::vector<SAFaultDescriptor> remove;
	std::vector<ds_structural::Signal*> signals;
	nl->get_signals(signals);
	for (ds_structural::Signal* s:signals){
		if (s->count_ports() == 2){
			auto it =  s->port_begin();
			ds_structural::PortBit *first = *it;
			ds_structural::PortBit *second = *(++it);

			SAFaultDescriptor aggressor1 = SAFaultDescriptor(first->get_gate()->get_instance_name(), first->get_instance_name(), ds_simulation::BIT_1);
			SAFaultDescriptor aggressor0 = SAFaultDescriptor(first->get_gate()->get_instance_name(), first->get_instance_name(), ds_simulation::BIT_0);
			SAFaultDescriptor victim1 = SAFaultDescriptor(second->get_gate()->get_instance_name(), second->get_instance_name(), ds_simulation::BIT_1);
			SAFaultDescriptor victim0 = SAFaultDescriptor(second->get_gate()->get_instance_name(), second->get_instance_name(), ds_simulation::BIT_0);

			std::map<SAFaultDescriptor, SAFaultDescriptor>::iterator f;

			f=descriptor_map.find(aggressor1);
			if (f==descriptor_map.end()){
				std::cout << "aggressor 1 not found: " << aggressor1.gate_name << " " << aggressor1.port_name << std::endl;
			}
			f=descriptor_map.find(aggressor0);
			if (f==descriptor_map.end()){
				std::cout << "aggressor 0 not found: " << aggressor0.gate_name << " " << aggressor0.port_name << std::endl;
			}
			f=descriptor_map.find(victim1);
			if (f==descriptor_map.end()){
				std::cout << "victim 1 not found " << std::endl;
			}
			f=descriptor_map.find(victim0);
			if (f==descriptor_map.end()){
				std::cout << "victim 0 not found " << std::endl;
			}
			SAFaultDescriptor rep_aggressor1 = descriptor_map[aggressor1];
			SAFaultDescriptor rep_aggressor0 = descriptor_map[aggressor0];
			SAFaultDescriptor rep_victim1 = descriptor_map[victim1];
			SAFaultDescriptor rep_victim0 = descriptor_map[victim0];

			std::map<SAFaultDescriptor, std::list<SAFaultDescriptor> >::iterator g;
			g=representatives.find(rep_aggressor1);
			if (g==representatives.end()){
				std::cout << "rep aggressor 1 not found: " << aggressor1.gate_name << " " << aggressor1.port_name << std::endl;
			}
			g=representatives.find(rep_aggressor0);
			if (g==representatives.end()){
				std::cout << "rep aggressor 0 not found: " << aggressor0.gate_name << " " << aggressor0.port_name << std::endl;
			}
			g=representatives.find(rep_victim1);
			if (g==representatives.end()){
				std::cout << "rep victim 1 not found: " << rep_victim1.gate_name << " " << rep_victim1.port_name << std::endl;
			}
			g=representatives.find(rep_victim0);
			if (g==representatives.end()){
				std::cout << "rep victim 0 not found: " << rep_victim0.gate_name << " " << rep_victim0.port_name << std::endl;
			}

			std::list<SAFaultDescriptor> equivalent1 = representatives[rep_aggressor1];
			std::list<SAFaultDescriptor> equivalent0 = representatives[rep_aggressor0];
			std::list<SAFaultDescriptor> merge1 = representatives[rep_victim1];
			std::list<SAFaultDescriptor> merge0 = representatives[rep_victim0];

			equivalent0.insert(equivalent0.begin(), merge0.begin(), merge0.end());
			equivalent1.insert(equivalent1.begin(), merge1.begin(), merge1.end());

			representatives[rep_aggressor0] = equivalent0;
			representatives[rep_aggressor1] = equivalent1;

			for (SAFaultDescriptor eq:merge1){
				descriptor_map[eq] = rep_aggressor1;
			}

			for (SAFaultDescriptor eq:merge0){
				descriptor_map[eq] = rep_aggressor0;
			}

			representatives.erase(rep_victim0);
			representatives.erase(rep_victim1);

			remove.push_back(rep_victim0);
			remove.push_back(rep_victim1);
		}
	}
	std::cout << "removed: " << remove.size() << " size " << representatives.size() << std::endl;

}

ds_lg::lg_v64* ds_faults::resolve(const PinReference *pr, ds_lg::LeveledGraph *lg){
	std::string name = pr->get_gate_name();
	std::string port = pr->get_port_name();
	ds_lg::LGNode *n = lg->get_node(name);
	if (n!=0){
		if (pr->is_input()){
			ds_lg::lg_v64 *p = *n->get_input(name);
			return p;
		}
		if (pr->is_output()){
			ds_lg::lg_v64 *p = n->get_output(port);
			return p;
		}
	} else {
		n = lg->get_node(port);
		if (pr->is_input()){
			return *n->get_input("o");
		}
		if (pr->is_output()){
			return n->get_output("a");
		}
	}
	return 0;
}
